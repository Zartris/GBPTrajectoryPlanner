use gbp_comms::{CommsInterface, ObservationUpdate, RobotBroadcast, RobotId};
use gbp_core::{Factor, FactorGraph, FactorKind, DynamicsFactor, InterRobotFactor, VelocityBoundFactor};
use gbp_map::{Map, MAX_HORIZON, MAX_NEIGHBOURS};
use gbp_map::map::EdgeId;
use heapless::Vec;
use crate::trajectory::Trajectory;
use crate::interrobot_set::InterRobotFactorSet;
use crate::dynamic_constraints::DynamicConstraints;

/// Number of dynamics factors = K-1 (one per adjacent timestep pair)
const NUM_DYN_FACTORS: usize = MAX_HORIZON - 1;
/// Max inter-robot factors = one per variable per neighbour (K * MAX_NEIGHBOURS)
const MAX_IR_FACTORS: usize = MAX_HORIZON * MAX_NEIGHBOURS;
/// Number of velocity bound factors = K-1 (one per adjacent timestep pair)
const NUM_VEL_BOUND_FACTORS: usize = MAX_HORIZON - 1;
/// Max total factors = dynamics + velocity bound + inter-robot
const MAX_FACTORS: usize = NUM_DYN_FACTORS + NUM_VEL_BOUND_FACTORS + MAX_IR_FACTORS;

/// Internal iterations (dynamics + velocity bound) per external iteration (inter-robot).
const GBP_INTERNAL_ITERS: usize = 3;
/// External iterations (inter-robot factors). Total iterations = (INTERNAL+1) * EXTERNAL.
const GBP_EXTERNAL_ITERS: usize = 5;
const DT: f32 = 0.1; // seconds per GBP timestep
const D_SAFE: f32 = 1.3; // minimum center-to-center clearance (m) — chassis 1.15m + 0.15m margin
const SIGMA_R: f32 = 0.12; // inter-robot factor noise (precision ≈ 69 vs dynamics precision = 4)
const IR_RAMP_START: f32 = 2.6; // 2× d_safe — distance where IR factor starts ramping up
const IR_ACTIVATION_RANGE: f32 = 3.0; // distance where IR factors are spawned/despawned
const AGENT_DT: f32 = 0.02; // 50 Hz agent tick

/// Output of one agent step.
pub struct StepOutput {
    pub velocity: f32,
    pub raw_gbp_velocity: f32,
    pub position_s: f32,
    pub current_edge: EdgeId,
    pub min_neighbour_dist_3d: f32,
    /// Belief means for variables 0..K (for diagnostics/visualiser)
    pub belief_means: [f32; MAX_HORIZON],
    /// Belief spread: max(means) - min(means). Large values indicate oscillation.
    pub belief_spread: f32,
}

pub struct RobotAgent<C: CommsInterface> {
    robot_id:  RobotId,
    comms:     C,
    map:       *const Map,
    graph:     FactorGraph<MAX_HORIZON, MAX_FACTORS>,
    trajectory: Option<Trajectory>,
    dyn_indices: [usize; NUM_DYN_FACTORS],
    vel_bound_indices: [usize; NUM_VEL_BOUND_FACTORS],
    ir_set: InterRobotFactorSet,
    position_s:   f32,
    current_edge: EdgeId,
    /// 3D world position (updated each step from map eval)
    pos_3d: [f32; 3],
    /// Post-GBP dynamic constraints (jerk, accel, speed limits).
    /// NOT part of the factor graph — applied after GBP solves for trajectory.
    constraints: DynamicConstraints,
    /// Cached last-known broadcasts — avoids dropping IR factors on empty ticks.
    last_broadcasts: Vec<RobotBroadcast, MAX_NEIGHBOURS>,
    /// Maximum allowed position (from safety cap). f32::MAX if unconstrained.
    last_max_position: f32,
}

// SAFETY: Map pointer is valid for the lifetime of the agent.
// RobotAgent is not Send/Sync in general, but this is needed for test compilation.
unsafe impl<C: CommsInterface + Send> Send for RobotAgent<C> {}

impl<C: CommsInterface> RobotAgent<C> {
    pub fn new(comms: C, map: &Map, robot_id: RobotId) -> Self {
        let mut graph: FactorGraph<MAX_HORIZON, MAX_FACTORS> = FactorGraph::new(0.0, 100.0);
        let mut dyn_indices = [0usize; NUM_DYN_FACTORS];

        // Add K-1 permanent dynamics factors
        for k in 0..NUM_DYN_FACTORS {
            let idx = graph.add_factor(FactorKind::Dynamics(
                DynamicsFactor::new([k, k + 1], DT, 0.5, 0.0)
            )).expect("BUG: MAX_FACTORS < K+1 dynamics factors — check capacity constants");
            dyn_indices[k] = idx;
        }

        let mut vel_bound_indices = [0usize; NUM_VEL_BOUND_FACTORS];
        for k in 0..NUM_VEL_BOUND_FACTORS {
            let idx = graph.add_factor(FactorKind::VelocityBound(
                VelocityBoundFactor::new([k, k + 1], DT, 2.5)
            )).expect("BUG: MAX_FACTORS too small for velocity bound factors");
            vel_bound_indices[k] = idx;
        }

        Self {
            robot_id,
            comms,
            map: map as *const Map,
            graph,
            trajectory: None,
            dyn_indices,
            vel_bound_indices,
            ir_set: InterRobotFactorSet::new(),
            position_s: 0.0,
            current_edge: EdgeId(0),
            pos_3d: [0.0; 3],
            constraints: DynamicConstraints::new(2.5, 5.0, 2.5),
            last_broadcasts: Vec::new(),
            last_max_position: f32::MAX,
        }
    }

    /// Assign a new planned trajectory as (edge_id, length) pairs starting at start_s.
    pub fn set_trajectory(&mut self, edges: Vec<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }>, start_s: f32) {
        self.trajectory = Some(Trajectory::new(edges, start_s));
    }

    /// Mutable access to comms (for broadcast_state in AgentRunner).
    pub fn comms_mut(&mut self) -> &mut C {
        &mut self.comms
    }

    /// Number of currently active inter-robot factors.
    pub fn interrobot_factor_count(&self) -> usize {
        self.ir_set.count()
    }

    /// Which variable timesteps have active IR factors (for visualiser).
    pub fn active_ir_timesteps(&self) -> Vec<u8, MAX_HORIZON> {
        let mut ts = Vec::new();
        for &(_, k, _) in self.ir_set.iter() {
            if !ts.iter().any(|&t| t == k as u8) {
                let _ = ts.push(k as u8);
            }
        }
        ts
    }

    /// Current belief means for all K variables.
    pub fn belief_means(&self) -> [f32; MAX_HORIZON] {
        let mut means = [0.0f32; MAX_HORIZON];
        for (i, v) in self.graph.variables.iter().enumerate() {
            means[i] = v.mean();
        }
        means
    }

    /// Cavity belief means — marginal minus IR factor messages.
    /// This is what should be broadcast so neighbours don't double-count evidence.
    pub fn cavity_belief_means(&self) -> [f32; MAX_HORIZON] {
        let mut means = [0.0f32; MAX_HORIZON];
        for (i, v) in self.graph.variables.iter().enumerate() {
            // Start with marginal
            let mut cav_eta = v.eta;
            let mut cav_lambda = v.lambda;
            // Subtract IR factor messages on this variable
            for &(_, k, fidx) in self.ir_set.iter() {
                if k == i {
                    if let Some(fnode) = self.graph.factor_node(fidx) {
                        cav_eta -= fnode.msg_eta[0];
                        cav_lambda -= fnode.msg_lambda[0];
                    }
                }
            }
            // Cavity mean = cav_eta / cav_lambda
            if cav_lambda > 1e-10 {
                means[i] = cav_eta / cav_lambda;
            } else {
                means[i] = v.mean(); // fallback to marginal
            }
        }
        means
    }

    /// Cavity belief variances — marginal minus IR factor messages.
    pub fn cavity_belief_vars(&self) -> [f32; MAX_HORIZON] {
        let mut vars = [0.0f32; MAX_HORIZON];
        for (i, v) in self.graph.variables.iter().enumerate() {
            let mut cav_lambda = v.lambda;
            for &(_, k, fidx) in self.ir_set.iter() {
                if k == i {
                    if let Some(fnode) = self.graph.factor_node(fidx) {
                        cav_lambda -= fnode.msg_lambda[0];
                    }
                }
            }
            if cav_lambda > 1e-10 {
                vars[i] = (1.0 / cav_lambda).min(1e6);
            } else {
                vars[i] = v.variance().min(1e6);
            }
        }
        vars
    }

    pub fn last_max_position(&self) -> f32 { self.last_max_position }
    pub fn set_pos_3d(&mut self, pos: [f32; 3]) { self.pos_3d = pos; }

    /// Current belief variances (marginal, for visualiser).
    pub fn belief_vars(&self) -> [f32; MAX_HORIZON] {
        let mut vars = [0.0f32; MAX_HORIZON];
        for (i, v) in self.graph.variables.iter().enumerate() {
            vars[i] = v.variance().min(1e6);
        }
        vars
    }

    /// Run one step of GBP and return commanded velocity.
    pub fn step(&mut self, obs: ObservationUpdate) -> StepOutput {
        self.position_s   = obs.position_s;
        self.current_edge = obs.current_edge;

        let map = unsafe { &*self.map };

        // Note: pos_3d is set by AgentRunner from the outside (it knows local_s).
        // The agent only uses pos_3d for safety cap distance calculations.

        // 0. Anchor variable[0] at observed position (strong prior).
        // Set both prior AND current eta/lambda so variables[0].mean() returns
        // the correct position_s immediately (used by update_dynamics_v_nom at step 3).
        self.graph.variables[0].prior_eta = self.position_s * 1000.0;
        self.graph.variables[0].prior_lambda = 1000.0;
        self.graph.variables[0].eta = self.position_s * 1000.0;
        self.graph.variables[0].lambda = 1000.0;

        // 1. Receive broadcasts from neighbours. If none arrived this tick,
        // use the cached version so IR factors don't flicker on/off.
        let fresh = self.comms.receive_broadcasts();
        if !fresh.is_empty() {
            self.last_broadcasts = fresh;
        }
        let broadcasts = self.last_broadcasts.clone();

        // 2. Update inter-robot factors (add/remove as planned edges change)
        self.update_interrobot_factors(&broadcasts, map);

        // 3. Update v_nom on dynamics factors from current trajectory
        self.update_dynamics_v_nom(map);

        // 4. Update inter-robot factor Jacobians and external beliefs
        self.update_interrobot_jacobians(&broadcasts, map);

        // 5. Run GBP
        self.graph.iterate_split(GBP_INTERNAL_ITERS, GBP_EXTERNAL_ITERS);

        // 6. Extract commanded velocity from GBP
        let s0 = self.graph.variables[0].mean();
        let s1 = self.graph.variables[1].mean();
        // Allow negative velocity (creep-back) — VelocityBoundFactor bounds at v_min=-0.3
        let raw_velocity = (s1 - s0) / DT;

        // 7. Post-GBP dynamic constraints (NOT part of factor graph).
        // Smooths the raw GBP velocity through jerk, accel, and speed limits.
        let max_speed = map.edges.iter()
            .find(|e| e.id == self.current_edge)
            .map(|e| e.speed.max)
            .unwrap_or(2.5);
        self.constraints.set_max_speed(max_speed);
        let velocity = self.constraints.apply(raw_velocity, AGENT_DT);

        // 8. Monitor 3D distance (diagnostic only — no clamp).
        // TODO: Replace with a proper CBF (Control Barrier Function) safety layer.
        let dist_3d = self.min_3d_distance_to_neighbours(&broadcasts);
        self.last_max_position = f32::MAX; // no position clamp — trust GBP + DynamicConstraints

        // 9. Broadcast state
        let _ = self.comms.broadcast(&self.make_broadcast(velocity));

        // 10. Compute belief diagnostics
        let means = self.belief_means();
        let mut bmin = f32::MAX;
        let mut bmax = f32::MIN;
        for &m in means.iter() {
            if m < bmin { bmin = m; }
            if m > bmax { bmax = m; }
        }

        StepOutput {
            velocity,
            raw_gbp_velocity: raw_velocity,
            position_s: self.position_s,
            current_edge: self.current_edge,
            min_neighbour_dist_3d: dist_3d,
            belief_means: means,
            belief_spread: bmax - bmin,
        }
    }


    /// Minimum 3D distance to any neighbour robot sharing planned edges.
    /// Returns f32::MAX if no neighbours are close.
    fn min_3d_distance_to_neighbours(&self, broadcasts: &Vec<RobotBroadcast, MAX_NEIGHBOURS>) -> f32 {
        let my_edges = self.planned_edge_ids();
        let mut min_dist = f32::MAX;
        for bcast in broadcasts.iter() {
            if bcast.robot_id == self.robot_id { continue; }
            // Only consider robots sharing edges
            let shares = bcast.planned_edges.iter().any(|e| my_edges.contains(e))
                || my_edges.contains(&bcast.current_edge);
            if !shares { continue; }

            let dx = self.pos_3d[0] - bcast.pos[0];
            let dy = self.pos_3d[1] - bcast.pos[1];
            let dz = self.pos_3d[2] - bcast.pos[2];
            let dist = libm::sqrtf(dx * dx + dy * dy + dz * dz);
            if dist < min_dist { min_dist = dist; }
        }
        min_dist
    }

    fn update_dynamics_v_nom(&mut self, map: &Map) {
        let traj = match &self.trajectory { Some(t) => t, None => return };
        let edge = map.edges.iter().find(|e| e.id == self.current_edge);
        let (nominal, decel) = edge.map(|e| (e.speed.nominal, e.speed.decel_limit))
            .unwrap_or((1.0, 1.0));

        for (k, &dyn_idx) in self.dyn_indices.iter().enumerate() {
            let s_k_global = self.position_s + self.graph.variables[k].mean() - self.graph.variables[0].mean();
            let v_nom = traj.v_nom_at(s_k_global, nominal, decel);
            if let Some(FactorKind::Dynamics(df)) = self.graph.get_factor_kind_mut(dyn_idx) {
                df.set_v_nom(v_nom);
            }
        }

        let max_speed = edge.map(|e| e.speed.max).unwrap_or(2.5);
        for &vb_idx in self.vel_bound_indices.iter() {
            if let Some(FactorKind::VelocityBound(vbf)) = self.graph.get_factor_kind_mut(vb_idx) {
                vbf.set_v_max(max_speed);
            }
        }
    }

    fn update_interrobot_factors(
        &mut self,
        broadcasts: &Vec<RobotBroadcast, MAX_NEIGHBOURS>,
        map: &Map,
    ) {
        let my_edges = self.planned_edge_ids();
        let mut active_ids: Vec<RobotId, MAX_NEIGHBOURS> = Vec::new();

        for bcast in broadcasts.iter() {
            if bcast.robot_id == self.robot_id { continue; }

            let shares_edge = bcast.planned_edges.iter().any(|e| my_edges.contains(e))
                || my_edges.contains(&bcast.current_edge);
            if !shares_edge {
                // Remove any existing factors for this robot
                if self.ir_set.contains_robot(bcast.robot_id) {
                    self.ir_set.remove_robot(bcast.robot_id, &mut self.graph);
                }
                continue;
            }

            let _ = active_ids.push(bcast.robot_id);
            let activation_range = IR_ACTIVATION_RANGE;

            // For each variable k (skip k=0 — anchor prior is too strong, factor would
            // have no effect), check if predicted positions are close enough in 3D.
            for k in 1..MAX_HORIZON {
                let my_s_k = self.graph.variables[k].mean();
                let their_s_k = if k < bcast.belief_means.len() {
                    bcast.belief_means[k]
                } else {
                    bcast.position_s
                };

                // Convert both arc-lengths to 3D positions via trajectory
                let my_pos = self.trajectory.as_ref()
                    .and_then(|t| t.edge_and_local_s(my_s_k))
                    .and_then(|(eid, ls, _)| map.eval_position(eid, ls))
                    .unwrap_or(self.pos_3d);

                // Convert B's predicted position at timestep k to 3D using B's planned edges
                let their_pos = {
                    let mut cumulative = 0.0f32;
                    let mut found = bcast.pos; // fallback to B's current position
                    for &eid in bcast.planned_edges.iter() {
                        if let Some(idx) = map.edge_index(eid) {
                            let len = map.edges[idx].geometry.length();
                            if their_s_k < cumulative + len {
                                let local = their_s_k - cumulative;
                                if let Some(p) = map.eval_position(eid, local) {
                                    found = p;
                                }
                                break;
                            }
                            cumulative += len;
                        }
                    }
                    found
                };

                let dx = my_pos[0] - their_pos[0];
                let dy = my_pos[1] - their_pos[1];
                let dz = my_pos[2] - their_pos[2];
                let dist = libm::sqrtf(dx * dx + dy * dy + dz * dz);

                if dist < activation_range {
                    // Add factor at this timestep if we don't have one already
                    if !self.ir_set.contains(bcast.robot_id, k) {
                        let factor = InterRobotFactor::new(k, D_SAFE, SIGMA_R);
                        if let Ok(idx) = self.graph.add_factor(FactorKind::InterRobot(factor)) {
                            if !self.ir_set.insert(bcast.robot_id, k, idx) {
                                // IR set full — remove orphaned factor
                                self.graph.remove_factor(idx);
                            }
                        }
                    }
                } else {
                    // Too far apart at this timestep — remove factor if it exists
                    if self.ir_set.contains(bcast.robot_id, k) {
                        self.ir_set.remove_single(bcast.robot_id, k, &mut self.graph);
                    }
                }
            }
        }

        // Remove all factors for robots no longer sharing edges
        let mut to_remove: Vec<RobotId, MAX_NEIGHBOURS> = Vec::new();
        for &(rid, _, _) in self.ir_set.iter() {
            if !active_ids.contains(&rid) && !to_remove.contains(&rid) {
                let _ = to_remove.push(rid);
            }
        }
        for rid in to_remove.iter() {
            self.ir_set.remove_robot(*rid, &mut self.graph);
        }
    }

    fn update_interrobot_jacobians(
        &mut self,
        broadcasts: &Vec<RobotBroadcast, MAX_NEIGHBOURS>,
        map: &Map,
    ) {
        /// Front robot gets a dampened push — yielding matters more than fleeing.
        /// 1.0 = full push (no dampening), 0.0 = no push at all.
        const FRONT_DAMPING: f32 = 0.3;

        for bcast in broadcasts.iter() {
            if bcast.robot_id == self.robot_id { continue; }

            // Collect (k, factor_idx) pairs for this robot to avoid borrow issues
            let mut pairs: Vec<(usize, usize), MAX_HORIZON> = Vec::new();
            for (k, fidx) in self.ir_set.factors_for(bcast.robot_id) {
                let _ = pairs.push((k, fidx));
            }

            for &(k, factor_idx) in pairs.iter() {
                let s_a = self.graph.variables[k].mean();
                let s_b = if k < bcast.belief_means.len() {
                    bcast.belief_means[k]
                } else {
                    bcast.position_s
                };

                // Compute 3D positions at timestep k for both robots
                let my_pos = self.trajectory.as_ref()
                    .and_then(|t| t.edge_and_local_s(s_a))
                    .and_then(|(eid, ls, _)| map.eval_position(eid, ls))
                    .unwrap_or(self.pos_3d);

                let my_tangent = self.trajectory.as_ref()
                    .and_then(|t| t.edge_and_local_s(s_a))
                    .and_then(|(eid, ls, _)| map.eval_tangent(eid, ls))
                    .unwrap_or([1.0, 0.0, 0.0]);

                // B's predicted 3D position at timestep k (walk B's planned edges)
                let their_pos = {
                    let mut cumulative = 0.0f32;
                    let mut found = bcast.pos;
                    for &eid in bcast.planned_edges.iter() {
                        if let Some(idx) = map.edge_index(eid) {
                            let len = map.edges[idx].geometry.length();
                            if s_b < cumulative + len {
                                let local = s_b - cumulative;
                                if let Some(p) = map.eval_position(eid, local) {
                                    found = p;
                                }
                                break;
                            }
                            cumulative += len;
                        }
                    }
                    found
                };

                // B's tangent at timestep k (for Schur complement coupling)
                let their_tangent = {
                    let mut cumulative = 0.0f32;
                    let mut found = [1.0, 0.0, 0.0];
                    for &eid in bcast.planned_edges.iter() {
                        if let Some(idx) = map.edge_index(eid) {
                            let len = map.edges[idx].geometry.length();
                            if s_b < cumulative + len {
                                let local = s_b - cumulative;
                                if let Some(t) = map.eval_tangent(eid, local) {
                                    found = t;
                                }
                                break;
                            }
                            cumulative += len;
                        }
                    }
                    found
                };

                // 3D separation and distance
                let sep = [
                    my_pos[0] - their_pos[0],
                    my_pos[1] - their_pos[1],
                    my_pos[2] - their_pos[2],
                ];
                let dist_3d = libm::sqrtf(sep[0] * sep[0] + sep[1] * sep[1] + sep[2] * sep[2]);

                if let Some(FactorKind::InterRobot(irf)) = self.graph.get_factor_kind_mut(factor_idx) {
                    if dist_3d < 1e-6 {
                        irf.dist = 0.0;
                        irf.set_active(true);
                        continue;
                    }

                    // 3D Jacobians: d(dist_3d)/d(s_a) and d(dist_3d)/d(s_b)
                    let dot_a = sep[0] * my_tangent[0] + sep[1] * my_tangent[1] + sep[2] * my_tangent[2];
                    let dot_b = sep[0] * their_tangent[0] + sep[1] * their_tangent[1] + sep[2] * their_tangent[2];

                    let mut jac_a = dot_a / dist_3d;
                    let jac_b = -dot_b / dist_3d;

                    // Asymmetric dampening: front robot gets gentler push
                    if jac_a > 0.0 {
                        jac_a *= FRONT_DAMPING;
                    }

                    irf.jacobian_a = jac_a;
                    irf.jacobian_b = jac_b;
                    irf.dist = dist_3d;

                    // Set external belief of robot B at timestep k from its broadcast
                    if k < bcast.belief_means.len() && k < bcast.belief_vars.len() {
                        let ext_mean = bcast.belief_means[k];
                        let ext_var = bcast.belief_vars[k].max(1e-6);
                        irf.ext_lambda_b = 1.0 / ext_var;
                        irf.ext_eta_b = irf.ext_lambda_b * ext_mean;
                    }

                    // Activate factor only when robots are within activation range (3D)
                    irf.set_active(dist_3d < IR_ACTIVATION_RANGE);
                }
            }
        }
    }

    /// Get planned edge IDs from the trajectory.
    fn planned_edge_ids(&self) -> Vec<EdgeId, { gbp_map::MAX_PATH_EDGES }> {
        match &self.trajectory {
            Some(t) => t.edge_ids(),
            None => Vec::new(),
        }
    }

    fn make_broadcast(&self, velocity: f32) -> RobotBroadcast {
        // Broadcast MARGINAL beliefs (standard in distributed GBP).
        // Circular evidence exists but is accepted — message damping stabilises it.
        let mut means: Vec<f32, MAX_HORIZON> = Vec::new();
        let mut vars: Vec<f32, MAX_HORIZON> = Vec::new();
        for v in &self.graph.variables {
            let _ = means.push(v.mean());
            let _ = vars.push(v.variance().min(1e6));
        }
        RobotBroadcast {
            robot_id: self.robot_id,
            current_edge: self.current_edge,
            position_s: self.position_s,
            velocity,
            pos: self.pos_3d,
            planned_edges: self.planned_edge_ids_horizon(),
            belief_means: means,
            belief_vars: vars,
            gbp_timesteps: Vec::new(),
        }
    }

    /// Planned edge IDs with MAX_HORIZON capacity (for broadcasts).
    fn planned_edge_ids_horizon(&self) -> Vec<EdgeId, MAX_HORIZON> {
        let mut pe = Vec::new();
        if let Some(t) = &self.trajectory {
            for &eid in t.edge_ids().iter().take(MAX_HORIZON) {
                let _ = pe.push(eid);
            }
        }
        pe
    }
}
