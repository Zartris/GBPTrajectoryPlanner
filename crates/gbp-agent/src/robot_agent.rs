use gbp_comms::{CommsInterface, ObservationUpdate, RobotBroadcast, RobotId};
use gbp_core::{Factor, FactorGraph, FactorKind, DynamicsFactor, InterRobotFactor};
use gbp_map::{Map, MAX_HORIZON, MAX_NEIGHBOURS};
use gbp_map::map::EdgeId;
use heapless::Vec;
use crate::trajectory::Trajectory;
use crate::interrobot_set::InterRobotFactorSet;

/// Number of dynamics factors = K-1 (one per adjacent timestep pair)
const NUM_DYN_FACTORS: usize = MAX_HORIZON - 1;
/// Max inter-robot factors = one per variable per neighbour (K * MAX_NEIGHBOURS)
const MAX_IR_FACTORS: usize = MAX_HORIZON * MAX_NEIGHBOURS;
/// Max total factors = dynamics + inter-robot
const MAX_FACTORS: usize = NUM_DYN_FACTORS + MAX_IR_FACTORS;

const GBP_ITERATIONS: usize = 15;
const DT: f32 = 0.1; // seconds per GBP timestep
const D_SAFE: f32 = 0.3; // minimum clearance (m)
const SIGMA_R: f32 = 0.5; // inter-robot factor noise (weaker to avoid overwhelming dynamics)

/// Output of one agent step.
pub struct StepOutput {
    pub velocity: f32,
    pub position_s: f32,
    pub current_edge: EdgeId,
}

pub struct RobotAgent<C: CommsInterface> {
    robot_id:  RobotId,
    comms:     C,
    map:       *const Map,
    graph:     FactorGraph<MAX_HORIZON, MAX_FACTORS>,
    trajectory: Option<Trajectory>,
    dyn_indices: [usize; NUM_DYN_FACTORS],
    ir_set: InterRobotFactorSet,
    position_s:   f32,
    current_edge: EdgeId,
    /// 3D world position (updated each step from map eval)
    pos_3d: [f32; 3],
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

        Self {
            robot_id,
            comms,
            map: map as *const Map,
            graph,
            trajectory: None,
            dyn_indices,
            ir_set: InterRobotFactorSet::new(),
            position_s: 0.0,
            current_edge: EdgeId(0),
            pos_3d: [0.0; 3],
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

    /// Current belief means for all K variables.
    pub fn belief_means(&self) -> [f32; MAX_HORIZON] {
        let mut means = [0.0f32; MAX_HORIZON];
        for (i, v) in self.graph.variables.iter().enumerate() {
            means[i] = v.mean();
        }
        means
    }

    /// Current belief variances for all K variables.
    pub fn last_max_position(&self) -> f32 { self.last_max_position }
    pub fn set_pos_3d(&mut self, pos: [f32; 3]) { self.pos_3d = pos; }

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

        // 0. Anchor variable[0] at observed position (strong prior)
        self.graph.variables[0].prior_eta = self.position_s * 1000.0;
        self.graph.variables[0].prior_lambda = 1000.0;
        self.graph.variables[0].eta = self.graph.variables[0].prior_eta;
        self.graph.variables[0].lambda = self.graph.variables[0].prior_lambda;

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
        self.graph.iterate(GBP_ITERATIONS);

        // 6. Extract commanded velocity from first dynamics factor (s_1 - s_0) / dt
        let s0 = self.graph.variables[0].mean();
        let s1 = self.graph.variables[1].mean();
        let mut velocity = ((s1 - s0) / DT).max(0.0);

        // 7. Safety: 3D distance-based velocity cap.
        let dist_3d = self.min_3d_distance_to_neighbours(&broadcasts);
        if dist_3d <= D_SAFE {
            velocity = 0.0;
            self.last_max_position = self.position_s;
        } else if dist_3d < D_SAFE * 3.0 {
            let safety_factor = ((dist_3d - D_SAFE) / (D_SAFE * 2.0)).clamp(0.0, 1.0);
            velocity *= safety_factor;
            self.last_max_position = f32::MAX;
        } else {
            self.last_max_position = f32::MAX;
        }

        // 8. Broadcast state
        let _ = self.comms.broadcast(&self.make_broadcast(velocity));

        StepOutput { velocity, position_s: self.position_s, current_edge: self.current_edge }
    }

    /// Find the distance to the nearest robot AHEAD of us on the trajectory.
    /// Returns f32::MAX if no robot is ahead.
    fn nearest_ahead_distance(&self, broadcasts: &Vec<RobotBroadcast, MAX_NEIGHBOURS>) -> f32 {
        let mut min_ahead = f32::MAX;
        for bcast in broadcasts.iter() {
            if bcast.robot_id == self.robot_id { continue; }
            let gap = bcast.position_s - self.position_s; // positive = ahead
            if gap > -D_SAFE && gap < min_ahead {
                // Include robots slightly behind us too (within d_safe) to prevent
                // position overshoot from one-tick lag
                min_ahead = gap;
            }
        }
        min_ahead
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
    }

    fn update_interrobot_factors(
        &mut self,
        broadcasts: &Vec<RobotBroadcast, MAX_NEIGHBOURS>,
        _map: &Map,
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
            let activation_range = D_SAFE * 3.0;

            // For EACH variable k, check if our predicted position is close
            // to the neighbour's predicted position at the same timestep.
            for k in 0..MAX_HORIZON {
                let my_s_k = self.graph.variables[k].mean();
                let their_s_k = if k < bcast.belief_means.len() {
                    bcast.belief_means[k]
                } else {
                    bcast.position_s
                };
                let dist = (my_s_k - their_s_k).abs();

                if dist < activation_range {
                    // Add factor at this timestep if we don't have one already
                    if !self.ir_set.contains(bcast.robot_id, k) {
                        let factor = InterRobotFactor::new(k, D_SAFE, SIGMA_R);
                        if let Ok(idx) = self.graph.add_factor(FactorKind::InterRobot(factor)) {
                            self.ir_set.insert(bcast.robot_id, k, idx);
                        }
                    }
                } else {
                    // Too far apart at this timestep — remove factor if it exists
                    // (skip for now to avoid complex mid-array removal; factors deactivate via is_active)
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
        _map: &Map,
    ) {
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

                if let Some(FactorKind::InterRobot(irf)) = self.graph.get_factor_kind_mut(factor_idx) {
                    let diff = s_a - s_b;
                    let dist = diff.abs();

                    // Jacobian: d(dist)/d(s_a) = sign(s_a - s_b)
                    let sign = if diff >= 0.0 { 1.0 } else { -1.0 };
                    irf.jacobian_a = -sign; // negative because residual = d_safe - dist
                    irf.jacobian_b = sign;
                    irf.dist = dist;

                    // Set external belief of robot B from its broadcast
                    if !bcast.belief_means.is_empty() && !bcast.belief_vars.is_empty() {
                        let ext_mean = bcast.belief_means[0];
                        let ext_var = bcast.belief_vars[0].max(1e-6);
                        irf.ext_lambda_b = 1.0 / ext_var;
                        irf.ext_eta_b = irf.ext_lambda_b * ext_mean;
                    }

                    // Activate factor only when robots are close enough to matter
                    irf.set_active(dist < D_SAFE * 5.0);
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
