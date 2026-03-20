use gbp_comms::{CommsInterface, ObservationUpdate, RobotBroadcast, RobotId};
use gbp_core::{Factor, FactorGraph, FactorKind, DynamicsFactor, InterRobotFactor};
use gbp_map::{Map, MAX_HORIZON, MAX_NEIGHBOURS};
use gbp_map::map::EdgeId;
use heapless::Vec;
use crate::trajectory::Trajectory;
use crate::interrobot_set::InterRobotFactorSet;

/// Number of dynamics factors = K-1 (one per adjacent timestep pair)
const NUM_DYN_FACTORS: usize = MAX_HORIZON - 1;
/// Max total factors = dynamics + inter-robot neighbours
const MAX_FACTORS: usize = NUM_DYN_FACTORS + MAX_NEIGHBOURS;

const GBP_ITERATIONS: usize = 15;
const DT: f32 = 0.1; // seconds per GBP timestep
const D_SAFE: f32 = 0.3; // minimum clearance (m)
const SIGMA_R: f32 = 0.1; // inter-robot factor noise

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

        // 0. Anchor variable[0] at observed position (strong prior)
        self.graph.variables[0].prior_eta = self.position_s * 1000.0;
        self.graph.variables[0].prior_lambda = 1000.0;
        self.graph.variables[0].eta = self.graph.variables[0].prior_eta;
        self.graph.variables[0].lambda = self.graph.variables[0].prior_lambda;

        // 1. Receive broadcasts from neighbours
        let broadcasts = self.comms.receive_broadcasts();

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
        let velocity = ((s1 - s0) / DT).max(0.0);

        // 7. Broadcast state
        let _ = self.comms.broadcast(&self.make_broadcast(velocity));

        StepOutput { velocity, position_s: self.position_s, current_edge: self.current_edge }
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
            if !shares_edge { continue; }

            let _ = active_ids.push(bcast.robot_id);

            // Find the timestep k where our predicted position is closest to theirs
            let mut best_k = 0usize;
            let mut best_dist = f32::MAX;
            for k in 0..MAX_HORIZON {
                let my_s_k = self.graph.variables[k].mean();
                let their_s_k = if k < bcast.belief_means.len() {
                    bcast.belief_means[k]
                } else {
                    bcast.position_s
                };
                let dist = (my_s_k - their_s_k).abs();
                if dist < best_dist {
                    best_dist = dist;
                    best_k = k;
                }
            }

            if !self.ir_set.contains(bcast.robot_id) {
                // Add new factor at the closest timestep
                let factor = InterRobotFactor::new(best_k, D_SAFE, SIGMA_R);
                if let Ok(idx) = self.graph.add_factor(FactorKind::InterRobot(factor)) {
                    self.ir_set.insert(bcast.robot_id, idx);
                }
            } else if let Some(factor_idx) = self.ir_set.factor_idx(bcast.robot_id) {
                // Update existing factor to connect at the closest timestep
                if let Some(FactorKind::InterRobot(irf)) = self.graph.get_factor_kind_mut(factor_idx) {
                    irf.set_variable_index(best_k);
                }
            }
        }

        // Remove factors for robots no longer sharing edges
        let mut to_remove: Vec<RobotId, MAX_NEIGHBOURS> = Vec::new();
        for &(rid, _) in self.ir_set.iter() {
            if !active_ids.contains(&rid) {
                let _ = to_remove.push(rid);
            }
        }
        for rid in to_remove.iter() {
            self.ir_set.remove(*rid, &mut self.graph);
        }
    }

    fn update_interrobot_jacobians(
        &mut self,
        broadcasts: &Vec<RobotBroadcast, MAX_NEIGHBOURS>,
        _map: &Map,
    ) {
        for bcast in broadcasts.iter() {
            if bcast.robot_id == self.robot_id { continue; }
            if let Some(factor_idx) = self.ir_set.factor_idx(bcast.robot_id) {
                // Read the variable index and its mean
                let var_idx = self.graph.get_factor_kind(factor_idx)
                    .and_then(|fk| if let FactorKind::InterRobot(irf) = fk { Some(irf.variable_indices()[0]) } else { None })
                    .unwrap_or(0);
                let s_a = self.graph.variables[var_idx].mean();

                // Neighbour's predicted position at the same timestep
                let s_b = if var_idx < bcast.belief_means.len() {
                    bcast.belief_means[var_idx]
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
            pos: [0.0; 3],
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
