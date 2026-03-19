use gbp_comms::{CommsInterface, ObservationUpdate, RobotBroadcast, RobotId};
use gbp_core::{FactorGraph, FactorKind, DynamicsFactor};
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
    pub fn set_trajectory(&mut self, edges: Vec<(EdgeId, f32), 32>, start_s: f32) {
        self.trajectory = Some(Trajectory::new(edges, start_s));
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
        _broadcasts: &Vec<RobotBroadcast, MAX_NEIGHBOURS>,
        _map: &Map,
    ) {
        // Stub: full implementation in M3 when multi-robot is introduced.
    }

    fn update_interrobot_jacobians(
        &mut self,
        _broadcasts: &Vec<RobotBroadcast, MAX_NEIGHBOURS>,
        _map: &Map,
    ) {
        // Stub: full implementation in M3.
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
            planned_edges: Vec::new(),
            belief_means: means,
            belief_vars: vars,
            gbp_timesteps: Vec::new(),
        }
    }
}
