#![no_std]

use heapless::Vec;
use gbp_map::{MAX_HORIZON, MAX_NEIGHBOURS};
use gbp_map::map::{EdgeId, NodeId};

pub type RobotId = u32;

#[derive(Clone, Copy, Debug, Default)]
pub struct GBPTimestep {
    pub eta:    f32,
    pub lambda: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RobotSource { Simulated, Hardware }

#[derive(Clone, Copy, Debug)]
pub enum ParameterTarget { Global, Robot(RobotId) }

/// Physics -> Agent (in-process only)
#[derive(Clone, Copy, Debug, Default)]
pub struct ObservationUpdate {
    pub position_s:   f32,
    pub velocity:     f32,
    pub current_edge: EdgeId,
}

/// Agent -> Agent broadcast (over SimComms or ESP-NOW)
#[derive(Clone, Debug, Default)]
pub struct RobotBroadcast {
    pub robot_id:      RobotId,
    pub current_edge:  EdgeId,
    pub position_s:    f32,
    pub velocity:      f32,
    pub pos:           [f32; 3],
    pub planned_edges: Vec<EdgeId, MAX_HORIZON>,
    pub belief_means:  Vec<f32, MAX_HORIZON>,
    pub belief_vars:   Vec<f32, MAX_HORIZON>,
    pub gbp_timesteps: Vec<GBPTimestep, MAX_HORIZON>,
}

/// Bridge/Simulator -> Visualiser (~20 Hz, JSON over WebSocket)
#[derive(Clone, Debug)]
pub struct RobotStateMsg {
    pub robot_id:       RobotId,
    pub current_edge:   EdgeId,
    pub position_s:     f32,
    pub velocity:       f32,
    pub pos_3d:         [f32; 3],
    pub source:         RobotSource,
    pub belief_means:   [f32; MAX_HORIZON],
    pub belief_vars:    [f32; MAX_HORIZON],
    pub planned_edges:  Vec<EdgeId, MAX_HORIZON>,
    pub active_factors: Vec<RobotId, MAX_NEIGHBOURS>,
}

/// Visualiser -> Bridge/Simulator
#[derive(Clone, Copy, Debug)]
pub struct TrajectoryCommand {
    pub robot_id:  RobotId,
    pub goal_node: NodeId,
}

/// Visualiser -> Bridge/Simulator -- live parameter update
#[derive(Clone, Debug)]
pub struct ParameterUpdate {
    pub target: ParameterTarget,
    pub key:    heapless::String<32>,
    pub value:  f32,
}

/// Trait implemented by SimComms (in-process) and ESPNowComms (firmware).
pub trait CommsInterface {
    type Error;
    /// Broadcast this robot's state to all neighbours.
    fn broadcast(&mut self, msg: &RobotBroadcast) -> Result<(), Self::Error>;
    /// Receive all pending broadcasts from other robots (non-blocking).
    fn receive_broadcasts(&mut self) -> Vec<RobotBroadcast, MAX_NEIGHBOURS>;
}

// Default impls for EdgeId and NodeId are in gbp-map
