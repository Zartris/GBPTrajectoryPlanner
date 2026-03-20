// src/bins/visualiser/src/state.rs
use bevy::prelude::*;
use std::sync::{Arc, Mutex};
use std::collections::VecDeque;
use gbp_comms::RobotStateMsg;

/// Bevy resource: queue of incoming messages from WebSocket thread.
#[derive(Resource, Default)]
pub struct WsInbox(pub Arc<Mutex<VecDeque<RobotStateMsg>>>);

/// Bevy resource: latest known state per robot, keyed by robot_id.
#[derive(Resource, Default)]
pub struct RobotStates(pub std::collections::HashMap<u32, RobotStateMsg>);

/// Bevy resource: parsed map.
#[derive(Resource)]
pub struct MapRes(pub gbp_map::map::Map);
