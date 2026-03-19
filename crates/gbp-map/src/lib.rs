#![no_std]

// -- Capacity constants -- locked at M0. Changing these requires a memory-budget re-check. --
pub const MAX_NODES:          usize = 64;
pub const MAX_EDGES:          usize = 96;
pub const MAX_CONTROL_POINTS: usize = 16;
pub const MAX_KNOTS:          usize = 32;
pub const ARC_TABLE_SAMPLES:  usize = 64;
pub const MAX_HORIZON:        usize = 12;   // K: GBP timesteps
pub const MAX_NEIGHBOURS:     usize = 8;    // max robots tracked simultaneously
pub const MAX_PATH_EDGES:     usize = 32;

pub mod map;
pub mod nurbs;
pub mod astar;

#[cfg(feature = "parse")]
pub mod parser;

// Re-exports will be added as modules are implemented
