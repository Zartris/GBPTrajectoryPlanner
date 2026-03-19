#![no_std]

pub mod variable_node;
pub mod factor_node;
pub mod factor_graph;
pub mod dynamics_factor;
pub mod interrobot_factor;  // FACTOR EXTENSION POINT 3/3: export new modules here

// Re-exports will be added as modules are implemented
