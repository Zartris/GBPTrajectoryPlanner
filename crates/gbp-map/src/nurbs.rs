// Minimal stubs so map.rs compiles. Full implementation in Task 5.

use crate::{MAX_CONTROL_POINTS, MAX_KNOTS};

/// Type alias for the arc-length table (placeholder until full impl).
pub type ArcLengthTable = ();

pub fn eval_point(
    _t: f32,
    _cps: &heapless::Vec<[f32; 3], MAX_CONTROL_POINTS>,
    _knots: &heapless::Vec<f32, MAX_KNOTS>,
    _degree: usize,
) -> [f32; 3] {
    [0.0; 3]
}

pub fn eval_tangent(
    _t: f32,
    _cps: &heapless::Vec<[f32; 3], MAX_CONTROL_POINTS>,
    _knots: &heapless::Vec<f32, MAX_KNOTS>,
    _degree: usize,
) -> [f32; 3] {
    [1.0, 0.0, 0.0]
}

pub fn arc_s_to_t(_s: f32, _n: &crate::map::NurbsGeometry) -> f32 {
    0.0
}

pub fn arc_t_to_s(_t: f32, _n: &crate::map::NurbsGeometry) -> f32 {
    0.0
}
