use heapless::Vec;
use crate::factor_node::{Factor, LinearizedFactor};
use crate::variable_node::VariableNode;

/// Velocity prior factor connecting (s_k, s_{k+1}).
/// Residual: e = (s_{k+1} - s_k)/dt - v_nom
/// Jacobian: [-1/dt, +1/dt]
pub struct DynamicsFactor {
    var_indices: [usize; 2],
    dt:    f32,
    sigma: f32,
    /// Nominal velocity at current s_k -- set by RobotAgent before each iterate().
    pub v_nom: f32,
}

impl DynamicsFactor {
    pub fn new(var_indices: [usize; 2], dt: f32, sigma: f32, v_nom: f32) -> Self {
        let dt = f32::max(dt, 1e-6);
        let sigma = f32::max(sigma, 1e-6);
        Self { var_indices, dt, sigma, v_nom }
    }
    pub fn set_v_nom(&mut self, v: f32) { self.v_nom = v; }
    pub fn set_sigma(&mut self, sigma: f32) { self.sigma = f32::max(sigma, 1e-6); }
    pub fn set_timestep(&mut self, dt: f32) { self.dt = f32::max(dt, 1e-6); }
}

impl Factor for DynamicsFactor {
    fn variable_indices(&self) -> &[usize] { &self.var_indices }

    fn linearize(&self, variables: &[VariableNode]) -> LinearizedFactor {
        let s_k  = variables[self.var_indices[0]].mean();
        let s_k1 = variables[self.var_indices[1]].mean();
        let residual = (s_k1 - s_k) / self.dt - self.v_nom;
        let inv_dt = 1.0 / self.dt;
        let mut jacobian: Vec<f32, 2> = Vec::new();
        let _ = jacobian.push(-inv_dt);
        let _ = jacobian.push( inv_dt);
        LinearizedFactor {
            jacobian,
            residual,
            precision: 1.0 / (self.sigma * self.sigma),
        }
    }
}
