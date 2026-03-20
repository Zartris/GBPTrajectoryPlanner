/// GBP variable in information (canonical) form.
/// Mean mu = eta/lambda, Variance sigma^2 = 1/lambda.
#[derive(Clone, Copy, Debug, Default)]
pub struct VariableNode {
    /// Information vector: eta = lambda * mu
    pub eta: f32,
    /// Precision (information scalar): lambda = 1/sigma^2
    pub lambda: f32,
    /// Prior information -- kept separate so iterate() can reset each step.
    pub prior_eta: f32,
    pub prior_lambda: f32,
}

impl VariableNode {
    pub fn new(mean: f32, variance: f32) -> Self {
        let lambda = if variance > 1e-30 { 1.0 / variance } else { 0.0 };
        let eta = lambda * mean;
        Self { eta, lambda, prior_eta: eta, prior_lambda: lambda }
    }

    pub fn mean(&self) -> f32 {
        if self.lambda.abs() < 1e-10 { 0.0 } else { self.eta / self.lambda }
    }

    pub fn variance(&self) -> f32 {
        if self.lambda.abs() < 1e-10 { f32::MAX } else { 1.0 / self.lambda }
    }

    /// Reset to prior (called at start of each iterate() to re-accumulate messages).
    pub fn reset_to_prior(&mut self) {
        self.eta    = self.prior_eta;
        self.lambda = self.prior_lambda;
    }
}
