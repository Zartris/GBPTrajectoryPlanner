//! Cox-de Boor B-spline evaluation, arc-length table, and tangent.
//! All weights are assumed 1.0 (non-rational B-spline).

use crate::{ARC_TABLE_SAMPLES, MAX_CONTROL_POINTS, MAX_KNOTS};

/// Type alias for the arc-length table.
pub type ArcLengthTable = ([f32; ARC_TABLE_SAMPLES], [f32; ARC_TABLE_SAMPLES]);

/// Evaluate point on B-spline at parameter t in [0, 1].
pub fn eval_point(
    t: f32,
    cps: &heapless::Vec<[f32; 3], MAX_CONTROL_POINTS>,
    knots: &heapless::Vec<f32, MAX_KNOTS>,
    degree: usize,
) -> [f32; 3] {
    let n = cps.len() - 1;
    let t_clamped = t.clamp(knots[degree], knots[n + 1]);
    let span = find_knot_span(n, degree, t_clamped, knots);
    let basis = basis_functions(span, t_clamped, degree, knots);
    let mut p = [0.0f32; 3];
    for i in 0..=degree {
        let cp = cps[span - degree + i];
        p[0] += basis[i] * cp[0];
        p[1] += basis[i] * cp[1];
        p[2] += basis[i] * cp[2];
    }
    p
}

/// Unit tangent at parameter t via central difference.
pub fn eval_tangent(
    t: f32,
    cps: &heapless::Vec<[f32; 3], MAX_CONTROL_POINTS>,
    knots: &heapless::Vec<f32, MAX_KNOTS>,
    degree: usize,
) -> [f32; 3] {
    let eps = 1e-4_f32;
    let t0 = (t - eps).max(0.0);
    let t1 = (t + eps).min(1.0);
    let p0 = eval_point(t0, cps, knots, degree);
    let p1 = eval_point(t1, cps, knots, degree);
    let dx = p1[0] - p0[0];
    let dy = p1[1] - p0[1];
    let dz = p1[2] - p0[2];
    let len = libm::sqrtf(dx * dx + dy * dy + dz * dz);
    if len < 1e-9 { return [1.0, 0.0, 0.0]; }
    [dx / len, dy / len, dz / len]
}

/// Build the arc-length lookup table: uniformly sample t, accumulate arc-length.
/// Returns (arc_t[N], arc_s[N]) where arc_s[N-1] = total_length.
pub fn build_arc_table(
    cps: &heapless::Vec<[f32; 3], MAX_CONTROL_POINTS>,
    knots: &heapless::Vec<f32, MAX_KNOTS>,
    degree: usize,
) -> ([f32; ARC_TABLE_SAMPLES], [f32; ARC_TABLE_SAMPLES]) {
    let mut arc_t = [0.0f32; ARC_TABLE_SAMPLES];
    let mut arc_s = [0.0f32; ARC_TABLE_SAMPLES];
    let n = ARC_TABLE_SAMPLES - 1;
    let mut prev = eval_point(0.0, cps, knots, degree);
    let mut cumulative = 0.0f32;
    arc_t[0] = 0.0;
    arc_s[0] = 0.0;
    for i in 1..=n {
        let t = i as f32 / n as f32;
        let cur = eval_point(t, cps, knots, degree);
        let dx = cur[0] - prev[0];
        let dy = cur[1] - prev[1];
        let dz = cur[2] - prev[2];
        cumulative += libm::sqrtf(dx * dx + dy * dy + dz * dz);
        arc_t[i] = t;
        arc_s[i] = cumulative;
        prev = cur;
    }
    (arc_t, arc_s)
}

/// Convert arc-length s to parameter t via binary search + linear interpolation.
pub fn arc_s_to_t(s: f32, n: &crate::map::NurbsGeometry) -> f32 {
    let total = n.length;
    if total < 1e-9 { return 0.0; }
    let s_clamped = s.clamp(0.0, total);
    let mut lo = 0usize;
    let mut hi = ARC_TABLE_SAMPLES - 1;
    while lo + 1 < hi {
        let mid = (lo + hi) / 2;
        if n.arc_s[mid] <= s_clamped { lo = mid; } else { hi = mid; }
    }
    let ds = n.arc_s[hi] - n.arc_s[lo];
    if ds < 1e-9 { return n.arc_t[lo]; }
    let frac = (s_clamped - n.arc_s[lo]) / ds;
    n.arc_t[lo] + frac * (n.arc_t[hi] - n.arc_t[lo])
}

/// Convert parameter t to arc-length s via linear interpolation.
pub fn arc_t_to_s(t: f32, n: &crate::map::NurbsGeometry) -> f32 {
    let t_clamped = t.clamp(0.0, 1.0);
    let mut lo = 0usize;
    let mut hi = ARC_TABLE_SAMPLES - 1;
    while lo + 1 < hi {
        let mid = (lo + hi) / 2;
        if n.arc_t[mid] <= t_clamped { lo = mid; } else { hi = mid; }
    }
    let dt = n.arc_t[hi] - n.arc_t[lo];
    if dt < 1e-9 { return n.arc_s[lo]; }
    let frac = (t_clamped - n.arc_t[lo]) / dt;
    n.arc_s[lo] + frac * (n.arc_s[hi] - n.arc_s[lo])
}

// -- Internal helpers --

fn find_knot_span(
    n: usize,
    p: usize,
    t: f32,
    knots: &heapless::Vec<f32, MAX_KNOTS>,
) -> usize {
    if (t - knots[n + 1]).abs() < 1e-9 { return n; }
    let mut lo = p;
    let mut hi = n + 1;
    let mut mid = (lo + hi) / 2;
    while t < knots[mid] || t >= knots[mid + 1] {
        if t < knots[mid] { hi = mid; } else { lo = mid; }
        mid = (lo + hi) / 2;
    }
    mid
}

fn basis_functions(
    span: usize,
    t: f32,
    p: usize,
    knots: &heapless::Vec<f32, MAX_KNOTS>,
) -> [f32; 16] {
    debug_assert!(p < 16, "degree {} exceeds basis_functions array capacity (max 15)", p);
    let mut b = [0.0f32; 16];
    let mut left  = [0.0f32; 16];
    let mut right = [0.0f32; 16];
    b[0] = 1.0;
    for j in 1..=p {
        left[j]  = t - knots[span + 1 - j];
        right[j] = knots[span + j] - t;
        let mut saved = 0.0f32;
        for r in 0..j {
            let denom = right[r + 1] + left[j - r];
            let temp = if denom.abs() < 1e-12 { 0.0 } else { b[r] / denom };
            b[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        b[j] = saved;
    }
    b
}
