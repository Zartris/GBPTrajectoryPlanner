use gbp_map::map::NurbsGeometry;
use gbp_map::nurbs;
use gbp_map::ARC_TABLE_SAMPLES;

/// A cubic NURBS with 4 control points forming a straight line (degenerate case).
fn make_test_nurbs() -> NurbsGeometry {
    let mut cps: heapless::Vec<[f32; 3], 16> = heapless::Vec::new();
    cps.push([0.0, 0.0, 0.0]).unwrap();
    cps.push([1.0 / 3.0, 0.0, 0.0]).unwrap();
    cps.push([2.0 / 3.0, 0.0, 0.0]).unwrap();
    cps.push([1.0, 0.0, 0.0]).unwrap();
    let mut knots: heapless::Vec<f32, 32> = heapless::Vec::new();
    for k in [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0] {
        knots.push(k).unwrap();
    }
    let (arc_t, arc_s) = nurbs::build_arc_table(&cps, &knots, 3);
    NurbsGeometry {
        control_points: cps,
        knots,
        degree: 3,
        length: arc_s[ARC_TABLE_SAMPLES - 1],
        arc_t,
        arc_s,
    }
}

#[test]
fn nurbs_straight_line_eval() {
    let n = make_test_nurbs();
    let p0 = nurbs::eval_point(0.0, &n.control_points, &n.knots, 3);
    let p1 = nurbs::eval_point(1.0, &n.control_points, &n.knots, 3);
    assert!((p0[0] - 0.0).abs() < 1e-5, "p0.x={}", p0[0]);
    assert!((p1[0] - 1.0).abs() < 1e-5, "p1.x={}", p1[0]);
}

#[test]
fn arc_length_table_monotone() {
    let n = make_test_nurbs();
    for i in 1..ARC_TABLE_SAMPLES {
        assert!(n.arc_s[i] >= n.arc_s[i - 1], "arc_s not monotone at {}", i);
    }
}

#[test]
fn arc_s_to_t_round_trip() {
    let n = make_test_nurbs();
    let total = n.length;
    for frac in [0.0f32, 0.25, 0.5, 0.75, 1.0] {
        let s = frac * total;
        let t = nurbs::arc_s_to_t(s, &n);
        let s_back = nurbs::arc_t_to_s(t, &n);
        assert!((s_back - s).abs() < 0.01, "round-trip error at s={}: got {}", s, s_back);
    }
}

#[test]
fn tangent_is_unit_length() {
    let n = make_test_nurbs();
    for frac in [0.1f32, 0.5, 0.9] {
        let t = nurbs::eval_tangent(frac, &n.control_points, &n.knots, 3);
        let len = (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]).sqrt();
        assert!((len - 1.0).abs() < 1e-4, "tangent not unit at t={}: len={}", frac, len);
    }
}
