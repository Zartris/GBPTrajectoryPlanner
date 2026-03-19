use gbp_agent::trajectory::Trajectory;
use gbp_map::map::EdgeId;

fn edges_1m_each() -> heapless::Vec<(EdgeId, f32), 32> {
    let mut v: heapless::Vec<(EdgeId, f32), 32> = heapless::Vec::new();
    v.push((EdgeId(0), 1.0)).unwrap();
    v.push((EdgeId(1), 1.0)).unwrap();
    v.push((EdgeId(2), 1.0)).unwrap();
    v
}

#[test]
fn local_s_on_first_edge() {
    let t = Trajectory::new(edges_1m_each(), 0.0);
    let (edge, local_s, is_final) = t.edge_and_local_s(0.5);
    assert_eq!(edge, EdgeId(0));
    assert!((local_s - 0.5).abs() < 1e-5);
    assert!(!is_final);
}

#[test]
fn local_s_on_second_edge() {
    let t = Trajectory::new(edges_1m_each(), 0.0);
    let (edge, local_s, is_final) = t.edge_and_local_s(1.7);
    assert_eq!(edge, EdgeId(1));
    assert!((local_s - 0.7).abs() < 1e-5);
    assert!(!is_final);
}

#[test]
fn local_s_on_final_edge() {
    let t = Trajectory::new(edges_1m_each(), 0.0);
    let (edge, local_s, is_final) = t.edge_and_local_s(2.5);
    assert_eq!(edge, EdgeId(2));
    assert!((local_s - 0.5).abs() < 1e-5);
    assert!(is_final);
}

#[test]
fn v_nom_tapered_on_final_edge() {
    let t = Trajectory::new(edges_1m_each(), 0.0);
    let v = t.v_nom_at(2.5, 2.0, 1.0);
    assert!((v - 1.0).abs() < 1e-4, "v_nom={}", v);
}

#[test]
fn v_nom_nominal_on_intermediate_edge() {
    let t = Trajectory::new(edges_1m_each(), 0.0);
    let v = t.v_nom_at(0.5, 2.0, 1.0);
    assert!((v - 2.0).abs() < 1e-4);
}
