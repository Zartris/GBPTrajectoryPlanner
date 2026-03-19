use gbp_comms::*;

#[test]
fn robot_broadcast_default_is_valid() {
    let b = RobotBroadcast::default();
    assert_eq!(b.planned_edges.len(), 0);
    assert_eq!(b.belief_means.len(), 0);
}
