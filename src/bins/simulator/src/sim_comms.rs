//! In-process broadcast CommsInterface for multi-robot simulation.
//! Each agent holds a private Receiver subscribed to a shared broadcast channel.

use gbp_comms::{CommsInterface, RobotBroadcast};
use heapless::Vec;
use tokio::sync::broadcast;

/// In-process broadcast. One instance per agent.
/// All agents share the same broadcast::Sender<RobotBroadcast>.
/// Each agent holds a private broadcast::Receiver to drain incoming.
pub struct SimComms {
    tx: broadcast::Sender<RobotBroadcast>,
    rx: broadcast::Receiver<RobotBroadcast>,
}

impl SimComms {
    pub fn new(tx: broadcast::Sender<RobotBroadcast>, rx: broadcast::Receiver<RobotBroadcast>) -> Self {
        Self { tx, rx }
    }
}

impl CommsInterface for SimComms {
    type Error = ();

    fn broadcast(&mut self, msg: &RobotBroadcast) -> Result<(), Self::Error> {
        self.tx.send(msg.clone()).map(|_| ()).map_err(|_| ())
    }

    fn receive_broadcasts(&mut self) -> Vec<RobotBroadcast, { gbp_map::MAX_NEIGHBOURS }> {
        let mut out = Vec::new();
        loop {
            match self.rx.try_recv() {
                Ok(msg) => { let _ = out.push(msg); }
                Err(_) => break,
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gbp_comms::CommsInterface;

    #[test]
    fn broadcast_is_received_by_second_receiver() {
        let (tx, _rx) = tokio::sync::broadcast::channel::<RobotBroadcast>(32);
        let rx2 = tx.subscribe();
        let mut comms = SimComms::new(tx.clone(), rx2);
        let msg = RobotBroadcast {
            robot_id: 1,
            current_edge: gbp_map::map::EdgeId(0),
            position_s: 1.5,
            ..Default::default()
        };
        comms.broadcast(&msg).unwrap();
        // a third subscriber can receive it
        let mut rx3 = tx.subscribe();
        // send again so rx3 sees it
        comms.broadcast(&msg).unwrap();
        let received = rx3.try_recv().unwrap();
        assert_eq!(received.robot_id, 1);
    }

    #[test]
    fn receive_returns_broadcasts_from_others() {
        let (tx, rx) = tokio::sync::broadcast::channel::<RobotBroadcast>(32);
        let mut comms = SimComms::new(tx.clone(), rx);
        let msg = RobotBroadcast {
            robot_id: 2,
            current_edge: gbp_map::map::EdgeId(0),
            position_s: 3.0,
            ..Default::default()
        };
        tx.send(msg.clone()).unwrap();
        let received = comms.receive_broadcasts();
        assert!(received.iter().any(|b| b.robot_id == 2));
    }

    #[test]
    fn two_simcomms_share_one_channel() {
        let (tx, rx0) = tokio::sync::broadcast::channel::<RobotBroadcast>(64);
        let rx1 = tx.subscribe();
        let _comms0 = SimComms::new(tx.clone(), rx0);
        let _comms1 = SimComms::new(tx.clone(), rx1);
    }
}
