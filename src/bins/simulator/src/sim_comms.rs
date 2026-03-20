//! No-op CommsInterface for single-robot simulation.
//! In M3 this is replaced with a real in-process broadcast.

use gbp_comms::{CommsInterface, RobotBroadcast};
use heapless::Vec;

pub struct SimComms;

impl CommsInterface for SimComms {
    type Error = ();
    fn broadcast(&mut self, _: &RobotBroadcast) -> Result<(), ()> {
        Ok(())
    }
    fn receive_broadcasts(&mut self) -> Vec<RobotBroadcast, { gbp_map::MAX_NEIGHBOURS }> {
        Vec::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gbp_comms::CommsInterface;

    #[test]
    fn broadcast_returns_ok() {
        let mut c = SimComms;
        let dummy = gbp_comms::RobotBroadcast::default();
        assert!(c.broadcast(&dummy).is_ok());
    }

    #[test]
    fn receive_returns_empty() {
        let mut c = SimComms;
        assert_eq!(c.receive_broadcasts().len(), 0);
    }
}
