//! Central event-loop skeleton for routing field-machine events off the hot
//! path. Drains a [`SpscRing`](crate::bus::SpscRing) of `E` and dispatches each
//! to a handler. Intentionally allocation-free in [`EventLoop::step`].

use crate::bus::{Consumer, Producer, SpscRing};
use std::sync::Arc;

/// An event loop that consumes `E` events from a [`Consumer`] and dispatches
/// them via a caller-supplied handler.
pub struct EventLoop<E, const N: usize> {
    rx: Consumer<E, N>,
}

impl<E, const N: usize> EventLoop<E, N> {
    /// Wrap a consumer half of an event ring.
    pub fn new(rx: Consumer<E, N>) -> Self {
        Self { rx }
    }

    /// Process a single buffered event, returning `false` when the ring is empty
    /// (no work done this tick).
    pub fn step<H: FnMut(E)>(&self, mut handler: H) -> bool {
        match self.rx.try_pop() {
            Some(e) => {
                handler(e);
                true
            }
            None => false,
        }
    }

    /// Drain all currently-buffered events, invoking `handler` for each.
    /// Returns the number of events processed.
    pub fn drain<H: FnMut(E)>(&self, mut handler: H) -> usize {
        let mut n = 0;
        while let Some(e) = self.rx.try_pop() {
            handler(e);
            n += 1;
        }
        n
    }
}

/// Build a paired producer + [`EventLoop`] over a shared `N`-slot event ring.
///
/// The returned [`Producer`] is the hot-path send side (e.g. fed by sensor /
/// safety threads); the [`EventLoop`] is driven on the dispatcher thread.
pub fn event_channel<E, const N: usize>() -> (Producer<E, N>, EventLoop<E, N>) {
    let ring = Arc::new(SpscRing::new());
    let (tx, rx) = ring.split();
    (tx, EventLoop::new(rx))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn event_channel_drains_in_order() {
        let (tx, loop_) = event_channel::<u32, 8>();
        for i in 0..5 {
            tx.push(i);
        }
        let mut collected = Vec::new();
        let processed = loop_.drain(|e| collected.push(e));
        assert_eq!(processed, 5);
        assert_eq!(collected, vec![0, 1, 2, 3, 4]);
    }

    #[test]
    fn step_reports_empty() {
        let (_tx, loop_) = event_channel::<u32, 8>();
        let mut count = 0;
        assert!(!loop_.step(|_| count += 1));
        assert_eq!(count, 0);
    }
}
