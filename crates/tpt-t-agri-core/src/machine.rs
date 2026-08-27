//! Field / operational state machine (mirrors `tpt-t-core::machine`).
//!
//! Models the high-level autonomy state of a single field machine (tractor,
//! combine, sprayer, grain cart). Transition legality is encoded by
//! [`FieldOpState::next`]; illegal transitions are rejected rather than
//! panicked, so the caller can route them through the safety subsystem.
//!
//! The universal safety state machine from `tpt-t-domain-bridge`
//! (`AUTONOMOUS → REQUESTING_TELEOP → TELEOP_ACTIVE → RETURNING_TO_AUTONOMY`)
//! and its `EMERGENCY_STOP` transitions compose *on top* of this machine; this
//! crate stays domain-agnostic about teleop and only models the field-work
//! lifecycle.

use core::fmt;

/// High-level operating state of a field machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FieldOpState {
    /// Machine is parked / powered down. No field work.
    Parked,
    /// Transit between depot and field (road or headland travel, not working).
    Transit,
    /// Actively performing field work (tillage, seeding, spraying, harvesting).
    Working,
    /// Executing a headland turn at the end of a pass.
    HeadlandTurn,
    /// Paused for refuel, seed/input refill, or operator swap.
    Servicing,
    /// Fault / degraded: control handed to the safety subsystem.
    Fault,
    /// Teleoperation engaged (operator driving remotely).
    Teleop,
}

impl FieldOpState {
    /// Returns `Some(target)` if a transition from `self` to `target` is legal,
    /// otherwise `None`.
    pub fn next(self, target: FieldOpState) -> Option<FieldOpState> {
        if self == target {
            return Some(self);
        }
        use FieldOpState::*;
        let ok = matches!(
            (self, target),
            (Parked, Transit)
                | (Transit, Working)
                | (Transit, Parked)
                | (Working, HeadlandTurn)
                | (HeadlandTurn, Working)
                | (Working, Servicing)
                | (Servicing, Working)
                | (Working, Transit)
                | (_, Fault) // any state may fault
                | (Fault, Parked) // recover from fault only to parked
                | (_, Teleop) // any state may request teleop
                | (Teleop, Working)
                | (Teleop, Transit)
                | (Teleop, Parked)
        );
        ok.then_some(target)
    }
}

/// A minimal generic state machine driven by an implementor of [`Transition`].
#[derive(Debug, Clone, Copy)]
pub struct StateMachine<S> {
    state: S,
}

impl<S: Copy + PartialEq + fmt::Debug> StateMachine<S> {
    /// Create a machine in `initial` state.
    pub fn new(initial: S) -> Self {
        Self { state: initial }
    }

    /// Current state.
    pub fn state(&self) -> S {
        self.state
    }

    /// Attempt a transition via [`Transition::next`].
    pub fn transition(&mut self, target: S) -> Result<(), TransitionError<S>>
    where
        S: Transition<State = S>,
    {
        match self.state.next(target) {
            Some(s) => {
                self.state = s;
                Ok(())
            }
            None => Err(TransitionError {
                from: self.state,
                to: target,
            }),
        }
    }
}

/// Transition policy for a state type.
pub trait Transition {
    /// The state space this policy operates over.
    type State;
    /// Return the next state, or `None` if the transition is illegal.
    fn next(self, target: Self::State) -> Option<Self::State>;
}

impl Transition for FieldOpState {
    type State = FieldOpState;
    fn next(self, target: Self::State) -> Option<Self::State> {
        FieldOpState::next(self, target)
    }
}

/// Error returned when a transition is rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransitionError<S> {
    /// State the machine was in.
    pub from: S,
    /// State that was requested.
    pub to: S,
}

impl<S: fmt::Debug> fmt::Display for TransitionError<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "illegal transition {:?} -> {:?}", self.from, self.to)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legal_working_path() {
        let mut m = StateMachine::new(FieldOpState::Parked);
        m.transition(FieldOpState::Transit).unwrap();
        m.transition(FieldOpState::Working).unwrap();
        m.transition(FieldOpState::HeadlandTurn).unwrap();
        m.transition(FieldOpState::Working).unwrap();
        assert_eq!(m.state(), FieldOpState::Working);
    }

    #[test]
    fn illegal_rejected_and_fault_recovery() {
        let mut m = StateMachine::new(FieldOpState::Parked);
        // Parked may not jump straight to Working.
        assert!(m.transition(FieldOpState::Working).is_err());
        // Any state may fault.
        assert!(m.transition(FieldOpState::Fault).is_ok());
        // Fault may only recover to Parked, not directly back to Working.
        assert!(m.transition(FieldOpState::Working).is_err());
        assert!(m.transition(FieldOpState::Parked).is_ok());
    }

    #[test]
    fn teleop_from_anywhere() {
        let mut m = StateMachine::new(FieldOpState::Working);
        assert!(m.transition(FieldOpState::Teleop).is_ok());
        assert!(m.transition(FieldOpState::Transit).is_ok());
        assert!(m.transition(FieldOpState::Parked).is_ok());
    }

    #[test]
    fn same_state_is_idempotent() {
        let mut m = StateMachine::new(FieldOpState::Working);
        assert!(m.transition(FieldOpState::Working).is_ok());
        assert_eq!(m.state(), FieldOpState::Working);
    }
}
