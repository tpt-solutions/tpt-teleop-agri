#![warn(missing_docs)]
//! `tpt-t-agri-core` — the shared foundation for the `tpt-teleop-agri`
//! workspace.
//!
//! Provides:
//! * [`machine`] — the field / operational state machine (mirrors
//!   `tpt-t-core::machine`).
//! * [`bus`] — a lock-free single-producer / single-consumer ring buffer used
//!   for zero-alloc inter-crate event routing on the hot path.
//! * [`event_loop`] — a central event-loop skeleton that drains the bus and
//!   dispatches events without allocating per tick.
//! * [`wire`] — the zero-copy rkyv wire-type prelude shared by the other agri
//!   crates.
//!
//! Design invariants (inherited from the `tpt-teleop` sibling repo): no async
//! runtime, no `serde`, no channels-with-mutexes. The forward data plane makes
//! no per-event heap allocations and takes no locks on the steady-state path.

pub mod bus;
pub mod event_loop;
pub mod machine;
pub mod wire;
