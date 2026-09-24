#![warn(missing_docs)]
//! `tpt-t-agri-isobus` — the tractor's nervous system.
//!
//! A custom, zero-allocation parser stack for ISOBUS (ISO 11783) over CAN,
//! replacing heavy C++ ISOBUS libraries and socketcan wrappers (spec §4.1 /
//! §7). Modules:
//!
//! * [`frame`] — 29-bit J1939/ISOBUS identifier codec (priority, PGN,
//!   source, destination) and the raw frame record.
//! * [`tp`] — ISO 11783 / J1939 transport-protocol reassembly (TP.CM/TP.DT)
//!   into a fixed 1785-byte buffer, so multi-packet VT/TC messages work
//!   without heap allocation.
//! * [`telemetry`] — decode of the core J1939 parameter groups (EEC1, CCVS1)
//!   and Task Controller process data (set/measurement values) into the
//!   rkyv wire structs shared across the workspace.
//! * [`vt`] — Virtual Terminal (ISO 11783-6) fast-packet message codec:
//!   soft-key/button activation, mask selection, object hide/show /
//!   enable/disable.
//! * [`tc`] — Task Controller (ISO 11783-10) applications layer: set-value /
//!   measurement-value process data, device-element registry, and the
//!   zero-copy command path `tpt-t-agri-crop` writes VRA setpoints through.
//! * [`xml`] — a stack-driven, zero-allocation XML generator for farm
//!   management software export; writes directly into the caller's file
//!   buffer (bypassing the heap entirely, per spec §4.1).
//! * [`section`] — section control: overlap detection and <50 ms
//!   nozzle/row-unit shutoff commands (spec §5.1).
//!
//! No CAN driver lives here — the crate consumes [`frame::Frame`] records
//! from whatever socket/interface layer the platform provides.

pub mod frame;
pub mod section;
pub mod tc;
pub mod telemetry;
pub mod tp;
pub mod vt;
pub mod xml;
