#![warn(missing_docs)]
//! `tpt-t-agri-nav` — localization, GNSS correction ingestion, and field
//! guidance for `tpt-teleop-agri`.
//!
//! Modules:
//! * [`geo`] — local tangent-plane (ENU) geometry.
//! * [`nmea`] — zero-allocation NMEA-0183 sentence parsing.
//! * [`rtcm3`] — zero-copy RTCM3 frame detection, CRC24Q validation, and
//!   observation decode (reference-station ECEF + MSM7 header).
//! * [`guidance`] — AB-line and curve (polyline) cross-track math.
//! * [`headland`] — headland-turn state machine.
//! * [`imu`] — scalar Kalman + complementary IMU fusion for diesel-vibration /
//!   soft-soil drift.
//! * [`slam`] — outdoor 2D LiDAR scan-matching SLAM for GPS-shadowed areas
//!   (tree lines, barns, silos).
//!
//! The ingest path is allocation-free where it matters (NMEA/RTCM3 parsing
//! reuses caller buffers); path/guidance state is allocated once at setup.

pub mod geo;
pub mod guidance;
pub mod headland;
pub mod imu;
pub mod nmea;
pub mod rtcm3;
pub mod slam;
