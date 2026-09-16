# tpt-teleop-agri — Project TODO

Precision Agriculture & Autonomous Farming middleware. Sister repo to `tpt-teleop`
(video/control transport) and `tpt-teleop-fleet` (indoor warehouse precision).
License: MIT OR Apache-2.0. Copyright TPT Solutions.

Source docs: `spec.txt` (primary design doc), `bridge spec.txt` (tpt-teleop
integration contract).

---

## Phase 0 — Repo Foundation & Tooling
- [x] `git init`, initial commit of spec docs
- [x] Cargo workspace `Cargo.toml` (resolver "3", edition 2024, rust-version 1.85,
      license `MIT OR Apache-2.0`, authors "TPT Solutions"), mirroring `tpt-teleop`'s
      workspace layout (`crates/*` members, shared workspace deps table)
- [x] `LICENSE-MIT` and `LICENSE-APACHE` at repo root (TPT Solutions, 2026)
- [x] `.gitignore` (target/, etc.)
- [x] `deny.toml` — MIT/BSD/ISC/Zlib/MPL-2.0 allow-chain, explicit Apache-2.0 deny,
      `[[licenses.clarify]]` overrides for dual-licensed deps (rkyv, windows-sys,
      proc-macro2/syn/quote, libc), copied/adapted from `tpt-teleop`'s policy
- [x] `.github/workflows/ci.yml` — build-test matrix (linux/macos/windows),
      cross-compile check, `RUSTFLAGS: -D warnings`
- [x] `.github/workflows/lint.yml` — fmt + clippy (`-D warnings`) + cargo-deny action
- [x] `rust-toolchain.toml` — stable default; document the nightly override needed for
      `tpt-t-agri-crop` (Phase 6)
- [x] Root `README.md` (Status / Workspace Crates / Zero-Copy Data Path / Developer
      Experience / Licensing — mirror `tpt-teleop`'s section structure)
- [x] `docs/quickstart.md` stub

## Phase 1 — Cross-Repo Bridge: `tpt-t-domain-bridge` (lives in `tpt-teleop` repo)
- [x] New crate `tpt-t-domain-bridge` under `tpt-teleop/crates/`
- [x] Define `DomainTeleopInterface` (DTI) trait: `on_teleop_engage`,
      `on_teleop_disengage`, `on_control_command`, `get_domain_state`,
      `get_sensor_feed`
- [x] Define wire types with `rkyv::Archive`: `ControlCommand`, `InputState`,
      `SensorFeed`, `DomainState`, `DomainTelemetry`, `HapticCmd`/`HapticFeedback`
- [x] Reconcile with existing 56-byte POD `ControlCommand` in
      `tpt-t-core/src/ser/cmd.rs` (bridge spec flags this as an intentional,
      unreconciled discrepancy — resolve or document the mapping)
- [x] Universal safety state machine: `AUTONOMOUS → REQUESTING_TELEOP →
      TELEOP_ACTIVE → RETURNING_TO_AUTONOMY → AUTONOMOUS`, plus `EMERGENCY_STOP`
      transitions
- [x] `request_teleop_assistance()` API surface
- [x] Unit tests + doc examples; publish path so `tpt-t-agri-teleop` can depend on it
      (path dep during co-development)

## Phase 2 — `tpt-t-agri-core`
- [x] Field state machine (mirrors `tpt-t-core::machine` pattern)
- [x] Lock-free message bus for inter-crate event routing
- [x] Central event loop (borrow platform eventloop pattern from `tpt-t-core`)
- [x] rkyv wire-type prelude shared by other agri crates
- [x] Zero-lock hot-path audit passes (reuse `tools/lock-audit.sh` pattern)

## Phase 3 — `tpt-t-agri-nav`
- [x] Custom zero-copy RTCM3 parser (CRC24Q validation, carrier-phase extraction),
       100Hz RTK correction ingestion
- [x] Raw NMEA parsing
- [x] IMU fusion / Kalman filtering tuned for diesel-engine vibration + soft-soil
      track drift
- [ ] Outdoor visual/LiDAR SLAM for GPS-shadowed areas (tree lines, etc.)
- [x] AB-line & curve guidance (sub-2cm target)
- [x] Automated headland turning logic
- [ ] Benchmarks against sub-2cm guidance accuracy target

## Phase 4 — `tpt-t-agri-isobus`
- [ ] Zero-allocation streaming CAN-bus parser for J1939/ISOBUS (ISO 11783) frames
- [ ] Frame → rkyv struct mapping (no heap allocation)
- [ ] Virtual Terminal (VT) support
- [ ] Task Controller (TC) support
- [ ] Implement telemetry decoding
- [ ] Zero-allocation XML generator for farm management software export
      (USB/cellular), writes directly to file buffer
- [ ] Section control: <50ms nozzle/row-unit section shutoff on overlap detection

## Phase 5 — `tpt-t-agri-env`
- [ ] Wind speed/direction sensor integration
- [ ] Soil moisture probe integration
- [ ] Rain sensor integration
- [ ] Dynamic machinery speed / spray nozzle adjustment based on micro-climate state

## Phase 6 — `tpt-t-agri-crop` (nightly Rust, scoped)
- [x] Crate-local nightly toolchain override + `#![feature(portable_simd)]`, CI job
      scoped to this crate only (rest of workspace stays stable)
- [x] SIMD Bayer-to-NDVI pipeline on raw multi-spectral (R/G/B/NIR) sensor data
- [x] Variable Rate Application (VRA) controller output: prescription map ingestion,
      hydraulic down-pressure + seed population adjustment by soil type/topography
- [ ] Target: 4K multi-spectral frame processed in <2ms
- [ ] Additional vegetation indices beyond NDVI (as needed)
- [ ] Zero-copy command path into `tpt-t-agri-isobus` Task Controller (e.g. rotor/fan
      speed adjustment on high-moisture patch detection)
- [ ] Benchmarks proving the <2ms/frame target

## Phase 7 — `tpt-t-agri-safety`
- [ ] ROPS (Rollover Protection System) monitoring
- [ ] PTO (Power Take-Off) safety interlocks
- [ ] Human-in-implement-zone detection → immediate emergency stop
- [ ] Integration with `tpt-t-domain-bridge` safety state machine
      (`REQUESTING_TELEOP` trigger on unrecoverable fault/stall)
- [ ] Local "reflex arc" latency budget defined and benchmarked

## Phase 8 — `tpt-t-agri-edge` (`no_std`)
- [ ] `no_std` path executor for tractor/combine
- [ ] AB-line path → low-level hydraulic proportional valve command translation
- [ ] Direct PWM/DAC register access (no heavy robotics framework)
- [ ] GPS coordinate + yield data logging hooks (feeds Phase 13 NVMe/O_DIRECT work)

## Phase 9 — `tpt-t-agri-swarm`
- [ ] Path dependency on `tpt-teleop`'s `tpt-t-link` crate (reuse `mesh`/`mux`
      transport instead of a new radio crate)
- [ ] Wheel encoder + RTK GPS data sharing over lock-free MPSC ring buffer via
      `tpt-t-link` mesh
- [ ] Dedicated pinned-core kinematic sync loop (target: <500µs relative
      velocity/steering calc)
- [ ] Mass flow tracking (grain yield + moisture content, continuous)
- [ ] Predictive cart routing: grain-tank-full prediction, nearest-cart dispatch,
      intercept vector calculation for moving-unload speed match
- [ ] Relative-distance hold logic (e.g. 1.5m @ 8mph) through GPS-shadowed segments,
      falling back to local LiDAR + wheel encoders
- [ ] Benchmarks proving the <500µs kinematic sync target

## Phase 10 — `tpt-t-agri-logistics`
- [ ] Grain bin management
- [ ] Silo fill-level estimation
- [ ] Automated grain-cart routing to nearest dump hopper via real-time mass flow
      sensor data
- [ ] Silo selection by moisture content + crop type of carried load

## Phase 11 — `tpt-t-agri-teleop` (adapter crate)
- [ ] Depend on `tpt-t-domain-bridge` (Phase 1) and implement `DomainTeleopInterface`
- [ ] `request_teleop_assistance()` call site wired to `tpt-t-agri-safety` stall/fault
      detection
- [ ] `ControlCommand` → ISOBUS implement command translation (raise plow, engage
      PTO, drive-forward speed commands, etc.) via `tpt-t-agri-isobus`
- [ ] Implement control surface: plow raise/lower, spray nozzle adjustment, not just
      tractor movement
- [ ] GPS-denied recovery: visual guidance handoff via `tpt-teleop`'s video link when
      under tree canopy / RTK-shadowed
- [ ] Smooth autonomy↔teleop handover (no jerk) via
      `on_teleop_engage`/`on_teleop_disengage` hooks
- [ ] End-to-end test: stuck-in-mud → teleop request → operator joystick control →
      extrication → control returned to autonomy

## Phase 12 — `tpt-t-agri-sim` + Visualizer
- [ ] Headless simulator: soil mechanics (slip, sinkage)
- [ ] Crop resistance modeling
- [ ] Diesel engine torque curve modeling
- [ ] Hydraulic lag modeling
- [ ] Scale target: 50-vehicle harvest swarm across 5,000-acre virtual field at 100x
      real-time
- [ ] `egui` (MIT) visualizer binary: 3D terrain rendering
- [ ] Visualizer: crop health heatmaps (NDVI)
- [ ] Visualizer: swarm kinematic path rendering for intercept-algorithm tuning

## Phase 13 — System Integration & End-to-End
- [ ] Full "Autonomous Harvest" data flow test (spec.txt §6): prescription map sync
      → ISOBUS header read → NDVI-driven rotor/fan adjustment → swarm intercept →
      moving unload → grain-cart dispatch to dump hopper
- [ ] Offline-first architecture: 12-hour zero-cellular-coverage field operation
- [ ] Local NVMe logging via `O_DIRECT` for telemetry, yield maps, as-applied data
- [ ] WiFi-boundary-crossing sync of rkyv buffers to cloud
- [ ] Cross-workspace zero-alloc/zero-lock verification (extend
      `tpt-t-integration`-style audit from `tpt-teleop` to cover agri crates)

## Phase 14 — Hardening, Docs & v1.0.0 Release
- [ ] Full `cargo-deny` audit clean across the workspace
- [ ] `cargo clippy --workspace --all-targets -- -D warnings` clean
- [ ] Performance benchmarks validated against every spec target (sub-2cm guidance,
      <50ms section control, <2ms/frame NDVI, <500µs swarm sync)
- [ ] Developer docs complete (`docs/quickstart.md`, crate-level rustdoc)
- [ ] README finalized
- [ ] Tag `v1.0.0` — "v1 Monolithic Release"
