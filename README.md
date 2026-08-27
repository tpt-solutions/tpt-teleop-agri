# tpt-teleop-agri

**Tele-Presence Teleoperation — Precision Agriculture & Autonomous Farming** — a
hyper-optimized, zero-bloat Rust middleware workspace for autonomous farming
machinery (tractors, combines, grain carts, sprayers).

Sister repo to [`tpt-teleop`](https://github.com/tpt-solutions/tpt-teleop)
(video/control transport) and [`tpt-teleop-fleet`](https://github.com/tpt-solutions/tpt-teleop-fleet)
(indoor warehouse precision).

Built for the same microsecond-level determinism as `tpt-teleop`: no async
runtime, no serde, no channels-with-mutexes. Lock-free SPSC rings, zero-copy
rkyv serialization, thread-per-core pinning, and raw OS interfaces all the way
down.

## Status

🚧 **Phase 0 — Repo Foundation & Tooling** in progress. The workspace skeleton
(Cargo workspace, license policy, CI/lint pipelines, `deny.toml` MIT-chain
audit) is in place and `tpt-t-agri-core` is scaffolded. Later phases (bridge,
nav, ISOBUS, env, crop, safety, edge, swarm, logistics, teleop, sim) follow the
checklist in [`todo.md`](todo.md). See
[`docs/quickstart.md`](docs/quickstart.md) for the developer quick start.

| CI | Lint | Deps |
|----|------|------|
| ![build](https://github.com/tpt-solutions/tpt-teleop-agri/actions/workflows/ci.yml/badge.svg) | ![lint](https://github.com/tpt-solutions/tpt-teleop-agri/actions/workflows/lint.yml/badge.svg) | ![deny](https://github.com/tpt-solutions/tpt-teleop-agri/actions/workflows/lint.yml/badge.svg) |

## Workspace Crates

| Crate | Purpose |
|-------|---------|
| `tpt-t-agri-core` | Field state machine, lock-free message bus, central event loop, rkyv wire prelude |
| `tpt-t-agri-nav` | RTCM3/RTK parsing, IMU fusion, AB-line/curve guidance, headland turns |
| `tpt-t-agri-isobus` | Zero-alloc J1939/ISOBUS parser, VT/TC, section control, XML export |
| `tpt-t-agri-env` | Wind/soil/rain micro-climate sensing and adaptive machinery control |
| `tpt-t-agri-crop` | SIMD Bayer→NDVI pipeline (nightly), VRA controller *(nightly crate)* |
| `tpt-t-agri-safety` | ROPS/PTO interlocks, human-in-zone detection, reflex-arc latency budget |
| `tpt-t-agri-edge` | `no_std` path executor, hydraulic valve translation, PWM/DAC access |
| `tpt-t-agri-swarm` | `tpt-t-link`-backed kinematic sync, mass-flow, predictive cart routing |
| `tpt-t-agri-logistics` | Grain bin/silo management, cart routing to dump hoppers |
| `tpt-t-agri-teleop` | `DomainTeleopInterface` adapter, autonomy↔teleop handover |
| `tpt-t-agri-sim` | Headless 50-vehicle harvest simulator + `egui` visualizer |

## The Zero-Copy Data Path

```
Sensor ──Ingest──▶ Fuse/Route ──▶ Field state (in-place)
                                          │
                                 Serialize ──▶ ISOBUS/wire
Total allocations (steady state): 0        Total mutex locks: 0
```

The forward data plane is built on lock-free SPSC rings (`tpt-t-agri-core`) and
proven zero-alloc on the hot path; see `tools/lock-audit.sh` (Phase 2) and the
`zero_alloc` tests added per crate.

## Developer Experience

Path deps during co-development: `tpt-t-agri-*` crates depend on
`tpt-t-agri-core` and, where relevant, on `tpt-teleop`'s `tpt-t-domain-bridge`
(via path dep) and `tpt-t-link` (swarm transport). See
[`docs/quickstart.md`](docs/quickstart.md).

## Licensing

Dual-licensed under either of:

 * Apache License, Version 2.0 — [LICENSE-APACHE](LICENSE-APACHE)
 * MIT license — [LICENSE-MIT](LICENSE-MIT)

Copyright © 2026 TPT Solutions.

Dependency policy ("the MIT chain"): dependencies are restricted to MIT,
BSD-2/3-Clause, ISC, Zlib, and MPL-2.0. Dual MIT/Apache crates are resolved
strictly under MIT; strictly Apache-2.0-only crates are banned. Enforced by
[`deny.toml`](deny.toml) in CI.
