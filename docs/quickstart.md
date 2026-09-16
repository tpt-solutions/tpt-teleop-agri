# Quickstart — tpt-teleop-agri

> Status: stub. This document will grow as the crates land. The scaffold below
> is enough to build and test the workspace today.

## Prerequisites

- Rust stable (toolchain pinned via `rust-toolchain.toml`).
- Nightly is **only** required for the `tpt-t-agri-crop` crate
  (`#![feature(portable_simd)]`); that crate is excluded from the stable
  workspace and built by a dedicated CI job.

## Build & test the workspace

```sh
# Whole workspace (stable)
cargo build --workspace --all-targets
cargo test  --workspace

# Lint / format
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings

# Dependency license audit (MIT chain)
cargo deny check
```

## Crate layout

```
crates/
  tpt-t-agri-core/      # field state machine + lock-free bus + rkyv prelude
  tpt-t-agri-nav/       # guidance / RTK (Phase 3)
  tpt-t-agri-isobus/    # J1939/ISOBUS (Phase 4)
  tpt-t-agri-env/       # micro-climate (Phase 5)
  tpt-t-agri-crop/      # SIMD NDVI/VRA, nightly (Phase 6)
  tpt-t-agri-safety/    # interlocks / reflex arc (Phase 7)
  tpt-t-agri-edge/      # no_std path executor (Phase 8)
  tpt-t-agri-swarm/     # kinematic sync / routing (Phase 9)
  tpt-t-agri-logistics/ # grain/silo management (Phase 10)
  tpt-t-agri-teleop/    # DomainTeleopInterface adapter (Phase 11)
  tpt-t-agri-sim/       # headless sim + egui visualizer (Phase 12)
```

## Cross-repo bridge

`Phase 1` is `tpt-t-domain-bridge`. Its canonical home is the `tpt-teleop`
repo (every domain repo depends on it); during co-development it is vendored
in this workspace at `crates/tpt-t-domain-bridge` so domain crates use a
plain path dependency:

```toml
[dependencies]
tpt-t-domain-bridge = { workspace = true }
```

When `tpt-teleop` publishes the bridge, switch the dependency over — the
public API (DTI trait, wire types, safety state machine, assist API) is the
stable contract.
