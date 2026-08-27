#![feature(portable_simd)]
//! SIMD Bayer → NDVI pipeline and variable-rate application (VRA) controller.
//!
//! Nightly-only crate: it relies on [`std::simd`] (`#![feature(portable_simd)]`).
//! It is excluded from the stable parent workspace (see the root `Cargo.toml`
//! `exclude`) and is built in isolation by the dedicated `crop-nightly` CI job.
//!
//! Design target (spec §6 / Phase 6): process a 4K multi-spectral frame in
//! `< 2 ms`, and emit VRA setpoints (hydraulic down-pressure + seed population)
//! from NDVI and soil/topography maps with zero heap allocation on the path.

use std::simd::prelude::*;

/// Compute NDVI = (NIR − Red) / (NIR + Red) for every pixel using 8-lane SIMD.
///
/// `red` and `nir` must be the same length; `out` is filled in place with the
/// same length. The trailing (< 8) pixels are handled scalarly so any length is
/// accepted. A near-zero denominator (both bands dark) yields `0.0`.
pub fn ndvi_simd(red: &[f32], nir: &[f32], out: &mut [f32]) {
    assert_eq!(red.len(), nir.len());
    assert_eq!(red.len(), out.len());
    let len = red.len();
    let mut i = 0;
    while i + 8 <= len {
        let r = f32x8::from_slice(&red[i..]);
        let n = f32x8::from_slice(&nir[i..]);
        let denom = n + r;
        // Guard the (rare) exact-zero denominator to avoid NaN propagation.
        let safe = denom.simd_ne(f32x8::splat(0.0)).select(denom, f32x8::splat(1.0));
        let ndvi = (n - r) / safe;
        ndvi.copy_to_slice(&mut out[i..]);
        i += 8;
    }
    while i < len {
        let r = red[i];
        let n = nir[i];
        let d = n + r;
        out[i] = if d == 0.0 { 0.0 } else { (n - r) / d };
        i += 1;
    }
}

/// A variable-rate application (VRA) setpoint emitted per management zone.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VraSetpoint {
    /// Hydraulic down-pressure command (kPa).
    pub down_pressure_kpa: f32,
    /// Seed population command (seeds per hectare).
    pub seed_population_ha: f32,
}

/// Compute a VRA setpoint from an NDVI value and a coarse soil-moisture class
/// (0 = dry/sandy, 1 = loam, 2 = wet/clay). Higher NDVI (vigorous canopy) and
/// wetter soil both call for more down-pressure and lower seed population.
pub fn vra_setpoint(ndvi: f32, soil_class: u8) -> VraSetpoint {
    let moisture = match soil_class {
        0 => 0.0_f32,
        1 => 0.5,
        _ => 1.0,
    };
    let vigor = ndvi.clamp(-1.0, 1.0);
    VraSetpoint {
        down_pressure_kpa: (60.0 + moisture * 40.0 + vigor * 10.0).clamp(40.0, 140.0),
        seed_population_ha: (320_000.0 - vigor * 40_000.0 - moisture * 30_000.0)
            .clamp(120_000.0, 360_000.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ndvi_matches_scalar() {
        let red = [0.1, 0.2, 0.3, 0.4, 0.5, 0.05, 0.15, 0.25, 0.0, 0.0];
        let nir = [0.4, 0.5, 0.6, 0.7, 0.8, 0.45, 0.55, 0.65, 0.0, 0.0];
        let mut out = [0.0f32; 10];
        ndvi_simd(&red, &nir, &mut out);
        for i in 0..10 {
            let expected = if red[i] + nir[i] == 0.0 {
                0.0
            } else {
                (nir[i] - red[i]) / (nir[i] + red[i])
            };
            assert!((out[i] - expected).abs() < 1e-5, "pixel {i}: {} vs {}", out[i], expected);
        }
    }

    #[test]
    fn ndvi_zero_denominator_is_zero() {
        let red = [0.0];
        let nir = [0.0];
        let mut out = [1.0f32; 1];
        ndvi_simd(&red, &nir, &mut out);
        assert_eq!(out[0], 0.0);
    }

    #[test]
    fn vra_bounds() {
        let dry = vra_setpoint(0.8, 0);
        let wet = vra_setpoint(0.8, 2);
        // Wetter soil → higher down-pressure, lower population.
        assert!(wet.down_pressure_kpa > dry.down_pressure_kpa);
        assert!(wet.seed_population_ha < dry.seed_population_ha);
        assert!((40.0..=140.0).contains(&wet.down_pressure_kpa));
        assert!((120_000.0..=360_000.0).contains(&wet.seed_population_ha));
    }
}
