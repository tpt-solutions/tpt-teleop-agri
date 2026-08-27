//! Local tangent-plane (ENU) geometry helpers for field-scale guidance.
//!
//! Uses the equirectangular approximation about a reference origin. Across a
//! single field this is accurate to well below the sub-2cm guidance target;
//! swap in a full ellipsoid model only if operating across basin scales.

const EARTH_RADIUS_M: f64 = 6_378_137.0;

/// East-North-Up coordinates in metres relative to a reference origin.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Enu {
    /// East (metres).
    pub east: f64,
    /// North (metres).
    pub north: f64,
    /// Up (metres).
    pub up: f64,
}

/// Convert WGS-84 geodetic coordinates to local ENU about `(ref_lat, ref_lon,
/// ref_alt)`.
pub fn geodetic_to_enu(
    lat_deg: f64,
    lon_deg: f64,
    alt_m: f64,
    ref_lat_deg: f64,
    ref_lon_deg: f64,
    ref_alt_m: f64,
) -> Enu {
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let rlat = ref_lat_deg.to_radians();
    let rlon = ref_lon_deg.to_radians();
    let east = (lon - rlon) * lat.cos() * EARTH_RADIUS_M;
    let north = (lat - rlat) * EARTH_RADIUS_M;
    let up = alt_m - ref_alt_m;
    Enu { east, north, up }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn origin_maps_to_zero() {
        let e = geodetic_to_enu(-40.0, 175.0, 100.0, -40.0, 175.0, 100.0);
        assert!(e.east.abs() < 1e-9);
        assert!(e.north.abs() < 1e-9);
        assert!(e.up.abs() < 1e-9);
    }

    #[test]
    fn north_is_positive() {
        let e = geodetic_to_enu(-39.999, 175.0, 0.0, -40.0, 175.0, 0.0);
        assert!(e.north > 0.0);
        let expect = 0.001f64.to_radians() * EARTH_RADIUS_M;
        assert!((e.north - expect).abs() < 1e-3);
    }

    #[test]
    fn east_depends_on_latitude() {
        let e = geodetic_to_enu(-40.0, 175.001, 0.0, -40.0, 175.0, 0.0);
        let expect = 0.001f64.to_radians() * EARTH_RADIUS_M * (-40.0f64).to_radians().cos();
        assert!((e.east - expect).abs() < 1e-3);
    }
}
