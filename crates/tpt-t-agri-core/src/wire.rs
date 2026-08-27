//! Zero-copy rkyv wire-type prelude shared by the other agri crates.
//!
//! All types here derive `rkyv::Archive` so they can be serialized to a
//! contiguous buffer and read back *in place* with zero allocations and zero
//! copies. They are plain-old-data-safe and contain no `Drop` glue, so archived
//! views are safe to map directly over shared memory, NVMe buffers, or the wire.
//!
//! Serialization ([`serialize`]) allocates only the output buffer; the read side
//! is zero-copy via [`access`] (returns a borrowed archived view). Use
//! [`deserialize`] only when an owned `T` is genuinely required.

use bytecheck::CheckBytes;
use rkyv::{Archive, Deserialize, Portable, Serialize};

/// A GNSS/RTK fix in WGS-84. Lat/lon in degrees, altitude in metres; the
/// sub-2cm guidance target is resolved downstream once `rtk_fixed` is true.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct GnssFix {
    /// Latitude (degrees, WGS-84).
    pub lat_deg: f64,
    /// Longitude (degrees, WGS-84).
    pub lon_deg: f64,
    /// Ellipsoidal altitude (metres).
    pub alt_m: f64,
    /// Horizontal precision estimate (m); `0.0` == unknown.
    pub hprec_m: f32,
    /// `true` when RTK-fixed (carrier-phase), `false` for float/standalone.
    pub rtk_fixed: bool,
}

/// A generic agri control command on the wire.
///
/// Distinct from the 56-byte POD `ControlCommand` in `tpt-t-core` — the bridge
/// spec (`bridge spec.txt`) flags that 56-byte layout as an intentional,
/// unreconciled discrepancy and defines the mapping. This struct is the agri
/// domain's native command and maps onto ISOBUS implement commands in
/// `tpt-t-agri-teleop`.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct AgriControlCommand {
    /// Desired ground speed (m/s), signed (negative = reverse).
    pub speed_m_s: f32,
    /// Implement / steering setpoint (degrees, 0 = straight ahead).
    pub steer_deg: f32,
    /// Section / master implement enable bitfield (bit 0 = master, 1.. = sections).
    pub sections: u32,
    /// Sequence number for loss / duplicate detection.
    pub seq: u32,
}

/// A field-machine event routed on the lock-free bus.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct FieldEvent {
    /// Event kind discriminator (crate-local enum encoded as `u16`).
    pub kind: u16,
    /// Monotonic sequence number.
    pub seq: u32,
    /// Opaque payload (interpretation depends on `kind`).
    pub payload: u64,
}

/// Serialize `value` into an owned byte buffer. Allocation occurs only for the
/// output; the read side stays zero-copy via [`access`].
pub fn serialize<T>(value: &T) -> Vec<u8>
where
    T: for<'a> Serialize<
        rkyv::api::high::HighSerializer<
            rkyv::util::AlignedVec,
            rkyv::ser::allocator::ArenaHandle<'a>,
            rkyv::rancor::Error,
        >,
    >,
{
    rkyv::api::high::to_bytes::<rkyv::rancor::Error>(value)
        .expect("rkyv serialize")
        .to_vec()
}

/// Zero-copy read: validate (`CheckBytes`) and return a borrowed archived view.
///
/// Returns `None` if the bytes fail validation (wrong size, corrupt data, or a
/// type/schema mismatch). The returned reference borrows `bytes`, so no copy or
/// allocation occurs.
pub fn access<T>(bytes: &[u8]) -> Option<&<T as Archive>::Archived>
where
    T: Archive,
    <T as Archive>::Archived:
        Portable + for<'a> CheckBytes<rkyv::api::high::HighValidator<'a, rkyv::rancor::Error>>,
{
    rkyv::api::high::access::<<T as Archive>::Archived, rkyv::rancor::Error>(bytes).ok()
}

/// Deserialize into an owned `T`. Use only when an owned value is required;
/// prefer [`access`] for zero-copy reads.
pub fn deserialize<T>(bytes: &[u8]) -> Option<T>
where
    T: Archive,
    <T as Archive>::Archived: Portable
        + for<'a> CheckBytes<rkyv::api::high::HighValidator<'a, rkyv::rancor::Error>>
        + Deserialize<T, rkyv::api::high::HighDeserializer<rkyv::rancor::Error>>,
{
    rkyv::api::high::from_bytes::<T, rkyv::rancor::Error>(bytes).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gnss_roundtrip_zero_copy() {
        let fix = GnssFix {
            lat_deg: -40.123456,
            lon_deg: 175.654321,
            alt_m: 123.5,
            hprec_m: 0.012,
            rtk_fixed: true,
        };
        let bytes = serialize(&fix);
        let view = access::<GnssFix>(&bytes).expect("valid archived GnssFix");
        assert_eq!(view.lat_deg, fix.lat_deg);
        assert_eq!(view.lon_deg, fix.lon_deg);
        assert_eq!(view.alt_m, fix.alt_m);
        assert_eq!(view.hprec_m, fix.hprec_m);
        assert_eq!(view.rtk_fixed, fix.rtk_fixed);
    }

    #[test]
    fn command_deserialize_owned() {
        let cmd = AgriControlCommand {
            speed_m_s: 2.5,
            steer_deg: -1.25,
            sections: 0b11,
            seq: 7,
        };
        let bytes = serialize(&cmd);
        let back = deserialize::<AgriControlCommand>(&bytes).expect("owned command");
        assert_eq!(back, cmd);
    }

    #[test]
    fn corrupt_bytes_rejected() {
        let bytes = serialize(&FieldEvent {
            kind: 1,
            seq: 2,
            payload: 3,
        });
        // A truncated buffer has no valid archived root, so validation must
        // reject it. (rkyv does not semantically validate plain primitive
        // payloads, so we test structural validity, not bit-flips.)
        let truncated = &bytes[..bytes.len().saturating_sub(1)];
        assert!(access::<FieldEvent>(truncated).is_none());
    }
}
