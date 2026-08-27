//! Zero-allocation NMEA-0183 sentence parsing for GPS/GNSS receivers.
//!
//! Sentences are parsed in place (no heap allocation); the parser only borrows
//! the input slice and fills a fixed field buffer.

/// Errors returned while parsing an NMEA sentence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NmeaError {
    /// Empty input.
    Empty,
    /// `*`-delimited checksum present but mismatched.
    BadChecksum,
    /// Unsupported / unknown talker sentence.
    Unsupported,
    /// A numeric field failed to parse.
    Parse,
}

/// GGA — global positioning system fix data.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gga {
    /// Fix quality (0 = invalid … 4 = RTK fixed).
    pub fix_quality: u8,
    /// Latitude (degrees, signed).
    pub lat_deg: f64,
    /// Longitude (degrees, signed).
    pub lon_deg: f64,
    /// Altitude above mean sea level (metres).
    pub altitude_m: f64,
    /// Horizontal dilution of precision.
    pub hdop: f32,
    /// Number of satellites used.
    pub sat_count: u8,
}

/// RMC — recommended minimum specific GNSS data.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rmc {
    /// Latitude (degrees, signed).
    pub lat_deg: f64,
    /// Longitude (degrees, signed).
    pub lon_deg: f64,
    /// Speed over ground (m/s).
    pub speed_m_s: f32,
    /// Course over ground (degrees, 0..360).
    pub course_deg: f32,
    /// `b'A'` = valid, `b'V'` = void.
    pub status: u8,
}

/// A parsed NMEA sentence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Nmea {
    /// GGA fix.
    Gga(Gga),
    /// RMC navigation.
    Rmc(Rmc),
}

fn dm_to_deg(value: f64, hemi: u8) -> f64 {
    let deg = (value / 100.0).floor();
    let min = value - deg * 100.0;
    let mut d = deg + min / 60.0;
    if hemi == b'S' || hemi == b'W' {
        d = -d;
    }
    d
}

fn field<'a>(body: &'a str, n: &mut usize) -> &'a str {
    let b = body.trim_start_matches('$');
    let bytes = b.as_bytes();
    let mut start = 0;
    let mut count = 0;
    let mut i = 0;
    while i <= bytes.len() {
        let at_end = i == bytes.len() || bytes[i] == b',';
        if at_end {
            if count == *n {
                *n = count;
                return &b[start..i];
            }
            count += 1;
            start = i + 1;
        }
        i += 1;
    }
    *n = count;
    ""
}

fn parse_f64(s: &str) -> f64 {
    s.parse::<f64>().unwrap_or(0.0)
}

fn parse_gga(body: &str) -> Result<Gga, NmeaError> {
    let mut n = 2;
    let lat = field(body, &mut n);
    n = 3;
    let lat_h = field(body, &mut n)
        .as_bytes()
        .first()
        .copied()
        .unwrap_or(b'N');
    n = 4;
    let lon = field(body, &mut n);
    n = 5;
    let lon_h = field(body, &mut n)
        .as_bytes()
        .first()
        .copied()
        .unwrap_or(b'E');
    n = 6;
    let fix = field(body, &mut n);
    n = 7;
    let sats = field(body, &mut n);
    n = 8;
    let hdop = field(body, &mut n);
    n = 9;
    let alt = field(body, &mut n);
    Ok(Gga {
        fix_quality: parse_f64(fix) as u8,
        lat_deg: dm_to_deg(parse_f64(lat), lat_h),
        lon_deg: dm_to_deg(parse_f64(lon), lon_h),
        altitude_m: parse_f64(alt),
        hdop: parse_f64(hdop) as f32,
        sat_count: parse_f64(sats) as u8,
    })
}

fn parse_rmc(body: &str) -> Result<Rmc, NmeaError> {
    let mut n = 2;
    let status = field(body, &mut n)
        .as_bytes()
        .first()
        .copied()
        .unwrap_or(b'V');
    n = 3;
    let lat = field(body, &mut n);
    n = 4;
    let lat_h = field(body, &mut n)
        .as_bytes()
        .first()
        .copied()
        .unwrap_or(b'N');
    n = 5;
    let lon = field(body, &mut n);
    n = 6;
    let lon_h = field(body, &mut n)
        .as_bytes()
        .first()
        .copied()
        .unwrap_or(b'E');
    n = 7;
    let speed_knots = field(body, &mut n);
    n = 8;
    let course = field(body, &mut n);
    Ok(Rmc {
        status,
        lat_deg: dm_to_deg(parse_f64(lat), lat_h),
        lon_deg: dm_to_deg(parse_f64(lon), lon_h),
        speed_m_s: parse_f64(speed_knots) as f32 * 0.514_444,
        course_deg: parse_f64(course) as f32,
    })
}

/// Parse a single NMEA sentence (`&[u8]`, with or without a trailing `*xx`
/// checksum). Returns the decoded [`Nmea`] or an error.
pub fn parse(line: &[u8]) -> Result<Nmea, NmeaError> {
    if line.is_empty() {
        return Err(NmeaError::Empty);
    }
    let s = core::str::from_utf8(line).map_err(|_| NmeaError::Parse)?;
    let (body, expected) = match s.split_once('*') {
        Some((b, c)) => {
            let exp = u8::from_str_radix(c.trim(), 16).map_err(|_| NmeaError::BadChecksum)?;
            (b, Some(exp))
        }
        None => (s, None),
    };
    if let Some(exp) = expected {
        let mut sum: u8 = 0;
        for b in body.trim_start_matches('$').as_bytes() {
            sum ^= b;
        }
        if sum != exp {
            return Err(NmeaError::BadChecksum);
        }
    }
    let mut n = 0;
    let tag = field(body, &mut n);
    match tag {
        "GPGGA" | "GNGGA" | "GAGGA" => parse_gga(body).map(Nmea::Gga),
        "GPRMC" | "GNRMC" | "GARMC" => parse_rmc(body).map(Nmea::Rmc),
        _ => Err(NmeaError::Unsupported),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gga_parses() {
        let s = b"$GPGGA,134730.00,4112.3456,N,17412.3456,W,1,08,1.0,123.4,M,1.2,M,,";
        let n = parse(s).unwrap();
        let Nmea::Gga(g) = n else { panic!("not gga") };
        assert_eq!(g.fix_quality, 1);
        assert!((g.lat_deg - 41.20576).abs() < 1e-4);
        assert!((g.lon_deg + 174.20576).abs() < 1e-4);
        assert!((g.altitude_m - 123.4).abs() < 1e-6);
        assert_eq!(g.sat_count, 8);
    }

    #[test]
    fn rmc_parses_speed_and_course() {
        let s2 = b"$GPRMC,225446,A,4112.3456,N,17412.3456,W,10.5,90.0,,,";
        let n = parse(s2).unwrap();
        let Nmea::Rmc(r) = n else { panic!("not rmc") };
        assert_eq!(r.status, b'A');
        assert!((r.speed_m_s - 10.5 * 0.514_444).abs() < 1e-3);
        assert!((r.course_deg - 90.0).abs() < 1e-3);
    }

    #[test]
    fn checksum_validated() {
        let no_cs = b"$GPGGA,000000.00,0000.0000,N,0000.0000,E,0,00,0.0,0.0,M,0.0,M,,";
        assert!(parse(no_cs).is_ok());
        let bad = b"$GPGGA,000000.00,0000.0000,N,0000.0000,E,0,00,0.0,0.0,M,0.0,M,,*00";
        assert_eq!(parse(bad).unwrap_err(), NmeaError::BadChecksum);
    }
}
