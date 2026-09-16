//! Sub-2cm guidance accuracy gate (Phase 3 / spec §5.1).
//!
//! Simulates a tractor following guidance paths under realistic disturbances
//! — RTK position noise (σ = 10 mm), steering actuator lag, and an initial
//! offset — with a Stanley-style controller built only on the crate's public
//! guidance math. The gate: **true steady-state cross-track error must stay
//! under 2 cm RMS** on both a straight AB pass and a curved pass.
//!
//! Deterministic (seeded LCG noise), so CI failures are reproducible.

use tpt_t_agri_nav::guidance::{AbLine, Polyline, Vec2};
use tpt_t_agri_nav::slam::Pose2;

/// Deterministic uniform noise source (no external rand dependency).
struct Lcg(u64);

impl Lcg {
    fn next_f64(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Standard normal via Box–Muller.
    fn normal(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-12);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * core::f64::consts::PI * u2).cos()
    }
}

/// Bicycle-model tractor with a first-order steering actuator.
struct Tractor {
    pose: Pose2,
    steer_rad: f64,
    wheelbase_m: f64,
    steer_tau_s: f64,
    steer_limit_rad: f64,
}

impl Tractor {
    fn step(&mut self, steer_cmd: f64, speed_m_s: f64, dt: f64) {
        // First-order lag toward the commanded angle, rate-limited by the
        // valve's physical slew.
        let error = steer_cmd.clamp(-self.steer_limit_rad, self.steer_limit_rad) - self.steer_rad;
        self.steer_rad += (dt / self.steer_tau_s) * error;
        let theta_dot = speed_m_s * self.steer_rad.tan() / self.wheelbase_m;
        self.pose.yaw += theta_dot * dt;
        self.pose.x += speed_m_s * self.pose.yaw.cos() * dt;
        self.pose.y += speed_m_s * self.pose.yaw.sin() * dt;
    }
}

/// Stanley controller: steer by heading error + cross-track proportional term.
///
/// Sign conventions: `heading_err` = path heading − vehicle yaw (steer
/// toward the path direction); `cross_track_m` positive = vehicle left of
/// the path, which requires steering right (negative delta, since +delta
/// turns left in the bicycle model).
fn stanley_steer(heading_err: f64, cross_track_m: f64, speed_m_s: f64) -> f64 {
    let k_xte = 1.2;
    heading_err - ((k_xte * cross_track_m) / (speed_m_s + 0.5)).atan()
}

/// Run one guidance pass and return `(rms_xte_m, max_xte_m)` over the
/// steady-state window (last 80% of the pass).
fn run_ab_pass() -> (f64, f64) {
    // AB line along +x from the origin; machine starts 0.5 m off-line.
    let line = AbLine::new(Vec2::new(0.0, 0.0), Vec2::new(400.0, 0.0));
    let mut tractor = Tractor {
        pose: Pose2 {
            x: 0.0,
            y: 0.5,
            yaw: 0.0,
        },
        steer_rad: 0.0,
        wheelbase_m: 2.8,
        steer_tau_s: 0.12,
        steer_limit_rad: 0.5,
    };
    let mut rng = Lcg(0x00a1_1ce5);
    let (dt, v) = (0.02, 1.5); // 50 Hz control, 1.5 m/s planting speed
    let steps = (400.0 / (v * dt)) as usize;

    let mut sum_sq = 0.0;
    let mut max = 0.0_f64;
    let mut n = 0usize;
    for step in 0..steps {
        // RTK fix: true position + 10 mm GNSS noise.
        let measured = Vec2::new(
            tractor.pose.x + 0.01 * rng.normal(),
            tractor.pose.y + 0.01 * rng.normal(),
        );
        let xte = line.cross_track(measured);
        let heading_err = -tractor.pose.yaw; // line heading is 0
        tractor.step(stanley_steer(heading_err, xte, v), v, dt);

        // True cross-track error, steady-state window only.
        if step >= steps / 5 {
            let true_xte = line.cross_track(Vec2::new(tractor.pose.x, tractor.pose.y));
            sum_sq += true_xte * true_xte;
            max = max.max(true_xte.abs());
            n += 1;
        }
    }
    ((sum_sq / n as f64).sqrt(), max)
}

/// Closest-segment index search window for the curve pass.
const SEG_WINDOW: usize = 8;

/// Run a curved pass over a polyline and return `(rms_xte_m, max_xte_m)`.
fn run_curve_pass() -> (f64, f64) {
    // Gentle S-curve y = 3·sin(x/30) sampled every 2 m over 400 m.
    let pts: Vec<Vec2> = (0..=200)
        .map(|i| {
            let x = i as f64 * 2.0;
            Vec2::new(x, 3.0 * (x / 30.0).sin())
        })
        .collect();
    let path = Polyline::new(pts.clone());
    let mut tractor = Tractor {
        pose: Pose2 {
            x: 0.0,
            y: 0.5,
            yaw: 0.0,
        },
        steer_rad: 0.0,
        wheelbase_m: 2.8,
        steer_tau_s: 0.12,
        steer_limit_rad: 0.5,
    };
    let mut rng = Lcg(0xc0ffee);
    let (dt, v) = (0.02, 1.5);
    let steps = (400.0 / (v * dt)) as usize;

    let mut seg_idx = 0usize;
    let mut sum_sq = 0.0;
    let mut max = 0.0_f64;
    let mut n = 0usize;
    for step in 0..steps {
        let measured = Vec2::new(
            tractor.pose.x + 0.01 * rng.normal(),
            tractor.pose.y + 0.01 * rng.normal(),
        );

        // Track the closest segment within a forward window (monotone
        // progression, as the guidance loop does on the machine).
        let mut best = seg_idx;
        let mut best_d = f64::INFINITY;
        let mut p = path_point(&pts, seg_idx);
        let mut hdg = path_heading(&pts, seg_idx);
        for cand in seg_idx.saturating_sub(2)..((seg_idx + SEG_WINDOW).min(pts.len() - 2)) {
            let a = pts[cand];
            let b = pts[cand + 1];
            let seg = b.sub(a);
            let len2 = seg.dot(seg);
            let t = (measured.sub(a).dot(seg) / len2).clamp(0.0, 1.0);
            let proj = a.add(seg.scale(t));
            let d = measured.sub(proj).len();
            if d < best_d {
                best_d = d;
                best = cand;
                p = proj;
                hdg = seg.y.atan2(seg.x);
            }
        }
        seg_idx = best;

        let xte = {
            let rel = measured.sub(p);
            let cross = hdg.cos() * rel.y - hdg.sin() * rel.x;
            cross.signum() * best_d
        };
        let heading_err = hdg - tractor.pose.yaw;
        tractor.step(stanley_steer(heading_err, xte, v), v, dt);

        if step >= steps / 5 {
            let true_xte = path.cross_track(Vec2::new(tractor.pose.x, tractor.pose.y));
            sum_sq += true_xte * true_xte;
            max = max.max(true_xte.abs());
            n += 1;
        }
    }
    ((sum_sq / n as f64).sqrt(), max)
}

fn path_point(pts: &[Vec2], seg: usize) -> Vec2 {
    pts[seg.min(pts.len() - 1)]
}

fn path_heading(pts: &[Vec2], seg: usize) -> f64 {
    let s = seg.min(pts.len() - 2);
    let d = pts[s + 1].sub(pts[s]);
    d.y.atan2(d.x)
}

const TARGET_M: f64 = 0.02;

#[test]
fn ab_line_steady_state_under_2cm() {
    let (rms, max) = run_ab_pass();
    println!("AB-line: rms {:.1} mm, max {:.1} mm", rms * 1000.0, max * 1000.0);
    assert!(
        rms < TARGET_M,
        "AB-line steady-state RMS {rms:.4} m exceeds the 2 cm target"
    );
}

#[test]
fn curve_steady_state_under_2cm() {
    let (rms, max) = run_curve_pass();
    println!("Curve: rms {:.1} mm, max {:.1} mm", rms * 1000.0, max * 1000.0);
    assert!(
        rms < TARGET_M,
        "Curve steady-state RMS {rms:.4} m exceeds the 2 cm target"
    );
}

#[test]
fn convergence_happens_within_50m() {
    // Sanity on the transient: after 50 m the machine must be inside 2 cm
    // of the line (true position), not just at the end of the pass.
    let line = AbLine::new(Vec2::new(0.0, 0.0), Vec2::new(400.0, 0.0));
    let mut tractor = Tractor {
        pose: Pose2 {
            x: 0.0,
            y: 1.0,
            yaw: 0.0,
        },
        steer_rad: 0.0,
        wheelbase_m: 2.8,
        steer_tau_s: 0.12,
        steer_limit_rad: 0.5,
    };
    let mut rng = Lcg(42);
    let (dt, v) = (0.02, 1.5);
    for _ in 0..100_000 {
        if tractor.pose.x >= 50.0 {
            break;
        }
        let measured = Vec2::new(
            tractor.pose.x + 0.01 * rng.normal(),
            tractor.pose.y + 0.01 * rng.normal(),
        );
        let xte = line.cross_track(measured);
        tractor.step(stanley_steer(-tractor.pose.yaw, xte, v), v, dt);
    }
    assert!(tractor.pose.x >= 50.0, "tractor never reached 50 m");
    let xte = line.cross_track(Vec2::new(tractor.pose.x, tractor.pose.y)).abs();
    println!("Cross-track after 50 m: {:.1} mm", xte * 1000.0);
    assert!(xte < TARGET_M, "still {xte:.4} m off-line after 50 m");
}
