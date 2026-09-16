//! Guidance and SLAM hot-path throughput benchmark (Phase 3).
//!
//! Dependency-free harness (`harness = false`): run with
//!
//! ```sh
//! cargo bench --bench guidance -p tpt-t-agri-nav
//! ```
//!
//! Reports per-operation timings for the cross-track hot path and one SLAM
//! scan-match iteration at the field-tuned grid resolution.

use std::hint::black_box;
use std::time::Instant;

use tpt_t_agri_nav::guidance::{AbLine, Vec2};
use tpt_t_agri_nav::slam::{correlative_match, MatchSearch, OccupancyGrid, Pose2};

fn time_per_op<F: FnMut()>(label: &str, iters: u64, mut f: F) {
    // Warmup.
    for _ in 0..(iters / 10).max(1) {
        f();
    }
    let start = Instant::now();
    for _ in 0..iters {
        f();
    }
    let elapsed = start.elapsed();
    println!(
        "{label:<42} {:>10.1} ns/op  ({iters} iters, {elapsed:.1?} total)",
        elapsed.as_nanos() as f64 / iters as f64
    );
}

/// Deterministic point field: two tree lines and a barn, as world points.
fn synthetic_field() -> Vec<Vec2> {
    let mut pts = Vec::new();
    for side in [-30.0, 30.0] {
        for i in 0..300 {
            pts.push(Vec2::new(i as f64 * 0.2, side + ((i % 7) as f64 - 3.0) * 0.1));
        }
    }
    for i in 0..60 {
        let t = i as f64 / 59.0;
        pts.push(Vec2::new(60.0 + 6.0 * t, -8.0));
        pts.push(Vec2::new(66.0, -8.0 + 6.0 * t));
    }
    pts
}

fn scan_from(pose: Pose2, field: &[Vec2], range: f64) -> Vec<Vec2> {
    let c = pose.yaw.cos();
    let s = pose.yaw.sin();
    field
        .iter()
        .filter_map(|w| {
            let dx = w.x - pose.x;
            let dy = w.y - pose.y;
            if dx * dx + dy * dy > range * range {
                return None;
            }
            Some(Vec2::new(c * dx + s * dy, -s * dx + c * dy))
        })
        .collect()
}

fn main() {
    println!("== guidance hot path ==");
    let line = AbLine::new(Vec2::new(0.0, 0.0), Vec2::new(1000.0, 0.0));
    let p = Vec2::new(123.456, 0.05);
    time_per_op("AbLine::cross_track + along_track", 10_000_000, || {
        black_box(line.cross_track(black_box(p)));
        black_box(line.along_track(black_box(p)));
    });

    println!("== SLAM scan match (0.25 m grid, ~500-pt scan) ==");
    let field = synthetic_field();
    let true_pose = Pose2 {
        x: 20.0,
        y: 0.0,
        yaw: 0.0,
    };
    let scan = scan_from(true_pose, &field, 40.0);
    let mut grid = OccupancyGrid::new(Vec2::new(-10.0, -60.0), 100.0, 120.0, 0.25);
    grid.insert_scan(true_pose, &scan, 40.0);
    let search = MatchSearch::default();
    let predicted = Pose2 {
        x: 20.15,
        y: -0.12,
        yaw: 0.02,
    };
    time_per_op("correlative_match (one 5 Hz update)", 50, || {
        let (pose, score) = correlative_match(black_box(&grid), black_box(&scan), predicted, &search);
        black_box(pose);
        black_box(score);
    });

    println!("== SLAM map update (insert + carve) ==");
    time_per_op("OccupancyGrid::insert_scan", 50, || {
        grid.insert_scan(predicted, &scan, 40.0);
    });
}
