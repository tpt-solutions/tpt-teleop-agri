//! Outdoor 2D LiDAR SLAM for GPS-shadowed field areas (tree lines, barns,
//! silos, headland shelter belts).
//!
//! When RTK corrections drop out under canopy, dead reckoning alone drifts
//! unacceptably over a pass. This module keeps the pose locked by matching
//! each new LiDAR scan against a persistent occupancy grid of the field's
//! static structure:
//!
//! 1. [`OccupancyGrid`] — a fixed-capacity log-odds map, allocated once at
//!    setup, updated with Bresenham ray carving (misses) and endpoint hits.
//!    Cell probabilities are cached alongside the log-odds so scan scoring
//!    never evaluates an exponential on the hot path.
//! 2. [`correlative_match`] — brute-force correlative scan matching (the
//!    core of real-time correlative scan matchers): transform the scan by
//!    candidate poses around the odometry prediction, score by summed
//!    occupancy, take the argmax, then refine below cell resolution.
//! 3. [`Slam`] — the session: prediction from odometry/IMU, match correction,
//!    optional RTK anchoring when fixes return, and a match-quality metric
//!    the guidance layer can gate on.
//!
//! The per-scan path allocates nothing: the grid is owned by the session and
//! scans are borrowed slices of [`Vec2`].

use crate::guidance::Vec2;

/// A 2D pose: position in the local ENU plane plus yaw.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose2 {
    /// East (metres).
    pub x: f64,
    /// North (metres).
    pub y: f64,
    /// Yaw (radians, atan2(north, east)).
    pub yaw: f64,
}

impl Pose2 {
    /// Transform a point from the scan (body) frame into the world frame.
    pub fn transform(self, p: Vec2) -> Vec2 {
        let (s, c) = self.yaw.sin_cos();
        Vec2::new(self.x + c * p.x - s * p.y, self.y + s * p.x + c * p.y)
    }
}

// Log-odds update constants (p_hit = 0.7, p_miss = 0.4) and the clamp that
// keeps cells from saturating (p in [0.06, 0.95]).
const LO_HIT: f32 = 0.847;
const LO_MISS: f32 = -0.405;
const LO_MIN: f32 = -2.75;
const LO_MAX: f32 = 2.94;

/// Log-odds occupancy grid over a rectangular field area.
///
/// Storage is log-odds (`cells`); `prob` caches the sigmoid-mapped
/// occupancy probability so the scan matcher reads a plain float per cell.
pub struct OccupancyGrid {
    cells: Box<[f32]>,
    prob: Box<[f32]>,
    w: usize,
    h: usize,
    cell_m: f64,
    origin: Vec2,
}

impl OccupancyGrid {
    /// Allocate a grid covering `width_m × height_m` at cell size `cell_m`,
    /// with its lower-left corner at world position `origin`.
    pub fn new(origin: Vec2, width_m: f64, height_m: f64, cell_m: f64) -> Self {
        assert!(cell_m > 0.0, "cell size must be positive");
        let w = (width_m / cell_m).ceil() as usize;
        let h = (height_m / cell_m).ceil() as usize;
        Self {
            cells: vec![0.0_f32; w * h].into_boxed_slice(),
            prob: vec![0.5_f32; w * h].into_boxed_slice(),
            w,
            h,
            cell_m,
            origin,
        }
    }

    /// World position of the grid's lower-left corner.
    pub fn origin(&self) -> Vec2 {
        self.origin
    }

    /// Cell edge length (metres).
    pub fn cell_m(&self) -> f64 {
        self.cell_m
    }

    /// Grid width in cells.
    pub fn width_cells(&self) -> usize {
        self.w
    }

    /// Grid height in cells.
    pub fn height_cells(&self) -> usize {
        self.h
    }

    fn cell_index(&self, p: Vec2) -> Option<(usize, usize)> {
        let fx = (p.x - self.origin.x) / self.cell_m;
        let fy = (p.y - self.origin.y) / self.cell_m;
        if !(fx >= 0.0 && fy >= 0.0) {
            return None;
        }
        let (ix, iy) = (fx as usize, fy as usize);
        if ix < self.w && iy < self.h {
            Some((ix, iy))
        } else {
            None
        }
    }

    fn bump_cell(&mut self, idx: usize, delta: f32) {
        let lo = (self.cells[idx] + delta).clamp(LO_MIN, LO_MAX);
        self.cells[idx] = lo;
        self.prob[idx] = 1.0 / (1.0 + (-lo).exp());
    }

    /// Occupancy probability at a world point; points outside the grid read
    /// as 0.5 (unknown — the matcher neither rewards nor punishes them).
    pub fn occupancy(&self, p: Vec2) -> f32 {
        match self.cell_index(p) {
            Some((ix, iy)) => self.prob[iy * self.w + ix],
            None => 0.5,
        }
    }

    /// Bilinearly interpolated occupancy at a world point (unknown-outside
    /// reads as 0.5). Used by the scan matcher: interpolation removes the
    /// cell-quantization plateaus of [`Self::occupancy`] so refinement
    /// converges below cell resolution.
    pub fn occupancy_interp(&self, p: Vec2) -> f32 {
        let fx = (p.x - self.origin.x) / self.cell_m - 0.5;
        let fy = (p.y - self.origin.y) / self.cell_m - 0.5;
        if !(fx >= 0.0 && fy >= 0.0) {
            return 0.5;
        }
        let (ix, iy) = (fx as usize, fy as usize);
        if ix + 1 >= self.w || iy + 1 >= self.h {
            return 0.5;
        }
        let tx = (fx - ix as f64) as f32;
        let ty = (fy - iy as f64) as f32;
        let base = iy * self.w + ix;
        let top = self.prob[base] * (1.0 - tx) + self.prob[base + 1] * tx;
        let bottom = self.prob[base + self.w] * (1.0 - tx) + self.prob[base + self.w + 1] * tx;
        top * (1.0 - ty) + bottom * ty
    }

    /// Update the log-odds of a single cell (`hit = true` for a scan
    /// endpoint, `false` for a carved miss).
    fn update_cell(&mut self, p: Vec2, hit: bool) {
        if let Some((ix, iy)) = self.cell_index(p) {
            self.bump_cell(iy * self.w + ix, if hit { LO_HIT } else { LO_MISS });
        }
    }

    /// Insert one scan: carve free space along each beam from `sensor_pose`
    /// and mark the endpoints as occupied. Points beyond `max_range_m` only
    /// carve (their endpoints are range-clipped, not real obstacles).
    pub fn insert_scan(&mut self, sensor_pose: Pose2, scan: &[Vec2], max_range_m: f64) {
        let start_world = sensor_pose.transform(Vec2::new(0.0, 0.0));
        for p in scan {
            let end_world = sensor_pose.transform(*p);
            let range = end_world.sub(start_world).len();
            self.carve_ray(start_world, end_world);
            if range <= max_range_m {
                self.update_cell(end_world, true);
            }
        }
    }

    /// Bresenham ray from `from` to `to` (exclusive of the endpoint cell),
    /// marking every traversed cell as a miss.
    fn carve_ray(&mut self, from: Vec2, to: Vec2) {
        let Some((mut x0, mut y0)) = self.cell_index(from) else {
            return;
        };
        let Some((x1, y1)) = self.cell_index(to) else {
            return;
        };
        let dx = (x1 as isize - x0 as isize).abs();
        let dy = (y1 as isize - y0 as isize).abs();
        let sx = if x1 >= x0 { 1 } else { -1 };
        let sy = if y1 >= y0 { 1 } else { -1 };
        let mut err = dx - dy;
        let mut steps = 0usize;
        let max_steps = self.w.max(self.h) * 2;
        while (x0 != x1 || y0 != y1) && steps < max_steps {
            let idx = y0 * self.w + x0;
            self.bump_cell(idx, LO_MISS);
            let e2 = 2 * err;
            if e2 > -dy {
                err -= dy;
                x0 = (x0 as isize + sx) as usize;
            }
            if e2 < dx {
                err += dx;
                y0 = (y0 as isize + sy) as usize;
            }
            steps += 1;
        }
    }
}

/// Search-window configuration for [`correlative_match`].
#[derive(Debug, Clone, Copy)]
pub struct MatchSearch {
    /// Half-width of the translation search window (metres).
    pub xy_window_m: f64,
    /// Half-width of the yaw search window (radians).
    pub yaw_window_rad: f64,
    /// Translation step of the coarse pass (metres; typically ≤ cell size).
    pub coarse_step_m: f64,
    /// Number of yaw samples in the coarse pass (≥ 1).
    pub yaw_samples: u32,
    /// Coarse-to-fine refinement factor per stage (two stages run).
    pub refine_divisor: f64,
}

impl Default for MatchSearch {
    fn default() -> Self {
        Self {
            xy_window_m: 0.30,
            yaw_window_rad: 0.05,
            coarse_step_m: 0.05,
            yaw_samples: 9,
            refine_divisor: 4.0,
        }
    }
}

/// Sum the grid occupancy under `scan` transformed by `pose`. Higher is a
/// better alignment.
fn score(grid: &OccupancyGrid, scan: &[Vec2], pose: Pose2) -> f64 {
    let mut sum = 0.0_f64;
    for p in scan {
        sum += grid.occupancy_interp(pose.transform(*p)) as f64;
    }
    sum
}

/// Brute-force correlative scan matching: find the pose near `init` that
/// best aligns `scan` with the grid, refining the coarse winner below the
/// coarse step size. Allocation-free.
///
/// Returns the refined pose and its *mean* occupancy score (`0.0..1.0`); a
/// score the caller considers too low means "do not trust this match".
pub fn correlative_match(
    grid: &OccupancyGrid,
    scan: &[Vec2],
    init: Pose2,
    search: &MatchSearch,
) -> (Pose2, f64) {
    let mut best = init;
    let mut best_score = f64::NEG_INFINITY;
    let yaw_n = search.yaw_samples.max(1);
    let yaw_lo = init.yaw - search.yaw_window_rad;
    let yaw_step = if yaw_n > 1 {
        2.0 * search.yaw_window_rad / (yaw_n - 1) as f64
    } else {
        0.0
    };

    let mut dyaw = -search.yaw_window_rad;
    for _ in 0..yaw_n {
        let mut dx = -search.xy_window_m;
        while dx <= search.xy_window_m {
            let mut dy = -search.xy_window_m;
            while dy <= search.xy_window_m {
                let cand = Pose2 {
                    x: init.x + dx,
                    y: init.y + dy,
                    yaw: yaw_lo + dyaw,
                };
                let s = score(grid, scan, cand);
                if s > best_score {
                    best_score = s;
                    best = cand;
                }
                dy += search.coarse_step_m;
            }
            dx += search.coarse_step_m;
        }
        dyaw += yaw_step;
    }

    // Refinement: shrinking coordinate steps around the coarse winner.
    let mut step = search.coarse_step_m / search.refine_divisor;
    for _ in 0..3 {
        let before = best;
        for dx in [-step, 0.0, step] {
            for dy in [-step, 0.0, step] {
                for dyaw in [-step * 0.5, 0.0, step * 0.5] {
                    let cand = Pose2 {
                        x: before.x + dx,
                        y: before.y + dy,
                        yaw: before.yaw + dyaw,
                    };
                    let s = score(grid, scan, cand);
                    if s > best_score {
                        best_score = s;
                        best = cand;
                    }
                }
            }
        }
        step /= search.refine_divisor;
    }

    let mean = if scan.is_empty() {
        0.0
    } else {
        best_score / scan.len() as f64
    };
    (best, mean)
}

/// A SLAM session over one field area.
pub struct Slam {
    grid: OccupancyGrid,
    pose: Pose2,
    quality: f64,
    last_match_good: bool,
    search: MatchSearch,
    max_range_m: f64,
    /// Mean score above which a match is trusted.
    min_good_score: f64,
    /// How far the correction may jump from the prediction before the match
    /// is treated as degenerate (metres).
    max_jump_m: f64,
}

impl Slam {
    /// Create a session over `width_m × height_m` at `cell_m` resolution.
    pub fn new(origin: Vec2, width_m: f64, height_m: f64, cell_m: f64) -> Self {
        Self {
            grid: OccupancyGrid::new(origin, width_m, height_m, cell_m),
            pose: Pose2::default(),
            quality: 0.0,
            last_match_good: false,
            search: MatchSearch::default(),
            max_range_m: 40.0,
            min_good_score: 0.4,
            max_jump_m: 1.0,
        }
    }

    /// Search configuration (read).
    pub fn search(&self) -> &MatchSearch {
        &self.search
    }

    /// Mutable access to the matcher's search configuration.
    pub fn search_mut(&mut self) -> &mut MatchSearch {
        &mut self.search
    }

    /// Current best pose estimate.
    pub fn pose(&self) -> Pose2 {
        self.pose
    }

    /// Mean occupancy score of the last match (`0.0` before the first).
    pub fn quality(&self) -> f64 {
        self.quality
    }

    /// Whether the last match met the trust threshold and jump gate.
    pub fn last_match_trusted(&self) -> bool {
        self.last_match_good
    }

    /// Grid accessor for map export.
    pub fn grid(&self) -> &OccupancyGrid {
        &self.grid
    }

    /// Re-anchor the pose from an RTK fix (already converted to ENU). Used
    /// on session start and whenever RTK returns after a shadowed segment.
    /// The scan is inserted so the map stays consistent with the anchored
    /// pose.
    pub fn anchor(&mut self, pose: Pose2, scan: &[Vec2]) {
        self.pose = pose;
        self.grid.insert_scan(pose, scan, self.max_range_m);
    }

    /// One SLAM iteration: predict from odometry, scan-match correct, update
    /// the map. `predicted` is the odometry/IMU dead-reckoned pose since the
    /// last call (absolute, not incremental).
    ///
    /// When the match is untrusted (low score or implausible jump), the
    /// prediction is kept and [`Slam::last_match_trusted`] reports `false`
    /// so guidance can degrade (slow down / request operator assistance).
    pub fn update(&mut self, predicted: Pose2, scan: &[Vec2]) -> Pose2 {
        let (matched, mean) = correlative_match(&self.grid, scan, predicted, &self.search);
        let jump = Vec2::new(matched.x - predicted.x, matched.y - predicted.y).len();
        let good = mean >= self.min_good_score && jump <= self.max_jump_m;
        if good {
            self.pose = matched;
        } else {
            self.pose = predicted;
        }
        self.quality = mean;
        self.last_match_good = good;
        self.grid.insert_scan(self.pose, scan, self.max_range_m);
        self.pose
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic LCG so tests never depend on an external rand crate.
    struct Lcg(u64);
    impl Lcg {
        fn next_f64(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
        fn range(&mut self, lo: f64, hi: f64) -> f64 {
            lo + (hi - lo) * self.next_f64()
        }
    }

    /// Build a synthetic field: two tree-line rows (dense point clusters)
    /// and a barn rectangle, as world-frame points.
    fn synthetic_field() -> Vec<Vec2> {
        let mut pts = Vec::new();
        let mut rng = Lcg(0x1234_5678);
        for side in [-30.0, 30.0] {
            for i in 0..300 {
                let x = i as f64 * 0.2;
                let y = side + rng.range(-0.4, 0.4);
                pts.push(Vec2::new(x, y));
            }
        }
        for i in 0..60 {
            let t = i as f64 / 59.0;
            pts.push(Vec2::new(60.0 + 6.0 * t, -8.0));
            pts.push(Vec2::new(60.0 + 6.0 * t, -2.0));
            pts.push(Vec2::new(60.0, -8.0 + 6.0 * t));
            pts.push(Vec2::new(66.0, -8.0 + 6.0 * t));
        }
        pts
    }

    /// Simulate a 2D LiDAR: field points within `range` of `pose`, in the
    /// body frame.
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

    #[test]
    fn grid_marks_hits_and_carves_misses() {
        let mut grid = OccupancyGrid::new(Vec2::new(0.0, 0.0), 20.0, 20.0, 0.5);
        let scan = [Vec2::new(2.0, 0.0), Vec2::new(2.0, 0.0)];
        let pose = Pose2 {
            x: 10.0,
            y: 10.0,
            yaw: 0.0,
        };
        for _ in 0..5 {
            grid.insert_scan(pose, &scan, 40.0);
        }
        // Endpoint cell (12,10) is occupied after 5 hits.
        let hit = grid.occupancy(Vec2::new(12.0, 10.0));
        assert!(hit > 0.9, "hit cell occupancy {hit}");
        // Cells traversed by the beam read as free (5 misses reach
        // p ≈ 0.12 from the 0.4-miss log-odds).
        let free = grid.occupancy(Vec2::new(11.0, 10.0));
        assert!(free < 0.15, "carved cell occupancy {free}");
        // Unknown area reads 0.5.
        assert!((grid.occupancy(Vec2::new(19.5, 19.5)) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn out_of_range_endpoints_are_not_marked() {
        let mut grid = OccupancyGrid::new(Vec2::new(0.0, 0.0), 20.0, 20.0, 0.5);
        let scan = [Vec2::new(19.0, 0.0)];
        let pose = Pose2 {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        };
        for _ in 0..5 {
            grid.insert_scan(pose, &scan, 10.0);
        }
        // The beam carves, but the range-clipped endpoint stays unknown.
        let mid = grid.occupancy(Vec2::new(15.0, 0.0));
        assert!(mid < 0.15, "mid-beam cell must be carved: {mid}");
        let endpoint = grid.occupancy(Vec2::new(19.25, 0.0));
        assert!(
            (endpoint - 0.5).abs() < 1e-6,
            "clamped endpoint must stay unknown: {endpoint}"
        );
    }

    #[test]
    fn scan_match_recovers_known_offset() {
        let field = synthetic_field();
        let true_pose = Pose2 {
            x: 20.0,
            y: 0.0,
            yaw: 0.0,
        };
        let scan = scan_from(true_pose, &field, 40.0);
        assert!(!scan.is_empty(), "synthetic field must be visible");

        let mut slam = Slam::new(Vec2::new(-10.0, -60.0), 100.0, 120.0, 0.25);
        slam.anchor(true_pose, &scan);

        // Prediction drifted 0.25 m east, 0.2 m south, 0.03 rad.
        let predicted = Pose2 {
            x: 20.25,
            y: -0.20,
            yaw: 0.03,
        };
        let (match_pose, score) =
            correlative_match(slam.grid(), &scan, predicted, slam.search());
        assert!(
            (match_pose.x - true_pose.x).abs() < 0.06
                && (match_pose.y - true_pose.y).abs() < 0.06,
            "match {match_pose:?} vs true {true_pose:?}"
        );
        assert!(
            (match_pose.yaw - true_pose.yaw).abs() < 0.01,
            "yaw error {}",
            (match_pose.yaw - true_pose.yaw).abs()
        );
        assert!(score > 0.35, "expected strong match, got {score}");
    }

    #[test]
    fn slam_session_tracks_through_gnss_shadow() {
        let field = synthetic_field();
        let mut slam = Slam::new(Vec2::new(-10.0, -60.0), 100.0, 120.0, 0.25);
        let mut rng = Lcg(0xdead_beef);

        // Ground-truth start; anchor with the first scan.
        let mut truth = Pose2 {
            x: 5.0,
            y: 0.0,
            yaw: 0.0,
        };
        let scan0 = scan_from(truth, &field, 40.0);
        slam.anchor(truth, &scan0);

        // Drive at 2 m/s for 25 s @ 5 Hz. RTK is shadowed the whole time:
        // predictions are odometry with a systematic +2% scale error.
        for step in 0..125 {
            truth.yaw = (step as f64 * 0.004).sin() * 0.05;
            truth.x += 0.4 * truth.yaw.cos();
            truth.y += 0.4 * truth.yaw.sin();

            let prev = slam.pose();
            let odo = Pose2 {
                x: prev.x + 0.4 * truth.yaw.cos() * 1.02,
                y: prev.y + 0.4 * truth.yaw.sin() * 1.02,
                yaw: truth.yaw + rng.range(-0.002, 0.002),
            };
            let scan = scan_from(truth, &field, 40.0);
            let pose = slam.update(odo, &scan);
            assert!(slam.last_match_trusted(), "match lost at step {step}");
            let err = Vec2::new(pose.x - truth.x, pose.y - truth.y).len();
            assert!(err < 0.5, "SLAM error {err:.3} m at step {step}");
        }
    }

    #[test]
    fn degraded_match_falls_back_to_prediction() {
        let mut slam = Slam::new(Vec2::new(0.0, 0.0), 50.0, 50.0, 0.5);
        slam.anchor(Pose2::default(), &[]);
        // An empty scan over an empty grid cannot produce a trusted match.
        let predicted = Pose2 {
            x: 1.0,
            y: 1.0,
            yaw: 0.0,
        };
        let pose = slam.update(predicted, &[]);
        assert_eq!(pose, predicted, "untrusted match keeps prediction");
        assert!(!slam.last_match_trusted());
        assert_eq!(slam.quality(), 0.0);
    }

    #[test]
    fn pose_transform_matches_rotation_math() {
        let p = Pose2 {
            x: 1.0,
            y: 2.0,
            yaw: core::f64::consts::FRAC_PI_2,
        };
        let q = p.transform(Vec2::new(1.0, 0.0));
        assert!((q.x - 1.0).abs() < 1e-9);
        assert!((q.y - 3.0).abs() < 1e-9);
    }
}
