//! Lightweight IMU fusion for agricultural machinery.
//!
//! Tuned for the two dominant error sources in field work: high-frequency
//! diesel-engine vibration (handled by the Kalman filter's measurement trust)
//! and slow soft-soil track drift (handled by the complementary tilt filter's
//! gyro integration). Both filters are allocation-free and single-state.

/// Scalar Kalman filter for a constant process observed with noisy measurements.
pub struct Kalman1D {
    x: f64,
    p: f64,
    q: f64,
    r: f64,
}

impl Kalman1D {
    /// `q` = process noise variance, `r` = measurement noise variance.
    pub fn new(q: f64, r: f64) -> Self {
        Self {
            x: 0.0,
            p: 1.0,
            q,
            r,
        }
    }

    /// Current estimate.
    pub fn state(&self) -> f64 {
        self.x
    }

    /// Prediction step (constant model: the estimate persists, uncertainty grows).
    pub fn predict(&mut self) {
        self.p += self.q;
    }

    /// Correction step given a measurement `z`.
    pub fn update(&mut self, z: f64) {
        let k = self.p / (self.p + self.r);
        self.x += k * (z - self.x);
        self.p *= 1.0 - k;
    }

    /// Predict + update in one call; returns the new estimate.
    pub fn step(&mut self, z: f64) -> f64 {
        self.predict();
        self.update(z);
        self.x
    }
}

/// Complementary filter fusing accelerometer-derived static tilt with
/// gyroscope integration (radians).
pub struct Complementary {
    angle: f64,
    alpha: f64,
}

impl Complementary {
    /// `alpha` ∈ (0,1): weight on the gyro-integrated estimate. Higher `alpha`
    /// trusts the gyro more (good when the machine is moving, so the
    /// accelerometer is dominated by dynamics).
    pub fn new(alpha: f64) -> Self {
        Self {
            angle: 0.0,
            alpha: alpha.clamp(0.0, 1.0),
        }
    }

    /// Update with `accel_angle` (static tilt from the accelerometer) and the
    /// gyro contribution `gyro_delta` (angular rate × dt). Returns the estimate.
    pub fn update(&mut self, accel_angle: f64, gyro_delta: f64) -> f64 {
        self.angle = self.alpha * (self.angle + gyro_delta) + (1.0 - self.alpha) * accel_angle;
        self.angle
    }

    /// Current tilt estimate (radians).
    pub fn angle(&self) -> f64 {
        self.angle
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kalman_converges_to_constant() {
        let mut k = Kalman1D::new(1e-3, 0.5);
        for _ in 0..50 {
            k.step(1.0);
        }
        assert!((k.state() - 1.0).abs() < 0.05);
    }

    #[test]
    fn kalman_rejects_spike() {
        let mut k = Kalman1D::new(1e-5, 1.0);
        for _ in 0..20 {
            k.step(0.0);
        }
        let before = k.state();
        k.step(100.0);
        assert!((k.state() - before).abs() < 5.0);
    }

    #[test]
    fn complementary_low_pass_steady_state() {
        let mut c = Complementary::new(0.98);
        let mut last = 0.0;
        for _ in 0..2000 {
            last = c.update(0.0, 0.01);
        }
        // Steady state = gyro_delta * alpha / (1 - alpha) = 0.01 * 0.98 / 0.02.
        assert!((last - 0.49).abs() < 1e-3);
    }
}
