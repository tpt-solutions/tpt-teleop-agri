//! Headland-turn state machine for end-of-pass maneuvers.
//!
//! Drives the brief `TurnIn → Turning → TurnOut` sequence at a field boundary.
//! The machine is allocation-free and advanced once per guidance tick with the
//! current signed cross-track error to the AB line and the heading error
//! between the machine's heading and the target line heading.

/// Headland maneuver phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeadlandPhase {
    /// Tracking the AB line normally.
    Following,
    /// Approaching the turn boundary; beginning to slow/curve in.
    TurnIn,
    /// Actively rotating through the headland (≈180°).
    Turning,
    /// Straightening back onto the next pass.
    TurnOut,
    /// Re-aligned with the AB line; hand back to guidance.
    Aligned,
}

/// Stateful planner for headland turns.
pub struct HeadlandPlanner {
    phase: HeadlandPhase,
    turn_radius: f64,
}

impl HeadlandPlanner {
    /// Create a planner with the machine's turn radius (metres) used as the
    /// boundary-approach threshold.
    pub fn new(turn_radius: f64) -> Self {
        Self {
            phase: HeadlandPhase::Following,
            turn_radius: turn_radius.max(1e-3),
        }
    }

    /// Current phase.
    pub fn phase(&self) -> HeadlandPhase {
        self.phase
    }

    /// Advance the machine given the signed cross-track error to the AB line and
    /// the heading error (radians, current − target line heading).
    pub fn update(&mut self, cross_track: f64, heading_error: f64) -> HeadlandPhase {
        match self.phase {
            HeadlandPhase::Following => {
                if cross_track.abs() > self.turn_radius * 0.95 {
                    self.phase = HeadlandPhase::TurnIn;
                }
            }
            HeadlandPhase::TurnIn => {
                if heading_error.abs() > 0.5 {
                    self.phase = HeadlandPhase::Turning;
                }
            }
            HeadlandPhase::Turning => {
                if heading_error.abs() < 0.05 {
                    self.phase = HeadlandPhase::TurnOut;
                }
            }
            HeadlandPhase::TurnOut => {
                if cross_track.abs() < 0.02 {
                    self.phase = HeadlandPhase::Aligned;
                } else {
                    self.phase = HeadlandPhase::Following;
                }
            }
            HeadlandPhase::Aligned => {
                self.phase = HeadlandPhase::Following;
            }
        }
        self.phase
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_headland_cycle() {
        let mut p = HeadlandPlanner::new(2.0);
        assert_eq!(p.phase(), HeadlandPhase::Following);
        p.update(2.5, 0.0); // past threshold → TurnIn
        assert_eq!(p.phase(), HeadlandPhase::TurnIn);
        p.update(3.0, 1.2); // heading diverged → Turning
        assert_eq!(p.phase(), HeadlandPhase::Turning);
        p.update(0.0, 0.01); // straightened → TurnOut
        assert_eq!(p.phase(), HeadlandPhase::TurnOut);
        p.update(-0.01, 0.0); // within tolerance → Aligned
        assert_eq!(p.phase(), HeadlandPhase::Aligned);
        p.update(0.0, 0.0); // hand back
        assert_eq!(p.phase(), HeadlandPhase::Following);
    }

    #[test]
    fn stays_following_within_radius() {
        let mut p = HeadlandPlanner::new(2.0);
        p.update(1.0, 0.0);
        assert_eq!(p.phase(), HeadlandPhase::Following);
    }
}
