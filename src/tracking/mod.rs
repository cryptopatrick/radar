//! Target tracking (feature `tracking`).

mod filter;
mod manager;
mod matrix;

pub use filter::{AlphaBeta, Covariance6, KalmanCv, TrackState, Tracker};
pub(crate) use manager::{BeamMotion, TrackManager, with_tracker};
pub use manager::{Track, TrackStatus};

/// Which tracking filter a radar runs.
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TrackerConfig {
    /// No tracking.
    None,
    /// Alpha-beta filter.
    AlphaBeta {
        /// Position gain, in `(0, 1]`.
        alpha: f64,
        /// Velocity gain, in `(0, 2)`.
        beta: f64,
    },
    /// Constant-velocity Kalman filter.
    Kalman {
        /// Process-noise acceleration sigma, m/s².
        process_accel_sigma: f64,
    },
    /// A user-supplied filter. Not serialisable and not validated.
    #[cfg_attr(feature = "serde", serde(skip))]
    Custom(Box<dyn Tracker>),
}

impl TrackerConfig {
    /// Clones the config, or returns `None` for [`TrackerConfig::Custom`].
    pub fn try_clone(&self) -> Option<TrackerConfig> {
        Some(match self {
            TrackerConfig::None => TrackerConfig::None,
            TrackerConfig::AlphaBeta { alpha, beta } => TrackerConfig::AlphaBeta {
                alpha: *alpha,
                beta: *beta,
            },
            TrackerConfig::Kalman {
                process_accel_sigma,
            } => TrackerConfig::Kalman {
                process_accel_sigma: *process_accel_sigma,
            },
            TrackerConfig::Custom(_) => return None,
        })
    }
}

/// Track lifecycle parameters.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TrackManagementConfig {
    /// Hits needed to confirm (M of M-of-N).
    pub confirm_m: u32,
    /// Window of illuminated outcomes (N of M-of-N).
    pub confirm_n: u32,
    /// Consecutive misses before deletion.
    pub max_coast: u32,
    /// Probability mass of the association gate, in `(0, 1)`.
    pub gate_probability: f64,
    /// Initial velocity sigma of a new track, m/s.
    pub initial_velocity_sigma: f64,
    /// Position sigma above which a track is deleted, m; `None` means
    /// `instrumented_range / 10`.
    pub max_position_sigma: Option<f64>,
}

impl Default for TrackManagementConfig {
    /// 3-of-5 confirmation, 5 coasts, 99 % gate, 300 m/s initial velocity
    /// sigma, automatic position-sigma limit.
    fn default() -> Self {
        Self {
            confirm_m: 3,
            confirm_n: 5,
            max_coast: 5,
            gate_probability: 0.99,
            initial_velocity_sigma: 300.0,
            max_position_sigma: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn try_clone_rejects_custom() {
        let k = TrackerConfig::Kalman {
            process_accel_sigma: 2.0,
        };
        assert!(matches!(
            k.try_clone(),
            Some(TrackerConfig::Kalman { process_accel_sigma }) if process_accel_sigma == 2.0
        ));
        let custom = TrackerConfig::Custom(Box::new(KalmanCv {
            process_accel_sigma: 1.0,
        }));
        assert!(custom.try_clone().is_none());
    }
}
