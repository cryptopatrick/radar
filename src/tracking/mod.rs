//! Target tracking (feature `tracking`).

/// Which tracking filter a radar runs.
#[derive(Debug, Clone, PartialEq)]
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
