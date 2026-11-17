//! Detection models.

use crate::random::RandomSource;
use crate::world::Swerling;

/// Everything a detection model needs to decide on one look at one target.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DetectionContext {
    /// Range to the target, m.
    pub range: f64,
    /// Mean radar cross-section, m².
    pub rcs_m2: f64,
    /// Fluctuation model.
    pub swerling: Swerling,
    /// Normalised one-way antenna pattern gain toward the target.
    pub pattern_gain: f64,
    /// Jamming-to-noise ratio (linear, 0 = none).
    pub jamming_to_noise: f64,
}

/// The outcome of one look.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DetectionResult {
    /// Whether the target was detected.
    pub detected: bool,
    /// Analytic probability of detection for this look.
    pub probability: f64,
    /// Mean SNR (linear), if the model computes one.
    pub mean_snr: Option<f64>,
    /// Measured SNR (linear), if the model computes one.
    pub measured_snr: Option<f64>,
}

/// A user-supplied detection model.
pub trait DetectionModel: Send + std::fmt::Debug {
    /// Decides whether the target in `ctx` is detected, drawing from `rng`.
    fn detect(&self, ctx: &DetectionContext, rng: &mut dyn RandomSource) -> DetectionResult;
}
