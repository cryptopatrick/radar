//! Measurement noise configuration.

/// Standard deviations of measurement noise.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MeasurementSigma {
    /// Range sigma, m.
    pub range: f64,
    /// Azimuth sigma, rad.
    pub azimuth: f64,
    /// Elevation sigma, rad.
    pub elevation: f64,
    /// Radial-velocity sigma, m/s.
    pub radial_velocity: f64,
}

/// How measurement noise is chosen.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum MeasurementConfig {
    /// Constant sigmas.
    Fixed(MeasurementSigma),
    /// Sigmas shrink with measured SNR (requires range-equation detection):
    /// range `res/√(2·SNR)`, angles `bw/(1.6·√(2·SNR))`, radial velocity
    /// `radial_velocity_resolution/√(2·SNR)`, each at least `floor`.
    SnrScaled {
        /// Radial-velocity resolution, m/s.
        radial_velocity_resolution: f64,
        /// Lower bound on each sigma.
        floor: MeasurementSigma,
    },
}
