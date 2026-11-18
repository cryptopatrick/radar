//! Measurements and measurement noise.

use crate::geometry::Vector3;
use crate::geometry::angle::wrap_two_pi;
use crate::ids::{SensorId, TransponderCode};
use crate::radar::antenna::AntennaConfig;
use crate::radar::config::Dimension;
use crate::random::RandomSource;
use crate::random::sample::standard_normal;
use crate::time::SimTime;

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

/// One noisy radar measurement of one detected target.
///
/// Deliberately carries no target identity: only what the sensor itself
/// could know.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Measurement {
    /// The sensor that made the measurement.
    pub sensor: SensorId,
    /// Simulation time of the measurement.
    pub time: SimTime,
    /// Measured slant range, m.
    pub range: f64,
    /// Measured world-frame compass bearing, rad in `[0, 2π)`.
    pub azimuth: f64,
    /// Measured elevation, rad (`None` for 2D radars).
    pub elevation: Option<f64>,
    /// Measured radial velocity, m/s (positive = opening), if measured.
    pub radial_velocity: Option<f64>,
    /// Measured SNR, dB, if the detection model provides one.
    pub snr_db: Option<f64>,
    /// Noise standard deviations applied to this measurement.
    pub sigma: MeasurementSigma,
    /// Transponder code, if the target replied to interrogation.
    pub identity: Option<TransponderCode>,
    /// World position of the sensor when the measurement was made.
    pub sensor_position: Vector3,
}

/// Computes the measurement sigmas for one detection.
///
/// [`MeasurementConfig::Fixed`] returns the stored sigmas and ignores the
/// SNR. [`MeasurementConfig::SnrScaled`] computes range `res/√(2·snr)`,
/// angles `bw/(1.6·√(2·snr))` and radial velocity
/// `radial_velocity_resolution/√(2·snr)`, each at least the floor. Without a
/// usable (finite, positive) `measured_snr` the floor is returned; without a
/// `range_resolution` the range sigma is the floor.
pub fn measurement_sigma(
    cfg: &MeasurementConfig,
    antenna: &AntennaConfig,
    range_resolution: Option<f64>,
    measured_snr: Option<f64>,
) -> MeasurementSigma {
    match *cfg {
        MeasurementConfig::Fixed(sigma) => sigma,
        MeasurementConfig::SnrScaled {
            radial_velocity_resolution,
            floor,
        } => {
            let Some(snr) = measured_snr.filter(|s| s.is_finite() && *s > 0.0) else {
                return floor;
            };
            let root = (2.0 * snr).sqrt();
            MeasurementSigma {
                range: range_resolution.map_or(floor.range, |res| (res / root).max(floor.range)),
                azimuth: (antenna.beamwidth_az / (1.6 * root)).max(floor.azimuth),
                elevation: (antenna.beamwidth_el / (1.6 * root)).max(floor.elevation),
                radial_velocity: (radial_velocity_resolution / root).max(floor.radial_velocity),
            }
        }
    }
}

/// Adds Gaussian noise to a true measurement. Draws in the fixed order
/// range, azimuth, elevation (3D only), radial velocity (if `Some`).
/// Range is clamped at 0 and azimuth wrapped to `[0, 2π)`.
#[cfg_attr(not(test), allow(dead_code))] // Used by the radar sensor.
pub(crate) fn noisy_measurement(
    true_range: f64,
    true_az: f64,
    true_el: f64,
    true_rv: Option<f64>,
    sigma: MeasurementSigma,
    dim: Dimension,
    rng: &mut dyn RandomSource,
) -> (f64, f64, Option<f64>, Option<f64>) {
    let range = (true_range + sigma.range * standard_normal(rng)).max(0.0);
    let az = wrap_two_pi(true_az + sigma.azimuth * standard_normal(rng));
    let el = match dim {
        Dimension::TwoD => None,
        Dimension::ThreeD => Some(true_el + sigma.elevation * standard_normal(rng)),
    };
    let rv = true_rv.map(|v| v + sigma.radial_velocity * standard_normal(rng));
    (range, az, el, rv)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::SensorId;
    use crate::radar::Dimension;
    use crate::radar::antenna::AntennaConfig;
    use crate::random::{ChaChaStream, Seed, StreamId};
    use approx::assert_relative_eq;

    const FLOOR: MeasurementSigma = MeasurementSigma {
        range: 1.0,
        azimuth: 1e-4,
        elevation: 1e-4,
        radial_velocity: 0.1,
    };

    #[test]
    fn snr_scaled_sigma() {
        let cfg = MeasurementConfig::SnrScaled {
            radial_velocity_resolution: 10.0,
            floor: FLOOR,
        };
        let ant = AntennaConfig::default();
        let s = measurement_sigma(&cfg, &ant, Some(150.0), Some(100.0));
        assert_relative_eq!(s.range, 150.0 / 200f64.sqrt(), max_relative = 1e-12);
        assert_relative_eq!(s.range, 10.607, epsilon = 1e-3);
        assert_relative_eq!(
            s.azimuth,
            ant.beamwidth_az / (1.6 * 200f64.sqrt()),
            max_relative = 1e-12
        );
        assert_relative_eq!(
            s.elevation,
            ant.beamwidth_el / (1.6 * 200f64.sqrt()),
            max_relative = 1e-12
        );
        assert_relative_eq!(
            s.radial_velocity,
            10.0 / 200f64.sqrt(),
            max_relative = 1e-12
        );
        let huge = measurement_sigma(&cfg, &ant, Some(150.0), Some(1e12));
        assert_eq!(huge, FLOOR);
        let none = measurement_sigma(&cfg, &ant, Some(150.0), None);
        assert_eq!(none, FLOOR);
        let fixed = measurement_sigma(&MeasurementConfig::Fixed(FLOOR), &ant, None, Some(1.0));
        assert_eq!(fixed, FLOOR);
    }

    #[test]
    fn noise_wraps_azimuth() {
        let sigma = MeasurementSigma {
            range: 10.0,
            azimuth: 0.1,
            elevation: 0.1,
            radial_velocity: 1.0,
        };
        let mut rng = ChaChaStream::new(Seed(7), StreamId::Sensor(SensorId(3)));
        let mut wrapped = false;
        for _ in 0..1_000 {
            let (r, az, el, rv) =
                noisy_measurement(5.0, 0.0, 0.0, None, sigma, Dimension::TwoD, &mut rng);
            assert!(r >= 0.0);
            assert!((0.0..std::f64::consts::TAU).contains(&az));
            wrapped |= az > std::f64::consts::PI;
            assert_eq!(el, None);
            assert_eq!(rv, None);
        }
        assert!(wrapped);
        let (_, _, el, rv) =
            noisy_measurement(5.0, 0.0, 0.0, Some(3.0), sigma, Dimension::ThreeD, &mut rng);
        assert!(el.is_some());
        assert!(rv.is_some());
    }
}
