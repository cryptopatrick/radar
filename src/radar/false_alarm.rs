//! False alarm generation.

use crate::geometry::angle::wrap_two_pi;
use crate::radar::antenna::AntennaConfig;
use crate::radar::config::{Dimension, RangeLimits};
use crate::radar::scan::Sweep;
use crate::random::RandomSource;
use crate::random::sample::{exponential, poisson, uniform01};
use std::f64::consts::TAU;

/// Number of resolution cells examined during a sweep:
/// `((instrumented − min) / res) × max(1, |swept_az| / beamwidth_az)`.
pub fn resolution_cells(
    limits: &RangeLimits,
    range_resolution: f64,
    swept_az: f64,
    beamwidth_az: f64,
) -> f64 {
    ((limits.instrumented_range - limits.min_range) / range_resolution)
        * (swept_az.abs() / beamwidth_az).max(1.0)
}

/// A spurious detection before measurement noise.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FalseAlarm {
    /// Range, m.
    pub range: f64,
    /// Azimuth in the scan frame, in `[0, 2π)`.
    pub azimuth_scan: f64,
    /// Elevation in the scan frame; 0 in 2D.
    pub elevation_scan: f64,
    /// Measured SNR (linear, power − 1); unused in radius mode.
    pub measured_snr: f64,
}

/// Draws range, two azimuth uniforms and (3D only) elevation for one alarm.
fn draw_position(
    sweep: &Sweep,
    limits: &RangeLimits,
    antenna: &AntennaConfig,
    dim: Dimension,
    rng: &mut dyn RandomSource,
) -> (f64, f64, f64) {
    let range = limits.min_range + uniform01(rng) * (limits.instrumented_range - limits.min_range);
    let u1 = uniform01(rng);
    let u2 = uniform01(rng);
    let az = wrap_two_pi(
        sweep.start_azimuth + u1 * sweep.delta_azimuth + (u2 - 0.5) * antenna.beamwidth_az,
    );
    let el = match dim {
        Dimension::TwoD => 0.0,
        Dimension::ThreeD => sweep.elevation + (uniform01(rng) - 0.5) * antenna.beamwidth_el,
    };
    (range, az, el)
}

/// Generates range-equation false alarms for one sweep:
/// `count = poisson(pfa · cells)`; per alarm the draw order is range, two
/// azimuth uniforms, elevation (3D only), then the power exponential.
#[allow(clippy::too_many_arguments)]
pub(crate) fn generate_false_alarms(
    sweep: &Sweep,
    limits: &RangeLimits,
    range_resolution: f64,
    pfa: f64,
    antenna: &AntennaConfig,
    dim: Dimension,
    rng: &mut dyn RandomSource,
    out: &mut Vec<FalseAlarm>,
) {
    let cells = resolution_cells(
        limits,
        range_resolution,
        sweep.delta_azimuth,
        antenna.beamwidth_az,
    );
    let count = poisson(rng, pfa * cells);
    for _ in 0..count {
        let (range, azimuth_scan, elevation_scan) = draw_position(sweep, limits, antenna, dim, rng);
        let power = -pfa.ln() + exponential(rng, 1.0);
        out.push(FalseAlarm {
            range,
            azimuth_scan,
            elevation_scan,
            measured_snr: power - 1.0,
        });
    }
}

/// Generates radius-mode false alarms:
/// `count = poisson(false_alarms_per_scan × |delta| / 2π)`, same position
/// draws, `measured_snr = 0.0`, no power draw.
pub(crate) fn generate_radius_false_alarms(
    sweep: &Sweep,
    limits: &RangeLimits,
    false_alarms_per_scan: f64,
    antenna: &AntennaConfig,
    dim: Dimension,
    rng: &mut dyn RandomSource,
    out: &mut Vec<FalseAlarm>,
) {
    let count = poisson(rng, false_alarms_per_scan * sweep.delta_azimuth.abs() / TAU);
    for _ in 0..count {
        let (range, azimuth_scan, elevation_scan) = draw_position(sweep, limits, antenna, dim, rng);
        out.push(FalseAlarm {
            range,
            azimuth_scan,
            elevation_scan,
            measured_snr: 0.0,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::angle::wrap_pi;
    use crate::ids::SensorId;
    use crate::random::{ChaChaStream, Seed, StreamId};
    use approx::assert_relative_eq;

    const LIMITS: RangeLimits = RangeLimits {
        min_range: 0.0,
        instrumented_range: 10_000.0,
    };

    fn sweep(start: f64, delta: f64) -> Sweep {
        Sweep {
            start_azimuth: start,
            delta_azimuth: delta,
            elevation: 0.2,
            completed: false,
        }
    }

    fn rng() -> ChaChaStream {
        ChaChaStream::new(Seed(11), StreamId::Sensor(SensorId(1)))
    }

    #[test]
    fn cells_formula() {
        let l = RangeLimits {
            min_range: 1_000.0,
            instrumented_range: 11_000.0,
        };
        assert_relative_eq!(resolution_cells(&l, 10.0, 0.0, 0.05), 1_000.0);
        assert_relative_eq!(resolution_cells(&l, 10.0, 0.1, 0.05), 2_000.0);
        assert_relative_eq!(resolution_cells(&l, 10.0, -0.1, 0.05), 2_000.0);
    }

    #[test]
    fn fa_rate_matches_pfa() {
        let antenna = AntennaConfig {
            beamwidth_az: 0.05,
            ..AntennaConfig::default()
        };
        let s = sweep(0.0, 0.01);
        let mut r = rng();
        let mut out = Vec::new();
        let n = 2_000;
        for _ in 0..n {
            generate_false_alarms(
                &s,
                &LIMITS,
                1.0,
                1e-3,
                &antenna,
                Dimension::TwoD,
                &mut r,
                &mut out,
            );
        }
        let mean = out.len() as f64 / n as f64;
        assert!((mean - 10.0).abs() < 0.3, "mean {mean}");
    }

    #[test]
    fn fa_within_sweep() {
        let antenna = AntennaConfig::default();
        let bw = antenna.beamwidth_az;
        let mut r = rng();
        for (start, delta) in [(6.2, 0.5), (1.0, -0.5), (0.0, 3.0)] {
            let s = sweep(start, delta);
            let mut out = Vec::new();
            for _ in 0..20 {
                generate_false_alarms(
                    &s,
                    &LIMITS,
                    1.0,
                    1e-3,
                    &antenna,
                    Dimension::ThreeD,
                    &mut r,
                    &mut out,
                );
            }
            assert!(!out.is_empty());
            for fa in &out {
                let off = wrap_pi(fa.azimuth_scan - start);
                assert!(off >= delta.min(0.0) - bw / 2.0 - 1e-12);
                assert!(off <= delta.max(0.0) + bw / 2.0 + 1e-12);
                assert!((0.0..TAU).contains(&fa.azimuth_scan));
                assert!((fa.elevation_scan - 0.2).abs() <= antenna.beamwidth_el / 2.0 + 1e-12);
                assert!((0.0..=10_000.0).contains(&fa.range));
            }
        }
    }

    #[test]
    fn fa_snr_above_threshold() {
        let antenna = AntennaConfig::default();
        let mut r = rng();
        let mut out = Vec::new();
        for _ in 0..50 {
            generate_false_alarms(
                &sweep(0.0, 0.5),
                &LIMITS,
                1.0,
                1e-3,
                &antenna,
                Dimension::TwoD,
                &mut r,
                &mut out,
            );
        }
        assert!(!out.is_empty());
        let floor = -(1e-3f64).ln() - 1.0;
        assert!(out.iter().all(|fa| fa.measured_snr >= floor));
        assert!(out.iter().all(|fa| fa.elevation_scan == 0.0));
    }

    #[test]
    fn radius_mode_count_and_snr() {
        let antenna = AntennaConfig::default();
        let mut r = rng();
        let mut out = Vec::new();
        let n = 2_000;
        for _ in 0..n {
            generate_radius_false_alarms(
                &sweep(0.0, TAU / 4.0),
                &LIMITS,
                8.0,
                &antenna,
                Dimension::TwoD,
                &mut r,
                &mut out,
            );
        }
        let mean = out.len() as f64 / n as f64;
        assert!((mean - 2.0).abs() < 0.15, "mean {mean}");
        assert!(out.iter().all(|fa| fa.measured_snr == 0.0));
    }
}
