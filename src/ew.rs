//! Electronic warfare (feature `ew`): noise-jammer interference and the
//! accuracy of jamming strobes.

use crate::radar::calibration::Calibration;
use crate::radar::config::{DetectionConfig, RadarConfig, RangeEquationConfig};
use crate::units::db_to_linear;
use crate::world::{EmitterKind, EmitterState};

/// Linear jamming-to-noise ratio that `emitter` at `range` (m) produces in
/// the radar's receiver, with normalised one-way pattern gain `gain` toward
/// the jammer. Line of sight is the caller's concern.
///
/// - 0 if the radar has no `receiver_band`.
/// - A [`Calibration::Physical`] range-equation radar:
///   `erp·gain·jnr_constant/R²·overlap`.
/// - Otherwise, with a `jamming_reference`:
///   `db_to_linear(jnr_db)·(erp/erp_ref)·(range_ref/R)²·gain·overlap`.
/// - Otherwise 0 (no receiver model to express J/N in).
///
/// `overlap` is the fraction of the jammer band inside the receiver band.
/// Non-finite or non-positive inputs and results give 0.
pub(crate) fn jammer_jnr(cfg: &RadarConfig, emitter: &EmitterState, range: f64, gain: f64) -> f64 {
    let Some(receiver) = &cfg.receiver_band else {
        return 0.0;
    };
    match emitter.kind {
        EmitterKind::NoiseJammer => {}
    }
    if !(range.is_finite() && range > 0.0) {
        return 0.0;
    }
    let overlap = emitter.band.overlap_fraction(receiver);
    let erp = emitter.erp_w;
    let base = match (&cfg.detection, &cfg.jamming_reference) {
        (
            DetectionConfig::RangeEquation(RangeEquationConfig {
                calibration: c @ Calibration::Physical { .. },
                ..
            }),
            _,
        ) => c.jnr_constant().unwrap_or(0.0) * erp / (range * range),
        (_, Some(j)) => {
            let ratio = j.range_m / range;
            db_to_linear(j.jnr_db) * (erp / j.erp_w) * ratio * ratio
        }
        (_, None) => 0.0,
    };
    let jnr = base * gain * overlap;
    if jnr.is_finite() && jnr > 0.0 {
        jnr
    } else {
        0.0
    }
}

/// Standard deviation (radians) of a strobe angle for a beam of width
/// `beamwidth` at linear J/N `jnr`: `beamwidth/(1.6·√(2·jnr))`.
pub(crate) fn strobe_sigma(beamwidth: f64, jnr: f64) -> f64 {
    beamwidth / (1.6 * (2.0 * jnr).sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Vector3;
    use crate::ids::EmitterId;
    use crate::radar::calibration::JammingReference;
    use crate::world::Band;
    use approx::assert_relative_eq;

    const RX: Band = Band {
        centre_hz: 10e9,
        bandwidth_hz: 10e6,
    };

    fn jammer(erp_w: f64, band: Band) -> EmitterState {
        EmitterState {
            id: EmitterId(1),
            platform: None,
            position: Vector3::ZERO,
            band,
            erp_w,
            kind: EmitterKind::NoiseJammer,
        }
    }

    fn reference_cfg(receiver_band: Option<Band>) -> RadarConfig {
        RadarConfig::builder()
            .receiver_band(receiver_band)
            .jamming_reference(Some(JammingReference {
                erp_w: 2.0,
                range_m: 1000.0,
                jnr_db: 20.0,
            }))
            .build()
            .unwrap()
    }

    #[test]
    fn reference_formula() {
        let cfg = reference_cfg(Some(RX));
        // 100 · (4/2) · (1000/2000)² · 0.5 · 1
        let j = jammer_jnr(&cfg, &jammer(4.0, RX), 2000.0, 0.5);
        assert_relative_eq!(j, 25.0, max_relative = 1e-12);
        // Half the jammer band overlaps the receiver.
        let wide = Band {
            centre_hz: 10e9,
            bandwidth_hz: 20e6,
        };
        let j = jammer_jnr(&cfg, &jammer(4.0, wide), 2000.0, 0.5);
        assert_relative_eq!(j, 12.5, max_relative = 1e-12);
    }

    #[test]
    fn physical_formula() {
        let calibration = Calibration::Physical {
            peak_power_w: 1000.0,
            gain_db: 30.0,
            frequency_hz: 10e9,
            noise_figure_db: 3.0,
            bandwidth_hz: 1e6,
            losses_db: 0.0,
        };
        let cfg = RadarConfig::builder()
            .detection(DetectionConfig::RangeEquation(RangeEquationConfig {
                calibration,
                pfa: 1e-6,
                range_resolution: 100.0,
            }))
            .receiver_band(Some(RX))
            .build()
            .unwrap();
        let k = calibration.jnr_constant().unwrap();
        let j = jammer_jnr(&cfg, &jammer(10.0, RX), 5000.0, 0.25);
        assert_relative_eq!(j, 10.0 * 0.25 * k / 25e6, max_relative = 1e-12);
    }

    #[test]
    fn no_band_or_bad_input_gives_zero() {
        assert_eq!(
            jammer_jnr(&reference_cfg(None), &jammer(4.0, RX), 2000.0, 1.0),
            0.0
        );
        let cfg = reference_cfg(Some(RX));
        for (erp, range) in [
            (f64::NAN, 1000.0),
            (f64::INFINITY, 1000.0),
            (1.0, f64::NAN),
            (1.0, 0.0),
            (1.0, -5.0),
            (1.0, f64::INFINITY),
        ] {
            assert_eq!(jammer_jnr(&cfg, &jammer(erp, RX), range, 1.0), 0.0);
        }
        // Radius detection without a jamming reference: no receiver model.
        let cfg = RadarConfig::builder()
            .receiver_band(Some(RX))
            .build()
            .unwrap();
        assert_eq!(jammer_jnr(&cfg, &jammer(1.0, RX), 1000.0, 1.0), 0.0);
    }

    #[test]
    fn strobe_sigma_value() {
        assert_relative_eq!(strobe_sigma(0.032, 50.0), 0.002, max_relative = 1e-12);
    }
}
