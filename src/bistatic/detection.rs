//! Target detection for a passive receiver.

use super::{BistaticGeometry, Illuminator, PassiveReceiver};
use crate::error::RadarError;
use crate::radar::calibration::{analytic_pd, required_snr_db};
use crate::random::RandomSource;
use crate::random::sample::bernoulli;
use crate::units::{db_to_linear, linear_to_db};
use crate::world::{RadarSignature, Swerling};

/// How a passive receiver decides a target is present.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum DetectionThreshold {
    /// A fixed threshold on the mean SNR after integration: the target is
    /// detected exactly when its mean SNR reaches `snr_db`. This is the
    /// link-budget convention of Gomez-del-Hoyo et al. (IRS 2022), which
    /// uses 16.5 dB ([`DetectionThreshold::PAPER`]).
    MeanSnr {
        /// Threshold, dB.
        snr_db: f64,
    },
    /// A square-law detector set for false-alarm probability `pfa` per
    /// cell. Detection is random, with probability [`analytic_pd`] for the
    /// target's fluctuation model.
    FalseAlarm {
        /// Probability of false alarm per resolution cell, in `(0, 1)`.
        pfa: f64,
    },
}

impl DetectionThreshold {
    /// The paper's 16.5 dB mean-SNR threshold.
    pub const PAPER: DetectionThreshold = DetectionThreshold::MeanSnr { snr_db: 16.5 };

    /// Checks the threshold is finite, and `pfa` is in `(0, 1)`.
    pub fn validate(&self) -> Result<(), RadarError> {
        match *self {
            DetectionThreshold::MeanSnr { snr_db } if !snr_db.is_finite() => {
                Err(RadarError::InvalidConfig {
                    field: "snr_db",
                    reason: "must be finite".into(),
                })
            }
            DetectionThreshold::FalseAlarm { pfa } if !(pfa > 0.0 && pfa < 1.0) => {
                Err(RadarError::InvalidConfig {
                    field: "pfa",
                    reason: "must be in (0, 1)".into(),
                })
            }
            _ => Ok(()),
        }
    }

    /// Probability of detecting a target of linear mean SNR `snr` and
    /// fluctuation model `swerling`. `MeanSnr` gives 1 at or above the
    /// threshold and 0 below it (and ignores `swerling`).
    pub fn probability(&self, snr: f64, swerling: Swerling) -> f64 {
        match *self {
            DetectionThreshold::MeanSnr { snr_db } => {
                if snr >= db_to_linear(snr_db) {
                    1.0
                } else {
                    0.0
                }
            }
            DetectionThreshold::FalseAlarm { pfa } => analytic_pd(snr, pfa, swerling),
        }
    }

    /// Mean SNR, dB, needed to detect with probability `pd`. `MeanSnr`
    /// returns its threshold for any `pd` in `(0, 1)`.
    ///
    /// Returns [`RadarError::InvalidConfig`] for `pd` outside `(0, 1)`, or
    /// as [`required_snr_db`] does.
    pub fn required_snr_db(&self, pd: f64, swerling: Swerling) -> Result<f64, RadarError> {
        match *self {
            DetectionThreshold::MeanSnr { snr_db } => {
                if pd > 0.0 && pd < 1.0 {
                    Ok(snr_db)
                } else {
                    Err(RadarError::InvalidConfig {
                        field: "pd",
                        reason: "must be in (0, 1)".into(),
                    })
                }
            }
            DetectionThreshold::FalseAlarm { pfa } => required_snr_db(pd, pfa, swerling),
        }
    }
}

/// The outcome of one passive detection decision.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PassiveDetection {
    /// Mean SNR after integration, dB.
    pub snr_db: f64,
    /// Probability of detection.
    pub probability: f64,
    /// Whether the target was detected.
    pub detected: bool,
}

impl PassiveReceiver {
    /// Decides whether a target with `signature` at `geometry` is detected,
    /// on the receiver's boresight. `MeanSnr` thresholds never draw from
    /// `rng`; `FalseAlarm` thresholds draw one uniform.
    pub fn detect(
        &self,
        illuminator: &Illuminator,
        signature: &RadarSignature,
        geometry: &BistaticGeometry,
        threshold: &DetectionThreshold,
        rng: &mut dyn RandomSource,
    ) -> PassiveDetection {
        let snr = self.snr_at(illuminator, signature.rcs_m2, geometry);
        let probability = threshold.probability(snr, signature.swerling);
        let detected = match threshold {
            DetectionThreshold::MeanSnr { .. } => probability >= 1.0,
            DetectionThreshold::FalseAlarm { .. } => bernoulli(rng, probability),
        };
        PassiveDetection {
            snr_db: linear_to_db(snr),
            probability,
            detected,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Vector3;
    use crate::ids::SensorId;
    use crate::random::{ChaChaStream, Seed, StreamId};
    use approx::assert_relative_eq;

    fn rng() -> ChaChaStream {
        ChaChaStream::new(Seed(7), StreamId::Sensor(SensorId(1)))
    }

    fn truck_at(il: &Illuminator, range: f64) -> BistaticGeometry {
        BistaticGeometry {
            transmitter: Vector3::new(0.0, 0.0, il.altitude_m),
            receiver: Vector3::ZERO,
            target: Vector3::new(0.0, range, 0.0),
        }
    }

    #[test]
    fn paper_threshold_is_a_step() {
        let t = DetectionThreshold::PAPER;
        t.validate().unwrap();
        let at = db_to_linear(16.5);
        assert_eq!(t.probability(at, Swerling::One), 1.0);
        assert_eq!(t.probability(at * 0.999, Swerling::One), 0.0);
        assert_eq!(t.required_snr_db(0.9, Swerling::Zero).unwrap(), 16.5);
        assert!(t.required_snr_db(1.0, Swerling::Zero).is_err());
    }

    #[test]
    fn paper_receiver_detects_a_truck_to_about_18_km() {
        // With the paper's losses (see budget tests), a 40 m² truck is
        // detected at 17 km and missed at 19 km.
        let il = Illuminator {
            frequency_hz: crate::units::SPEED_OF_LIGHT / 0.03,
            ..Illuminator::starlink()
        };
        let rx = PassiveReceiver {
            losses_db: 4.74,
            ..PassiveReceiver::default()
        };
        let truck = RadarSignature {
            rcs_m2: 40.0,
            swerling: Swerling::One,
        };
        let mut r = rng();
        let near = rx.detect(
            &il,
            &truck,
            &truck_at(&il, 17e3),
            &DetectionThreshold::PAPER,
            &mut r,
        );
        let far = rx.detect(
            &il,
            &truck,
            &truck_at(&il, 19e3),
            &DetectionThreshold::PAPER,
            &mut r,
        );
        assert!(near.detected && near.snr_db > 16.5);
        assert!(!far.detected && far.snr_db < 16.5);
    }

    #[test]
    fn false_alarm_threshold_detects_at_the_analytic_rate() {
        let t = DetectionThreshold::FalseAlarm { pfa: 1e-6 };
        t.validate().unwrap();
        let il = Illuminator::starlink();
        let rx = PassiveReceiver::default();
        let geometry = truck_at(&il, 20e3);
        // Pick the RCS that puts the target at Pd = 0.7 for Swerling 1.
        let need = db_to_linear(t.required_snr_db(0.7, Swerling::One).unwrap());
        let rcs = need / rx.snr_at(&il, 1.0, &geometry);
        let target = RadarSignature {
            rcs_m2: rcs,
            swerling: Swerling::One,
        };
        let mut r = rng();
        let n = 20_000;
        let mut hits = 0;
        for _ in 0..n {
            let d = rx.detect(&il, &target, &geometry, &t, &mut r);
            assert_relative_eq!(d.probability, 0.7, epsilon = 1e-5);
            hits += usize::from(d.detected);
        }
        let rate = hits as f64 / n as f64;
        // 5σ of a binomial(20 000, 0.7) is about 0.016.
        assert!((rate - 0.7).abs() < 0.016, "{rate}");
    }

    #[test]
    fn fluctuating_targets_need_more_snr_for_high_pd() {
        let t = DetectionThreshold::FalseAlarm { pfa: 1e-6 };
        let steady = t.required_snr_db(0.9, Swerling::Zero).unwrap();
        let s3 = t.required_snr_db(0.9, Swerling::Three).unwrap();
        let s1 = t.required_snr_db(0.9, Swerling::One).unwrap();
        assert!(steady < s3 && s3 < s1, "{steady} {s3} {s1}");
    }

    #[test]
    fn validate_rejects_bad_thresholds() {
        assert!(
            DetectionThreshold::MeanSnr { snr_db: f64::NAN }
                .validate()
                .is_err()
        );
        assert!(
            DetectionThreshold::FalseAlarm { pfa: 0.0 }
                .validate()
                .is_err()
        );
        assert!(
            DetectionThreshold::FalseAlarm { pfa: 1.0 }
                .validate()
                .is_err()
        );
    }
}
