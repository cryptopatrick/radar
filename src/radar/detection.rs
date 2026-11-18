//! Detection models.

use crate::radar::calibration::analytic_pd;
use crate::random::RandomSource;
use crate::random::sample::{bernoulli, exponential, gamma2, standard_normal};
use crate::world::Swerling;
use std::f64::consts::FRAC_1_SQRT_2;

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

/// Cookie-cutter detection: a target within `range` is detected with fixed
/// probability `pd`; beyond it, never. Always draws exactly once.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RadiusDetector {
    /// Maximum detection range, m.
    pub range: f64,
    /// Probability of detection inside `range`.
    pub pd: f64,
}

impl DetectionModel for RadiusDetector {
    fn detect(&self, ctx: &DetectionContext, rng: &mut dyn RandomSource) -> DetectionResult {
        let in_range = ctx.range <= self.range;
        // Draw unconditionally so the draw count does not depend on geometry.
        let hit = bernoulli(rng, self.pd);
        DetectionResult {
            detected: in_range && hit,
            probability: if in_range { self.pd } else { 0.0 },
            mean_snr: None,
            measured_snr: None,
        }
    }
}

/// Range-equation detection with a sampled square-law envelope.
///
/// `mean_snr = k·σ·g²/R⁴/(1+jnr)`. A fluctuation factor is drawn per
/// Swerling case (0: none, 1: `exponential(1)`, 3: `gamma2(0.5)`), then two
/// unit-power complex noise components; the target is detected when the
/// envelope power exceeds `-ln(pfa)`. The draw count never depends on the
/// outcome.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RangeEquationDetector {
    /// Range-equation constant `K` (see [`crate::radar::Calibration::snr_constant`]).
    pub k: f64,
    /// Probability of false alarm per resolution cell.
    pub pfa: f64,
}

impl DetectionModel for RangeEquationDetector {
    fn detect(&self, ctx: &DetectionContext, rng: &mut dyn RandomSource) -> DetectionResult {
        let range = if ctx.range.is_finite() {
            ctx.range.max(1e-3)
        } else if ctx.range == f64::INFINITY {
            ctx.range
        } else {
            1e-3
        };
        let g = ctx.pattern_gain;
        let mean_snr = self.k * ctx.rcs_m2 * g * g / range.powi(4) / (1.0 + ctx.jamming_to_noise);
        let f = match ctx.swerling {
            Swerling::Zero => 1.0,
            Swerling::One => exponential(rng, 1.0),
            Swerling::Three => gamma2(rng, 0.5),
        };
        let nr = standard_normal(rng) * FRAC_1_SQRT_2;
        let ni = standard_normal(rng) * FRAC_1_SQRT_2;
        let a = (mean_snr * f).sqrt() + nr;
        let power = a * a + ni * ni;
        DetectionResult {
            detected: power > -self.pfa.ln(),
            probability: analytic_pd(mean_snr, self.pfa, ctx.swerling),
            mean_snr: Some(mean_snr),
            measured_snr: Some((power - 1.0).max(1e-3)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::SensorId;
    use crate::random::{ChaChaStream, Seed, StreamId};
    use approx::assert_relative_eq;

    fn stream() -> ChaChaStream {
        ChaChaStream::new(Seed(42), StreamId::Sensor(SensorId(1)))
    }

    fn ctx(range: f64, rcs_m2: f64, swerling: Swerling) -> DetectionContext {
        DetectionContext {
            range,
            rcs_m2,
            swerling,
            pattern_gain: 1.0,
            jamming_to_noise: 0.0,
        }
    }

    #[test]
    fn radius_detector() {
        let det = RadiusDetector {
            range: 1_000.0,
            pd: 1.0,
        };
        let mut rng = stream();
        for _ in 0..100 {
            let r = det.detect(&ctx(999.0, 1.0, Swerling::Zero), &mut rng);
            assert!(r.detected);
            assert_eq!(r.probability, 1.0);
            assert_eq!(r.mean_snr, None);
            assert_eq!(r.measured_snr, None);
            let r = det.detect(&ctx(1_001.0, 1.0, Swerling::Zero), &mut rng);
            assert!(!r.detected);
            assert_eq!(r.probability, 0.0);
        }
    }

    #[test]
    fn rcs_and_range_monotone() {
        let r0: f64 = 10_000.0;
        let det = RangeEquationDetector {
            k: 10.0 * r0.powi(4),
            pfa: 1e-6,
        };
        let rate = |range: f64, rcs: f64| {
            let mut rng = stream();
            (0..5_000)
                .filter(|_| {
                    det.detect(&ctx(range, rcs, Swerling::One), &mut rng)
                        .detected
                })
                .count()
        };
        assert!(rate(r0, 1.0) > rate(2.0 * r0, 1.0));
        assert!(rate(r0, 10.0) > rate(r0, 1.0));
    }

    #[test]
    fn draw_count_constant() {
        let det = RangeEquationDetector { k: 1e20, pfa: 1e-6 };
        for case in [Swerling::Zero, Swerling::One, Swerling::Three] {
            let mut a = stream();
            let mut b = stream();
            let near = det.detect(&ctx(1_000.0, 1.0, case), &mut a);
            let far = det.detect(&ctx(1e9, 1.0, case), &mut b);
            assert!(near.detected, "{case:?}");
            assert!(!far.detected, "{case:?}");
            assert_eq!(a.next_u64(), b.next_u64(), "{case:?}");
        }
    }

    #[test]
    fn jamming_reduces_snr() {
        let det = RangeEquationDetector { k: 1e18, pfa: 1e-6 };
        let clean = det
            .detect(&ctx(10_000.0, 1.0, Swerling::One), &mut stream())
            .mean_snr
            .unwrap();
        let jammed_ctx = DetectionContext {
            jamming_to_noise: 10.0,
            ..ctx(10_000.0, 1.0, Swerling::One)
        };
        let jammed = det.detect(&jammed_ctx, &mut stream()).mean_snr.unwrap();
        assert_relative_eq!(jammed, clean / 11.0, max_relative = 1e-12);
    }

    #[test]
    fn zero_range_is_safe() {
        let det = RangeEquationDetector { k: 1e18, pfa: 1e-6 };
        for range in [0.0, -5.0, f64::NAN] {
            let r = det.detect(&ctx(range, 1.0, Swerling::Zero), &mut stream());
            assert!(r.mean_snr.unwrap().is_finite());
            assert!(r.probability.is_finite());
        }
    }
}
