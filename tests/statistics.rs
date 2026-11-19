//! Acceptance (spec §14.9): Monte-Carlo detection rates match the analytic
//! probability of detection, and the false-alarm rate matches Pfa.
//!
//! Detection is driven through the public [`DetectionModel`] trait on a
//! [`RangeEquationDetector`] with unit range, RCS and gain, so `k` is the
//! mean SNR.

use radar::prelude::*;

const N: u32 = 20_000;
const PFA: f64 = 1e-3;

/// A look whose mean SNR is `snr` (linear), and its detector.
fn look(snr: f64, swerling: Swerling) -> (DetectionContext, RangeEquationDetector) {
    let ctx = DetectionContext {
        range: 1.0,
        rcs_m2: 1.0,
        swerling,
        pattern_gain: 1.0,
        jamming_to_noise: 0.0,
    };
    (ctx, RangeEquationDetector { k: snr, pfa: PFA })
}

/// Fraction of `N` looks detected.
fn empirical(ctx: &DetectionContext, detector: &RangeEquationDetector, seed: u64) -> f64 {
    let mut rng = ChaChaStream::new(Seed(seed), StreamId::Scenario);
    let hits = (0..N)
        .filter(|_| detector.detect(ctx, &mut rng).detected)
        .count();
    hits as f64 / f64::from(N)
}

fn sigma(p: f64) -> f64 {
    (p * (1.0 - p) / f64::from(N)).sqrt()
}

#[test]
fn sampled_pd_matches_analytic() {
    let mut seed = 0;
    for swerling in [Swerling::Zero, Swerling::One, Swerling::Three] {
        for snr_db in [6.0f64, 10.0, 13.0] {
            let snr = 10f64.powf(snr_db / 10.0);
            let (ctx, detector) = look(snr, swerling);
            let expected = analytic_pd(snr, PFA, swerling);
            seed += 1;
            let got = empirical(&ctx, &detector, seed);
            // North's approximation for Swerling 0 is itself off by up to
            // about 0.01.
            let slack = if swerling == Swerling::Zero {
                0.01
            } else {
                0.0
            };
            let tolerance = 4.0 * sigma(expected) + slack;
            assert!(
                (got - expected).abs() <= tolerance,
                "{swerling:?} at {snr_db} dB: empirical {got}, analytic {expected}, tolerance {tolerance}"
            );
        }
    }
}

#[test]
fn false_alarm_rate_matches_pfa() {
    // No target: only noise crosses the threshold.
    let (ctx, detector) = look(0.0, Swerling::Zero);
    let got = empirical(&ctx, &detector, 99);
    assert!(
        (got - PFA).abs() <= 4.0 * sigma(PFA),
        "false-alarm rate {got}, expected {PFA}"
    );
}
