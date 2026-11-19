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

#[test]
fn pipeline_false_alarm_rate_matches_pfa_times_cells() {
    // A surveillance radar in an empty world: every measurement is a false
    // alarm. Per update the count is Poisson(pfa · cells), so the mean over
    // `UPDATES` updates has standard deviation √(pfa · cells / UPDATES).
    const UPDATES: u32 = 5_000;
    let mut cfg = RadarConfig::surveillance_2d().unwrap();
    let pfa = 1e-4;
    let resolution = match &mut cfg.detection {
        DetectionConfig::RangeEquation(c) => {
            c.pfa = pfa;
            c.range_resolution
        }
        _ => unreachable!("surveillance_2d uses the range equation"),
    };
    #[cfg(feature = "tracking")]
    {
        cfg.tracker = TrackerConfig::None;
    }
    let ScanConfig::Circular { rate, .. } = cfg.scan else {
        unreachable!("surveillance_2d scans circularly")
    };
    let swept = rate * cfg.update_period.as_secs_f64();
    let cells = resolution_cells(&cfg.limits, resolution, swept, cfg.antenna.beamwidth_az);
    let expected = pfa * cells;

    let (radar, own) = (SensorId(1), PlatformId(1));
    let mut sim = Simulation::new(SimulationConfig::default().seed(Seed(5)));
    sim.add_radar(radar, own, Pose3::IDENTITY, cfg).unwrap();
    let mut w = WorldSnapshot::new(SimTime::ZERO);
    w.platforms.push(PlatformState {
        id: own,
        pose: Pose3::IDENTITY,
        velocity: Vector3::ZERO,
    });
    sim.update_world(w).unwrap();
    let mut out = Vec::new();
    let mut alarms = 0usize;
    for _ in 0..UPDATES {
        sim.step_open(SimDuration::from_millis(100)).unwrap();
        sim.drain(radar, &mut out).unwrap();
        alarms += out
            .iter()
            .filter(|o| matches!(o, Observation::Measurement(_)))
            .count();
        out.clear();
    }
    let mean = alarms as f64 / f64::from(UPDATES);
    let sd = (expected / f64::from(UPDATES)).sqrt();
    assert!(
        (mean - expected).abs() <= 4.0 * sd,
        "mean {mean} false alarms per update, expected {expected} ± {}",
        4.0 * sd
    );
}
