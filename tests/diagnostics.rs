//! Truth-annotated diagnostics: linkage to the observation stream and
//! non-interference with it.
#![cfg(feature = "diagnostics")]

use radar::diagnostics::{DiagnosticOutcome, DiagnosticRecord};
use radar::prelude::*;

const RADAR: SensorId = SensorId(1);
const OWN: PlatformId = PlatformId(7);

fn target(id: u64, pos: Vector3) -> TargetState {
    TargetState {
        id: TargetId(id),
        pose: Pose3::from_position_heading(pos, 0.0),
        velocity: Vector3::ZERO,
        extent_radius: 1.0,
        signature: RadarSignature {
            rcs_m2: 1.0,
            swerling: Swerling::Zero,
        },
        transponder: None,
        platform: None,
    }
}

fn sim() -> Simulation {
    let cfg = RadarConfig::builder()
        .detection(DetectionConfig::Radius {
            range: 1000.0,
            pd: 0.7,
            false_alarms_per_scan: 4.0,
        })
        .limits(RangeLimits {
            min_range: 0.0,
            instrumented_range: 1200.0,
        })
        .build()
        .unwrap();
    let mut sim = Simulation::new(SimulationConfig::default());
    sim.add_radar(RADAR, OWN, Pose3::IDENTITY, cfg).unwrap();
    let mut w = WorldSnapshot::new(SimTime::ZERO);
    w.platforms = vec![PlatformState {
        id: OWN,
        pose: Pose3::from_position_heading(Vector3::ZERO, 0.0),
        velocity: Vector3::ZERO,
    }];
    w.targets = vec![
        target(1, Vector3::new(500.0, 0.0, 0.0)),
        target(2, Vector3::new(0.0, 700.0, 0.0)),
        target(3, Vector3::new(-300.0, 0.0, 0.0)),
        target(4, Vector3::new(5000.0, 0.0, 0.0)),
    ];
    sim.update_world(w).unwrap();
    sim
}

fn run(sim: &mut Simulation, drain_diag: bool) -> (Vec<Observation>, Vec<DiagnosticRecord>) {
    let (mut obs, mut diag) = (Vec::new(), Vec::new());
    for _ in 0..30 {
        sim.step(SimDuration::from_millis(100), &OpenSky).unwrap();
        sim.drain(RADAR, &mut obs).unwrap();
        if drain_diag {
            sim.drain_diagnostics(&mut diag);
        }
    }
    (obs, diag)
}

#[test]
fn diagnostics_link_truth() {
    let (obs, diag) = run(&mut sim(), true);
    let ms: Vec<&Measurement> = obs
        .iter()
        .filter_map(|o| match o {
            Observation::Measurement(m) => Some(m),
            _ => None,
        })
        .collect();
    let linked: Vec<&Measurement> = diag.iter().filter_map(|r| r.measurement.as_ref()).collect();
    assert!(!linked.is_empty());
    assert_eq!(linked.len(), ms.len());
    for m in &linked {
        assert!(ms.contains(m), "record measurement missing from stream");
    }
    let mut saw = [false; 6];
    for r in &diag {
        saw[r.outcome as usize] = true;
        match r.outcome {
            DiagnosticOutcome::FalseAlarm => {
                assert!(r.target.is_none() && r.true_range.is_none());
                assert!(r.measurement.is_some());
            }
            DiagnosticOutcome::Detected => {
                assert!(r.target.is_some() && r.measurement.is_some());
                assert_eq!(r.pd, Some(0.7));
            }
            _ => {
                assert!(r.target.is_some() && r.measurement.is_none());
                assert!(r.true_range.is_some());
            }
        }
    }
    // Detected, Missed, OutOfBeam, OutOfRange and FalseAlarm all occur.
    for o in [
        DiagnosticOutcome::Detected,
        DiagnosticOutcome::Missed,
        DiagnosticOutcome::OutOfBeam,
        DiagnosticOutcome::OutOfRange,
        DiagnosticOutcome::FalseAlarm,
    ] {
        assert!(saw[o as usize], "no {o:?} record");
    }
    // Target records appear in target order within each update.
    let first_update: Vec<_> = diag
        .iter()
        .take_while(|r| r.time == diag[0].time)
        .filter_map(|r| r.target)
        .collect();
    assert_eq!(
        first_update,
        vec![TargetId(1), TargetId(2), TargetId(3), TargetId(4)]
    );
}

#[test]
fn diagnostics_do_not_change_outputs() {
    let (a, _) = run(&mut sim(), true);
    let (b, _) = run(&mut sim(), false);
    assert!(!a.is_empty());
    assert_eq!(a, b);
}
