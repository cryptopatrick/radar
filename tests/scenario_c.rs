//! Acceptance scenario C (spec §14.3): sensors are independent. Adding or
//! removing a sensor never changes another sensor's observations.

use radar::prelude::*;

const S1: SensorId = SensorId(1);
const S2: SensorId = SensorId(2);
/// Lower than both, so it runs first within every step.
const S3: SensorId = SensorId(0);
const P1: PlatformId = PlatformId(1);
const P2: PlatformId = PlatformId(2);
const STEP: SimDuration = SimDuration::from_millis(100);
/// 30 s.
const STEPS: usize = 300;

/// What happens to the extra sensors in a run.
#[derive(Clone, Copy, PartialEq)]
enum Variant {
    Baseline,
    ThirdAdded,
    SecondRemovedAt15s,
}

fn radar(period_ms: u64) -> RadarConfig {
    let mut cfg = RadarConfig::surveillance_2d().unwrap();
    cfg.update_period = SimDuration::from_millis(period_ms);
    // Frequent false alarms make the streams rich in random draws.
    if let DetectionConfig::RangeEquation(c) = &mut cfg.detection {
        c.pfa = 1e-4;
    }
    cfg
}

fn world() -> WorldSnapshot {
    let mut w = WorldSnapshot::new(SimTime::ZERO);
    w.platforms = vec![
        PlatformState {
            id: P1,
            pose: Pose3::IDENTITY,
            velocity: Vector3::ZERO,
        },
        PlatformState {
            id: P2,
            pose: Pose3::from_position_heading(Vector3::new(30_000.0, 0.0, 0.0), 0.0),
            velocity: Vector3::new(0.0, 100.0, 0.0),
        },
    ];
    w.targets = (1..=4u32)
        .map(|i| {
            let b = f64::from(i) * 1.3;
            TargetState {
                id: TargetId(u64::from(i)),
                pose: Pose3::from_position_heading(
                    Vector3::new(b.sin(), b.cos(), 0.0) * (10_000.0 * f64::from(i)),
                    0.0,
                ),
                velocity: Vector3::new(50.0, -30.0, 0.0),
                extent_radius: 10.0,
                signature: RadarSignature {
                    rcs_m2: 5.0,
                    swerling: Swerling::One,
                },
                transponder: None,
                platform: None,
            }
        })
        .collect();
    w
}

/// Runs 30 s and returns the streams of sensors 1 and 2 (sensor 2's stream
/// ends at its removal).
fn run(variant: Variant) -> (Vec<Observation>, Vec<Observation>) {
    let mut sim = Simulation::new(SimulationConfig::default().seed(Seed(7)));
    sim.add_radar(S1, P1, Pose3::IDENTITY, radar(100)).unwrap();
    sim.add_radar(S2, P2, Pose3::IDENTITY, radar(1000)).unwrap();
    if variant == Variant::ThirdAdded {
        sim.add_radar(S3, P2, Pose3::IDENTITY, radar(100)).unwrap();
    }
    sim.update_world(world()).unwrap();
    let (mut s1, mut s2, mut s3) = (Vec::new(), Vec::new(), Vec::new());
    for i in 1..=STEPS {
        sim.step_open(STEP).unwrap();
        sim.drain(S1, &mut s1).unwrap();
        if variant != Variant::SecondRemovedAt15s || i <= STEPS / 2 {
            sim.drain(S2, &mut s2).unwrap();
        }
        if variant == Variant::ThirdAdded {
            sim.drain(S3, &mut s3).unwrap();
        }
        if variant == Variant::SecondRemovedAt15s && i == STEPS / 2 {
            sim.remove_sensor(S2).unwrap();
        }
    }
    if variant == Variant::ThirdAdded {
        assert!(!s3.is_empty());
    }
    (s1, s2)
}

fn measurements(obs: &[Observation]) -> usize {
    obs.iter()
        .filter(|o| matches!(o, Observation::Measurement(_)))
        .count()
}

#[test]
fn scenario_c_sensors_are_independent() {
    let (s1, s2) = run(Variant::Baseline);
    assert!(measurements(&s1) > 20, "sensor 1 saw {}", measurements(&s1));
    assert!(measurements(&s2) > 5, "sensor 2 saw {}", measurements(&s2));
    assert!(s1.iter().all(|o| match o {
        Observation::Measurement(m) => m.sensor == S1,
        _ => true,
    }));

    let (s1_third, s2_third) = run(Variant::ThirdAdded);
    assert_eq!(s1, s1_third, "adding a sensor changed sensor 1");
    assert_eq!(s2, s2_third, "adding a sensor changed sensor 2");

    let (s1_removed, s2_removed) = run(Variant::SecondRemovedAt15s);
    assert_eq!(s1, s1_removed, "removing sensor 2 changed sensor 1");
    // Sensor 2's stream up to its removal is the baseline's prefix.
    assert!(!s2_removed.is_empty());
    assert_eq!(s2_removed[..], s2[..s2_removed.len()]);
    assert!(s2_removed.len() < s2.len());
}
