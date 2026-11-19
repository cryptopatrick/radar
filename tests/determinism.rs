//! Acceptance (spec §14.7): the same seed and inputs reproduce the same
//! observation stream exactly; a different seed does not; `reset` followed
//! by the same inputs replays the first run.

use radar::prelude::*;

const S1: SensorId = SensorId(1);
const S2: SensorId = SensorId(2);
const OWN: PlatformId = PlatformId(1);
const STEP: SimDuration = SimDuration::from_millis(100);

fn world(time: SimTime) -> WorldSnapshot {
    let mut w = WorldSnapshot::new(time);
    w.platforms = vec![PlatformState {
        id: OWN,
        pose: Pose3::IDENTITY,
        velocity: Vector3::new(0.0, 20.0, 0.0),
    }];
    let t = time.as_secs_f64();
    w.targets = (1..=6u32)
        .map(|i| {
            let b = f64::from(i);
            TargetState {
                id: TargetId(u64::from(i)),
                pose: Pose3::from_position_heading(
                    Vector3::new(b.sin(), b.cos(), 0.0) * (8_000.0 * b)
                        + Vector3::new(40.0 * t, 0.0, 0.0),
                    0.0,
                ),
                velocity: Vector3::new(40.0, 0.0, 0.0),
                extent_radius: 10.0,
                signature: RadarSignature {
                    rcs_m2: b,
                    swerling: Swerling::One,
                },
                transponder: (i % 2 == 0).then_some(TransponderCode(i)),
                platform: None,
            }
        })
        .collect();
    w
}

fn new_sim(seed: Seed) -> Simulation {
    let mut sim = Simulation::new(SimulationConfig::default().seed(seed));
    let mut cfg = RadarConfig::surveillance_2d().unwrap();
    if let DetectionConfig::RangeEquation(c) = &mut cfg.detection {
        c.pfa = 1e-4;
    }
    cfg.identification = Some(IdentificationConfig {
        reply_probability: 0.8,
        max_range: 60_000.0,
    });
    sim.add_radar(S1, OWN, Pose3::IDENTITY, cfg).unwrap();
    sim.add_radar(
        S2,
        OWN,
        Pose3::IDENTITY,
        RadarConfig::surveillance_3d().unwrap(),
    )
    .unwrap();
    sim
}

/// 40 s, with a fresh world snapshot every second.
fn replay(sim: &mut Simulation) -> Vec<Observation> {
    let mut out = Vec::new();
    for i in 0..400u64 {
        if i % 10 == 0 {
            sim.update_world(world(SimTime::from_nanos(STEP.as_nanos() * i)))
                .unwrap();
        }
        sim.step_open(STEP).unwrap();
        sim.drain(S1, &mut out).unwrap();
        sim.drain(S2, &mut out).unwrap();
    }
    out
}

#[test]
fn same_seed_same_stream() {
    let first = replay(&mut new_sim(Seed(1)));
    assert!(
        first
            .iter()
            .filter(|o| matches!(o, Observation::Measurement(_)))
            .count()
            > 100
    );
    let second = replay(&mut new_sim(Seed(1)));
    assert_eq!(first, second);
}

#[test]
fn different_seed_different_stream() {
    let a = replay(&mut new_sim(Seed(1)));
    let b = replay(&mut new_sim(Seed(2)));
    assert_ne!(a, b);
}

#[test]
fn reset_replays_the_first_run() {
    let mut sim = new_sim(Seed(1));
    let first = replay(&mut sim);
    sim.reset();
    assert_eq!(sim.now(), SimTime::ZERO);
    let again = replay(&mut sim);
    assert_eq!(first, again);
}
