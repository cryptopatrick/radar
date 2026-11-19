//! One 100 ms update of a `surveillance_2d` radar against 1 000 targets
//! spread uniformly within 90 km (spec §14.12; target ≥ 60 updates/s).
//!
//! The scenario restarts every 60 s of simulated time so the targets stay
//! within range; only the steps are timed.
use criterion::{Criterion, criterion_group, criterion_main};
use radar::prelude::*;
use std::hint::black_box;
use std::time::{Duration, Instant};

const RADAR: SensorId = SensorId(1);
const OWN: PlatformId = PlatformId(1);

fn uniform(rng: &mut ChaChaStream) -> f64 {
    (rng.next_u64() >> 11) as f64 / (1u64 << 53) as f64
}

fn world() -> WorldSnapshot {
    let mut rng = ChaChaStream::new(Seed(1), StreamId::Scenario);
    let mut w = WorldSnapshot::new(SimTime::ZERO);
    w.platforms.push(PlatformState {
        id: OWN,
        pose: Pose3::IDENTITY,
        velocity: Vector3::ZERO,
    });
    for i in 0..1_000 {
        let r = 90_000.0 * uniform(&mut rng).sqrt();
        let b = std::f64::consts::TAU * uniform(&mut rng);
        let course = std::f64::consts::TAU * uniform(&mut rng);
        w.targets.push(TargetState {
            id: TargetId(i),
            pose: Pose3::from_position_heading(Vector3::new(r * b.sin(), r * b.cos(), 0.0), course),
            velocity: Vector3::new(course.sin(), course.cos(), 0.0) * 150.0,
            extent_radius: 15.0,
            signature: RadarSignature {
                rcs_m2: 5.0,
                swerling: Swerling::One,
            },
            transponder: None,
            platform: None,
        });
    }
    w
}

fn step_100ms(c: &mut Criterion) {
    let world = world();
    let mut sim = Simulation::new(SimulationConfig::default().seed(Seed(1)));
    sim.add_radar(
        RADAR,
        OWN,
        Pose3::IDENTITY,
        RadarConfig::surveillance_2d().unwrap(),
    )
    .unwrap();
    sim.update_world(world.clone()).unwrap();
    let restart = SimTime::ZERO + SimDuration::from_secs(60);
    let mut out = Vec::new();
    c.bench_function("surveillance_2d step(100 ms), 1000 targets", |b| {
        b.iter_custom(|iters| {
            let mut timed = Duration::ZERO;
            for _ in 0..iters {
                if sim.now() >= restart {
                    sim.reset();
                    sim.update_world(world.clone()).unwrap();
                }
                let start = Instant::now();
                sim.step_open(black_box(SimDuration::from_millis(100)))
                    .unwrap();
                sim.drain(RADAR, &mut out).unwrap();
                timed += start.elapsed();
                out.clear();
            }
            timed
        })
    });
}

criterion_group!(benches, step_100ms);
criterion_main!(benches);
