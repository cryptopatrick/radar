//! Pins the full observation stream of a fixed seeded scenario. The same
//! constant must hold with the `diagnostics` feature on or off, which
//! proves that recording truth draws no random numbers.

use radar::prelude::*;

const RADAR: SensorId = SensorId(1);
const OWN: PlatformId = PlatformId(7);

/// Blocks the line of sight to anything north of y = 6000.
struct Wall;
impl Environment for Wall {
    fn line_of_sight_blocked(&self, _from: Vector3, to: Vector3) -> bool {
        to.y > 6000.0
    }
}

fn target(id: u64, pos: Vector3, vel: Vector3) -> TargetState {
    TargetState {
        id: TargetId(id),
        pose: Pose3::from_position_heading(pos, 0.0),
        velocity: vel,
        extent_radius: 1.0,
        signature: RadarSignature {
            rcs_m2: 10.0,
            swerling: Swerling::One,
        },
        transponder: None,
        platform: None,
    }
}

fn stream() -> Vec<Observation> {
    let mut sim = Simulation::new(SimulationConfig::default().seed(Seed(42)));
    sim.add_radar(
        RADAR,
        OWN,
        Pose3::IDENTITY,
        RadarConfig::surveillance_2d().unwrap(),
    )
    .unwrap();
    let mut w = WorldSnapshot::new(SimTime::ZERO);
    w.platforms = vec![PlatformState {
        id: OWN,
        pose: Pose3::from_position_heading(Vector3::ZERO, 0.0),
        velocity: Vector3::ZERO,
    }];
    w.targets = vec![
        target(
            1,
            Vector3::new(1000.0, 0.0, 0.0),
            Vector3::new(0.0, 50.0, 0.0),
        ),
        target(
            2,
            Vector3::new(0.0, -3000.0, 0.0),
            Vector3::new(30.0, 0.0, 0.0),
        ),
        target(3, Vector3::new(-5000.0, 0.0, 0.0), Vector3::ZERO),
        target(4, Vector3::new(0.0, 8000.0, 0.0), Vector3::ZERO),
    ];
    sim.update_world(w).unwrap();
    let mut obs = Vec::new();
    for _ in 0..200 {
        sim.step(SimDuration::from_millis(100), &Wall).unwrap();
        sim.drain(RADAR, &mut obs).unwrap();
        #[cfg(feature = "diagnostics")]
        sim.drain_diagnostics(&mut Vec::new());
    }
    obs
}

/// FNV-1a over the Debug text of every observation.
fn fnv1a(obs: &[Observation]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for o in obs {
        for b in format!("{o:?}\n").bytes() {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x100000001b3);
        }
    }
    h
}

#[test]
fn observation_stream_is_pinned() {
    let obs = stream();
    assert!(obs.iter().any(|o| matches!(o, Observation::Measurement(_))));
    let h = fnv1a(&obs);
    #[cfg(feature = "tracking")]
    const EXPECTED: u64 = 0xe5c0_727c_e136_06c1;
    #[cfg(not(feature = "tracking"))]
    const EXPECTED: u64 = 0xf711_37d6_21fe_9117;
    assert_eq!(h, EXPECTED, "stream hash {h:#x}");
}
