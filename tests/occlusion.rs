//! Acceptance (spec §14.4): a target behind an `Environment` blocker is
//! never detected, while an unobstructed one is tracked.
#![cfg(feature = "tracking")]

use radar::prelude::*;

const RADAR: SensorId = SensorId(1);
const OWN: PlatformId = PlatformId(1);
const HIDDEN: Vector3 = Vector3::new(20_000.0, 0.0, 0.0);
const VISIBLE: Vector3 = Vector3::new(0.0, 20_000.0, 0.0);

/// A wall at x = 10 km spanning y = −3 km..3 km, between the radar and the
/// eastern target.
struct Wall;
impl Environment for Wall {
    fn line_of_sight_blocked(&self, from: Vector3, to: Vector3) -> bool {
        const X: f64 = 10_000.0;
        if (from.x - X) * (to.x - X) > 0.0 || from.x == to.x {
            return false;
        }
        let t = (X - from.x) / (to.x - from.x);
        (from.y + t * (to.y - from.y)).abs() < 3_000.0
    }
}

fn target(id: u64, position: Vector3) -> TargetState {
    TargetState {
        id: TargetId(id),
        pose: Pose3::from_position_heading(position, 0.0),
        velocity: Vector3::new(0.0, 30.0, 0.0),
        extent_radius: 10.0,
        signature: RadarSignature {
            rcs_m2: 10.0,
            swerling: Swerling::One,
        },
        transponder: None,
        platform: None,
    }
}

#[test]
fn blocked_target_is_never_detected() {
    let mut sim = Simulation::new(SimulationConfig::default().seed(Seed(1)));
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
        pose: Pose3::IDENTITY,
        velocity: Vector3::ZERO,
    }];
    w.targets = vec![target(1, HIDDEN), target(2, VISIBLE)];
    sim.update_world(w.clone()).unwrap();

    let mut obs = Vec::new();
    for _ in 0..600 {
        sim.step(SimDuration::from_millis(100), &Wall).unwrap();
        sim.drain(RADAR, &mut obs).unwrap();
    }

    let near = |t: &TargetState, m: &Measurement| {
        let el = m.elevation.unwrap_or(0.0);
        let p = m.sensor_position
            + Vector3::new(
                m.azimuth.sin() * el.cos(),
                m.azimuth.cos() * el.cos(),
                el.sin(),
            ) * m.range;
        p.distance(t.position_at(SimTime::ZERO, m.time)) < 2_000.0
    };
    let measurements: Vec<&Measurement> = obs
        .iter()
        .filter_map(|o| match o {
            Observation::Measurement(m) => Some(m),
            _ => None,
        })
        .collect();
    assert_eq!(
        measurements
            .iter()
            .filter(|m| near(&w.targets[0], m))
            .count(),
        0
    );
    assert!(
        measurements
            .iter()
            .filter(|m| near(&w.targets[1], m))
            .count()
            >= 10
    );

    let now = sim.now();
    let tracks = sim.tracks(RADAR).unwrap();
    let truth = w.targets[1].position_at(SimTime::ZERO, now);
    assert!(
        tracks.iter().any(
            |t| matches!(t.status, TrackStatus::Confirmed | TrackStatus::Coasting)
                && t.predict(now).position.distance(truth) < 1_000.0
        ),
        "{tracks:?}"
    );
    let hidden = w.targets[0].position_at(SimTime::ZERO, now);
    assert!(
        tracks
            .iter()
            .all(|t| t.predict(now).position.distance(hidden) > 2_000.0),
        "{tracks:?}"
    );
}
