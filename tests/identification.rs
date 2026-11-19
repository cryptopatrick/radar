//! Acceptance (spec §14.6): a transponder target's track carries its code;
//! a target without a transponder never gets an identity.
#![cfg(feature = "tracking")]

use radar::prelude::*;

const RADAR: SensorId = SensorId(1);
const OWN: PlatformId = PlatformId(1);
const CODE: TransponderCode = TransponderCode(0o7421);

fn target(id: u64, position: Vector3, transponder: Option<TransponderCode>) -> TargetState {
    TargetState {
        id: TargetId(id),
        pose: Pose3::from_position_heading(position, 0.0),
        velocity: Vector3::new(-60.0, 40.0, 0.0),
        extent_radius: 10.0,
        signature: RadarSignature {
            rcs_m2: 5.0,
            swerling: Swerling::One,
        },
        transponder,
        platform: None,
    }
}

#[test]
fn only_transponder_tracks_are_identified() {
    let mut cfg = RadarConfig::surveillance_2d().unwrap();
    cfg.identification = Some(IdentificationConfig {
        reply_probability: 0.9,
        max_range: 100_000.0,
    });
    let mut sim = Simulation::new(SimulationConfig::default().seed(Seed(1)));
    sim.add_radar(RADAR, OWN, Pose3::IDENTITY, cfg).unwrap();
    let mut w = WorldSnapshot::new(SimTime::ZERO);
    w.platforms = vec![PlatformState {
        id: OWN,
        pose: Pose3::IDENTITY,
        velocity: Vector3::ZERO,
    }];
    w.targets = vec![
        target(1, Vector3::new(15_000.0, 25_000.0, 0.0), Some(CODE)),
        target(2, Vector3::new(-20_000.0, -18_000.0, 0.0), None),
    ];
    sim.update_world(w.clone()).unwrap();

    let mut obs = Vec::new();
    for _ in 0..600 {
        sim.step_open(SimDuration::from_millis(100)).unwrap();
        sim.drain(RADAR, &mut obs).unwrap();
    }

    let now = sim.now();
    let tracks = sim.tracks(RADAR).unwrap();
    let track_of = |t: &TargetState| {
        let truth = t.position_at(SimTime::ZERO, now);
        tracks
            .iter()
            .filter(|tr| matches!(tr.status, TrackStatus::Confirmed | TrackStatus::Coasting))
            .find(|tr| tr.predict(now).position.distance(truth) < 1_000.0)
            .unwrap_or_else(|| panic!("no established track near {truth:?}: {tracks:?}"))
    };
    let identified = track_of(&w.targets[0]);
    let anonymous = track_of(&w.targets[1]);
    assert_eq!(identified.identity, Some(CODE));
    assert_eq!(anonymous.identity, None);
    // Never identified at any point of its life.
    assert!(obs.iter().all(|o| match o {
        Observation::TrackUpdated(t) if t.id == anonymous.id => t.identity.is_none(),
        _ => true,
    }));
}
