//! Integration tests for track management (feature `tracking`).
#![cfg(feature = "tracking")]

use radar::prelude::*;
use radar::radar::IdentificationConfig;
use std::f64::consts::FRAC_PI_2;

const RADAR: SensorId = SensorId(1);
const OWN: PlatformId = PlatformId(7);
const STEP: SimDuration = SimDuration::from_millis(100);

fn target(pos: Vector3, velocity: Vector3) -> TargetState {
    TargetState {
        id: TargetId(1),
        pose: Pose3::from_position_heading(pos, 0.0),
        velocity,
        extent_radius: 10.0,
        signature: RadarSignature {
            rcs_m2: 1.0,
            swerling: Swerling::One,
        },
        transponder: None,
        platform: None,
    }
}

fn world(time: SimTime, targets: Vec<TargetState>) -> WorldSnapshot {
    moving_world(time, Vector3::ZERO, targets)
}

/// A world whose radar platform starts at the origin moving at `velocity`.
fn moving_world(time: SimTime, velocity: Vector3, targets: Vec<TargetState>) -> WorldSnapshot {
    let mut w = WorldSnapshot::new(time);
    w.platforms = vec![PlatformState {
        id: OWN,
        pose: Pose3::IDENTITY,
        velocity,
    }];
    w.targets = targets;
    w
}

fn sim_with(cfg: RadarConfig, targets: Vec<TargetState>) -> Simulation {
    let mut sim = Simulation::new(SimulationConfig::default());
    sim.add_radar(RADAR, OWN, Pose3::IDENTITY, cfg).unwrap();
    sim.update_world(world(SimTime::ZERO, targets)).unwrap();
    sim
}

/// Steps `n` 100 ms updates and drains the radar.
fn run(sim: &mut Simulation, n: usize, env: &dyn Environment) -> Vec<Observation> {
    for _ in 0..n {
        sim.step(STEP, env).unwrap();
    }
    let mut out = Vec::new();
    sim.drain(RADAR, &mut out).unwrap();
    out
}

fn confirmed(sim: &Simulation) -> Vec<Track> {
    sim.tracks(RADAR)
        .unwrap()
        .into_iter()
        .filter(|t| t.status == TrackStatus::Confirmed)
        .collect()
}

/// Target 20 km north moving east at 100 m/s.
fn eastbound() -> TargetState {
    target(
        Vector3::new(0.0, 20_000.0, 0.0),
        Vector3::new(100.0, 0.0, 0.0),
    )
}

/// A `surveillance_2d` radar with the given false-alarm probability.
fn surveillance_with_pfa(pfa: f64) -> RadarConfig {
    let mut cfg = RadarConfig::surveillance_2d().unwrap();
    if let DetectionConfig::RangeEquation(c) = &mut cfg.detection {
        c.pfa = pfa;
    }
    cfg
}

/// Runs the eastbound scenario for 30 s and returns the confirmed track.
fn established(cfg: RadarConfig) -> (Simulation, Track) {
    let mut sim = sim_with(cfg, vec![eastbound()]);
    run(&mut sim, 300, &OpenSky);
    let c = confirmed(&sim);
    assert_eq!(c.len(), 1, "{c:?}");
    let track = c[0].clone();
    (sim, track)
}

#[test]
fn track_confirms_and_estimates_course() {
    let mut sim = sim_with(RadarConfig::surveillance_2d().unwrap(), vec![eastbound()]);
    let obs = run(&mut sim, 600, &OpenSky);
    let c = confirmed(&sim);
    assert_eq!(c.len(), 1, "{c:?}");
    let t = &c[0];
    assert!(
        (t.course() - FRAC_PI_2).abs() < 5f64.to_radians(),
        "course {}",
        t.course().to_degrees()
    );
    assert!((t.speed() - 100.0).abs() < 10.0, "speed {}", t.speed());
    assert_eq!(t.sensor, RADAR);
    // The estimate extrapolated to now lies near the truth.
    let truth = Vector3::new(100.0 * sim.now().as_secs_f64(), 20_000.0, 0.0);
    assert!(t.predict(sim.now()).position.distance(truth) < 200.0);
    assert!(
        obs.iter()
            .any(|o| matches!(o, Observation::TrackUpdated(u) if u.id == t.id))
    );
}

struct Wall;
impl Environment for Wall {
    fn line_of_sight_blocked(&self, _from: Vector3, _to: Vector3) -> bool {
        true
    }
}

#[test]
fn coasts_through_misses() {
    let (mut sim, track) = established(RadarConfig::surveillance_2d().unwrap());
    // One revolution (4 s) with the target occluded.
    let obs = run(&mut sim, 40, &Wall);
    assert!(
        obs.iter().any(|o| matches!(o,
            Observation::TrackUpdated(u)
                if u.id == track.id && u.status == TrackStatus::Coasting && u.consecutive_misses >= 1)),
        "{obs:?}"
    );
    let now: Vec<Track> = sim.tracks(RADAR).unwrap();
    let t = now.iter().find(|t| t.id == track.id).unwrap();
    assert_eq!(t.status, TrackStatus::Coasting);
    // Two clear revolutions bring it back.
    run(&mut sim, 80, &OpenSky);
    let c = confirmed(&sim);
    assert_eq!(c.len(), 1, "{c:?}");
    assert_eq!(c[0].id, track.id);
    assert_eq!(c[0].consecutive_misses, 0);
}

#[test]
fn deleted_after_max_coast() {
    // A negligible false-alarm rate so no stray tentative tracks appear.
    let (mut sim, track) = established(surveillance_with_pfa(1e-12));
    sim.update_world(world(sim.now(), vec![])).unwrap();
    let obs = run(&mut sim, 240, &OpenSky);
    let deleted = obs
        .iter()
        .position(|o| matches!(o, Observation::TrackDeleted { id, .. } if *id == track.id))
        .expect("track deleted");
    let misses: Vec<u32> = obs[..deleted]
        .iter()
        .filter_map(|o| match o {
            Observation::TrackUpdated(u) if u.id == track.id => Some(u.consecutive_misses),
            _ => None,
        })
        .collect();
    assert_eq!(misses, vec![1, 2, 3, 4, 5], "{obs:?}");
    match &obs[deleted - 1] {
        Observation::TrackUpdated(u) => assert_eq!(u.status, TrackStatus::Deleted),
        other => panic!("expected the final update, got {other:?}"),
    }
    assert!(sim.tracks(RADAR).unwrap().is_empty());
}

#[test]
fn not_penalised_outside_beam() {
    // Noise-free radius radar sweeping a 180° sector at 90°/s; target at 10°.
    let cfg = RadarConfig::builder()
        .scan(ScanConfig::Sector {
            centre: FRAC_PI_2,
            width: std::f64::consts::PI,
            rate: FRAC_PI_2,
            elevation: 0.0,
        })
        .detection(DetectionConfig::Radius {
            range: 20_000.0,
            pd: 1.0,
            false_alarms_per_scan: 0.0,
        })
        .limits(RangeLimits {
            min_range: 0.0,
            instrumented_range: 20_000.0,
        })
        .measurement(MeasurementConfig::Fixed(MeasurementSigma {
            range: 0.0,
            azimuth: 0.0,
            elevation: 0.0,
            radial_velocity: 0.0,
        }))
        .tracker(TrackerConfig::Kalman {
            process_accel_sigma: 1.0,
        })
        .build()
        .unwrap();
    let az = 10f64.to_radians();
    let pos = Vector3::new(10_000.0 * az.sin(), 10_000.0 * az.cos(), 0.0);
    let mut sim = sim_with(cfg, vec![target(pos, Vector3::ZERO)]);
    // Dwells (one outcome each): 0.1–0.2 s, 3.9–4.2 s (reversal at 0°),
    // 7.8–8.2 s → confirmed on the third.
    run(&mut sim, 82, &OpenSky);
    let c = confirmed(&sim);
    assert_eq!(c.len(), 1, "{c:?}");
    assert_eq!(c[0].consecutive_misses, 0);
    // The beam now sweeps away from the track for 10 updates.
    let obs = run(&mut sim, 10, &OpenSky);
    let beam = sim.sensor_status(RADAR).unwrap().boresight_azimuth;
    assert!(beam > 90f64.to_radians(), "beam at {}", beam.to_degrees());
    assert!(
        !obs.iter()
            .any(|o| matches!(o, Observation::TrackUpdated(_))),
        "{obs:?}"
    );
    let t = &sim.tracks(RADAR).unwrap()[0];
    assert_eq!(t.consecutive_misses, 0);
    assert_eq!(t.status, TrackStatus::Confirmed);
}

#[test]
fn identity_majority() {
    let mut cfg = RadarConfig::surveillance_2d().unwrap();
    cfg.identification = Some(IdentificationConfig {
        reply_probability: 1.0,
        max_range: 100_000.0,
    });
    let mut t = eastbound();
    t.transponder = Some(TransponderCode(0o1234));
    let mut sim = sim_with(cfg, vec![t]);
    run(&mut sim, 300, &OpenSky);
    let c = confirmed(&sim);
    assert_eq!(c.len(), 1, "{c:?}");
    assert_eq!(c[0].identity, Some(TransponderCode(0o1234)));
}

#[test]
fn false_alarms_do_not_confirm() {
    let mut sim = sim_with(surveillance_with_pfa(1e-4), vec![]);
    let obs = run(&mut sim, 600, &OpenSky);
    assert!(
        obs.iter()
            .filter(|o| matches!(o, Observation::Measurement(_)))
            .count()
            > 50,
        "false alarms are generated"
    );
    assert!(
        !obs.iter().any(
            |o| matches!(o, Observation::TrackUpdated(t) if t.status == TrackStatus::Confirmed)
        )
    );
    assert!(confirmed(&sim).is_empty());
}

#[test]
fn no_tracker_no_tracks() {
    let mut sim = sim_with(RadarConfig::simple_2d(30_000.0).unwrap(), vec![eastbound()]);
    let obs = run(&mut sim, 50, &OpenSky);
    assert!(obs.iter().all(|o| !matches!(
        o,
        Observation::TrackUpdated(_) | Observation::TrackDeleted { .. }
    )));
    assert!(sim.tracks(RADAR).unwrap().is_empty());
    assert!(sim.tracks(SensorId(99)).is_err());
}

#[test]
fn overlap_does_not_cause_misses() {
    // Sweeps end every 9°; a target at 9° is looked at by two consecutive
    // updates every revolution (one dwell).
    let az = 9f64.to_radians();
    let pos = Vector3::new(20_000.0 * az.sin(), 20_000.0 * az.cos(), 0.0);
    let mut sim = sim_with(
        RadarConfig::surveillance_2d().unwrap(),
        vec![target(pos, Vector3::ZERO)],
    );
    let warmup = run(&mut sim, 100, &OpenSky);
    let obs = run(&mut sim, 200, &OpenSky);
    let looks = warmup
        .iter()
        .chain(&obs)
        .filter(|o| matches!(o, Observation::Measurement(_)))
        .count();
    assert!(looks > 10, "doubly illuminated: {looks} looks in 7.5 turns");
    let c = confirmed(&sim);
    assert_eq!(c.len(), 1, "{c:?}");
    assert_eq!(c[0].consecutive_misses, 0);
    // The target's track never misses or coasts (other tracks, from false
    // alarms or an occasional ungated second look, are not checked).
    let id = c[0].id;
    let ours: Vec<&Track> = obs
        .iter()
        .filter_map(|o| match o {
            Observation::TrackUpdated(t) if t.id == id => Some(t),
            _ => None,
        })
        .collect();
    assert!(ours.len() >= 4, "{ours:?}");
    assert!(
        ours.iter()
            .all(|t| t.consecutive_misses == 0 && t.status == TrackStatus::Confirmed),
        "{ours:?}"
    );
    // One hit per dwell: 8 dwells in 30 s (the first at t = 0.1 s).
    assert!(c[0].hits <= 8, "hits {}", c[0].hits);
}

#[test]
fn alpha_beta_surveillance_holds_one_track() {
    let mut cfg = RadarConfig::surveillance_2d().unwrap();
    cfg.tracker = TrackerConfig::AlphaBeta {
        alpha: 0.5,
        beta: 0.3,
    };
    let mut sim = sim_with(cfg, vec![eastbound()]);
    run(&mut sim, 600, &OpenSky);
    let c = confirmed(&sim);
    assert_eq!(c.len(), 1, "{c:?}");
    assert_eq!(c[0].id, TrackId(1));
}

#[test]
fn moving_platform_doppler_unbiased() {
    let mut sim = Simulation::new(SimulationConfig::default());
    sim.add_radar(
        RADAR,
        OWN,
        Pose3::IDENTITY,
        RadarConfig::surveillance_2d().unwrap(),
    )
    .unwrap();
    sim.update_world(moving_world(
        SimTime::ZERO,
        Vector3::new(0.0, 50.0, 0.0),
        vec![eastbound()],
    ))
    .unwrap();
    let obs = run(&mut sim, 600, &OpenSky);
    let rv: Vec<f64> = obs
        .iter()
        .filter_map(|o| match o {
            Observation::Measurement(m) => m.radial_velocity,
            _ => None,
        })
        .collect();
    assert!(!rv.is_empty());
    let c = confirmed(&sim);
    assert_eq!(c.len(), 1, "{c:?}");
    let t = &c[0];
    assert!(
        (t.course() - FRAC_PI_2).abs() < 5f64.to_radians(),
        "course {}",
        t.course().to_degrees()
    );
    assert!((t.speed() - 100.0).abs() < 10.0, "speed {}", t.speed());
    assert!(t.velocity.y.abs() < 10.0, "vy {}", t.velocity.y);
}

#[test]
fn noise_free_3d_keeps_one_track() {
    let cfg = RadarConfig::builder()
        .dimension(Dimension::ThreeD)
        .detection(DetectionConfig::Radius {
            range: 20_000.0,
            pd: 1.0,
            false_alarms_per_scan: 0.0,
        })
        // Long enough that the default sigma limit (2 km) keeps a fresh
        // tentative track through the 1 s revisit.
        .limits(RangeLimits {
            min_range: 0.0,
            instrumented_range: 20_000.0,
        })
        .tracker(TrackerConfig::Kalman {
            process_accel_sigma: 0.0,
        })
        .build()
        .unwrap();
    let mut sim = sim_with(
        cfg,
        vec![target(Vector3::new(500.0, 500.0, 0.0), Vector3::ZERO)],
    );
    run(&mut sim, 200, &OpenSky);
    let ts = sim.tracks(RADAR).unwrap();
    assert_eq!(ts.len(), 1, "{ts:?}");
    assert_eq!(ts[0].id, TrackId(1));
    assert_eq!(ts[0].status, TrackStatus::Confirmed);
    assert!(ts[0].hits >= 19, "hits {}", ts[0].hits);
}
