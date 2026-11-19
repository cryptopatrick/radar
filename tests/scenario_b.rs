//! Acceptance scenario B (spec §14.2): a 3D volume-scan radar on an
//! aircraft flying north at 200 m/s and 8 km altitude, against targets at
//! 1, 5 and 10 km altitude.
//!
//! Ground truth is used only here, to match tracks to targets.
#![cfg(feature = "tracking")]

use radar::prelude::*;

const RADAR: SensorId = SensorId(1);
const OWN: PlatformId = PlatformId(1);
const STEP: SimDuration = SimDuration::from_millis(100);
/// 180 s = 15 volumes of six bars at one turn per 2 s.
const STEPS: usize = 1800;

/// (altitude m, bearing deg, velocity east m/s, velocity north m/s); every
/// target starts 60 km from the aircraft horizontally.
const TARGETS: [(f64, f64, f64, f64); 3] = [
    (1_000.0, 45.0, 0.0, 220.0),
    (5_000.0, 160.0, 20.0, 180.0),
    (10_000.0, 280.0, -15.0, 200.0),
];

fn targets() -> Vec<TargetState> {
    TARGETS
        .iter()
        .enumerate()
        .map(|(i, &(alt, bearing_deg, ve, vn))| {
            let b = bearing_deg.to_radians();
            TargetState {
                id: TargetId(i as u64 + 1),
                pose: Pose3::from_position_heading(
                    Vector3::new(60_000.0 * b.sin(), 60_000.0 * b.cos(), alt),
                    ve.atan2(vn),
                ),
                velocity: Vector3::new(ve, vn, 0.0),
                extent_radius: 10.0,
                signature: RadarSignature {
                    rcs_m2: 10.0,
                    swerling: Swerling::One,
                },
                transponder: None,
                platform: None,
            }
        })
        .collect()
}

#[test]
fn scenario_b_airborne_3d_surveillance() {
    // The preset, with its bars moved to look down from 8 km: targets lie
    // between about −7° and +2° elevation.
    let mut cfg = RadarConfig::surveillance_3d().unwrap();
    cfg.scan = ScanConfig::Volume {
        elevation_bars: [-7.0f64, -5.0, -3.0, -1.0, 1.0, 3.0]
            .iter()
            .map(|d| d.to_radians())
            .collect(),
        rate: std::f64::consts::PI,
    };
    let targets = targets();
    let mut sim = Simulation::new(SimulationConfig::default().seed(Seed(1)));
    sim.add_radar(RADAR, OWN, Pose3::IDENTITY, cfg).unwrap();
    let mut w = WorldSnapshot::new(SimTime::ZERO);
    w.platforms = vec![PlatformState {
        id: OWN,
        pose: Pose3::from_position_heading(Vector3::new(0.0, 0.0, 8_000.0), 0.0),
        velocity: Vector3::new(0.0, 200.0, 0.0),
    }];
    w.targets = targets.clone();
    sim.update_world(w).unwrap();

    let mut obs = Vec::new();
    for _ in 0..STEPS {
        sim.step_open(STEP).unwrap();
        sim.drain(RADAR, &mut obs).unwrap();
    }

    let measurements: Vec<&Measurement> = obs
        .iter()
        .filter_map(|o| match o {
            Observation::Measurement(m) => Some(m),
            _ => None,
        })
        .collect();
    assert!(
        measurements.len() > 30,
        "{} measurements",
        measurements.len()
    );
    assert!(measurements.iter().all(|m| m.elevation.is_some()));
    // Measured from the moving aircraft.
    let last = measurements.last().unwrap();
    let expected = 200.0 * last.time.as_secs_f64();
    assert!((last.sensor_position.y - expected).abs() < 1e-6);
    assert!((last.sensor_position.z - 8_000.0).abs() < 1e-6);

    let tracks = sim.tracks(RADAR).unwrap();
    let now = sim.now();
    for (i, t) in targets.iter().enumerate() {
        let truth = t.position_at(SimTime::ZERO, now);
        let track = tracks
            .iter()
            .filter(|tr| tr.status == TrackStatus::Confirmed)
            .min_by(|a, b| {
                let da = a.predict(now).position.distance(truth);
                let db = b.predict(now).position.distance(truth);
                da.total_cmp(&db)
            })
            .unwrap_or_else(|| panic!("no confirmed track: {tracks:?}"));
        let estimate = track.predict(now).position;
        let horizontal = Vector3::new(estimate.x - truth.x, estimate.y - truth.y, 0.0).length();
        assert!(horizontal < 2_000.0, "target {i}: {horizontal} m off");
        assert!(
            (estimate.z - truth.z).abs() < 500.0,
            "target {i}: z {} vs {}",
            estimate.z,
            truth.z
        );
    }
}
