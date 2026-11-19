//! Acceptance (spec §14.8): no public observation type carries ground truth
//! such as a `TargetId`. The destructuring below lists every field without
//! `..`, so a new field fails to compile until it has been reviewed.

use radar::prelude::*;

const RADAR: SensorId = SensorId(1);
const OWN: PlatformId = PlatformId(1);

/// Runs a short noise-free scenario and returns everything sensor 1 said.
fn observations() -> Vec<Observation> {
    let cfg = RadarConfig::builder()
        .detection(DetectionConfig::Radius {
            range: 2_000.0,
            pd: 1.0,
            false_alarms_per_scan: 0.0,
        })
        .limits(RangeLimits {
            min_range: 0.0,
            instrumented_range: 2_000.0,
        })
        .doppler(true)
        .identification(Some(IdentificationConfig {
            reply_probability: 1.0,
            max_range: 2_000.0,
        }));
    #[cfg(feature = "tracking")]
    let cfg = cfg.tracker(TrackerConfig::Kalman {
        process_accel_sigma: 1.0,
    });
    let mut sim = Simulation::new(SimulationConfig::default());
    sim.add_radar(RADAR, OWN, Pose3::IDENTITY, cfg.build().unwrap())
        .unwrap();
    let mut w = WorldSnapshot::new(SimTime::ZERO);
    w.platforms = vec![PlatformState {
        id: OWN,
        pose: Pose3::IDENTITY,
        velocity: Vector3::ZERO,
    }];
    w.targets = vec![TargetState {
        id: TargetId(42),
        pose: Pose3::from_position_heading(Vector3::new(500.0, 500.0, 0.0), 0.0),
        velocity: Vector3::new(5.0, 0.0, 0.0),
        extent_radius: 5.0,
        signature: RadarSignature {
            rcs_m2: 1.0,
            swerling: Swerling::Zero,
        },
        transponder: Some(TransponderCode(0o1200)),
        platform: None,
    }];
    sim.update_world(w).unwrap();
    let mut out = Vec::new();
    for _ in 0..100 {
        sim.step_open(SimDuration::from_millis(100)).unwrap();
    }
    sim.drain(RADAR, &mut out).unwrap();
    out
}

#[test]
fn measurement_fields_are_reviewed() {
    // adding a field breaks this test on purpose; confirm it carries no ground truth before updating
    let obs = observations();
    let mut measurements = 0;
    #[cfg(feature = "tracking")]
    let mut tracks = 0;
    for o in &obs {
        match o {
            Observation::Measurement(m) => {
                let Measurement {
                    sensor,
                    time: _,
                    range: _,
                    azimuth: _,
                    elevation: _,
                    radial_velocity,
                    snr_db: _,
                    sigma: _,
                    identity,
                    sensor_position: _,
                    sensor_velocity: _,
                } = m;
                assert_eq!(*sensor, RADAR);
                assert!(radial_velocity.is_some());
                assert!(identity.is_some());
                measurements += 1;
            }
            Observation::ScanCompleted { sensor, time: _ } => assert_eq!(*sensor, RADAR),
            #[cfg(feature = "tracking")]
            Observation::TrackUpdated(t) => {
                let Track {
                    id: _,
                    sensor,
                    status: _,
                    position: _,
                    velocity: _,
                    covariance: _,
                    identity: _,
                    first_seen: _,
                    last_update: _,
                    hits: _,
                    consecutive_misses: _,
                } = t;
                assert_eq!(*sensor, RADAR);
                tracks += 1;
            }
            #[cfg(feature = "tracking")]
            Observation::TrackDeleted {
                sensor,
                id: _,
                time: _,
            } => assert_eq!(*sensor, RADAR),
            #[cfg(feature = "ew")]
            Observation::JammingStrobe {
                sensor,
                time: _,
                azimuth: _,
                elevation: _,
                j_over_n_db: _,
            } => assert_eq!(*sensor, RADAR),
        }
    }
    assert!(measurements > 0);
    #[cfg(feature = "tracking")]
    assert!(tracks > 0);
}
