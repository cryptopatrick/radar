//! Acceptance scenario A (spec §14.1): a 2D rotating surveillance radar
//! against five moving targets of different RCS and range.
//!
//! Ground truth is used only here, in the test, to match observations to
//! targets by nearest position; the library never sees it.
#![cfg(feature = "tracking")]

use radar::prelude::*;

const RADAR: SensorId = SensorId(1);
const OWN: PlatformId = PlatformId(1);
const STEP: SimDuration = SimDuration::from_millis(100);
/// 120 s = 30 revolutions of the 4 s antenna.
const STEPS: usize = 1200;
const REVOLUTIONS: f64 = 30.0;
const SPEED: f64 = 80.0;

/// (RCS m², range km, bearing deg), strongest echo (σ/R⁴) first.
const TARGETS: [(f64, f64, f64); 5] = [
    (10.0, 20.0, 36.0),
    (1.0, 20.0, 108.0),
    (10.0, 40.0, 180.0),
    (1.0, 40.0, 252.0),
    (1.0, 60.0, 324.0),
];

fn targets() -> Vec<TargetState> {
    TARGETS
        .iter()
        .enumerate()
        .map(|(i, &(rcs, range_km, bearing_deg))| {
            let b = bearing_deg.to_radians();
            let position = Vector3::new(b.sin(), b.cos(), 0.0) * (range_km * 1000.0);
            // Crossing: course 90° clockwise from the bearing.
            let course = b + std::f64::consts::FRAC_PI_2;
            TargetState {
                id: TargetId(i as u64 + 1),
                pose: Pose3::from_position_heading(position, course),
                velocity: Vector3::new(course.sin(), course.cos(), 0.0) * SPEED,
                extent_radius: 10.0,
                signature: RadarSignature {
                    rcs_m2: rcs,
                    swerling: Swerling::One,
                },
                transponder: None,
                platform: None,
            }
        })
        .collect()
}

fn truth(t: &TargetState, at: SimTime) -> Vector3 {
    t.position_at(SimTime::ZERO, at)
}

fn measured_position(m: &Measurement) -> Vector3 {
    let el = m.elevation.unwrap_or(0.0);
    m.sensor_position
        + Vector3::new(
            m.azimuth.sin() * el.cos(),
            m.azimuth.cos() * el.cos(),
            el.sin(),
        ) * m.range
}

/// Index of the target nearest `p` at `at`, if within `gate` metres.
fn nearest(targets: &[TargetState], p: Vector3, at: SimTime, gate: f64) -> Option<usize> {
    targets
        .iter()
        .enumerate()
        .map(|(i, t)| (i, truth(t, at).distance(p)))
        .filter(|&(_, d)| d < gate)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i)
}

#[test]
fn scenario_a_rotating_surveillance() {
    // The preset with a higher false-alarm probability, so false alarms occur.
    let mut cfg = RadarConfig::surveillance_2d().unwrap();
    if let DetectionConfig::RangeEquation(c) = &mut cfg.detection {
        c.pfa = 1e-4;
    }
    let targets = targets();
    let mut sim = Simulation::new(SimulationConfig::default().seed(Seed(1)));
    sim.add_radar(RADAR, OWN, Pose3::IDENTITY, cfg).unwrap();
    let mut w = WorldSnapshot::new(SimTime::ZERO);
    w.platforms = vec![PlatformState {
        id: OWN,
        pose: Pose3::IDENTITY,
        velocity: Vector3::ZERO,
    }];
    w.targets = targets.clone();
    sim.update_world(w).unwrap();

    let mut obs = Vec::new();
    for _ in 0..STEPS {
        sim.step_open(STEP).unwrap();
        sim.drain(RADAR, &mut obs).unwrap();
    }

    // Detections per target, grouped into dwells (looks less than 1 s apart
    // belong to the same pass of the beam).
    let mut last_hit: Vec<Option<SimTime>> = vec![None; targets.len()];
    let mut dwells = vec![0u32; targets.len()];
    let mut false_alarms = 0;
    for o in &obs {
        if let Observation::Measurement(m) = o {
            match nearest(&targets, measured_position(m), m.time, 1_000.0) {
                Some(i) => {
                    let new_dwell = last_hit[i].is_none_or(|prev| {
                        m.time.duration_since(prev).unwrap() > SimDuration::from_secs(1)
                    });
                    if new_dwell {
                        dwells[i] += 1;
                    }
                    last_hit[i] = Some(m.time);
                }
                None => false_alarms += 1,
            }
        }
    }
    let rates: Vec<f64> = dwells.iter().map(|&d| f64::from(d) / REVOLUTIONS).collect();

    // Detection rate falls with σ/R⁴. The strongest four have Pd above
    // 0.97, so neighbours may swap by sampling noise: each must be at least
    // its weaker neighbour minus two binomial standard deviations (and at
    // least one dwell). The strongest and weakest must be strictly ordered.
    assert!(rates.iter().all(|&r| r <= 1.0), "{rates:?}");
    for pair in rates.windows(2) {
        let p = (pair[0] + pair[1]) / 2.0;
        let slack = (2.0 * (p * (1.0 - p) / REVOLUTIONS).sqrt()).max(1.0 / REVOLUTIONS);
        assert!(pair[0] + slack >= pair[1], "rates not ordered: {rates:?}");
    }
    assert!(rates[0] > rates[4], "{rates:?}");
    assert!(false_alarms >= 1, "no false alarm at pfa 1e-4");

    // Every target ends with a confirmed track on the right course.
    let tracks = sim.tracks(RADAR).unwrap();
    let now = sim.now();
    for (i, t) in targets.iter().enumerate() {
        let track = tracks
            .iter()
            .filter(|tr| tr.status == TrackStatus::Confirmed)
            .min_by(|a, b| {
                let da = a.predict(now).position.distance(truth(t, now));
                let db = b.predict(now).position.distance(truth(t, now));
                da.total_cmp(&db)
            })
            .unwrap_or_else(|| panic!("no confirmed track: {tracks:?}"));
        let distance = track.predict(now).position.distance(truth(t, now));
        assert!(
            distance < 1_000.0,
            "target {i}: nearest track {distance} m away"
        );
        let course = t.velocity.x.atan2(t.velocity.y);
        let error = (track.course() - course + std::f64::consts::PI)
            .rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI;
        assert!(
            error.abs() < 10f64.to_radians(),
            "target {i}: course error {}°",
            error.to_degrees()
        );
    }
}
