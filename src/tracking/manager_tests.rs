//! Unit tests for track management.

use super::*;
use crate::geometry::direction;
use crate::radar::measurement::MeasurementSigma;

const SENSOR: SensorId = SensorId(3);

fn meas(range: f64, az: f64, el: Option<f64>) -> Measurement {
    Measurement {
        sensor: SENSOR,
        time: SimTime::ZERO,
        range,
        azimuth: az,
        elevation: el,
        radial_velocity: None,
        snr_db: None,
        sigma: MeasurementSigma {
            range: 10.0,
            azimuth: 0.01,
            elevation: 0.02,
            radial_velocity: 1.0,
        },
        identity: None,
        sensor_position: Vector3::new(100.0, -50.0, 20.0),
    }
}

fn kalman() -> KalmanCv {
    KalmanCv {
        process_accel_sigma: 1.0,
    }
}

fn manager(dim: Dimension) -> TrackManager {
    TrackManager::new(&TrackManagementConfig::default(), dim, SENSOR, 100_000.0)
}

fn at(secs: u64) -> SimTime {
    SimTime::from_nanos(secs * 1_000_000_000)
}

fn updates(out: &[Observation]) -> Vec<&Track> {
    out.iter()
        .filter_map(|o| match o {
            Observation::TrackUpdated(t) => Some(t),
            _ => None,
        })
        .collect()
}

#[test]
fn jacobian_matches_finite_differences() {
    let f = |r: f64, az: f64, el: f64| direction(az, el) * r;
    let h = 1e-6;
    for &(r, az, el) in &[(1000.0, 0.3, 0.1), (5000.0, 2.5, -0.4), (200.0, 4.0, 1.2)] {
        let j = jacobian(r, az, el);
        let cols = [
            (f(r + h, az, el) - f(r - h, az, el)) * (0.5 / h),
            (f(r, az + h, el) - f(r, az - h, el)) * (0.5 / h),
            (f(r, az, el + h) - f(r, az, el - h)) * (0.5 / h),
        ];
        for (c, d) in cols.iter().enumerate() {
            let scale = if c == 0 { 1.0 } else { r };
            for (i, v) in [d.x, d.y, d.z].iter().enumerate() {
                assert!(
                    (j[i][c] - v).abs() < 1e-5 * scale,
                    "J[{i}][{c}] = {} vs {v} at ({r}, {az}, {el})",
                    j[i][c]
                );
            }
        }
    }
}

#[test]
fn polar_conversion_3d() {
    let m = meas(1000.0, 0.7, Some(0.2));
    let (p, cov) = polar_to_cartesian(&m);
    let expect = m.sensor_position + direction(0.7, 0.2) * 1000.0;
    assert!(p.distance(expect) < 1e-9);
    let j = jacobian(1000.0, 0.7, 0.2);
    let s2 = [100.0, 1e-4, 4e-4];
    for i in 0..3 {
        for k in 0..3 {
            let want: f64 = (0..3).map(|c| j[i][c] * s2[c] * j[k][c]).sum();
            assert!((cov[i][k] - want).abs() < 1e-9, "cov[{i}][{k}]");
            assert_eq!(cov[i][k], cov[k][i]);
        }
    }
}

#[test]
fn polar_conversion_2d_east() {
    // Due east: range error along x, azimuth error along y (r·σaz).
    let m = meas(1000.0, FRAC_PI_2_TEST, None);
    let (p, cov) = polar_to_cartesian(&m);
    assert!(p.distance(m.sensor_position + Vector3::new(1000.0, 0.0, 0.0)) < 1e-9);
    assert!((cov[0][0] - 100.0).abs() < 1e-9);
    assert!((cov[1][1] - 100.0).abs() < 1e-9); // (1000 · 0.01)²
    assert!(cov[0][1].abs() < 1e-9);
    assert_eq!(cov[2], [0.0, 0.0, 1e-6]);
    assert_eq!((cov[0][2], cov[1][2]), (0.0, 0.0));
    // Noise-free measurements still have a positive diagonal.
    let mut z = m.clone();
    z.sigma = MeasurementSigma {
        range: 0.0,
        azimuth: 0.0,
        elevation: 0.0,
        radial_velocity: 0.0,
    };
    let (_, cov) = polar_to_cartesian(&z);
    assert!((0..3).all(|i| cov[i][i] >= 1e-6));
}

const FRAC_PI_2_TEST: f64 = std::f64::consts::FRAC_PI_2;

#[test]
fn track_kinematics() {
    let t = Track {
        id: TrackId(1),
        sensor: SENSOR,
        status: TrackStatus::Confirmed,
        position: Vector3::new(0.0, 0.0, 0.0),
        velocity: Vector3::new(-10.0, 0.0, 0.0),
        covariance: Covariance6::new([[1.0; 6]; 6]),
        identity: None,
        first_seen: SimTime::ZERO,
        last_update: at(2),
        hits: 3,
        consecutive_misses: 0,
    };
    assert!((t.course() - 1.5 * std::f64::consts::PI).abs() < 1e-12);
    assert!((t.speed() - 10.0).abs() < 1e-12);
    let p = t.predict(at(5));
    assert_eq!(p.position, Vector3::new(-30.0, 0.0, 0.0));
    assert_eq!(p.covariance, t.covariance);
    assert_eq!(p.last_update, t.last_update);
    assert_eq!(
        t.predict(SimTime::ZERO).position,
        Vector3::new(20.0, 0.0, 0.0)
    );
}

#[test]
fn identity_majority_tie_smallest() {
    let mut mgr = manager(Dimension::TwoD);
    let mut out = Vec::new();
    let mut m = meas(1000.0, 1.0, None);
    let codes = [7, 3, 7, 3, 9];
    let expect = [7, 3, 7, 3, 3];
    for (k, code) in codes.iter().enumerate() {
        m.identity = Some(TransponderCode(*code));
        m.time = at(k as u64);
        mgr.update(
            &kalman(),
            at(k as u64),
            std::slice::from_ref(&m),
            |_| true,
            &mut out,
        );
        let t = &mgr.tracks()[0];
        assert_eq!(t.identity, Some(TransponderCode(expect[k])), "after {k}");
    }
    assert_eq!(mgr.tracks().len(), 1);
}

#[test]
fn m_of_n_confirms_and_tentative_dies() {
    let mut mgr = manager(Dimension::TwoD);
    let mut out = Vec::new();
    let m = meas(1000.0, 1.0, None);
    // Hits at t = 0, 1, 2 confirm (3 of 5).
    for k in 0..3 {
        mgr.update(
            &kalman(),
            at(k),
            std::slice::from_ref(&m),
            |_| true,
            &mut out,
        );
    }
    let t = &mgr.tracks()[0];
    assert_eq!(
        (t.status, t.hits, t.id),
        (TrackStatus::Confirmed, 3, TrackId(1))
    );

    // A single hit then four covered misses: window [H, M, M, M, M] → deleted.
    let mut mgr = manager(Dimension::TwoD);
    out.clear();
    mgr.update(
        &kalman(),
        at(0),
        std::slice::from_ref(&m),
        |_| true,
        &mut out,
    );
    for k in 1..4 {
        mgr.update(&kalman(), at(k), &[], |_| true, &mut out);
        assert_eq!(mgr.tracks()[0].status, TrackStatus::Tentative);
    }
    out.clear();
    mgr.update(&kalman(), at(4), &[], |_| true, &mut out);
    assert!(mgr.tracks().is_empty());
    assert!(matches!(&out[0], Observation::TrackUpdated(t) if t.status == TrackStatus::Deleted));
    assert!(matches!(
        out[1],
        Observation::TrackDeleted { id: TrackId(1), .. }
    ));
}

#[test]
fn uncovered_tracks_are_not_missed() {
    let mut mgr = manager(Dimension::TwoD);
    let mut out = Vec::new();
    let m = meas(1000.0, 1.0, None);
    mgr.update(
        &kalman(),
        at(0),
        std::slice::from_ref(&m),
        |_| true,
        &mut out,
    );
    out.clear();
    for k in 1..10 {
        mgr.update(&kalman(), at(k), &[], |_| false, &mut out);
    }
    assert!(out.is_empty());
    let t = &mgr.tracks()[0];
    assert_eq!(t.consecutive_misses, 0);
    assert_eq!(t.last_update, at(0));
}

#[test]
fn greedy_assignment_and_ids() {
    let mut mgr = manager(Dimension::TwoD);
    let mut out = Vec::new();
    let a = meas(1000.0, 1.0, None);
    let b = meas(3000.0, 2.0, None);
    mgr.update(
        &kalman(),
        at(0),
        &[a.clone(), b.clone()],
        |_| true,
        &mut out,
    );
    let ids: Vec<_> = updates(&out).iter().map(|t| t.id).collect();
    assert_eq!(ids, vec![TrackId(1), TrackId(2)]);
    // Next update, measurements in the other order: each still goes to its
    // own track and no new track starts.
    out.clear();
    mgr.update(&kalman(), at(1), &[b, a.clone()], |_| true, &mut out);
    let ts = updates(&out);
    assert_eq!(ts.len(), 2);
    assert!(ts.iter().all(|t| t.hits == 2));
    let (pa, _) = polar_to_cartesian(&a);
    assert!(ts[0].position.distance(pa) < 50.0);
    // A far measurement fails the gate and starts track 3.
    out.clear();
    mgr.update(
        &kalman(),
        at(2),
        &[meas(50_000.0, 3.0, None)],
        |_| false,
        &mut out,
    );
    assert_eq!(updates(&out)[0].id, TrackId(3));
    mgr.reset();
    mgr.update(
        &kalman(),
        at(3),
        std::slice::from_ref(&a),
        |_| true,
        &mut out,
    );
    assert_eq!(mgr.tracks()[0].id, TrackId(1));
}

#[test]
fn two_d_tracks_pin_vertical() {
    let mut mgr = manager(Dimension::TwoD);
    let mut out = Vec::new();
    for k in 0..5 {
        let m = meas(1000.0 + 10.0 * k as f64, 1.0, None);
        mgr.update(
            &kalman(),
            at(k),
            std::slice::from_ref(&m),
            |_| true,
            &mut out,
        );
    }
    let t = &mgr.tracks()[0];
    assert_eq!(t.velocity.z, 0.0);
    assert_eq!(t.covariance.get(5, 5), 1e-6);
    assert_eq!(t.covariance.get(2, 0), 0.0);
    assert!(t.speed() > 5.0);
}

#[test]
fn position_sigma_limit_deletes() {
    let cfg = TrackManagementConfig {
        max_position_sigma: Some(500.0),
        ..TrackManagementConfig::default()
    };
    let mut mgr = TrackManager::new(&cfg, Dimension::TwoD, SENSOR, 100_000.0);
    let mut out = Vec::new();
    mgr.update(
        &kalman(),
        at(0),
        &[meas(1000.0, 1.0, None)],
        |_| true,
        &mut out,
    );
    out.clear();
    // 300 m/s velocity sigma: after 2 s the predicted sigma exceeds 500 m.
    mgr.update(&kalman(), at(2), &[], |_| false, &mut out);
    assert!(mgr.tracks().is_empty());
    assert_eq!(out.len(), 1);
    assert!(matches!(
        out[0],
        Observation::TrackDeleted { id: TrackId(1), .. }
    ));
}

#[test]
fn with_tracker_selects_filter() {
    assert!(with_tracker(&TrackerConfig::None, |_| ()).is_none());
    let ab = TrackerConfig::AlphaBeta {
        alpha: 0.5,
        beta: 0.1,
    };
    assert_eq!(
        with_tracker(&ab, |t| format!("{t:?}")).unwrap(),
        "AlphaBeta { alpha: 0.5, beta: 0.1 }"
    );
}
