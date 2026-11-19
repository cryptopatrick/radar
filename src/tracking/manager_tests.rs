//! Unit tests for track management.

use super::*;
use crate::geometry::direction;
use crate::radar::measurement::MeasurementSigma;

const SENSOR: SensorId = SensorId(3);
/// Every update its own dwell (Fixed/Steered behaviour).
const STEPPED: BeamMotion = BeamMotion {
    sweeping: false,
    swept: 0.0,
};
/// A sweeping scan moving 9° per update.
const SWEEP: BeamMotion = BeamMotion {
    sweeping: true,
    swept: 0.157,
};

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
        sensor_velocity: Vector3::ZERO,
    }
}

fn kalman() -> KalmanCv {
    KalmanCv {
        process_accel_sigma: 1.0,
    }
}

fn manager(dim: Dimension) -> TrackManager {
    TrackManager::new(
        &TrackManagementConfig::default(),
        dim,
        SENSOR,
        100_000.0,
        1e9,
    )
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
            STEPPED,
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
fn m_of_n_confirms() {
    let mut mgr = manager(Dimension::TwoD);
    let mut out = Vec::new();
    let m = meas(1000.0, 1.0, None);
    // Hits at t = 0, 1, 2 confirm (3 of 5).
    for k in 0..3 {
        mgr.update(
            &kalman(),
            at(k),
            STEPPED,
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
}

#[test]
fn doomed_tentative_deleted_early() {
    // 3 of 5: after [H, M, M] three hits are still reachable; after
    // [H, M, M, M] they are not, so the track goes on its third miss.
    let mut mgr = manager(Dimension::TwoD);
    let mut out = Vec::new();
    let m = meas(1000.0, 1.0, None);
    mgr.update(
        &kalman(),
        at(0),
        STEPPED,
        std::slice::from_ref(&m),
        |_| true,
        &mut out,
    );
    for k in 1..3 {
        mgr.update(&kalman(), at(k), STEPPED, &[], |_| true, &mut out);
        assert_eq!(mgr.tracks()[0].status, TrackStatus::Tentative);
    }
    out.clear();
    mgr.update(&kalman(), at(3), STEPPED, &[], |_| true, &mut out);
    assert!(mgr.tracks().is_empty());
    assert!(matches!(&out[0], Observation::TrackUpdated(t) if t.status == TrackStatus::Deleted));
    assert!(matches!(
        out[1],
        Observation::TrackDeleted { id: TrackId(1), .. }
    ));
}

#[test]
fn established_tracks_take_plots_before_tentatives() {
    let mut mgr = manager(Dimension::TwoD);
    let mut out = Vec::new();
    let a = meas(1000.0, 1.0, None);
    let (pa, _) = polar_to_cartesian(&a);
    // Only the area around `a` is ever covered, so the tentative started
    // 200 m beyond it never misses and just grows stale.
    let covered = |p: Vector3| p.distance(pa) < 100.0;
    mgr.update(
        &kalman(),
        at(0),
        STEPPED,
        &[a.clone(), meas(1200.0, 1.0, None)],
        covered,
        &mut out,
    );
    for k in 1..3 {
        mgr.update(
            &kalman(),
            at(k),
            STEPPED,
            std::slice::from_ref(&a),
            covered,
            &mut out,
        );
    }
    let ts = mgr.tracks();
    assert_eq!(ts[0].status, TrackStatus::Confirmed);
    assert_eq!((ts[1].status, ts[1].hits), (TrackStatus::Tentative, 1));
    // 30 s later a plot 40 m beyond `a`: the stale tentative (≈9 km sigma)
    // has the smaller d², but the confirmed track gates it and wins.
    let plot = meas(1040.0, 1.0, None);
    out.clear();
    mgr.update(
        &kalman(),
        at(32),
        STEPPED,
        std::slice::from_ref(&plot),
        covered,
        &mut out,
    );
    let ts = mgr.tracks();
    assert_eq!((ts[0].status, ts[0].hits), (TrackStatus::Confirmed, 4));
    assert_eq!(ts[0].consecutive_misses, 0);
    assert_eq!(ts[1].hits, 1);
    assert_eq!(ts.len(), 2, "no new track: {ts:?}");
}

#[test]
fn uncovered_tracks_are_not_missed() {
    let mut mgr = manager(Dimension::TwoD);
    let mut out = Vec::new();
    let m = meas(1000.0, 1.0, None);
    mgr.update(
        &kalman(),
        at(0),
        STEPPED,
        std::slice::from_ref(&m),
        |_| true,
        &mut out,
    );
    out.clear();
    for k in 1..10 {
        mgr.update(&kalman(), at(k), STEPPED, &[], |_| false, &mut out);
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
        STEPPED,
        &[a.clone(), b.clone()],
        |_| true,
        &mut out,
    );
    let ids: Vec<_> = updates(&out).iter().map(|t| t.id).collect();
    assert_eq!(ids, vec![TrackId(1), TrackId(2)]);
    // Next update, measurements in the other order: each still goes to its
    // own track and no new track starts.
    out.clear();
    mgr.update(
        &kalman(),
        at(1),
        STEPPED,
        &[b, a.clone()],
        |_| true,
        &mut out,
    );
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
        STEPPED,
        &[meas(50_000.0, 3.0, None)],
        |_| false,
        &mut out,
    );
    assert_eq!(updates(&out)[0].id, TrackId(3));
    mgr.reset();
    mgr.update(
        &kalman(),
        at(3),
        STEPPED,
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
            STEPPED,
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
    let mut mgr = TrackManager::new(&cfg, Dimension::TwoD, SENSOR, 100_000.0, 1e9);
    let mut out = Vec::new();
    mgr.update(
        &kalman(),
        at(0),
        STEPPED,
        &[meas(1000.0, 1.0, None)],
        |_| true,
        &mut out,
    );
    out.clear();
    // 300 m/s velocity sigma: after 2 s the predicted sigma exceeds 500 m,
    // but a tentative track is exempt.
    mgr.update(&kalman(), at(2), STEPPED, &[], |_| false, &mut out);
    assert!(out.is_empty());
    assert_eq!(mgr.tracks()[0].status, TrackStatus::Tentative);
    // Confirm it (hits at 0, 3, 4), then leave it unseen: once its predicted
    // sigma exceeds 500 m it is deleted.
    let m = meas(1000.0, 1.0, None);
    for k in 3..5 {
        mgr.update(
            &kalman(),
            at(k),
            STEPPED,
            std::slice::from_ref(&m),
            |_| true,
            &mut out,
        );
    }
    assert_eq!(mgr.tracks()[0].status, TrackStatus::Confirmed);
    out.clear();
    mgr.update(&kalman(), at(200), STEPPED, &[], |_| false, &mut out);
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

#[test]
fn dwell_absorbs_second_look_and_misses_once() {
    let mut mgr = manager(Dimension::TwoD);
    let mut out = Vec::new();
    let m = meas(1000.0, 1.0, None);
    let one = std::slice::from_ref(&m);
    // Dwell 1: two consecutive looks, both detected → one hit, no new track.
    mgr.update(&kalman(), at(0), SWEEP, one, |_| true, &mut out);
    out.clear();
    mgr.update(&kalman(), at(1), SWEEP, one, |_| true, &mut out);
    assert!(out.is_empty(), "absorbed: {out:?}");
    let t = &mgr.tracks()[0];
    assert_eq!((mgr.tracks().len(), t.hits, t.last_update), (1, 1, at(0)));
    // Beam leaves: the dwell ended with a hit, so no miss.
    mgr.update(&kalman(), at(2), SWEEP, &[], |_| false, &mut out);
    assert!(out.is_empty());
    // Dwell 2: two covered looks with no detection → one miss, at dwell end.
    mgr.update(&kalman(), at(3), SWEEP, &[], |_| true, &mut out);
    mgr.update(&kalman(), at(4), SWEEP, &[], |_| true, &mut out);
    assert!(out.is_empty());
    mgr.update(&kalman(), at(5), SWEEP, &[], |_| false, &mut out);
    let ts = updates(&out);
    assert_eq!(ts.len(), 1);
    assert_eq!(ts[0].consecutive_misses, 1);
    // Dwell 3: miss on the first look, hit on the second → a hit.
    out.clear();
    mgr.update(&kalman(), at(6), SWEEP, &[], |_| true, &mut out);
    mgr.update(&kalman(), at(7), SWEEP, one, |_| true, &mut out);
    let t = &updates(&out)[0];
    assert_eq!((t.hits, t.consecutive_misses), (2, 0));
}

#[test]
fn dwell_ends_after_half_turn() {
    let mut mgr = manager(Dimension::TwoD);
    let mut out = Vec::new();
    let half = BeamMotion {
        sweeping: true,
        swept: 2.0,
    };
    mgr.update(
        &kalman(),
        at(0),
        half,
        &[meas(1000.0, 1.0, None)],
        |_| true,
        &mut out,
    );
    out.clear();
    // Covered throughout. Dwell 1 (hit) has swept 4 rad ≥ π by t = 2, so it
    // ends there; dwell 2 (t = 2, 3) ends at t = 4 without a hit: one miss.
    for k in 1..4 {
        mgr.update(&kalman(), at(k), half, &[], |_| true, &mut out);
    }
    assert!(out.is_empty(), "{out:?}");
    mgr.update(&kalman(), at(4), half, &[], |_| true, &mut out);
    assert_eq!(updates(&out)[0].consecutive_misses, 1);
}

/// A filter whose position update is always skipped.
#[derive(Debug)]
struct Skipping(KalmanCv);
impl Tracker for Skipping {
    fn predict(&self, s: &TrackState, dt: f64) -> TrackState {
        self.0.predict(s, dt)
    }
    fn update_position(&self, s: &TrackState, _z: Vector3, _r: [[f64; 3]; 3]) -> TrackState {
        s.clone()
    }
}

#[test]
fn skipped_update_keeps_last_hit_state() {
    let ab = AlphaBeta {
        alpha: 0.5,
        beta: 0.3,
    };
    let mut mgr = manager(Dimension::TwoD);
    let mut out = Vec::new();
    let m = meas(1000.0, 1.0, None);
    mgr.update(
        &ab,
        at(0),
        STEPPED,
        std::slice::from_ref(&m),
        |_| true,
        &mut out,
    );
    let before = mgr.tracks()[0].clone();
    let skip = Skipping(kalman());
    mgr.update(
        &skip,
        at(2),
        STEPPED,
        std::slice::from_ref(&m),
        |_| true,
        &mut out,
    );
    let after = &mgr.tracks()[0];
    assert_eq!(after.hits, 2);
    assert_eq!(after.last_update, before.last_update);
    assert_eq!(after.position, before.position);
    assert_eq!(after.covariance, before.covariance);
    // Elapsed restarts from the original last hit: one 3 s predict.
    assert_eq!(mgr.tracks[0].state.elapsed, 0.0);
    mgr.update(
        &ab,
        at(3),
        STEPPED,
        std::slice::from_ref(&m),
        |_| true,
        &mut out,
    );
    assert_eq!(mgr.tracks()[0].last_update, at(3));
}

#[test]
fn distance_rejects_degenerate_covariance() {
    let mgr = manager(Dimension::ThreeD);
    let s = TrackState::new([0.0; 6], Covariance6::new([[0.0; 6]; 6]));
    assert_eq!(
        mgr.distance2(&s, Vector3::new(1.0, 0.0, 0.0), &[[0.0; 3]; 3]),
        None
    );
    // Tiny but well-conditioned covariances still gate.
    let mut p = [[0.0; 6]; 6];
    for (i, row) in p.iter_mut().enumerate().take(3) {
        row[i] = 1e-8;
    }
    let s = TrackState::new([0.0; 6], Covariance6::new(p));
    let r = [[1e-8, 0.0, 0.0], [0.0, 1e-8, 0.0], [0.0, 0.0, 1e-8]];
    let d2 = mgr.distance2(&s, Vector3::new(1e-4, 0.0, 0.0), &r).unwrap();
    assert!((d2 - 0.5).abs() < 1e-9);
    // A non-positive-definite innovation giving d² < 0 is rejected.
    let mgr2 = manager(Dimension::TwoD);
    let mut q = [[0.0; 6]; 6];
    q[0][0] = -1.0;
    q[1][1] = 1.0;
    let s = TrackState::new([0.0; 6], Covariance6::new(q));
    assert_eq!(
        mgr2.distance2(&s, Vector3::new(1.0, 0.0, 0.0), &[[0.0; 3]; 3]),
        None
    );
}
