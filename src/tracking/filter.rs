//! Tracking filters: constant-velocity Kalman (with a radial-velocity EKF
//! update) and alpha-beta.

use super::matrix::{Matrix, inverse3};
use crate::geometry::Vector3;
use std::fmt::Debug;

/// A 6×6 state covariance over `[x, y, z, vx, vy, vz]` (m, m/s).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Covariance6([[f64; 6]; 6]);

impl Covariance6 {
    /// Wraps a row-major 6×6 covariance (for custom [`Tracker`]s).
    pub fn new(m: [[f64; 6]; 6]) -> Self {
        Covariance6(m)
    }

    /// Element `(i, j)`, or `NaN` if `i` or `j` is 6 or more.
    pub fn get(&self, i: usize, j: usize) -> f64 {
        self.0
            .get(i)
            .and_then(|row| row.get(j))
            .copied()
            .unwrap_or(f64::NAN)
    }

    /// The position block, m².
    pub fn position(&self) -> [[f64; 3]; 3] {
        self.block(0)
    }

    /// The velocity block, (m/s)².
    pub fn velocity(&self) -> [[f64; 3]; 3] {
        self.block(3)
    }

    /// Square root of the trace of the position block, m.
    pub fn position_sigma(&self) -> f64 {
        (self.0[0][0] + self.0[1][1] + self.0[2][2]).sqrt()
    }

    fn block(&self, o: usize) -> [[f64; 3]; 3] {
        let mut b = [[0.0; 3]; 3];
        for (i, row) in b.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v = self.0[o + i][o + j];
            }
        }
        b
    }

    fn matrix(&self) -> Matrix<6, 6> {
        Matrix(self.0)
    }
}

/// A filter state: position and velocity `[x, y, z, vx, vy, vz]` and its
/// covariance.
#[derive(Debug, Clone, PartialEq)]
pub struct TrackState {
    /// State vector `[x, y, z, vx, vy, vz]`, m and m/s.
    pub x: [f64; 6],
    /// State covariance.
    pub p: Covariance6,
    /// Seconds predicted since the last measurement update.
    pub elapsed: f64,
}

impl TrackState {
    /// A freshly updated state (`elapsed` 0).
    pub fn new(x: [f64; 6], p: Covariance6) -> Self {
        TrackState { x, p, elapsed: 0.0 }
    }

    fn column(&self) -> Matrix<6, 1> {
        Matrix(self.x.map(|v| [v]))
    }

    fn from_parts(x: Matrix<6, 1>, p: Matrix<6, 6>, elapsed: f64) -> TrackState {
        TrackState {
            x: x.0.map(|[v]| v),
            p: Covariance6(p.0),
            elapsed,
        }
    }

    fn is_finite(&self) -> bool {
        self.x
            .iter()
            .chain(self.p.0.iter().flatten())
            .all(|v| v.is_finite())
    }
}

/// A tracking filter. Implementations must be pure functions of their
/// inputs so runs replay and one tracker can serve many tracks; per-track
/// timing lives in [`TrackState::elapsed`].
pub trait Tracker: Send + Debug {
    /// Propagates `s` forward by `dt` seconds. Built-in filters add `dt` to
    /// `elapsed`; a `dt` that is not finite and positive is a no-op.
    fn predict(&self, s: &TrackState, dt: f64) -> TrackState;

    /// Updates `s` with a world-frame position measurement `z` of covariance
    /// `r` (m²). Built-in filters reset `elapsed` to 0 (a skipped update
    /// returns `s` unchanged).
    fn update_position(&self, s: &TrackState, z: Vector3, r: [[f64; 3]; 3]) -> TrackState;

    /// Updates `s` with a radial velocity `rv` (m/s, positive = opening) of
    /// standard deviation `sigma`, measured relative to a sensor at
    /// `sensor_pos` moving at `sensor_vel`. The default ignores the
    /// measurement.
    fn update_radial_velocity(
        &self,
        s: &TrackState,
        rv: f64,
        sigma: f64,
        sensor_pos: Vector3,
        sensor_vel: Vector3,
    ) -> TrackState {
        let _ = (rv, sigma, sensor_pos, sensor_vel);
        s.clone()
    }
}

/// The constant-velocity transition `F` for `dt`.
fn transition(dt: f64) -> Matrix<6, 6> {
    let mut f = Matrix::<6, 6>::identity();
    for i in 0..3 {
        f.0[i][i + 3] = dt;
    }
    f
}

/// Kalman update with a linear(ised) measurement matrix `h`, innovation `y`
/// and inverse innovation covariance `s_inv`, using the Joseph form.
fn kalman_update<const M: usize>(
    s: &TrackState,
    h: Matrix<M, 6>,
    y: Matrix<M, 1>,
    r: Matrix<M, M>,
    s_inv: Matrix<M, M>,
) -> TrackState {
    let p = s.p.matrix();
    let k = p * h.transpose() * s_inv;
    let x = s.column() + k * y;
    let ikh = Matrix::<6, 6>::identity() - k * h;
    let p = ikh * p * ikh.transpose() + k * r * k.transpose();
    let next = TrackState::from_parts(x, p, 0.0);
    if next.is_finite() { next } else { s.clone() }
}

/// Constant-velocity Kalman filter with white-acceleration process noise and
/// an extended (EKF) radial-velocity update.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KalmanCv {
    /// Process-noise acceleration sigma, m/s².
    pub process_accel_sigma: f64,
}

impl Tracker for KalmanCv {
    /// `x' = F·x`, `P' = F·P·Fᵀ + Q` with per-axis
    /// `Q = q²·[[dt⁴/4, dt³/2], [dt³/2, dt²]]`. A `dt` that is not finite
    /// and positive leaves the state unchanged; a non-finite result returns
    /// the input state with only `elapsed` advanced.
    fn predict(&self, s: &TrackState, dt: f64) -> TrackState {
        if !dt.is_finite() || dt <= 0.0 {
            return s.clone();
        }
        let f = transition(dt);
        let q2 = self.process_accel_sigma * self.process_accel_sigma;
        let mut q = Matrix::<6, 6>::zeros();
        for i in 0..3 {
            q.0[i][i] = q2 * dt.powi(4) / 4.0;
            q.0[i][i + 3] = q2 * dt.powi(3) / 2.0;
            q.0[i + 3][i] = q2 * dt.powi(3) / 2.0;
            q.0[i + 3][i + 3] = q2 * dt * dt;
        }
        let p = f * s.p.matrix() * f.transpose() + q;
        let elapsed = s.elapsed + dt;
        let next = TrackState::from_parts(f * s.column(), p, elapsed);
        if next.is_finite() {
            next
        } else {
            TrackState {
                elapsed,
                ..s.clone()
            }
        }
    }

    /// Linear update with `H = [I 0]`. Skipped (state returned unchanged)
    /// if the innovation covariance is singular or anything is non-finite.
    fn update_position(&self, s: &TrackState, z: Vector3, r: [[f64; 3]; 3]) -> TrackState {
        if !z.is_finite() {
            return s.clone();
        }
        let mut h = Matrix::<3, 6>::zeros();
        for (i, row) in h.0.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        let r = Matrix(r);
        let Some(s_inv) = inverse3(&(h * s.p.matrix() * h.transpose() + r)) else {
            return s.clone();
        };
        let y = Matrix([[z.x - s.x[0]], [z.y - s.x[1]], [z.z - s.x[2]]]);
        kalman_update(s, h, y, r, s_inv)
    }

    /// EKF update with `h(x) = u·w` and `H = [(w − (u·w)·u)/r, u]`, where
    /// `w = v − sensor_vel` is the velocity relative to the sensor, `u` the
    /// line-of-sight unit vector from `sensor_pos` to the predicted position
    /// and `r` its range. Skipped if `r < 1e-6`, the innovation variance is
    /// not positive, or anything is non-finite.
    fn update_radial_velocity(
        &self,
        s: &TrackState,
        rv: f64,
        sigma: f64,
        sensor_pos: Vector3,
        sensor_vel: Vector3,
    ) -> TrackState {
        if !rv.is_finite()
            || !sigma.is_finite()
            || !sensor_pos.is_finite()
            || !sensor_vel.is_finite()
        {
            return s.clone();
        }
        let los = Vector3::new(s.x[0], s.x[1], s.x[2]) - sensor_pos;
        let range = los.length();
        if !range.is_finite() || range < 1e-6 {
            return s.clone();
        }
        let u = los * (1.0 / range);
        let v = Vector3::new(s.x[3], s.x[4], s.x[5]) - sensor_vel;
        let uv = u.dot(v);
        let dpos = (v - u * uv) * (1.0 / range);
        let h = Matrix([[dpos.x, dpos.y, dpos.z, u.x, u.y, u.z]]);
        let r = Matrix([[sigma * sigma]]);
        let innovation_var = (h * s.p.matrix() * h.transpose() + r).0[0][0];
        if !innovation_var.is_finite() || innovation_var <= 0.0 {
            return s.clone();
        }
        kalman_update(
            s,
            h,
            Matrix([[rv - uv]]),
            r,
            Matrix([[1.0 / innovation_var]]),
        )
    }
}

/// Alpha-beta filter. Its covariance is approximate: after an update each
/// block is the larger (by trace) of the nominal value `P_pos = α·R`,
/// `P_vel = (β/dt)²·R` and the predicted block scaled by `ρ²`, where `ρ` is
/// the spectral radius of the closed loop
/// `λ² − (2 − α − β)·λ + (1 − α) = 0` (the rate at which estimate errors
/// decay). This keeps a fresh track's large start-up uncertainty while it
/// decays. Cross terms are zero; `dt` is the track's
/// [`TrackState::elapsed`] (1 s if that is not finite and positive).
///
/// With a constant `dt`, the velocity block settles at `(β/dt)²·R`. The
/// position block settles at `α·R` only if `ρ²·(α + β²) ≤ α` (roughly
/// `β ≲ α`, e.g. α = 0.3, β = 0.05). Otherwise it settles at the larger,
/// conservative fixed point `ρ²·β²·R / (1 − ρ²)`: for α = 0.2, β = 0.5
/// (`ρ² = 0.8`) that is `1.0·R` rather than `0.2·R`. Radial velocity is
/// ignored.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AlphaBeta {
    /// Position gain, in `(0, 1]`.
    pub alpha: f64,
    /// Velocity gain, in `(0, 2)`.
    pub beta: f64,
}

impl AlphaBeta {
    /// Per-update variance decay `ρ²` of the error dynamics (at most 1).
    fn transient_decay(&self) -> f64 {
        let a = 2.0 - self.alpha - self.beta;
        let c = 1.0 - self.alpha;
        let disc = a * a - 4.0 * c;
        let rho2 = if disc < 0.0 {
            c
        } else {
            let rho = ((a.abs() + disc.sqrt()) / 2.0).max(0.0);
            rho * rho
        };
        rho2.clamp(0.0, 1.0)
    }
}

impl Tracker for AlphaBeta {
    /// `x' = F·x`, `P' = F·P·Fᵀ` (no process noise), `elapsed += dt`. A `dt`
    /// that is not finite and positive leaves the state unchanged; a
    /// non-finite result returns the input state with only `elapsed`
    /// advanced.
    fn predict(&self, s: &TrackState, dt: f64) -> TrackState {
        if !dt.is_finite() || dt <= 0.0 {
            return s.clone();
        }
        let f = transition(dt);
        let elapsed = s.elapsed + dt;
        let next =
            TrackState::from_parts(f * s.column(), f * s.p.matrix() * f.transpose(), elapsed);
        if next.is_finite() {
            next
        } else {
            TrackState {
                elapsed,
                ..s.clone()
            }
        }
    }

    /// `pos += α·(z − pos)`, `vel += (β/dt)·(z − pos)`. Skipped if anything
    /// is non-finite.
    fn update_position(&self, s: &TrackState, z: Vector3, r: [[f64; 3]; 3]) -> TrackState {
        let dt = if s.elapsed.is_finite() && s.elapsed > 0.0 {
            s.elapsed
        } else {
            1.0
        };
        let g = self.beta / dt;
        let resid = [z.x - s.x[0], z.y - s.x[1], z.z - s.x[2]];
        let mut next = s.clone();
        for (i, e) in resid.iter().enumerate() {
            next.x[i] += self.alpha * e;
            next.x[i + 3] += g * e;
        }
        // Per block: steady state, or the decaying predicted uncertainty
        // while it is larger (so a fresh track's gate follows convergence).
        let trace = |m: &[[f64; 3]; 3]| m[0][0] + m[1][1] + m[2][2];
        let scaled = |m: [[f64; 3]; 3], k: f64| m.map(|row| row.map(|v| v * k));
        let block = |steady: [[f64; 3]; 3], predicted: [[f64; 3]; 3], decay: f64| {
            let transient = scaled(predicted, decay);
            if trace(&transient) > trace(&steady) {
                transient
            } else {
                steady
            }
        };
        let decay = self.transient_decay();
        let pos = block(scaled(r, self.alpha), s.p.position(), decay);
        let vel = block(scaled(r, g * g), s.p.velocity(), decay);
        let mut p = [[0.0; 6]; 6];
        for i in 0..3 {
            for j in 0..3 {
                p[i][j] = pos[i][j];
                p[i + 3][j + 3] = vel[i][j];
            }
        }
        next.p = Covariance6(p);
        next.elapsed = 0.0;
        if next.is_finite() { next } else { s.clone() }
    }
}

/// Chi-square threshold enclosing `probability` mass with `dof` degrees of
/// freedom: closed form `−2·ln(1 − p)` for 2, bisection on the CDF
/// `erf(√(x/2)) − √(2x/π)·e^(−x/2)` for 3. `NaN` for other `dof` or `p`
/// outside `(0, 1)`.
pub(crate) fn chi2_gate(probability: f64, dof: u32) -> f64 {
    if !(probability > 0.0 && probability < 1.0) {
        return f64::NAN;
    }
    match dof {
        2 => -2.0 * (1.0 - probability).ln(),
        3 => {
            let cdf = |x: f64| {
                1.0 - crate::radar::calibration::erfc((x / 2.0).sqrt())
                    - (2.0 * x / std::f64::consts::PI).sqrt() * (-x / 2.0).exp()
            };
            let (mut lo, mut hi) = (0.0_f64, 1000.0_f64);
            for _ in 0..200 {
                let mid = 0.5 * (lo + hi);
                if cdf(mid) < probability {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            0.5 * (lo + hi)
        }
        _ => f64::NAN,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::random::sample::standard_normal;
    use crate::random::{ChaChaStream, Seed, StreamId};

    const TRUTH_V: Vector3 = Vector3::new(10.0, 5.0, 0.0);
    const START: Vector3 = Vector3::new(1000.0, 2000.0, 100.0);
    const SENSOR: Vector3 = Vector3::new(0.0, 0.0, 0.0);

    fn rng() -> ChaChaStream {
        ChaChaStream::new(Seed(42), StreamId::Scenario)
    }

    fn noisy(truth: Vector3, sigma: f64, r: &mut ChaChaStream) -> Vector3 {
        Vector3::new(
            truth.x + sigma * standard_normal(r),
            truth.y + sigma * standard_normal(r),
            truth.z + sigma * standard_normal(r),
        )
    }

    fn diag(s: f64) -> [[f64; 3]; 3] {
        [[s, 0.0, 0.0], [0.0, s, 0.0], [0.0, 0.0, s]]
    }

    fn initial(pos: Vector3) -> TrackState {
        let mut p = [[0.0; 6]; 6];
        for i in 0..3 {
            p[i][i] = 25.0;
            p[i + 3][i + 3] = 300.0 * 300.0;
        }
        TrackState::new([pos.x, pos.y, pos.z, 0.0, 0.0, 0.0], Covariance6::new(p))
    }

    fn velocity_error(s: &TrackState) -> f64 {
        (Vector3::new(s.x[3], s.x[4], s.x[5]) - TRUTH_V).length()
    }

    fn trace(m: [[f64; 3]; 3]) -> f64 {
        m[0][0] + m[1][1] + m[2][2]
    }

    /// Runs `n` 1 s updates; returns the final state and the position-trace
    /// history after each update.
    fn run(tracker: &dyn Tracker, n: usize, radial_sigma: Option<f64>) -> (TrackState, Vec<f64>) {
        let mut r = rng();
        let mut s = initial(noisy(START, 5.0, &mut r));
        let mut traces = Vec::new();
        for k in 1..=n {
            let truth = START + TRUTH_V * k as f64;
            s = tracker.predict(&s, 1.0);
            s = tracker.update_position(&s, noisy(truth, 5.0, &mut r), diag(25.0));
            if let Some(sig) = radial_sigma {
                let u = (truth - SENSOR).normalized().unwrap();
                let rv = TRUTH_V.dot(u) + sig * standard_normal(&mut r);
                s = tracker.update_radial_velocity(&s, rv, sig, SENSOR, Vector3::ZERO);
            }
            traces.push(trace(s.p.position()));
        }
        (s, traces)
    }

    #[test]
    fn chi2_values() {
        assert!((chi2_gate(0.99, 2) - 9.2103).abs() < 1e-3);
        assert!((chi2_gate(0.99, 3) - 11.345).abs() < 1e-3);
    }

    #[test]
    fn kalman_converges_cv() {
        let kf = KalmanCv {
            process_accel_sigma: 0.1,
        };
        let (s, traces) = run(&kf, 50, None);
        assert!(
            velocity_error(&s) < 1.0,
            "velocity error {}",
            velocity_error(&s)
        );
        assert!(traces.windows(2).all(|w| w[1] <= w[0] + 1e-9), "{traces:?}");
        assert!(traces[49] < traces[0]);
        assert!(s.x.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn radial_velocity_improves_velocity() {
        let kf = KalmanCv {
            process_accel_sigma: 0.1,
        };
        let (without, _) = run(&kf, 5, None);
        let (with, _) = run(&kf, 5, Some(0.5));
        assert!(
            velocity_error(&with) < velocity_error(&without),
            "with {} without {}",
            velocity_error(&with),
            velocity_error(&without)
        );
    }

    #[test]
    fn radial_update_skips_degenerate_inputs() {
        let kf = KalmanCv {
            process_accel_sigma: 0.1,
        };
        let s = initial(SENSOR);
        assert_eq!(
            kf.update_radial_velocity(&s, 5.0, 0.5, SENSOR, Vector3::ZERO),
            s
        );
        let s = initial(START);
        assert_eq!(
            kf.update_radial_velocity(&s, f64::NAN, 0.5, SENSOR, Vector3::ZERO),
            s
        );
        assert_eq!(
            kf.update_position(&s, START, [[f64::NAN; 3]; 3]),
            s,
            "singular innovation covariance skips the update"
        );
    }

    #[test]
    fn radial_velocity_compensates_sensor_motion() {
        let kf = KalmanCv {
            process_accel_sigma: 0.1,
        };
        let sensor_vel = Vector3::new(0.0, 50.0, 0.0);
        let s = TrackState::new(
            [START.x, START.y, START.z, TRUTH_V.x, TRUTH_V.y, TRUTH_V.z],
            initial(START).p,
        );
        let u = (START - SENSOR).normalized().unwrap();
        let rv = (TRUTH_V - sensor_vel).dot(u);
        let next = kf.update_radial_velocity(&s, rv, 0.5, SENSOR, sensor_vel);
        // A perfect relative radial velocity has zero innovation.
        for (a, b) in next.x.iter().zip(&s.x) {
            assert!((a - b).abs() < 1e-9, "{:?} vs {:?}", next.x, s.x);
        }
        assert!(next.p.velocity()[1][1] < s.p.velocity()[1][1]);
        // Ignoring the sensor motion would pull the velocity.
        let biased = kf.update_radial_velocity(&s, rv, 0.5, SENSOR, Vector3::ZERO);
        assert!((biased.x[4] - s.x[4]).abs() > 1.0);
    }

    #[test]
    fn alpha_beta_covariance_follows_startup() {
        let ab = AlphaBeta {
            alpha: 0.5,
            beta: 0.3,
        };
        // Complex poles: ρ² = 1 − α = 0.5. A fresh track's 300 m/s
        // velocity sigma decays by ρ² instead of collapsing.
        assert!((ab.transient_decay() - 0.5).abs() < 1e-12);
        let s = ab.predict(&initial(START), 4.0);
        let u = ab.update_position(&s, START, diag(25.0));
        let want = 0.5 * s.p.get(3, 3);
        assert!((u.p.get(3, 3) - want).abs() < 1e-6 * want);
        assert!(u.p.get(3, 3) > (0.3 / 4.0_f64).powi(2) * 25.0);
    }

    #[test]
    fn alpha_beta_position_fixed_point() {
        let run = |alpha: f64, beta: f64| {
            let ab = AlphaBeta { alpha, beta };
            let mut s = initial(START);
            for _ in 0..500 {
                s = ab.update_position(&ab.predict(&s, 1.0), START, diag(25.0));
            }
            (s.p.get(0, 0) / 25.0, s.p.get(3, 3) / 25.0)
        };
        // β ≲ α: back to the nominal α·R and β²·R.
        let (pos, vel) = run(0.5, 0.3);
        assert!((pos - 0.5).abs() < 1e-9 && (vel - 0.09).abs() < 1e-9);
        // ρ²(α + β²) > α: conservative fixed point ρ²β²/(1 − ρ²) = 1.0.
        let (pos, vel) = run(0.2, 0.5);
        assert!((pos - 1.0).abs() < 1e-6, "pos {pos}");
        assert!((vel - 0.25).abs() < 1e-9);
    }

    #[test]
    fn alpha_beta_tracks_cv() {
        // Near-critically damped gains (β ≈ α²/(2 − α)).
        let ab = AlphaBeta {
            alpha: 0.3,
            beta: 0.05,
        };
        let (s, _) = run(&ab, 50, None);
        assert!(
            velocity_error(&s) < 2.0,
            "velocity error {}",
            velocity_error(&s)
        );
        // Covariance after an update: P_pos = αR, P_vel = (β/dt)²R.
        assert!((s.p.get(0, 0) - 0.3 * 25.0).abs() < 1e-12);
        assert!((s.p.get(3, 3) - 0.0025 * 25.0).abs() < 1e-12);
        assert_eq!(s.p.get(0, 3), 0.0);
        // Radial update is a no-op.
        assert_eq!(
            ab.update_radial_velocity(&s, 3.0, 0.5, SENSOR, Vector3::ZERO),
            s
        );
    }

    #[test]
    fn predict_grows_covariance() {
        let s = initial(START);
        let kf = KalmanCv {
            process_accel_sigma: 1.0,
        };
        let p = kf.predict(&s, 2.0);
        assert!(trace(p.p.position()) > trace(s.p.position()));
        assert!(p.p.position_sigma() > s.p.position_sigma());
        assert_eq!(p.x[0], s.x[0]);
        let moving = TrackState::new([0.0, 0.0, 0.0, 1.0, 2.0, 3.0], s.p);
        let q = kf.predict(&moving, 2.0);
        assert_eq!(&q.x[..3], &[2.0, 4.0, 6.0]);
        // Zero process noise still grows position via velocity uncertainty.
        let still = KalmanCv {
            process_accel_sigma: 0.0,
        };
        assert!(still.predict(&s, 1.0).p.position_sigma() > s.p.position_sigma());
        // Alpha-beta prediction grows too.
        let ab = AlphaBeta {
            alpha: 0.5,
            beta: 0.2,
        };
        assert!(ab.predict(&s, 1.0).p.position_sigma() > s.p.position_sigma());
        // Non-finite dt leaves the state unchanged.
        assert_eq!(kf.predict(&s, f64::NAN), s);
    }

    #[test]
    fn alpha_beta_per_track_dt() {
        let ab = AlphaBeta {
            alpha: 0.3,
            beta: 0.05,
        };
        let z = START + Vector3::new(4.0, -2.0, 1.0);
        let a = ab.predict(&initial(START), 1.0);
        let b = ab.predict(&initial(START * 2.0), 3.0);
        assert_eq!((a.elapsed, b.elapsed), (1.0, 3.0));
        let a = ab.update_position(&a, z, diag(25.0));
        // A fresh single-track run with the same inputs gives the same result.
        let fresh = AlphaBeta {
            alpha: 0.3,
            beta: 0.05,
        };
        let solo = fresh.update_position(&fresh.predict(&initial(START), 1.0), z, diag(25.0));
        assert_eq!(a, solo);
        // Velocity gain β/1 applied to the residual (4, −2, 1).
        assert!((a.x[3] - 0.05 * 4.0).abs() < 1e-12);
        assert!((a.x[4] + 0.05 * 2.0).abs() < 1e-12);
        assert_eq!(a.elapsed, 0.0);
        // Elapsed accumulates over coasts: two 1.5 s predicts use dt = 3.
        let coast = ab.predict(&ab.predict(&initial(START), 1.5), 1.5);
        assert_eq!(coast.elapsed, 3.0);
        let c = ab.update_position(&coast, z, diag(25.0));
        assert!((c.x[3] - 0.05 / 3.0 * 4.0).abs() < 1e-12);
    }

    #[test]
    fn kalman_elapsed_and_nan_safety() {
        let kf = KalmanCv {
            process_accel_sigma: 0.1,
        };
        let s = kf.predict(&initial(START), 2.0);
        assert_eq!(s.elapsed, 2.0);
        assert_eq!(kf.update_position(&s, START, diag(25.0)).elapsed, 0.0);
        let rv = kf.update_radial_velocity(&s, 1.0, 0.5, SENSOR, Vector3::ZERO);
        assert_eq!(rv.elapsed, 0.0);
        assert_eq!(kf.predict(&s, 0.0), s);
        assert_eq!(kf.predict(&s, -1.0), s);
        // Overflowing prediction keeps the input state, elapsed advanced.
        let huge = kf.predict(&s, 1e300);
        assert_eq!(huge.x, s.x);
        assert_eq!(huge.p, s.p);
        assert_eq!(huge.elapsed, 2.0 + 1e300);
    }

    #[test]
    fn covariance_accessors() {
        let mut m = [[0.0; 6]; 6];
        for (i, row) in m.iter_mut().enumerate() {
            row[i] = (i + 1) as f64;
        }
        let c = Covariance6::new(m);
        assert_eq!(c.get(4, 4), 5.0);
        assert_eq!(
            c.position(),
            [[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]]
        );
        assert_eq!(c.velocity()[2][2], 6.0);
        assert!((c.position_sigma() - 6.0_f64.sqrt()).abs() < 1e-12);
        assert!(c.get(6, 0).is_nan());
        assert!(c.get(0, 6).is_nan());
    }
}
