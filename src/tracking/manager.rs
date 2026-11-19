//! Track management: measurement conversion, gating, greedy association,
//! M-of-N confirmation, coasting and deletion.

use super::filter::{Covariance6, TrackState, Tracker, chi2_gate};
use super::matrix::{Matrix, inverse3};
use super::{AlphaBeta, KalmanCv, TrackManagementConfig, TrackerConfig};
use crate::geometry::{Vector3, azimuth_of};
use crate::ids::{SensorId, TrackId, TransponderCode};
use crate::radar::config::Dimension;
use crate::radar::measurement::Measurement;
use crate::sensor::Observation;
use crate::time::SimTime;
use std::collections::{BTreeMap, VecDeque};
use std::f64::consts::PI;

/// Variance of the pinned vertical states (z in m², vz in (m/s)²) of 2D
/// tracks, and of the z component of a 2D measurement.
const VERTICAL_VARIANCE: f64 = 1e-6;

/// Lower bound on each diagonal element of a converted measurement
/// covariance, m², so noise-free radars still give invertible gates.
const MIN_POSITION_VARIANCE: f64 = 1e-6;

/// Where a track is in its lifecycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TrackStatus {
    /// Newly started; not yet confirmed by M-of-N.
    Tentative,
    /// Confirmed and updated on its last illumination.
    Confirmed,
    /// Confirmed, but missed on its last illumination(s).
    Coasting,
    /// Dropped; reported once in the update that deleted it.
    Deleted,
}

/// An anonymous track maintained by one radar's tracker.
///
/// `position`, `velocity` and `covariance` are the filtered estimate at
/// `last_update` (the last update with an associated measurement); use
/// [`Track::predict`] to extrapolate. Like every observation it carries no
/// target identity, only what the sensor could know.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Track {
    /// Track number, assigned monotonically from 1 per sensor.
    pub id: TrackId,
    /// The sensor maintaining the track.
    pub sensor: SensorId,
    /// Lifecycle status.
    pub status: TrackStatus,
    /// Estimated world position at `last_update`, m.
    pub position: Vector3,
    /// Estimated world velocity, m/s (`z` is 0 for 2D radars).
    pub velocity: Vector3,
    /// State covariance over `[x, y, z, vx, vy, vz]` at `last_update`.
    pub covariance: Covariance6,
    /// Most frequently received transponder code (ties go to the smallest).
    pub identity: Option<TransponderCode>,
    /// Time the track was started.
    pub first_seen: SimTime,
    /// Time of the last associated measurement; the estimate is valid here.
    pub last_update: SimTime,
    /// Total number of associated measurements.
    pub hits: u32,
    /// Illuminations without an associated measurement since the last hit.
    pub consecutive_misses: u32,
}

impl Track {
    /// The track with its position extrapolated at constant velocity to
    /// `at` (which may be earlier than `last_update`). Every other field,
    /// including the covariance and `last_update`, is unchanged, so
    /// predict from the original track rather than from a prediction.
    pub fn predict(&self, at: SimTime) -> Track {
        let dt =
            (i128::from(at.as_nanos()) - i128::from(self.last_update.as_nanos())) as f64 * 1e-9;
        Track {
            position: self.position + self.velocity * dt,
            ..self.clone()
        }
    }

    /// Course over ground: compass bearing of the velocity, rad in
    /// `[0, 2π)` (0 when stationary).
    pub fn course(&self) -> f64 {
        azimuth_of(self.velocity)
    }

    /// Speed, m/s. 2D tracks keep `velocity.z` at 0, so this is the
    /// horizontal speed for 2D radars and the full speed for 3D radars.
    pub fn speed(&self) -> f64 {
        self.velocity.length()
    }
}

/// Jacobian of `r · direction(az, el)` (compass convention: x = r·sin az·cos el,
/// y = r·cos az·cos el, z = r·sin el). Rows x, y, z; columns r, az, el.
fn jacobian(r: f64, az: f64, el: f64) -> [[f64; 3]; 3] {
    let (sa, ca) = az.sin_cos();
    let (se, ce) = el.sin_cos();
    [
        [sa * ce, r * ca * ce, -r * sa * se],
        [ca * ce, -r * sa * ce, -r * ca * se],
        [se, 0.0, r * ce],
    ]
}

/// Converts a polar measurement to a world position
/// `sensor_position + r · direction(az, el)` and its first-order covariance
/// `J · diag(σr², σaz², σel²) · Jᵀ`. In 2D the elevation is 0, the
/// elevation term is dropped and the z variance is 1e-6 m². The result is
/// symmetrised and each diagonal element is at least 1e-6 m².
pub(crate) fn polar_to_cartesian(m: &Measurement) -> (Vector3, [[f64; 3]; 3]) {
    let el = m.elevation.unwrap_or(0.0);
    let (r, az) = (m.range, m.azimuth);
    let j = Matrix(jacobian(r, az, el));
    let sigma_el = if m.elevation.is_some() {
        m.sigma.elevation
    } else {
        0.0
    };
    let mut d = Matrix::<3, 3>::zeros();
    d.0[0][0] = m.sigma.range * m.sigma.range;
    d.0[1][1] = m.sigma.azimuth * m.sigma.azimuth;
    d.0[2][2] = sigma_el * sigma_el;
    let jdj = j * d * j.transpose();
    // Exactly symmetric.
    let mut cov = ((jdj + jdj.transpose()).scale(0.5)).0;
    if m.elevation.is_none() {
        cov[2] = [0.0, 0.0, VERTICAL_VARIANCE];
        cov[0][2] = 0.0;
        cov[1][2] = 0.0;
    }
    for (i, row) in cov.iter_mut().enumerate() {
        row[i] = row[i].max(MIN_POSITION_VARIANCE);
    }
    let offset = Vector3::new(j.0[0][0], j.0[1][0], j.0[2][0]) * r;
    (m.sensor_position + offset, cov)
}

/// Runs `f` with the filter `cfg` selects; `None` for [`TrackerConfig::None`].
pub(crate) fn with_tracker<R>(cfg: &TrackerConfig, f: impl FnOnce(&dyn Tracker) -> R) -> Option<R> {
    match cfg {
        TrackerConfig::None => None,
        TrackerConfig::AlphaBeta { alpha, beta } => Some(f(&AlphaBeta {
            alpha: *alpha,
            beta: *beta,
        })),
        TrackerConfig::Kalman {
            process_accel_sigma,
        } => Some(f(&KalmanCv {
            process_accel_sigma: *process_accel_sigma,
        })),
        TrackerConfig::Custom(t) => Some(f(t.as_ref())),
    }
}

/// How the beam moved during one update, for dwell bookkeeping.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct BeamMotion {
    /// Whether the scan sweeps (Circular, Sector, Raster, Volume). For other
    /// scans every update is its own dwell.
    pub(crate) sweeping: bool,
    /// Azimuth swept this update, `|delta|`, rad.
    pub(crate) swept: f64,
}

/// A run of consecutive updates in which a track was in the beam.
#[derive(Debug, Clone, Copy, Default)]
struct Dwell {
    /// Azimuth swept since the dwell started, rad.
    swept: f64,
    /// Whether the dwell already produced a hit.
    hit: bool,
}

/// One live track and its bookkeeping.
#[derive(Debug, Clone)]
struct Record {
    id: TrackId,
    status: TrackStatus,
    /// Filter state, valid at `time` (the last hit).
    state: TrackState,
    time: SimTime,
    first_seen: SimTime,
    hits: u32,
    consecutive_misses: u32,
    /// The last N illuminated outcomes (true = hit), oldest first.
    window: VecDeque<bool>,
    codes: BTreeMap<TransponderCode, u32>,
    /// The current dwell (sweeping scans only).
    dwell: Option<Dwell>,
}

impl Record {
    /// Records one dwell outcome in the M-of-N window and applies
    /// confirmation; returns true if a tentative track fails M-of-N, i.e.
    /// can no longer reach M hits within its first N outcomes.
    fn record(&mut self, hit: bool, m: u32, n: u32) -> bool {
        self.window.push_back(hit);
        while self.window.len() > n as usize {
            self.window.pop_front();
        }
        if self.status != TrackStatus::Tentative {
            return false;
        }
        let hits = self.window.iter().filter(|h| **h).count();
        if hits >= m as usize {
            self.status = TrackStatus::Confirmed;
            false
        } else {
            // Fails as soon as the outcomes left in the window cannot reach M.
            let remaining = (n as usize).saturating_sub(self.window.len());
            hits + remaining < m as usize
        }
    }

    /// Records a miss; returns true if a tentative track fails M-of-N.
    fn miss(&mut self, m: u32, n: u32) -> bool {
        self.consecutive_misses = self.consecutive_misses.saturating_add(1);
        if self.status == TrackStatus::Confirmed {
            self.status = TrackStatus::Coasting;
        }
        self.record(false, m, n)
    }

    fn identity(&self) -> Option<TransponderCode> {
        // Ascending iteration with a strict comparison keeps the smallest
        // code on a tie.
        let mut best: Option<(TransponderCode, u32)> = None;
        for (&code, &count) in &self.codes {
            if best.is_none_or(|(_, c)| count > c) {
                best = Some((code, count));
            }
        }
        best.map(|(code, _)| code)
    }
}

/// Maintains one radar's tracks.
#[derive(Debug, Clone)]
pub(crate) struct TrackManager {
    sensor: SensorId,
    dim: Dimension,
    mgmt: TrackManagementConfig,
    /// χ² gate threshold on d².
    gate: f64,
    max_position_sigma: f64,
    /// Tentative tracks unhit for longer than this (s) are deleted.
    tentative_max_age: f64,
    next_id: u64,
    /// Live tracks in ascending id order.
    tracks: Vec<Record>,
}

impl TrackManager {
    /// Creates an empty manager. `instrumented_range` sets the default
    /// position-sigma limit (a tenth of it); a tentative track is deleted
    /// once more than `tentative_max_age` (default
    /// `(confirm_n + 1) × nominal_revisit` seconds) passes since its last
    /// hit.
    pub(crate) fn new(
        mgmt: &TrackManagementConfig,
        dim: Dimension,
        sensor: SensorId,
        instrumented_range: f64,
        nominal_revisit: f64,
    ) -> Self {
        let dof = if dim == Dimension::TwoD { 2 } else { 3 };
        Self {
            sensor,
            dim,
            mgmt: mgmt.clone(),
            gate: chi2_gate(mgmt.gate_probability, dof),
            max_position_sigma: mgmt.max_position_sigma.unwrap_or(instrumented_range / 10.0),
            tentative_max_age: mgmt.tentative_max_age.map_or_else(
                || f64::from(mgmt.confirm_n.saturating_add(1)) * nominal_revisit,
                |d| d.as_secs_f64(),
            ),
            next_id: 1,
            tracks: Vec::new(),
        }
    }

    /// Drops every track and restarts numbering at 1.
    pub(crate) fn reset(&mut self) {
        self.tracks.clear();
        self.next_id = 1;
    }

    /// Snapshots of the live tracks in id order.
    pub(crate) fn tracks(&self) -> Vec<Track> {
        self.tracks.iter().map(|r| self.snapshot(r)).collect()
    }

    fn snapshot(&self, r: &Record) -> Track {
        let x = &r.state.x;
        Track {
            id: r.id,
            sensor: self.sensor,
            status: r.status,
            position: Vector3::new(x[0], x[1], x[2]),
            velocity: Vector3::new(x[3], x[4], x[5]),
            covariance: r.state.p,
            identity: r.identity(),
            first_seen: r.first_seen,
            last_update: r.time,
            hits: r.hits,
            consecutive_misses: r.consecutive_misses,
        }
    }

    /// In 2D, pins vz to 0 and decouples z and vz with a tiny variance.
    fn pin(&self, s: TrackState) -> TrackState {
        if self.dim == Dimension::ThreeD {
            return s;
        }
        let mut x = s.x;
        x[5] = 0.0;
        let mut p = [[0.0; 6]; 6];
        for (i, row) in p.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                let vertical = matches!(i, 2 | 5) || matches!(j, 2 | 5);
                *v = match (vertical, i == j) {
                    (false, _) => s.p.get(i, j),
                    (true, true) => VERTICAL_VARIANCE,
                    (true, false) => 0.0,
                };
            }
        }
        TrackState {
            x,
            p: Covariance6::new(p),
            elapsed: s.elapsed,
        }
    }

    /// Squared Mahalanobis distance of `z` from the predicted position, over
    /// x, y in 2D and x, y, z in 3D; `None` if the innovation covariance is
    /// singular (`|det| ≤ 1e-12·∏|diagᵢ|`) or the result is not finite and
    /// non-negative.
    fn distance2(&self, s: &TrackState, z: Vector3, r: &[[f64; 3]; 3]) -> Option<f64> {
        let y = [z.x - s.x[0], z.y - s.x[1], z.z - s.x[2]];
        let sc = |i: usize, j: usize| s.p.get(i, j) + r[i][j];
        let d2 = if self.dim == Dimension::TwoD {
            let (a, b, c, d) = (sc(0, 0), sc(0, 1), sc(1, 0), sc(1, 1));
            let det = a * d - b * c;
            if !det.is_finite() || det.abs() <= 1e-12 * (a * d).abs() {
                return None;
            }
            (d * y[0] * y[0] - (b + c) * y[0] * y[1] + a * y[1] * y[1]) / det
        } else {
            let mut m = Matrix::<3, 3>::zeros();
            for (i, row) in m.0.iter_mut().enumerate() {
                for (j, v) in row.iter_mut().enumerate() {
                    *v = sc(i, j);
                }
            }
            let inv = inverse3(&m)?;
            let yv = Matrix([[y[0]], [y[1]], [y[2]]]);
            (yv.transpose() * inv * yv).0[0][0]
        };
        (d2.is_finite() && d2 >= 0.0).then_some(d2)
    }

    /// Applies an associated measurement to the predicted state `pred` and
    /// records a hit; returns true if the filter state was committed. A
    /// skipped position update (the filter returned `pred` unchanged) counts
    /// as a hit for M-of-N but leaves the last-hit state, time and misses
    /// untouched, so elapsed time is not double-counted.
    #[allow(clippy::too_many_arguments)]
    fn hit(
        &self,
        tracker: &dyn Tracker,
        rec: &mut Record,
        pred: &TrackState,
        m: &Measurement,
        (z, r): (Vector3, [[f64; 3]; 3]),
        time: SimTime,
        delete: &mut bool,
    ) -> bool {
        let updated = tracker.update_position(pred, z, r);
        let applied = updated != *pred;
        if applied {
            let mut s = updated;
            if let Some(rv) = m.radial_velocity {
                s = tracker.update_radial_velocity(
                    &s,
                    rv,
                    m.sigma.radial_velocity,
                    m.sensor_position,
                    m.sensor_velocity,
                );
            }
            rec.state = self.pin(s);
            rec.time = time;
            rec.consecutive_misses = 0;
            if rec.status == TrackStatus::Coasting {
                rec.status = TrackStatus::Confirmed;
            }
        }
        rec.hits = rec.hits.saturating_add(1);
        if let Some(code) = m.identity {
            *rec.codes.entry(code).or_insert(0) += 1;
        }
        *delete |= rec.record(true, self.mgmt.confirm_m, self.mgmt.confirm_n);
        applied
    }

    /// Processes one radar update at `time`: predicts, gates, associates,
    /// updates lifecycles and starts tracks, appending `TrackUpdated` for
    /// every created, updated or missed track (in id order; a track deleted
    /// by its miss is reported with status `Deleted`) and then
    /// `TrackDeleted` for every deleted track (in id order). `covered(p, sigma)`
    /// says whether the beam confidently illuminated a predicted position
    /// `p` with position sigma `sigma` this update; only covered tracks can
    /// miss.
    ///
    /// With a sweeping scan, a track's consecutive in-beam updates form one
    /// dwell with at most one outcome: the first associated measurement
    /// updates the filter; later ones in the same dwell are absorbed (no
    /// update, no new track); a miss is recorded when the dwell ends (the
    /// track leaves the beam, or π has been swept) without a hit. Otherwise
    /// every update is its own dwell.
    pub(crate) fn update(
        &mut self,
        tracker: &dyn Tracker,
        time: SimTime,
        beam: BeamMotion,
        measurements: &[Measurement],
        covered: impl Fn(Vector3, f64) -> bool,
        out: &mut Vec<Observation>,
    ) {
        // 1. Predict each track from its own last-hit time.
        let predicted: Vec<TrackState> = self
            .tracks
            .iter()
            .map(|t| {
                let dt = time.duration_since(t.time).map_or(0.0, |d| d.as_secs_f64());
                self.pin(tracker.predict(&t.state, dt))
            })
            .collect();
        let converted: Vec<(Vector3, [[f64; 3]; 3])> =
            measurements.iter().map(polar_to_cartesian).collect();

        // 2–3. Gate, then assign greedily in two passes: established
        // (Confirmed/Coasting) tracks first, then Tentative ones, each by
        // (d², track id, measurement index). A stale tentative's wide
        // covariance gives it small d², so it must not compete with
        // established tracks for their plots.
        let mut pairs = Vec::new();
        for (ti, (t, s)) in self.tracks.iter().zip(&predicted).enumerate() {
            let tentative = t.status == TrackStatus::Tentative;
            for (mi, (z, r)) in converted.iter().enumerate() {
                if let Some(d2) = self.distance2(s, *z, r) {
                    if d2 <= self.gate {
                        pairs.push((tentative, d2, t.id, mi, ti));
                    }
                }
            }
        }
        pairs.sort_by(|a, b| {
            a.0.cmp(&b.0)
                .then(a.1.total_cmp(&b.1))
                .then(a.2.cmp(&b.2))
                .then(a.3.cmp(&b.3))
        });
        let mut assigned: Vec<Option<usize>> = vec![None; self.tracks.len()];
        let mut used = vec![false; measurements.len()];
        for (_, _, _, mi, ti) in pairs {
            if assigned[ti].is_none() && !used[mi] {
                assigned[ti] = Some(mi);
                used[mi] = true;
            }
        }

        // 4–8. Dwell outcomes, confirmation and deletion.
        let (m_of_n, n) = (self.mgmt.confirm_m, self.mgmt.confirm_n);
        let mut deleted = Vec::new();
        let mut records = std::mem::take(&mut self.tracks);
        let mut kept = Vec::with_capacity(records.len() + measurements.len());
        for ((mut rec, pred), assignment) in records.drain(..).zip(predicted).zip(assigned) {
            let p_pred = Vector3::new(pred.x[0], pred.x[1], pred.x[2]);
            let in_cover = covered(p_pred, pred.p.position_sigma());
            let mut delete = false;
            let mut reported = false;
            let mut applied = false;
            let mut on_hit = |rec: &mut Record, delete: &mut bool| {
                if let Some(mi) = assignment {
                    let zr = converted[mi];
                    applied = self.hit(tracker, rec, &pred, &measurements[mi], zr, time, delete);
                }
            };
            if beam.sweeping {
                let in_beam = in_cover || assignment.is_some();
                if let Some(d) = rec.dwell {
                    if !in_beam || d.swept >= PI {
                        if !d.hit {
                            delete |= rec.miss(m_of_n, n);
                            reported = true;
                        }
                        rec.dwell = None;
                    }
                }
                if in_beam {
                    let d = rec.dwell.get_or_insert_with(Dwell::default);
                    d.swept += beam.swept;
                    if assignment.is_some() && !d.hit {
                        d.hit = true;
                        on_hit(&mut rec, &mut delete);
                        reported = true;
                    }
                }
            } else if assignment.is_some() {
                on_hit(&mut rec, &mut delete);
                reported = true;
            } else if in_cover {
                delete |= rec.miss(m_of_n, n);
                reported = true;
            }
            let sigma = if applied {
                rec.state.p.position_sigma()
            } else {
                pred.p.position_sigma()
            };
            // The sigma limit applies to established tracks only; tentative
            // tracks are governed by M-of-N (and a non-finite covariance).
            // Tentative tracks the beam no longer visits age out instead.
            let established = rec.status != TrackStatus::Tentative;
            let age = time
                .duration_since(rec.time)
                .map_or(0.0, |d| d.as_secs_f64());
            delete |= rec.consecutive_misses >= self.mgmt.max_coast
                || !sigma.is_finite()
                || (established && sigma > self.max_position_sigma)
                || (!established && age > self.tentative_max_age);
            if delete {
                rec.status = TrackStatus::Deleted;
                deleted.push(rec.id);
            }
            if reported {
                out.push(Observation::TrackUpdated(self.snapshot(&rec)));
            }
            if !delete {
                kept.push(rec);
            }
        }
        self.tracks = kept;

        // 9–10. Unassigned measurements start tentative tracks.
        for (mi, m) in measurements.iter().enumerate() {
            if used[mi] {
                continue;
            }
            let (z, r) = converted[mi];
            let v2 = self.mgmt.initial_velocity_sigma * self.mgmt.initial_velocity_sigma;
            let mut p = [[0.0; 6]; 6];
            for i in 0..3 {
                for j in 0..3 {
                    p[i][j] = r[i][j];
                }
                p[i + 3][i + 3] = v2;
            }
            let initial = TrackState::new([z.x, z.y, z.z, 0.0, 0.0, 0.0], Covariance6::new(p));
            let state = match m.radial_velocity {
                Some(rv) => tracker.update_radial_velocity(
                    &initial,
                    rv,
                    m.sigma.radial_velocity,
                    m.sensor_position,
                    m.sensor_velocity,
                ),
                None => initial,
            };
            let mut rec = Record {
                id: TrackId(self.next_id),
                status: TrackStatus::Tentative,
                state: self.pin(state),
                time,
                first_seen: time,
                hits: 1,
                consecutive_misses: 0,
                window: VecDeque::new(),
                codes: BTreeMap::new(),
                dwell: beam.sweeping.then_some(Dwell {
                    swept: beam.swept,
                    hit: true,
                }),
            };
            self.next_id += 1;
            if let Some(code) = m.identity {
                rec.codes.insert(code, 1);
            }
            rec.record(true, m_of_n, n);
            out.push(Observation::TrackUpdated(self.snapshot(&rec)));
            self.tracks.push(rec);
        }

        // 12. Deletions, in id order (already ascending).
        out.extend(deleted.into_iter().map(|id| Observation::TrackDeleted {
            sensor: self.sensor,
            id,
            time,
        }));
    }
}

#[cfg(test)]
#[path = "manager_tests.rs"]
mod tests;
