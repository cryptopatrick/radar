//! Scan patterns: how the beam moves between updates.

use crate::geometry::angle::{wrap_pi, wrap_two_pi};
use crate::radar::antenna::AntennaConfig;
use crate::radar::config::Dimension;
use std::f64::consts::{PI, TAU};

/// Tolerance absorbing float rounding in completion and edge tests.
const EPS: f64 = 1e-9;

/// A user-supplied scan pattern.
pub trait ScanPattern: Send + std::fmt::Debug {
    /// Advances the beam by `dt_secs` seconds, updating `state`, and returns
    /// the arc swept during the interval.
    fn advance(&mut self, state: &mut ScanState, dt_secs: f64) -> Sweep;
}

/// How the beam moves. Angles are radians in the scan frame
/// ([`ScanFrame`](crate::radar::config::ScanFrame)); rates are rad/s.
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ScanConfig {
    /// A stationary beam.
    Fixed {
        /// Boresight azimuth.
        azimuth: f64,
        /// Boresight elevation.
        elevation: f64,
    },
    /// Continuous rotation at a constant rate.
    Circular {
        /// Rotation rate.
        rate: f64,
        /// Beam elevation.
        elevation: f64,
    },
    /// Back-and-forth sweep across `centre ± width/2`.
    Sector {
        /// Sector centre azimuth.
        centre: f64,
        /// Full sector width.
        width: f64,
        /// Sweep rate.
        rate: f64,
        /// Beam elevation.
        elevation: f64,
    },
    /// Sector sweeps stepping in elevation over `rows` rows.
    Raster {
        /// Centre azimuth of the raster.
        azimuth_centre: f64,
        /// Full azimuth span.
        azimuth_span: f64,
        /// Elevation of the lowest row.
        elevation_low: f64,
        /// Elevation of the highest row.
        elevation_high: f64,
        /// Number of rows.
        rows: u32,
        /// Azimuth sweep rate.
        rate: f64,
    },
    /// Circular rotation, one full turn per elevation bar.
    Volume {
        /// Elevation of each bar, visited in order.
        elevation_bars: Vec<f64>,
        /// Rotation rate.
        rate: f64,
    },
    /// The beam slews toward a host-commanded direction.
    Steered {
        /// Maximum slew rate.
        max_slew_rate: f64,
    },
    /// A user-supplied pattern. Not serialisable.
    #[cfg_attr(feature = "serde", serde(skip))]
    Custom(Box<dyn ScanPattern>),
}

impl ScanConfig {
    /// Clones the config, or returns `None` for [`ScanConfig::Custom`].
    pub(crate) fn try_clone(&self) -> Option<ScanConfig> {
        Some(match self {
            ScanConfig::Fixed { azimuth, elevation } => ScanConfig::Fixed {
                azimuth: *azimuth,
                elevation: *elevation,
            },
            ScanConfig::Circular { rate, elevation } => ScanConfig::Circular {
                rate: *rate,
                elevation: *elevation,
            },
            ScanConfig::Sector {
                centre,
                width,
                rate,
                elevation,
            } => ScanConfig::Sector {
                centre: *centre,
                width: *width,
                rate: *rate,
                elevation: *elevation,
            },
            ScanConfig::Raster {
                azimuth_centre,
                azimuth_span,
                elevation_low,
                elevation_high,
                rows,
                rate,
            } => ScanConfig::Raster {
                azimuth_centre: *azimuth_centre,
                azimuth_span: *azimuth_span,
                elevation_low: *elevation_low,
                elevation_high: *elevation_high,
                rows: *rows,
                rate: *rate,
            },
            ScanConfig::Volume {
                elevation_bars,
                rate,
            } => ScanConfig::Volume {
                elevation_bars: elevation_bars.clone(),
                rate: *rate,
            },
            ScanConfig::Steered { max_slew_rate } => ScanConfig::Steered {
                max_slew_rate: *max_slew_rate,
            },
            ScanConfig::Custom(_) => return None,
        })
    }
}

/// Runtime state of a scan.
#[derive(Debug, Clone, PartialEq)]
pub struct ScanState {
    /// Current boresight azimuth (scan frame).
    pub azimuth: f64,
    /// Current boresight elevation (scan frame).
    pub elevation: f64,
    /// Pending steering command `(azimuth, elevation)`, if any.
    pub command: Option<(f64, f64)>,
    /// Sweep direction, `+1` or `−1` (sector and raster scans).
    pub direction: f64,
    /// Current elevation bar or raster row.
    pub bar: usize,
    /// Azimuth travelled since the last completed scan.
    pub accumulated: f64,
}

impl Default for ScanState {
    /// Boresight at (0, 0), no command, direction `+1`, bar 0.
    fn default() -> Self {
        Self {
            azimuth: 0.0,
            elevation: 0.0,
            command: None,
            direction: 1.0,
            bar: 0,
            accumulated: 0.0,
        }
    }
}

/// The arc swept by the boresight during one update.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sweep {
    /// Boresight azimuth at the start of the interval.
    pub start_azimuth: f64,
    /// Signed azimuth change over the interval, `|·| ≤ 2π`.
    pub delta_azimuth: f64,
    /// Beam elevation during the interval.
    pub elevation: f64,
    /// Whether a full scan completed during the interval.
    pub completed: bool,
}

/// Initial scan state for a config: Fixed at its pointing; Circular at
/// azimuth 0; Sector and Raster at the low azimuth edge moving `+1` (Raster
/// on its lowest row); Volume at azimuth 0 on the first bar; Steered at
/// (0, 0); Custom at [`ScanState::default`].
#[cfg_attr(not(test), allow(dead_code))] // Used by the radar sensor.
pub(crate) fn initial_state(cfg: &ScanConfig) -> ScanState {
    let (azimuth, elevation) = match cfg {
        ScanConfig::Fixed { azimuth, elevation } => (*azimuth, *elevation),
        ScanConfig::Circular { elevation, .. } => (0.0, *elevation),
        ScanConfig::Sector {
            centre,
            width,
            elevation,
            ..
        } => (centre - width / 2.0, *elevation),
        ScanConfig::Raster {
            azimuth_centre,
            azimuth_span,
            elevation_low,
            ..
        } => (azimuth_centre - azimuth_span / 2.0, *elevation_low),
        ScanConfig::Volume { elevation_bars, .. } => {
            (0.0, elevation_bars.first().copied().unwrap_or(0.0))
        }
        ScanConfig::Steered { .. } | ScanConfig::Custom(_) => (0.0, 0.0),
    };
    ScanState {
        azimuth: wrap_two_pi(azimuth),
        elevation,
        ..ScanState::default()
    }
}

/// Advances the beam by `dt_secs`, updating `state`, and returns the swept
/// arc. After the call `state.azimuth == wrap_two_pi(start + delta)`.
#[cfg_attr(not(test), allow(dead_code))] // Used by the radar sensor.
pub(crate) fn advance(cfg: &mut ScanConfig, state: &mut ScanState, dt_secs: f64) -> Sweep {
    let start = wrap_two_pi(state.azimuth);
    let elevation = state.elevation;
    let (delta, completed) = match cfg {
        ScanConfig::Fixed { azimuth, elevation } => {
            state.azimuth = wrap_two_pi(*azimuth);
            state.elevation = *elevation;
            return Sweep {
                start_azimuth: state.azimuth,
                delta_azimuth: 0.0,
                elevation: *elevation,
                completed: false,
            };
        }
        ScanConfig::Circular { rate, .. } => {
            let delta = *rate * dt_secs;
            (delta, full_turn(state, delta))
        }
        ScanConfig::Volume {
            elevation_bars,
            rate,
        } => {
            let delta = *rate * dt_secs;
            let mut completed = false;
            if full_turn(state, delta) {
                state.bar += 1;
                if state.bar >= elevation_bars.len() {
                    state.bar = 0;
                    completed = true;
                }
                if let Some(el) = elevation_bars.get(state.bar) {
                    state.elevation = *el;
                }
            }
            (delta, completed)
        }
        ScanConfig::Sector {
            centre,
            width,
            rate,
            ..
        } => sector_step(state, *centre, *width, *rate * dt_secs),
        ScanConfig::Raster {
            azimuth_centre,
            azimuth_span,
            elevation_low,
            elevation_high,
            rows,
            rate,
        } => {
            let (delta, reversed) =
                sector_step(state, *azimuth_centre, *azimuth_span, *rate * dt_secs);
            let mut completed = false;
            if reversed {
                let rows = (*rows).max(1) as usize;
                state.bar += 1;
                if state.bar >= rows {
                    state.bar = 0;
                    completed = true;
                }
                let frac = if rows > 1 {
                    state.bar as f64 / (rows - 1) as f64
                } else {
                    0.0
                };
                state.elevation = *elevation_low + (*elevation_high - *elevation_low) * frac;
            }
            (delta, completed)
        }
        ScanConfig::Steered { max_slew_rate } => {
            let Some((az, el)) = state.command else {
                state.azimuth = start;
                return Sweep {
                    start_azimuth: start,
                    delta_azimuth: 0.0,
                    elevation,
                    completed: false,
                };
            };
            let cap = (*max_slew_rate * dt_secs).abs();
            // wrap_pi returns (-π, π], so an exact 180° tie goes positive.
            let delta = wrap_pi(az - start).clamp(-cap, cap);
            state.elevation += (el - state.elevation).clamp(-cap, cap);
            (delta, false)
        }
        ScanConfig::Custom(pattern) => return pattern.advance(state, dt_secs),
    };
    state.azimuth = wrap_two_pi(start + delta);
    Sweep {
        start_azimuth: start,
        delta_azimuth: delta,
        elevation,
        completed,
    }
}

/// Adds `|delta|` to the accumulated travel; returns whether a full turn
/// completed (consuming 2π of it).
fn full_turn(state: &mut ScanState, delta: f64) -> bool {
    state.accumulated += delta.abs();
    if state.accumulated >= TAU - EPS {
        state.accumulated = (state.accumulated - TAU).max(0.0);
        true
    } else {
        false
    }
}

/// Moves the beam `travel` radians in `state.direction` within
/// `centre ± width/2`. A sweep that reaches an edge stops there and the
/// direction reverses for the next update. Returns `(delta, reversed)`.
fn sector_step(state: &mut ScanState, centre: f64, width: f64, travel: f64) -> (f64, bool) {
    let half = width.abs() / 2.0;
    let pos = wrap_pi(state.azimuth - centre).clamp(-half, half);
    let dir = if state.direction < 0.0 { -1.0 } else { 1.0 };
    let room = if dir > 0.0 { half - pos } else { pos + half };
    let travel = travel.abs();
    if travel >= room - EPS {
        state.direction = -dir;
        (dir * room, true)
    } else {
        (dir * travel, false)
    }
}

/// Largest azimuth (or slew) travel a scan can make in one update of
/// `update_secs`: `|rate|·dt` for Circular, Volume, Sector and Raster,
/// `|max_slew_rate|·dt` for Steered, 0 for Fixed and Custom (unknown).
/// Validation rejects configs where this exceeds 2π.
pub fn max_sweep_per_update(cfg: &ScanConfig, update_secs: f64) -> f64 {
    match cfg {
        ScanConfig::Circular { rate, .. }
        | ScanConfig::Volume { rate, .. }
        | ScanConfig::Sector { rate, .. }
        | ScanConfig::Raster { rate, .. } => rate.abs() * update_secs,
        ScanConfig::Steered { max_slew_rate } => max_slew_rate.abs() * update_secs,
        ScanConfig::Fixed { .. } | ScanConfig::Custom(_) => 0.0,
    }
}

/// Angular half-extent (radians) of a target of radius `extent_radius`
/// seen from `range`: `π` if `range <= extent_radius`, else
/// `asin(extent_radius / range)`.
pub fn half_extent(extent_radius: f64, range: f64) -> f64 {
    if range <= extent_radius {
        PI
    } else {
        (extent_radius / range).asin()
    }
}

/// Whether a sweep illuminates a target, with the off-boresight angles at
/// closest approach.
///
/// Arguments are in the scan frame. `off_az` is 0 if `target_az` lies in the
/// swept arc, else the wrap-aware distance to the nearer arc endpoint;
/// `off_el` is `|target_el − sweep.elevation|`. Both are reduced by
/// `half_extent` and clamped at 0. Returns `None` if `off_az` exceeds half
/// the azimuth beamwidth or, in [`Dimension::ThreeD`], `off_el` exceeds half
/// the elevation beamwidth.
pub fn illumination(
    sweep: &Sweep,
    target_az: f64,
    target_el: f64,
    half_extent: f64,
    antenna: &AntennaConfig,
    dim: Dimension,
) -> Option<(f64, f64)> {
    let span = sweep.delta_azimuth.abs();
    let off_az = if span >= TAU {
        0.0
    } else {
        // Offset of the target from the arc start, measured in the sweep direction.
        let rel = if sweep.delta_azimuth >= 0.0 {
            wrap_two_pi(target_az - sweep.start_azimuth)
        } else {
            wrap_two_pi(sweep.start_azimuth - target_az)
        };
        if rel <= span {
            0.0
        } else {
            (rel - span).min(TAU - rel)
        }
    };
    let off_az = (off_az - half_extent).max(0.0);
    let off_el = ((target_el - sweep.elevation).abs() - half_extent).max(0.0);
    if off_az > antenna.beamwidth_az / 2.0 {
        return None;
    }
    if dim == Dimension::ThreeD && off_el > antenna.beamwidth_el / 2.0 {
        return None;
    }
    Some((off_az, off_el))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radar::antenna::{AntennaConfig, BeamPattern};
    use crate::radar::config::{Dimension, RadarConfig};
    use crate::time::SimDuration;
    use crate::units::deg;
    use approx::assert_relative_eq;
    use std::f64::consts::{PI, TAU};

    fn beam(bw: f64) -> AntennaConfig {
        AntennaConfig {
            beamwidth_az: bw,
            beamwidth_el: bw,
            pattern: BeamPattern::Gaussian,
            sidelobe_level_db: -30.0,
        }
    }

    fn sweep(start: f64, delta: f64, elevation: f64) -> Sweep {
        Sweep {
            start_azimuth: start,
            delta_azimuth: delta,
            elevation,
            completed: false,
        }
    }

    #[test]
    fn circular_advances_and_completes() {
        let mut cfg = ScanConfig::Circular {
            rate: TAU,
            elevation: 0.0,
        };
        let mut state = initial_state(&cfg);
        let mut completions = Vec::new();
        for i in 1..=10 {
            let s = advance(&mut cfg, &mut state, 0.1);
            assert_relative_eq!(s.delta_azimuth, TAU * 0.1, epsilon = 1e-12);
            assert_relative_eq!(
                state.azimuth,
                wrap_two_pi(s.start_azimuth + s.delta_azimuth),
                epsilon = 1e-12
            );
            if s.completed {
                completions.push(i);
            }
        }
        assert_eq!(completions, vec![10]);
    }

    #[test]
    fn sweep_across_north() {
        let s = sweep(deg(359.0), deg(2.0), 0.0);
        let a = beam(deg(1.0));
        let hit = illumination(&s, 0.0, 0.0, 0.0, &a, Dimension::TwoD);
        assert_eq!(hit, Some((0.0, 0.0)));
        assert_eq!(
            illumination(&s, deg(5.0), 0.0, 0.0, &a, Dimension::TwoD),
            None
        );
        // Negative sweeps across north are handled the same way.
        let back = sweep(deg(1.0), deg(-2.0), 0.0);
        assert!(illumination(&back, deg(359.5), 0.0, 0.0, &a, Dimension::TwoD).is_some());
        assert!(illumination(&back, deg(355.0), 0.0, 0.0, &a, Dimension::TwoD).is_none());
    }

    #[test]
    fn illumination_reports_offset_to_nearer_endpoint() {
        let s = sweep(deg(10.0), deg(5.0), 0.0);
        let a = beam(deg(4.0));
        let (off_az, _) = illumination(&s, deg(16.0), 0.0, 0.0, &a, Dimension::TwoD).unwrap();
        assert_relative_eq!(off_az, deg(1.0), epsilon = 1e-12);
        let (off_az, _) = illumination(&s, deg(9.5), 0.0, 0.0, &a, Dimension::TwoD).unwrap();
        assert_relative_eq!(off_az, deg(0.5), epsilon = 1e-12);
        // Extent reduces the offset.
        let (off_az, _) = illumination(&s, deg(16.0), 0.0, deg(0.4), &a, Dimension::TwoD).unwrap();
        assert_relative_eq!(off_az, deg(0.6), epsilon = 1e-12);
    }

    #[test]
    fn negative_steer() {
        let mut cfg = ScanConfig::Steered {
            max_slew_rate: deg(45.0),
        };
        let mut state = ScanState {
            azimuth: deg(10.0),
            command: Some((deg(350.0), 0.0)),
            ..ScanState::default()
        };
        let s = advance(&mut cfg, &mut state, 1.0);
        assert_relative_eq!(s.start_azimuth, deg(10.0), epsilon = 1e-12);
        assert_relative_eq!(s.delta_azimuth, deg(-20.0), epsilon = 1e-12);
        assert_relative_eq!(state.azimuth, deg(350.0), epsilon = 1e-12);
        assert!(!s.completed);
    }

    #[test]
    fn steer_capped() {
        let mut cfg = ScanConfig::Steered {
            max_slew_rate: deg(45.0),
        };
        let mut state = ScanState {
            command: Some((PI, deg(80.0))),
            ..ScanState::default()
        };
        let s = advance(&mut cfg, &mut state, 1.0);
        assert_relative_eq!(s.delta_azimuth, deg(45.0), epsilon = 1e-12);
        assert_relative_eq!(state.elevation, deg(45.0), epsilon = 1e-12);
        // No command: the beam holds.
        state.command = None;
        let s = advance(&mut cfg, &mut state, 1.0);
        assert_eq!(s.delta_azimuth, 0.0);
        assert_relative_eq!(state.azimuth, deg(45.0), epsilon = 1e-12);
    }

    #[test]
    fn zero_range_target() {
        assert_eq!(half_extent(18.0, 0.0), PI);
        assert_eq!(half_extent(18.0, 10.0), PI);
        assert_relative_eq!(half_extent(18.0, 36.0), (0.5f64).asin(), epsilon = 1e-12);
        let s = sweep(deg(90.0), deg(1.0), 0.0);
        let a = beam(deg(1.0));
        let he = half_extent(18.0, 0.0);
        assert_eq!(
            illumination(&s, deg(270.0), deg(-60.0), he, &a, Dimension::ThreeD),
            Some((0.0, 0.0))
        );
    }

    #[test]
    fn elevation_gate_3d() {
        let s = sweep(0.0, deg(1.0), deg(2.0));
        let a = beam(deg(2.0));
        assert_eq!(
            illumination(&s, deg(0.5), deg(7.0), 0.0, &a, Dimension::ThreeD),
            None
        );
        assert!(illumination(&s, deg(0.5), deg(7.0), 0.0, &a, Dimension::TwoD).is_some());
    }

    #[test]
    fn sector_reverses() {
        let mut cfg = ScanConfig::Sector {
            centre: 0.0,
            width: deg(20.0),
            rate: deg(10.0),
            elevation: deg(1.0),
        };
        let mut state = initial_state(&cfg);
        assert_relative_eq!(state.azimuth, deg(350.0), epsilon = 1e-12);
        let s1 = advance(&mut cfg, &mut state, 1.0);
        assert_relative_eq!(s1.delta_azimuth, deg(10.0), epsilon = 1e-12);
        assert_relative_eq!(s1.elevation, deg(1.0), epsilon = 1e-12);
        assert!(!s1.completed);
        let s2 = advance(&mut cfg, &mut state, 1.0);
        assert_relative_eq!(s2.delta_azimuth, deg(10.0), epsilon = 1e-12);
        assert!(s2.completed);
        assert_eq!(state.direction, -1.0);
        assert_relative_eq!(state.azimuth, deg(10.0), epsilon = 1e-12);
        let s3 = advance(&mut cfg, &mut state, 1.5);
        assert_relative_eq!(s3.delta_azimuth, deg(-15.0), epsilon = 1e-12);
        assert!(!s3.completed);
        // The sweep stops at the edge and reverses.
        let s4 = advance(&mut cfg, &mut state, 1.0);
        assert_relative_eq!(s4.delta_azimuth, deg(-5.0), epsilon = 1e-12);
        assert!(s4.completed);
        assert_eq!(state.direction, 1.0);
    }

    #[test]
    fn raster_steps_rows() {
        let mut cfg = ScanConfig::Raster {
            azimuth_centre: deg(90.0),
            azimuth_span: deg(10.0),
            elevation_low: deg(0.0),
            elevation_high: deg(4.0),
            rows: 3,
            rate: deg(10.0),
        };
        let mut state = initial_state(&cfg);
        let mut els = Vec::new();
        let mut completions = Vec::new();
        for i in 1..=4 {
            let s = advance(&mut cfg, &mut state, 1.0);
            els.push(s.elevation);
            if s.completed {
                completions.push(i);
            }
        }
        assert_relative_eq!(els[0], 0.0, epsilon = 1e-12);
        assert_relative_eq!(els[1], deg(2.0), epsilon = 1e-12);
        assert_relative_eq!(els[2], deg(4.0), epsilon = 1e-12);
        assert_relative_eq!(els[3], 0.0, epsilon = 1e-12);
        assert_eq!(completions, vec![3]);
    }

    #[test]
    fn volume_steps_bars() {
        let mut cfg = ScanConfig::Volume {
            elevation_bars: vec![deg(1.0), deg(3.0)],
            rate: PI,
        };
        let mut state = initial_state(&cfg);
        assert_relative_eq!(state.elevation, deg(1.0), epsilon = 1e-12);
        let mut els = Vec::new();
        let mut completions = Vec::new();
        for i in 1..=8 {
            let s = advance(&mut cfg, &mut state, 1.0);
            els.push(s.elevation);
            if s.completed {
                completions.push(i);
            }
        }
        assert_relative_eq!(els[1], deg(1.0), epsilon = 1e-12);
        assert_relative_eq!(els[2], deg(3.0), epsilon = 1e-12);
        assert_relative_eq!(els[3], deg(3.0), epsilon = 1e-12);
        assert_relative_eq!(els[4], deg(1.0), epsilon = 1e-12);
        assert_eq!(completions, vec![4, 8]);
    }

    #[test]
    fn fixed_holds() {
        let mut cfg = ScanConfig::Fixed {
            azimuth: deg(30.0),
            elevation: deg(2.0),
        };
        let mut state = initial_state(&cfg);
        let s = advance(&mut cfg, &mut state, 1.0);
        assert_eq!(s.delta_azimuth, 0.0);
        assert_relative_eq!(s.start_azimuth, deg(30.0), epsilon = 1e-12);
        assert!(!s.completed);
    }

    #[test]
    fn validation_rejects_fast_scan() {
        let r = RadarConfig::builder()
            .scan(ScanConfig::Circular {
                rate: 30.0,
                elevation: 0.0,
            })
            .update_period(SimDuration::from_secs(1))
            .build();
        assert!(matches!(
            r,
            Err(crate::error::RadarError::InvalidConfig { field: "scan", .. })
        ));
        assert_relative_eq!(
            max_sweep_per_update(&ScanConfig::Steered { max_slew_rate: 2.0 }, 0.5),
            1.0
        );
        assert!(RadarConfig::builder().build().is_ok());
    }
}
