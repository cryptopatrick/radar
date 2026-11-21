//! Detection coverage and resolution maps for a passive receiver.

use super::{BistaticGeometry, Illuminator, PassiveReceiver};
use crate::error::RadarError;
use crate::geometry::angle::{azimuth_of, elevation_of, wrap_pi};
use crate::geometry::{Vector2, Vector3};
use crate::radar::{AntennaConfig, BeamPattern, Dimension};
use crate::units::{db_to_linear, deg, linear_to_db};

/// A surveillance antenna: peak gain and beam shape.
///
/// The presets are the three options compared in Section VI of
/// Gomez-del-Hoyo et al. (IRS 2022).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SurveillanceAntenna {
    /// Peak gain, dBi.
    pub gain_db: f64,
    /// Beamwidths, shape and sidelobe level.
    pub pattern: AntennaConfig,
}

impl SurveillanceAntenna {
    /// High-directivity parabolic antenna: 40 dBi, 2° beam.
    pub fn parabolic() -> Self {
        Self::gaussian(40.0, 2.0)
    }

    /// Wide-beam horn: 17 dBi, 30° beam.
    pub fn horn() -> Self {
        Self::gaussian(17.0, 30.0)
    }

    /// Digital array of five horns: 17 + 7 dBi, 30° instantaneous beam.
    pub fn horn_array() -> Self {
        Self::gaussian(24.0, 30.0)
    }

    fn gaussian(gain_db: f64, beamwidth_deg: f64) -> Self {
        Self {
            gain_db,
            pattern: AntennaConfig {
                beamwidth_az: deg(beamwidth_deg),
                beamwidth_el: deg(beamwidth_deg),
                pattern: BeamPattern::Gaussian,
                ..AntennaConfig::default()
            },
        }
    }

    /// Gain, dBi, towards `direction` (world frame) for a beam pointed at
    /// compass `azimuth` and `elevation` (radians).
    pub fn gain_db_towards(&self, azimuth: f64, elevation: f64, direction: Vector3) -> f64 {
        let off_az = wrap_pi(azimuth_of(direction) - azimuth);
        let off_el = elevation_of(direction) - elevation;
        self.gain_db + linear_to_db(self.pattern.gain(off_az, off_el, Dimension::ThreeD))
    }
}

/// A surveillance antenna pointed in a fixed direction.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SteeredAntenna {
    /// The antenna.
    pub antenna: SurveillanceAntenna,
    /// Boresight compass azimuth, radians.
    pub azimuth: f64,
    /// Boresight elevation, radians.
    pub elevation: f64,
}

impl PassiveReceiver {
    /// Largest receiver-to-target range, m, at which a target of `rcs_m2`
    /// reaches `required_snr_db` on boresight, holding the
    /// transmitter-to-target range at `tx_range_m` (a good approximation
    /// for a satellite far from a local surveillance area):
    ///
    /// `R_R = √(K·σ / (R_T²·SNR_req))`
    pub fn max_rx_range(
        &self,
        illuminator: &Illuminator,
        rcs_m2: f64,
        tx_range_m: f64,
        required_snr_db: f64,
    ) -> f64 {
        (self.snr_constant(illuminator) * rcs_m2
            / (tx_range_m * tx_range_m * db_to_linear(required_snr_db)))
        .sqrt()
    }
}

/// What a coverage map evaluates: one illuminator, one receiver, one
/// target size and one detection threshold.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CoverageScenario {
    /// The illuminator.
    pub illuminator: Illuminator,
    /// Transmitter position, m.
    pub transmitter: Vector3,
    /// The receiver. With an [`antenna`](Self::antenna), its `gain_db` is
    /// replaced by the antenna's gain towards each cell.
    pub receiver: PassiveReceiver,
    /// Receiver position, m.
    pub receiver_position: Vector3,
    /// The surveillance antenna and where it points; `None` treats every
    /// cell as on boresight at `receiver.gain_db`.
    pub antenna: Option<SteeredAntenna>,
    /// Bistatic cross-section of the target, m².
    pub rcs_m2: f64,
    /// Mean SNR a cell needs to count as covered, dB.
    pub threshold_snr_db: f64,
}

/// A horizontal grid of target positions.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CoverageGrid {
    /// South-west corner (x = east, y = north), m.
    pub min: Vector2,
    /// Side of one square cell, m (> 0).
    pub cell_size_m: f64,
    /// Cells from west to east (≥ 1).
    pub columns: usize,
    /// Cells from south to north (≥ 1).
    pub rows: usize,
    /// Target height, m.
    pub height_m: f64,
}

impl CoverageGrid {
    /// A square grid of `2·half_cells + 1` cells per side, centred on
    /// `centre` (so one cell sits exactly on it).
    pub fn centred(centre: Vector2, half_cells: usize, cell_size_m: f64, height_m: f64) -> Self {
        let n = 2 * half_cells + 1;
        let half = half_cells as f64 * cell_size_m;
        Self {
            min: Vector2::new(centre.x - half, centre.y - half),
            cell_size_m,
            columns: n,
            rows: n,
            height_m,
        }
    }

    /// Checks the grid is finite and non-empty.
    pub fn validate(&self) -> Result<(), RadarError> {
        let invalid = |field: &'static str, reason: &str| {
            Err(RadarError::InvalidConfig {
                field,
                reason: reason.into(),
            })
        };
        if !(self.cell_size_m.is_finite() && self.cell_size_m > 0.0) {
            return invalid("cell_size_m", "must be finite and > 0");
        }
        if self.columns == 0 || self.rows == 0 {
            return invalid("columns", "grid must have at least one cell");
        }
        if !(self.min.x.is_finite() && self.min.y.is_finite() && self.height_m.is_finite()) {
            return invalid("min", "must be finite");
        }
        Ok(())
    }

    /// Centre of the cell at `column`, `row`.
    pub fn cell_centre(&self, column: usize, row: usize) -> Vector3 {
        Vector3::new(
            self.min.x + column as f64 * self.cell_size_m,
            self.min.y + row as f64 * self.cell_size_m,
            self.height_m,
        )
    }

    fn len(&self) -> usize {
        self.columns * self.rows
    }
}

/// Mean SNR and range resolution over a [`CoverageGrid`] (Figs. 6 and 9 of
/// the paper). Cells are stored row by row, south to north.
#[derive(Debug, Clone, PartialEq)]
pub struct CoverageMap {
    /// The grid.
    pub grid: CoverageGrid,
    /// The threshold a covered cell meets, dB.
    pub threshold_snr_db: f64,
    /// Mean SNR per cell, dB (`+∞` on the receiver itself).
    pub snr_db: Vec<f64>,
    /// Range resolution along the bisector per cell, m (`+∞` where the
    /// bistatic angle is undefined or at forward scatter).
    pub resolution_m: Vec<f64>,
}

impl CoverageMap {
    /// Evaluates `scenario` over `grid`.
    ///
    /// Returns [`RadarError::InvalidConfig`] if the illuminator, receiver,
    /// grid, RCS or threshold is invalid.
    pub fn compute(scenario: &CoverageScenario, grid: &CoverageGrid) -> Result<Self, RadarError> {
        scenario.illuminator.validate()?;
        scenario.receiver.validate()?;
        grid.validate()?;
        if !(scenario.rcs_m2.is_finite() && scenario.rcs_m2 > 0.0) {
            return Err(RadarError::InvalidConfig {
                field: "rcs_m2",
                reason: "must be finite and > 0".into(),
            });
        }
        if !scenario.threshold_snr_db.is_finite() {
            return Err(RadarError::InvalidConfig {
                field: "threshold_snr_db",
                reason: "must be finite".into(),
            });
        }
        let mut snr_db = Vec::with_capacity(grid.len());
        let mut resolution_m = Vec::with_capacity(grid.len());
        for row in 0..grid.rows {
            for column in 0..grid.columns {
                let target = grid.cell_centre(column, row);
                let geometry = BistaticGeometry {
                    transmitter: scenario.transmitter,
                    receiver: scenario.receiver_position,
                    target,
                };
                let mut receiver = scenario.receiver;
                if let Some(s) = scenario.antenna {
                    let direction = target - scenario.receiver_position;
                    // On the receiver itself there is no direction: use the peak.
                    receiver.gain_db = if direction.length_squared() > 0.0 {
                        s.antenna.gain_db_towards(s.azimuth, s.elevation, direction)
                    } else {
                        s.antenna.gain_db
                    };
                }
                snr_db.push(receiver.snr_db_at(&scenario.illuminator, scenario.rcs_m2, &geometry));
                resolution_m.push(
                    geometry
                        .range_resolution(scenario.illuminator.bandwidth_hz)
                        .unwrap_or(f64::INFINITY),
                );
            }
        }
        Ok(Self {
            grid: *grid,
            threshold_snr_db: scenario.threshold_snr_db,
            snr_db,
            resolution_m,
        })
    }

    fn index(&self, column: usize, row: usize) -> usize {
        assert!(
            column < self.grid.columns && row < self.grid.rows,
            "cell out of range"
        );
        row * self.grid.columns + column
    }

    /// Mean SNR of a cell, dB. Panics if the cell is outside the grid.
    pub fn snr_db_at(&self, column: usize, row: usize) -> f64 {
        self.snr_db[self.index(column, row)]
    }

    /// Range resolution of a cell, m. Panics if the cell is outside the
    /// grid.
    pub fn resolution_at(&self, column: usize, row: usize) -> f64 {
        self.resolution_m[self.index(column, row)]
    }

    /// Whether a cell meets the threshold. Panics if the cell is outside
    /// the grid.
    pub fn is_covered(&self, column: usize, row: usize) -> bool {
        self.snr_db_at(column, row) >= self.threshold_snr_db
    }

    /// Centres of the covered cells, row by row.
    pub fn covered_cells(&self) -> impl Iterator<Item = Vector3> + '_ {
        (0..self.grid.rows).flat_map(move |row| {
            (0..self.grid.columns)
                .filter(move |&column| self.is_covered(column, row))
                .map(move |column| self.grid.cell_centre(column, row))
        })
    }

    /// Fraction of cells covered, in `[0, 1]`.
    pub fn covered_fraction(&self) -> f64 {
        self.covered_cells().count() as f64 / self.snr_db.len() as f64
    }

    /// Covered area, m².
    pub fn covered_area_m2(&self) -> f64 {
        self.covered_cells().count() as f64 * self.grid.cell_size_m * self.grid.cell_size_m
    }

    /// Horizontal distance from `from` to the farthest covered cell centre,
    /// m; `None` if no cell is covered.
    pub fn farthest_covered_m(&self, from: Vector3) -> Option<f64> {
        self.covered_cells()
            .map(|c| (c.x - from.x).hypot(c.y - from.y))
            .reduce(f64::max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::SPEED_OF_LIGHT;
    use approx::assert_relative_eq;

    /// The paper's receiver: Table II's rounded Starlink wavelength and its
    /// implied 4.74 dB of losses (see the budget tests).
    fn paper() -> (Illuminator, PassiveReceiver) {
        (
            Illuminator {
                frequency_hz: SPEED_OF_LIGHT / 0.03,
                ..Illuminator::starlink()
            },
            PassiveReceiver {
                losses_db: 4.74,
                ..PassiveReceiver::default()
            },
        )
    }

    fn scenario(antenna: Option<SteeredAntenna>) -> CoverageScenario {
        let (illuminator, receiver) = paper();
        CoverageScenario {
            illuminator,
            transmitter: Vector3::new(0.0, 0.0, illuminator.altitude_m),
            receiver,
            receiver_position: Vector3::ZERO,
            antenna,
            rcs_m2: 40.0,
            threshold_snr_db: 16.5,
        }
    }

    #[test]
    fn max_range_matches_the_paper() {
        let (il, rx) = paper();
        let r = rx.max_rx_range(&il, 40.0, il.altitude_m, 16.5);
        assert!((r - 18e3).abs() < 0.5e3, "{r}");
        // The SNR at that range is the threshold.
        assert_relative_eq!(rx.snr_db(&il, 40.0, il.altitude_m, r), 16.5, epsilon = 1e-9);
        // Range grows with √σ: 1 m² gives 2.8 km.
        let r1 = rx.max_rx_range(&il, 1.0, il.altitude_m, 16.5);
        assert_relative_eq!(r1, r / 40f64.sqrt(), max_relative = 1e-12);
        assert!((r1 - 2.8e3).abs() < 0.1e3, "{r1}");
    }

    #[test]
    fn antenna_options_follow_fig_8() {
        // Fig. 8: the parabola reaches ~18 km, the 5-horn array ~3.1 km and
        // a single horn ~1.4 km. Peak gain alone gives 18, 2.85 and 1.27 km;
        // the paper's curves sit about 10% higher for the two horns.
        let (il, rx) = paper();
        let range = |a: SurveillanceAntenna| {
            PassiveReceiver {
                gain_db: a.gain_db,
                ..rx
            }
            .max_rx_range(&il, 40.0, il.altitude_m, 16.5)
        };
        let parabolic = range(SurveillanceAntenna::parabolic());
        let array = range(SurveillanceAntenna::horn_array());
        let horn = range(SurveillanceAntenna::horn());
        assert!((parabolic - 18e3).abs() < 0.5e3, "{parabolic}");
        assert!((array - 3.1e3).abs() < 0.4e3, "{array}");
        assert!((horn - 1.4e3).abs() < 0.2e3, "{horn}");
    }

    #[test]
    fn antenna_gain_falls_off_boresight() {
        let a = SurveillanceAntenna::parabolic();
        let north = Vector3::new(0.0, 1.0, 0.0);
        assert_relative_eq!(a.gain_db_towards(0.0, 0.0, north), 40.0, epsilon = 1e-9);
        // Half a beamwidth off is the half-power point.
        let off = Vector3::new(deg(1.0).sin(), deg(1.0).cos(), 0.0);
        assert_relative_eq!(
            a.gain_db_towards(0.0, 0.0, off),
            40.0 - 3.0103,
            epsilon = 1e-3
        );
        // Far off, the sidelobe floor.
        let east = Vector3::new(1.0, 0.0, 0.0);
        assert_relative_eq!(a.gain_db_towards(0.0, 0.0, east), 10.0, epsilon = 1e-9);
    }

    #[test]
    fn omnidirectional_coverage_is_a_disc_of_max_range() {
        let s = scenario(None);
        let grid = CoverageGrid::centred(Vector2::ZERO, 40, 500.0, 0.0);
        let map = CoverageMap::compute(&s, &grid).unwrap();
        let far = map.farthest_covered_m(Vector3::ZERO).unwrap();
        let r = s
            .receiver
            .max_rx_range(&s.illuminator, 40.0, s.illuminator.altitude_m, 16.5);
        assert!((far - r).abs() < 750.0, "{far} vs {r}");
        // About π·r² of the 40 km × 40 km grid.
        let disc = std::f64::consts::PI * r * r;
        assert!((map.covered_area_m2() - disc).abs() / disc < 0.05);
        assert!(map.is_covered(40, 40));
        assert_eq!(map.snr_db_at(40, 40), f64::INFINITY);
    }

    #[test]
    fn a_parabola_covers_a_narrow_wedge() {
        let s = scenario(Some(SteeredAntenna {
            antenna: SurveillanceAntenna::parabolic(),
            azimuth: 0.0,
            elevation: 0.0,
        }));
        let grid = CoverageGrid::centred(Vector2::ZERO, 40, 500.0, 0.0);
        let map = CoverageMap::compute(&s, &grid).unwrap();
        let far = map.farthest_covered_m(Vector3::ZERO).unwrap();
        assert!((far - 18e3).abs() < 1e3, "{far}");
        for c in map.covered_cells() {
            if c.length() > 1e3 {
                let az = c.x.atan2(c.y).abs();
                assert!(az < deg(5.0), "covered {c:?} at {}°", az.to_degrees());
            }
        }
        // The horn covers a wider, shorter wedge.
        let horn = CoverageMap::compute(
            &scenario(Some(SteeredAntenna {
                antenna: SurveillanceAntenna::horn(),
                azimuth: 0.0,
                elevation: 0.0,
            })),
            &CoverageGrid::centred(Vector2::ZERO, 40, 50.0, 0.0),
        )
        .unwrap();
        assert!(horn.farthest_covered_m(Vector3::ZERO).unwrap() < 1.6e3);
        let widest = horn
            .covered_cells()
            .filter(|c| c.length() > 300.0)
            .map(|c| c.x.atan2(c.y).abs())
            .fold(0.0, f64::max);
        assert!(widest > deg(10.0), "{}°", widest.to_degrees());
    }

    #[test]
    fn resolution_map_matches_the_geometry() {
        let s = scenario(None);
        let grid = CoverageGrid::centred(Vector2::new(1e3, 2e3), 3, 1e3, 0.0);
        let map = CoverageMap::compute(&s, &grid).unwrap();
        for row in 0..grid.rows {
            for column in 0..grid.columns {
                let g = BistaticGeometry {
                    transmitter: s.transmitter,
                    receiver: s.receiver_position,
                    target: grid.cell_centre(column, row),
                };
                assert_eq!(
                    map.resolution_at(column, row),
                    g.range_resolution(s.illuminator.bandwidth_hz)
                        .unwrap_or(f64::INFINITY)
                );
            }
        }
        // The grid includes the receiver's own cell, where β is undefined.
        assert_eq!(map.resolution_at(2, 1), f64::INFINITY);
        assert_relative_eq!(grid.cell_centre(3, 3).x, 1e3);
        assert_relative_eq!(grid.cell_centre(3, 3).y, 2e3);
    }

    #[test]
    fn compute_rejects_bad_input() {
        let grid = CoverageGrid::centred(Vector2::ZERO, 1, 1.0, 0.0);
        let mut s = scenario(None);
        s.rcs_m2 = 0.0;
        assert!(CoverageMap::compute(&s, &grid).is_err());
        let s = scenario(None);
        let empty = CoverageGrid { columns: 0, ..grid };
        assert!(CoverageMap::compute(&s, &empty).is_err());
        let bad = CoverageGrid {
            cell_size_m: f64::NAN,
            ..grid
        };
        assert!(CoverageMap::compute(&s, &bad).is_err());
    }
}
