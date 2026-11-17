//! Scan patterns: how the beam moves between updates.

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
