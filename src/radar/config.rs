//! Radar configuration, builder, validation and presets.

use crate::error::RadarError;
use crate::radar::antenna::{AntennaConfig, BeamPattern};
use crate::radar::calibration::Calibration;
#[cfg(feature = "ew")]
use crate::radar::calibration::JammingReference;
use crate::radar::detection::DetectionModel;
use crate::radar::measurement::{MeasurementConfig, MeasurementSigma};
use crate::radar::scan::ScanConfig;
use crate::time::SimDuration;
#[cfg(feature = "tracking")]
use crate::tracking::{TrackManagementConfig, TrackerConfig};
use crate::units::deg;
#[cfg(feature = "ew")]
use crate::world::Band;
use crate::world::Swerling;
use std::f64::consts::{PI, TAU};

/// Whether the radar measures elevation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Dimension {
    /// Range and azimuth only; elevation is ignored.
    TwoD,
    /// Range, azimuth and elevation.
    ThreeD,
}

/// The frame in which scan angles are expressed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ScanFrame {
    /// Relative to the platform pose combined with the sensor mount, so
    /// platform rotation drags the beam (default).
    #[default]
    PlatformRelative,
    /// Relative to the world axes at the sensor's mount position.
    WorldStabilised,
}

/// Minimum and maximum (instrumented) range, m.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RangeLimits {
    /// Minimum range, `>= 0`.
    pub min_range: f64,
    /// Maximum range, finite and `> min_range`.
    pub instrumented_range: f64,
}

/// Parameters of range-equation detection.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RangeEquationConfig {
    /// How the range-equation constant is obtained.
    pub calibration: Calibration,
    /// Probability of false alarm per resolution cell, in `(0, 0.1]`.
    pub pfa: f64,
    /// Range resolution, m (> 0).
    pub range_resolution: f64,
}

/// How targets are detected.
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum DetectionConfig {
    /// Illuminated targets within `range` are detected with probability `pd`.
    Radius {
        /// Detection range, m.
        range: f64,
        /// Probability of detection, in `[0, 1]`.
        pd: f64,
        /// Mean number of false alarms per full 2π scan, `>= 0`.
        false_alarms_per_scan: f64,
    },
    /// Single-look square-law detection from the radar range equation.
    RangeEquation(RangeEquationConfig),
    /// A user-supplied model. Not serialisable.
    #[cfg_attr(feature = "serde", serde(skip))]
    Custom(Box<dyn DetectionModel>),
}

impl DetectionConfig {
    /// Clones the config, or returns `None` for [`DetectionConfig::Custom`].
    fn try_clone(&self) -> Option<DetectionConfig> {
        Some(match self {
            DetectionConfig::Radius {
                range,
                pd,
                false_alarms_per_scan,
            } => DetectionConfig::Radius {
                range: *range,
                pd: *pd,
                false_alarms_per_scan: *false_alarms_per_scan,
            },
            DetectionConfig::RangeEquation(c) => DetectionConfig::RangeEquation(*c),
            DetectionConfig::Custom(_) => return None,
        })
    }
}

/// Cooperative identification (a co-mounted secondary radar).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IdentificationConfig {
    /// Probability that a detected transponder replies, in `[0, 1]`.
    pub reply_probability: f64,
    /// Maximum interrogation range, m.
    pub max_range: f64,
}

/// Full configuration of one radar. Build it with [`RadarConfig::builder`]
/// or a preset; [`RadarConfig::validate`] checks a hand-assembled value.
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RadarConfig {
    /// 2D or 3D.
    pub dimension: Dimension,
    /// Time between updates (> 0).
    pub update_period: SimDuration,
    /// Antenna beamwidths and pattern.
    pub antenna: AntennaConfig,
    /// Beam motion.
    pub scan: ScanConfig,
    /// Frame of the scan angles.
    pub scan_frame: ScanFrame,
    /// Detection model.
    pub detection: DetectionConfig,
    /// Measurement noise model.
    pub measurement: MeasurementConfig,
    /// Range limits.
    pub limits: RangeLimits,
    /// Whether radial velocity is measured.
    pub doppler: bool,
    /// Cooperative identification, if fitted.
    pub identification: Option<IdentificationConfig>,
    /// Tracking filter.
    #[cfg(feature = "tracking")]
    pub tracker: TrackerConfig,
    /// Track lifecycle parameters.
    #[cfg(feature = "tracking")]
    pub track_management: TrackManagementConfig,
    /// Receiver band, for jamming overlap; `None` makes the radar immune.
    #[cfg(feature = "ew")]
    pub receiver_band: Option<Band>,
    /// Jamming reference, required with a [`Calibration::Reference`]
    /// calibration when `receiver_band` is set.
    #[cfg(feature = "ew")]
    pub jamming_reference: Option<JammingReference>,
    /// Main-beam J/N (dB) at or above which a jamming strobe is reported.
    #[cfg(feature = "ew")]
    pub strobe_threshold_db: f64,
}

fn invalid(field: &'static str, reason: &str) -> RadarError {
    RadarError::InvalidConfig {
        field,
        reason: reason.into(),
    }
}

fn check(ok: bool, field: &'static str, reason: &str) -> Result<(), RadarError> {
    if ok {
        Ok(())
    } else {
        Err(invalid(field, reason))
    }
}

fn positive(x: f64) -> bool {
    x.is_finite() && x > 0.0
}

fn non_negative(x: f64) -> bool {
    x.is_finite() && x >= 0.0
}

fn check_sigma(s: &MeasurementSigma, field: &'static str) -> Result<(), RadarError> {
    let ok = [s.range, s.azimuth, s.elevation, s.radial_velocity]
        .into_iter()
        .all(non_negative);
    check(ok, field, "every sigma must be finite and >= 0")
}

fn check_calibration(c: &Calibration) -> Result<(), RadarError> {
    match *c {
        Calibration::Physical {
            peak_power_w,
            gain_db,
            frequency_hz,
            noise_figure_db,
            bandwidth_hz,
            losses_db,
        } => {
            let gt0 = "must be finite and > 0";
            let fin = "must be finite";
            check(
                positive(peak_power_w),
                "detection.calibration.peak_power_w",
                gt0,
            )?;
            check(gain_db.is_finite(), "detection.calibration.gain_db", fin)?;
            check(
                positive(frequency_hz),
                "detection.calibration.frequency_hz",
                gt0,
            )?;
            check(
                noise_figure_db.is_finite(),
                "detection.calibration.noise_figure_db",
                fin,
            )?;
            check(
                positive(bandwidth_hz),
                "detection.calibration.bandwidth_hz",
                gt0,
            )?;
            check(
                losses_db.is_finite(),
                "detection.calibration.losses_db",
                fin,
            )
        }
        Calibration::Reference {
            rcs_m2,
            range_m,
            snr_db,
        } => {
            check(
                positive(rcs_m2),
                "detection.calibration.rcs_m2",
                "must be finite and > 0",
            )?;
            check(
                positive(range_m),
                "detection.calibration.range_m",
                "must be finite and > 0",
            )?;
            check(
                snr_db.is_finite(),
                "detection.calibration.snr_db",
                "must be finite",
            )
        }
    }
}

impl RadarConfig {
    /// Starts a builder with the defaults documented on [`RadarConfigBuilder`].
    pub fn builder() -> RadarConfigBuilder {
        RadarConfigBuilder::default()
    }

    /// Checks every field, returning [`RadarError::InvalidConfig`] naming
    /// the first offending field.
    pub fn validate(&self) -> Result<(), RadarError> {
        let a = &self.antenna;
        let bw = |x: f64| x > 0.0 && x <= PI;
        check(
            bw(a.beamwidth_az),
            "antenna.beamwidth_az",
            "must be in (0, π]",
        )?;
        check(
            bw(a.beamwidth_el),
            "antenna.beamwidth_el",
            "must be in (0, π]",
        )?;
        check(
            a.sidelobe_level_db.is_finite() && a.sidelobe_level_db <= 0.0,
            "antenna.sidelobe_level_db",
            "must be finite and <= 0",
        )?;
        check(
            !self.update_period.is_zero(),
            "update_period",
            "must be > 0",
        )?;
        let l = &self.limits;
        check(
            non_negative(l.min_range)
                && l.instrumented_range.is_finite()
                && l.min_range < l.instrumented_range,
            "limits",
            "need finite 0 <= min_range < instrumented_range",
        )?;
        match &self.detection {
            DetectionConfig::Radius {
                range,
                pd,
                false_alarms_per_scan,
            } => {
                check(
                    non_negative(*range),
                    "detection.range",
                    "must be finite and >= 0",
                )?;
                check(
                    (0.0..=1.0).contains(pd),
                    "detection.pd",
                    "must be in [0, 1]",
                )?;
                check(
                    non_negative(*false_alarms_per_scan),
                    "detection.false_alarms_per_scan",
                    "must be finite and >= 0",
                )?;
            }
            DetectionConfig::RangeEquation(c) => {
                check(
                    c.pfa > 0.0 && c.pfa <= 0.1,
                    "detection.pfa",
                    "must be in (0, 0.1]",
                )?;
                check(
                    positive(c.range_resolution),
                    "detection.range_resolution",
                    "must be finite and > 0",
                )?;
                check_calibration(&c.calibration)?;
            }
            DetectionConfig::Custom(_) => {}
        }
        match &self.measurement {
            MeasurementConfig::Fixed(s) => check_sigma(s, "measurement.sigma")?,
            MeasurementConfig::SnrScaled {
                radial_velocity_resolution,
                floor,
            } => {
                check(
                    matches!(self.detection, DetectionConfig::RangeEquation(_)),
                    "measurement",
                    "SnrScaled requires RangeEquation detection",
                )?;
                check(
                    positive(*radial_velocity_resolution),
                    "measurement.radial_velocity_resolution",
                    "must be finite and > 0",
                )?;
                check_sigma(floor, "measurement.floor")?;
            }
        }
        if let Some(id) = &self.identification {
            check(
                (0.0..=1.0).contains(&id.reply_probability),
                "identification.reply_probability",
                "must be in [0, 1]",
            )?;
            check(
                non_negative(id.max_range),
                "identification.max_range",
                "must be finite and >= 0",
            )?;
        }
        #[cfg(feature = "tracking")]
        self.validate_tracking()?;
        #[cfg(feature = "ew")]
        self.validate_ew()?;
        Ok(())
    }

    #[cfg(feature = "tracking")]
    fn validate_tracking(&self) -> Result<(), RadarError> {
        match self.tracker {
            TrackerConfig::None => {}
            TrackerConfig::AlphaBeta { alpha, beta } => {
                check(
                    alpha > 0.0 && alpha <= 1.0,
                    "tracker.alpha",
                    "must be in (0, 1]",
                )?;
                check(
                    beta > 0.0 && beta < 2.0,
                    "tracker.beta",
                    "must be in (0, 2)",
                )?;
            }
            TrackerConfig::Kalman {
                process_accel_sigma,
            } => check(
                non_negative(process_accel_sigma),
                "tracker.process_accel_sigma",
                "must be finite and >= 0",
            )?,
        }
        let m = &self.track_management;
        check(
            m.confirm_m <= m.confirm_n,
            "track_management.confirm_m",
            "must be <= confirm_n",
        )?;
        check(
            m.max_coast >= 1,
            "track_management.max_coast",
            "must be >= 1",
        )?;
        check(
            m.gate_probability > 0.0 && m.gate_probability < 1.0,
            "track_management.gate_probability",
            "must be in (0, 1)",
        )?;
        check(
            positive(m.initial_velocity_sigma),
            "track_management.initial_velocity_sigma",
            "must be finite and > 0",
        )?;
        if let Some(s) = m.max_position_sigma {
            check(
                positive(s),
                "track_management.max_position_sigma",
                "must be finite and > 0",
            )?;
        }
        Ok(())
    }

    #[cfg(feature = "ew")]
    fn validate_ew(&self) -> Result<(), RadarError> {
        if let Some(band) = &self.receiver_band {
            check(
                positive(band.centre_hz) && positive(band.bandwidth_hz),
                "receiver_band",
                "centre and bandwidth must be finite and > 0",
            )?;
            if let DetectionConfig::RangeEquation(RangeEquationConfig {
                calibration: Calibration::Reference { .. },
                ..
            }) = self.detection
            {
                check(
                    self.jamming_reference.is_some(),
                    "jamming_reference",
                    "required with a Reference calibration and a receiver band",
                )?;
            }
        }
        if let Some(j) = &self.jamming_reference {
            check(
                positive(j.erp_w) && positive(j.range_m) && j.jnr_db.is_finite(),
                "jamming_reference",
                "erp_w and range_m must be finite and > 0, jnr_db finite",
            )?;
        }
        check(
            self.strobe_threshold_db.is_finite(),
            "strobe_threshold_db",
            "must be finite",
        )
    }

    /// Copies the config, or returns `None` if any stage is a `Custom`
    /// boxed model (which cannot be cloned).
    pub fn try_clone(&self) -> Option<RadarConfig> {
        Some(RadarConfig {
            dimension: self.dimension,
            update_period: self.update_period,
            antenna: self.antenna,
            scan: self.scan.try_clone()?,
            scan_frame: self.scan_frame,
            detection: self.detection.try_clone()?,
            measurement: self.measurement,
            limits: self.limits,
            doppler: self.doppler,
            identification: self.identification,
            #[cfg(feature = "tracking")]
            tracker: self.tracker.clone(),
            #[cfg(feature = "tracking")]
            track_management: self.track_management.clone(),
            #[cfg(feature = "ew")]
            receiver_band: self.receiver_band,
            #[cfg(feature = "ew")]
            jamming_reference: self.jamming_reference,
            #[cfg(feature = "ew")]
            strobe_threshold_db: self.strobe_threshold_db,
        })
    }

    /// The "arcade" radar: 2D, 100 ms updates, 3° × 3° Gaussian beam rotating
    /// once per second, radius detection with Pd 1 and no false alarms out to
    /// `range_m`, noise-free measurements, limits `0..range_m`, no doppler,
    /// no tracking.
    pub fn simple_2d(range_m: f64) -> Result<RadarConfig, RadarError> {
        RadarConfig::builder()
            .detection(DetectionConfig::Radius {
                range: range_m,
                pd: 1.0,
                false_alarms_per_scan: 0.0,
            })
            .limits(RangeLimits {
                min_range: 0.0,
                instrumented_range: range_m,
            })
            .build()
    }

    /// A 2D surveillance radar: 100 ms updates, 1.5° Gaussian beam rotating
    /// every 4 s, range-equation detection calibrated so a 1 m² Swerling 1
    /// target at 50 km has Pd 0.9 at Pfa 1e-6, 150 m range resolution,
    /// limits 500 m..100 km, SNR-scaled noise (floors 5 m, 0.05°, 0.05°,
    /// 0.5 m/s; 1 m/s velocity resolution), doppler on, Kalman tracking
    /// (process noise 5 m/s²).
    pub fn surveillance_2d() -> Result<RadarConfig, RadarError> {
        Self::surveillance_builder()?.build()
    }

    /// As [`RadarConfig::surveillance_2d`] but 3D, with a 2° elevation
    /// beamwidth and a volume scan over bars at 0.5°, 2°, 4° and 8°, one
    /// turn every 2 s per bar.
    pub fn surveillance_3d() -> Result<RadarConfig, RadarError> {
        Self::surveillance_builder()?
            .dimension(Dimension::ThreeD)
            .antenna(AntennaConfig {
                beamwidth_az: deg(1.5),
                beamwidth_el: deg(2.0),
                pattern: BeamPattern::Gaussian,
                sidelobe_level_db: -30.0,
            })
            .scan(ScanConfig::Volume {
                elevation_bars: vec![deg(0.5), deg(2.0), deg(4.0), deg(8.0)],
                rate: TAU / 2.0,
            })
            .build()
    }

    fn surveillance_builder() -> Result<RadarConfigBuilder, RadarError> {
        let calibration = Calibration::reference_for_pd(1.0, 50_000.0, 0.9, 1e-6, Swerling::One)?;
        let b = RadarConfig::builder()
            .antenna(AntennaConfig {
                beamwidth_az: deg(1.5),
                beamwidth_el: deg(1.5),
                pattern: BeamPattern::Gaussian,
                sidelobe_level_db: -30.0,
            })
            .scan(ScanConfig::Circular {
                rate: TAU / 4.0,
                elevation: 0.0,
            })
            .detection(DetectionConfig::RangeEquation(RangeEquationConfig {
                calibration,
                pfa: 1e-6,
                range_resolution: 150.0,
            }))
            .limits(RangeLimits {
                min_range: 500.0,
                instrumented_range: 100_000.0,
            })
            .measurement(MeasurementConfig::SnrScaled {
                radial_velocity_resolution: 1.0,
                floor: MeasurementSigma {
                    range: 5.0,
                    azimuth: deg(0.05),
                    elevation: deg(0.05),
                    radial_velocity: 0.5,
                },
            })
            .doppler(true);
        #[cfg(feature = "tracking")]
        let b = b.tracker(TrackerConfig::Kalman {
            process_accel_sigma: 5.0,
        });
        Ok(b)
    }
}

/// Builder for [`RadarConfig`]. Every field has a setter of the same name.
///
/// Defaults (the [`RadarConfig::simple_2d`] radar at 1 000 m):
/// - `dimension`: [`Dimension::TwoD`]; `update_period`: 100 ms
/// - `antenna`: [`AntennaConfig::default`] (3° × 3° Gaussian, −30 dB sidelobes)
/// - `scan`: `Circular { rate: 2π, elevation: 0 }`; `scan_frame`: `PlatformRelative`
/// - `detection`: `Radius { range: 1000, pd: 1, false_alarms_per_scan: 0 }`
/// - `measurement`: `Fixed` with all sigmas 0; `limits`: `0..1000`
/// - `doppler`: false; `identification`: None
/// - `[tracking]` `tracker`: `None`; `track_management`: default
/// - `[ew]` `receiver_band`: None; `jamming_reference`: None; `strobe_threshold_db`: 3.0
#[derive(Debug)]
pub struct RadarConfigBuilder {
    config: RadarConfig,
}

impl Default for RadarConfigBuilder {
    fn default() -> Self {
        Self {
            config: RadarConfig {
                dimension: Dimension::TwoD,
                update_period: SimDuration::from_millis(100),
                antenna: AntennaConfig::default(),
                scan: ScanConfig::Circular {
                    rate: TAU,
                    elevation: 0.0,
                },
                scan_frame: ScanFrame::PlatformRelative,
                detection: DetectionConfig::Radius {
                    range: 1000.0,
                    pd: 1.0,
                    false_alarms_per_scan: 0.0,
                },
                measurement: MeasurementConfig::Fixed(MeasurementSigma {
                    range: 0.0,
                    azimuth: 0.0,
                    elevation: 0.0,
                    radial_velocity: 0.0,
                }),
                limits: RangeLimits {
                    min_range: 0.0,
                    instrumented_range: 1000.0,
                },
                doppler: false,
                identification: None,
                #[cfg(feature = "tracking")]
                tracker: TrackerConfig::None,
                #[cfg(feature = "tracking")]
                track_management: TrackManagementConfig::default(),
                #[cfg(feature = "ew")]
                receiver_band: None,
                #[cfg(feature = "ew")]
                jamming_reference: None,
                #[cfg(feature = "ew")]
                strobe_threshold_db: 3.0,
            },
        }
    }
}

macro_rules! setters {
    ($($(#[$m:meta])* $name:ident: $ty:ty;)*) => {
        $(
            $(#[$m])*
            #[doc = concat!("Sets `", stringify!($name), "`.")]
            pub fn $name(mut self, value: $ty) -> Self {
                self.config.$name = value;
                self
            }
        )*
    };
}

impl RadarConfigBuilder {
    setters! {
        dimension: Dimension;
        update_period: SimDuration;
        antenna: AntennaConfig;
        scan: ScanConfig;
        scan_frame: ScanFrame;
        detection: DetectionConfig;
        measurement: MeasurementConfig;
        limits: RangeLimits;
        doppler: bool;
        identification: Option<IdentificationConfig>;
        #[cfg(feature = "tracking")] tracker: TrackerConfig;
        #[cfg(feature = "tracking")] track_management: TrackManagementConfig;
        #[cfg(feature = "ew")] receiver_band: Option<Band>;
        #[cfg(feature = "ew")] jamming_reference: Option<JammingReference>;
        #[cfg(feature = "ew")] strobe_threshold_db: f64;
    }

    /// Validates and returns the config (see [`RadarConfig::validate`]).
    pub fn build(self) -> Result<RadarConfig, RadarError> {
        self.config.validate()?;
        Ok(self.config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radar::detection::{DetectionContext, DetectionResult};

    fn field_of(r: Result<RadarConfig, RadarError>) -> &'static str {
        match r {
            Err(RadarError::InvalidConfig { field, .. }) => field,
            other => panic!("expected InvalidConfig, got {other:?}"),
        }
    }

    #[test]
    fn presets_build() {
        assert!(RadarConfig::simple_2d(1000.0).is_ok());
        assert!(RadarConfig::surveillance_2d().is_ok());
        let s3 = RadarConfig::surveillance_3d().unwrap();
        assert_eq!(s3.dimension, Dimension::ThreeD);
        assert!(RadarConfig::simple_2d(-5.0).is_err());
    }

    #[test]
    fn builder_default_builds() {
        assert!(RadarConfig::builder().build().is_ok());
    }

    #[test]
    fn try_clone_presets() {
        for cfg in [
            RadarConfig::simple_2d(1200.0).unwrap(),
            RadarConfig::surveillance_2d().unwrap(),
            RadarConfig::surveillance_3d().unwrap(),
        ] {
            let copy = cfg.try_clone().expect("preset has no Custom stage");
            assert_eq!(format!("{copy:?}"), format!("{cfg:?}"));
        }
    }

    #[derive(Debug)]
    struct Never;
    impl DetectionModel for Never {
        fn detect(
            &self,
            _ctx: &DetectionContext,
            _rng: &mut dyn crate::random::RandomSource,
        ) -> DetectionResult {
            DetectionResult {
                detected: false,
                probability: 0.0,
                mean_snr: None,
                measured_snr: None,
            }
        }
    }

    #[test]
    fn try_clone_custom_is_none() {
        let cfg = RadarConfig::builder()
            .detection(DetectionConfig::Custom(Box::new(Never)))
            .build()
            .unwrap();
        assert!(cfg.try_clone().is_none());
    }

    #[test]
    fn builder_rejects_zero_beamwidth() {
        let antenna = AntennaConfig {
            beamwidth_az: 0.0,
            ..AntennaConfig::default()
        };
        assert_eq!(
            field_of(RadarConfig::builder().antenna(antenna).build()),
            "antenna.beamwidth_az"
        );
        let antenna = AntennaConfig {
            beamwidth_el: 4.0,
            ..AntennaConfig::default()
        };
        assert_eq!(
            field_of(RadarConfig::builder().antenna(antenna).build()),
            "antenna.beamwidth_el"
        );
    }

    #[test]
    fn builder_rejects_snr_scaled_with_radius() {
        let floor = MeasurementSigma {
            range: 1.0,
            azimuth: 0.0,
            elevation: 0.0,
            radial_velocity: 0.0,
        };
        let r = RadarConfig::builder()
            .measurement(MeasurementConfig::SnrScaled {
                radial_velocity_resolution: 1.0,
                floor,
            })
            .build();
        assert_eq!(field_of(r), "measurement");
    }

    #[test]
    fn builder_rejects_invalid_values() {
        let b = RadarConfig::builder;
        assert_eq!(
            field_of(b().update_period(SimDuration::ZERO).build()),
            "update_period"
        );
        let limits = RangeLimits {
            min_range: 10.0,
            instrumented_range: 10.0,
        };
        assert_eq!(field_of(b().limits(limits).build()), "limits");
        let limits = RangeLimits {
            min_range: 0.0,
            instrumented_range: f64::INFINITY,
        };
        assert_eq!(field_of(b().limits(limits).build()), "limits");
        let det = DetectionConfig::Radius {
            range: 100.0,
            pd: 1.1,
            false_alarms_per_scan: 0.0,
        };
        assert_eq!(field_of(b().detection(det).build()), "detection.pd");
        let det = DetectionConfig::Radius {
            range: 100.0,
            pd: 0.5,
            false_alarms_per_scan: -1.0,
        };
        assert_eq!(
            field_of(b().detection(det).build()),
            "detection.false_alarms_per_scan"
        );
        let re = |pfa: f64, res: f64| {
            DetectionConfig::RangeEquation(RangeEquationConfig {
                calibration: Calibration::Reference {
                    rcs_m2: 1.0,
                    range_m: 1000.0,
                    snr_db: 13.0,
                },
                pfa,
                range_resolution: res,
            })
        };
        assert_eq!(
            field_of(b().detection(re(0.2, 10.0)).build()),
            "detection.pfa"
        );
        assert_eq!(
            field_of(b().detection(re(0.0, 10.0)).build()),
            "detection.pfa"
        );
        assert_eq!(
            field_of(b().detection(re(1e-6, 0.0)).build()),
            "detection.range_resolution"
        );
        assert!(b().detection(re(0.1, 10.0)).build().is_ok());
        let sigma = MeasurementSigma {
            range: f64::NAN,
            azimuth: 0.0,
            elevation: 0.0,
            radial_velocity: 0.0,
        };
        assert_eq!(
            field_of(b().measurement(MeasurementConfig::Fixed(sigma)).build()),
            "measurement.sigma"
        );
        let sigma = MeasurementSigma {
            range: 0.0,
            azimuth: -0.1,
            elevation: 0.0,
            radial_velocity: 0.0,
        };
        assert_eq!(
            field_of(b().measurement(MeasurementConfig::Fixed(sigma)).build()),
            "measurement.sigma"
        );
        let id = IdentificationConfig {
            reply_probability: 1.5,
            max_range: 1000.0,
        };
        assert_eq!(
            field_of(b().identification(Some(id)).build()),
            "identification.reply_probability"
        );
        let phys = Calibration::Physical {
            peak_power_w: 0.0,
            gain_db: 30.0,
            frequency_hz: 1e9,
            noise_figure_db: 3.0,
            bandwidth_hz: 1e6,
            losses_db: 0.0,
        };
        let det = DetectionConfig::RangeEquation(RangeEquationConfig {
            calibration: phys,
            pfa: 1e-6,
            range_resolution: 150.0,
        });
        assert_eq!(
            field_of(b().detection(det).build()),
            "detection.calibration.peak_power_w"
        );
    }

    #[cfg(feature = "tracking")]
    #[test]
    fn builder_rejects_invalid_tracking() {
        use crate::tracking::{TrackManagementConfig, TrackerConfig};
        let b = RadarConfig::builder;
        let ab = |alpha, beta| TrackerConfig::AlphaBeta { alpha, beta };
        assert_eq!(field_of(b().tracker(ab(0.0, 0.5)).build()), "tracker.alpha");
        assert_eq!(field_of(b().tracker(ab(1.0, 2.0)).build()), "tracker.beta");
        assert!(b().tracker(ab(1.0, 0.5)).build().is_ok());
        let mgmt = |f: fn(&mut TrackManagementConfig)| {
            let mut m = TrackManagementConfig::default();
            f(&mut m);
            m
        };
        let m = mgmt(|m| m.confirm_m = 6);
        assert_eq!(
            field_of(b().track_management(m).build()),
            "track_management.confirm_m"
        );
        let m = mgmt(|m| m.max_coast = 0);
        assert_eq!(
            field_of(b().track_management(m).build()),
            "track_management.max_coast"
        );
        let m = mgmt(|m| m.gate_probability = 1.0);
        assert_eq!(
            field_of(b().track_management(m).build()),
            "track_management.gate_probability"
        );
    }

    #[cfg(feature = "ew")]
    #[test]
    fn reference_with_receiver_band_requires_jamming_reference() {
        let band = Band {
            centre_hz: 3e9,
            bandwidth_hz: 1e6,
        };
        let base = || RadarConfig::surveillance_2d().unwrap();
        let mut cfg = base();
        cfg.receiver_band = Some(band);
        let r = RadarConfig::builder()
            .detection(cfg.detection)
            .receiver_band(Some(band))
            .build();
        assert_eq!(field_of(r), "jamming_reference");
        let r = RadarConfig::builder()
            .detection(base().detection)
            .receiver_band(Some(band))
            .jamming_reference(Some(JammingReference {
                erp_w: 1000.0,
                range_m: 10_000.0,
                jnr_db: 20.0,
            }))
            .build();
        assert!(r.is_ok());
    }

    #[test]
    fn preset_values() {
        let s = RadarConfig::simple_2d(1200.0).unwrap();
        assert_eq!(s.update_period, SimDuration::from_millis(100));
        assert_eq!(
            s.limits,
            RangeLimits {
                min_range: 0.0,
                instrumented_range: 1200.0
            }
        );
        assert!(!s.doppler);
        assert_eq!(s.scan_frame, ScanFrame::PlatformRelative);
        let s2 = RadarConfig::surveillance_2d().unwrap();
        assert!(s2.doppler);
        assert!(
            matches!(s2.scan, ScanConfig::Circular { rate, .. } if (rate - std::f64::consts::FRAC_PI_2).abs() < 1e-12)
        );
        let s3 = RadarConfig::surveillance_3d().unwrap();
        assert!((s3.antenna.beamwidth_el - deg(2.0)).abs() < 1e-12);
        match s3.scan {
            ScanConfig::Volume {
                ref elevation_bars,
                rate,
            } => {
                assert_eq!(elevation_bars.len(), 4);
                assert!((rate - std::f64::consts::PI).abs() < 1e-12);
            }
            _ => panic!("expected Volume"),
        }
    }
}
