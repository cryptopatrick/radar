//! The radar as a [`SensorModel`]: scan, detect, measure, identify.

use crate::error::RadarError;
use crate::geometry::{Rotation3, Vector3, azimuth_of, direction, elevation_of};
use crate::ids::{SensorId, TransponderCode};
use crate::radar::config::{DetectionConfig, Dimension, RadarConfig, ScanFrame};
use crate::radar::detection::{
    DetectionContext, DetectionModel, DetectionResult, RadiusDetector, RangeEquationDetector,
};
use crate::radar::false_alarm::{FalseAlarm, generate_false_alarms, generate_radius_false_alarms};
use crate::radar::measurement::{Measurement, measurement_sigma, noisy_measurement};
#[cfg(feature = "tracking")]
use crate::radar::scan::ScanConfig;
use crate::radar::scan::{ScanState, Sweep, advance, half_extent, illumination, initial_state};
use crate::random::RandomSource;
use crate::random::sample::{bernoulli, uniform01};
use crate::sensor::{Observation, SensorContext, SensorModel};
#[cfg(feature = "tracking")]
use crate::tracking::{BeamMotion, Track, TrackManager, TrackerConfig, with_tracker};
use crate::units::linear_to_db;
use std::f64::consts::FRAC_PI_2;

/// Half-width (m/s) of the uniform radial velocity given to false alarms
/// when doppler is on, so they carry the same fields as real returns.
const FALSE_ALARM_DOPPLER_SPAN: f64 = 300.0;

/// The sensor position and scan-frame rotation (scan frame → world).
#[derive(Debug, Clone, Copy)]
struct Frame {
    position: Vector3,
    rotation: Rotation3,
}

impl Frame {
    /// Range and scan-frame azimuth and elevation of a world point.
    fn look_at(&self, p: Vector3) -> (f64, f64, f64) {
        let rel = p - self.position;
        let local = self.rotation.inverse().rotate(rel);
        (rel.length(), azimuth_of(local), elevation_of(local))
    }
}

/// A real detection awaiting identification.
#[derive(Debug)]
struct Pending {
    measurement: Measurement,
    transponder: Option<TransponderCode>,
    true_range: f64,
}

/// A radar driven by a [`RadarConfig`].
#[derive(Debug)]
pub(crate) struct RadarSensor {
    cfg: RadarConfig,
    state: ScanState,
    /// Range-equation constant `K` (unused by other detection configs).
    snr_constant: f64,
    /// Track manager, unless the tracker is [`TrackerConfig::None`].
    #[cfg(feature = "tracking")]
    tracks: Option<TrackManager>,
}

impl RadarSensor {
    /// Creates sensor `id` with its scan at the initial state and no
    /// tracks. The config is assumed valid.
    #[cfg_attr(not(feature = "tracking"), allow(unused_variables))]
    pub(crate) fn new(id: SensorId, cfg: RadarConfig) -> Self {
        let snr_constant = match &cfg.detection {
            DetectionConfig::RangeEquation(c) => c.calibration.snr_constant(),
            _ => 0.0,
        };
        let state = initial_state(&cfg.scan);
        #[cfg(feature = "tracking")]
        let tracks = (!matches!(cfg.tracker, TrackerConfig::None)).then(|| {
            TrackManager::new(
                &cfg.track_management,
                cfg.dimension,
                id,
                cfg.limits.instrumented_range,
            )
        });
        Self {
            cfg,
            state,
            snr_constant,
            #[cfg(feature = "tracking")]
            tracks,
        }
    }

    /// Snapshots of the live tracks (empty without a tracker).
    #[cfg(feature = "tracking")]
    pub(crate) fn tracks(&self) -> Vec<Track> {
        self.tracks
            .as_ref()
            .map_or_else(Vec::new, TrackManager::tracks)
    }

    /// The radar's configuration.
    pub(crate) fn config(&self) -> &RadarConfig {
        &self.cfg
    }

    /// The scan state (boresight in the scan frame).
    pub(crate) fn scan_state(&self) -> &ScanState {
        &self.state
    }

    /// Mutable scan state, for beam commands.
    pub(crate) fn scan_state_mut(&mut self) -> &mut ScanState {
        &mut self.state
    }

    /// Returns the scan to its initial state and drops every track.
    pub(crate) fn reset(&mut self) {
        self.state = initial_state(&self.cfg.scan);
        #[cfg(feature = "tracking")]
        if let Some(t) = &mut self.tracks {
            t.reset();
        }
    }

    /// Runs the configured detector: Radius → [`RadiusDetector`],
    /// RangeEquation → [`RangeEquationDetector`], Custom → the boxed model.
    fn detect(&self, look: &DetectionContext, rng: &mut dyn RandomSource) -> DetectionResult {
        match &self.cfg.detection {
            DetectionConfig::Radius { range, pd, .. } => RadiusDetector {
                range: *range,
                pd: *pd,
            }
            .detect(look, rng),
            DetectionConfig::RangeEquation(c) => RangeEquationDetector {
                k: self.snr_constant,
                pfa: c.pfa,
            }
            .detect(look, rng),
            DetectionConfig::Custom(model) => model.detect(look, rng),
        }
    }

    fn range_resolution(&self) -> Option<f64> {
        match &self.cfg.detection {
            DetectionConfig::RangeEquation(c) => Some(c.range_resolution),
            _ => None,
        }
    }

    /// Sensor position and scan-frame rotation for this update.
    fn frame(&self, ctx: &SensorContext<'_>) -> Frame {
        let pose = ctx.platform.pose.compose(ctx.mount);
        let rotation = match self.cfg.scan_frame {
            ScanFrame::PlatformRelative => pose.rotation,
            ScanFrame::WorldStabilised => Rotation3::IDENTITY,
        };
        Frame {
            position: pose.position,
            rotation,
        }
    }

    /// Builds a noisy measurement from scan-frame truth; angles are
    /// reported in the world frame.
    fn measure(
        &self,
        ctx: &mut SensorContext<'_>,
        frame: Frame,
        truth: (f64, f64, f64, Option<f64>),
        measured_snr: Option<f64>,
    ) -> Measurement {
        let (range, az_s, el_s, rv) = truth;
        let dim = self.cfg.dimension;
        let sigma = measurement_sigma(
            &self.cfg.measurement,
            &self.cfg.antenna,
            self.range_resolution(),
            measured_snr,
        );
        let el_s = if dim == Dimension::TwoD { 0.0 } else { el_s };
        let (range, az, el, radial_velocity) =
            noisy_measurement(range, az_s, el_s, rv, sigma, dim, &mut *ctx.rng);
        let d_world = frame.rotation.rotate(direction(az, el.unwrap_or(0.0)));
        Measurement {
            sensor: ctx.sensor,
            time: ctx.time,
            range,
            azimuth: azimuth_of(d_world),
            elevation: el.map(|_| elevation_of(d_world).clamp(-FRAC_PI_2, FRAC_PI_2)),
            radial_velocity,
            snr_db: measured_snr.map(linear_to_db),
            sigma,
            identity: None,
            sensor_position: frame.position,
            sensor_velocity: ctx.platform.velocity,
        }
    }

    /// Looks at every target in id order and measures the detected ones.
    fn detect_targets(
        &self,
        ctx: &mut SensorContext<'_>,
        frame: Frame,
        sweep: &Sweep,
        pending: &mut Vec<Pending>,
    ) {
        let world = ctx.world;
        let limits = self.cfg.limits;
        for target in &world.targets {
            if target.platform == Some(ctx.platform.id) {
                continue;
            }
            let p_target = target.position_at(world.time, ctx.time);
            let rel = p_target - frame.position;
            let (range, az_s, el_s) = frame.look_at(p_target);
            if !(range >= limits.min_range && range <= limits.instrumented_range) {
                continue;
            }
            let Some((off_az, off_el)) = illumination(
                sweep,
                az_s,
                el_s,
                half_extent(target.extent_radius, range),
                &self.cfg.antenna,
                self.cfg.dimension,
            ) else {
                continue;
            };
            if ctx
                .environment
                .line_of_sight_blocked(frame.position, p_target)
            {
                continue;
            }
            let look = DetectionContext {
                range,
                rcs_m2: target.signature.rcs_m2,
                swerling: target.signature.swerling,
                pattern_gain: self.cfg.antenna.gain(off_az, off_el, self.cfg.dimension),
                jamming_to_noise: 0.0,
            };
            let result = self.detect(&look, &mut *ctx.rng);
            if !result.detected {
                continue;
            }
            let rv = self.cfg.doppler.then(|| {
                let unit = rel.normalized().unwrap_or(Vector3::ZERO);
                (target.velocity - ctx.platform.velocity).dot(unit)
            });
            let measurement =
                self.measure(ctx, frame, (range, az_s, el_s, rv), result.measured_snr);
            pending.push(Pending {
                measurement,
                transponder: target.transponder,
                true_range: range,
            });
        }
    }

    /// Generates this sweep's false alarms as unidentified measurements.
    fn false_alarms(
        &self,
        ctx: &mut SensorContext<'_>,
        frame: Frame,
        sweep: &Sweep,
        out: &mut Vec<Measurement>,
    ) {
        let cfg = &self.cfg;
        let mut alarms: Vec<FalseAlarm> = Vec::new();
        let snr_of = match &cfg.detection {
            DetectionConfig::RangeEquation(c) => {
                generate_false_alarms(
                    sweep,
                    &cfg.limits,
                    c.range_resolution,
                    c.pfa,
                    &cfg.antenna,
                    cfg.dimension,
                    &mut *ctx.rng,
                    &mut alarms,
                );
                true
            }
            DetectionConfig::Radius {
                false_alarms_per_scan,
                ..
            } => {
                generate_radius_false_alarms(
                    sweep,
                    &cfg.limits,
                    *false_alarms_per_scan,
                    &cfg.antenna,
                    cfg.dimension,
                    &mut *ctx.rng,
                    &mut alarms,
                );
                false
            }
            DetectionConfig::Custom(_) => false,
        };
        for fa in alarms {
            let rv = cfg
                .doppler
                .then(|| (2.0 * uniform01(&mut *ctx.rng) - 1.0) * FALSE_ALARM_DOPPLER_SPAN);
            let snr = snr_of.then_some(fa.measured_snr);
            let truth = (fa.range, fa.azimuth_scan, fa.elevation_scan, rv);
            out.push(self.measure(ctx, frame, truth, snr));
        }
    }

    /// Interrogates transponders of detected targets within range.
    fn identify(&self, ctx: &mut SensorContext<'_>, pending: &mut [Pending]) {
        let Some(id) = self.cfg.identification else {
            return;
        };
        for p in pending {
            if let Some(code) = p.transponder {
                if p.true_range <= id.max_range && bernoulli(&mut *ctx.rng, id.reply_probability) {
                    p.measurement.identity = Some(code);
                }
            }
        }
    }
}

impl SensorModel for RadarSensor {
    fn update(
        &mut self,
        ctx: &mut SensorContext<'_>,
        out: &mut Vec<Observation>,
    ) -> Result<(), RadarError> {
        let sweep = advance(
            &mut self.cfg.scan,
            &mut self.state,
            ctx.interval.as_secs_f64(),
        );
        let frame = self.frame(ctx);

        let mut pending = Vec::new();
        self.detect_targets(ctx, frame, &sweep, &mut pending);
        let mut false_alarms = Vec::new();
        self.false_alarms(ctx, frame, &sweep, &mut false_alarms);
        self.identify(ctx, &mut pending);

        let mut measurements: Vec<Measurement> =
            pending.into_iter().map(|p| p.measurement).collect();
        measurements.append(&mut false_alarms);
        // List position must not reveal targets or false alarms.
        measurements.sort_by(|a, b| {
            a.range
                .total_cmp(&b.range)
                .then(a.azimuth.total_cmp(&b.azimuth))
        });

        #[cfg(feature = "tracking")]
        let mut track_obs = Vec::new();
        #[cfg(feature = "tracking")]
        if let Some(manager) = &mut self.tracks {
            let (antenna, dim) = (&self.cfg.antenna, self.cfg.dimension);
            let covered = |p: Vector3| {
                let (_, az, el) = frame.look_at(p);
                illumination(&sweep, az, el, 0.0, antenna, dim).is_some()
            };
            let beam = BeamMotion {
                sweeping: matches!(
                    self.cfg.scan,
                    ScanConfig::Circular { .. }
                        | ScanConfig::Sector { .. }
                        | ScanConfig::Raster { .. }
                        | ScanConfig::Volume { .. }
                ),
                swept: sweep.delta_azimuth.abs(),
            };
            with_tracker(&self.cfg.tracker, |tracker| {
                manager.update(
                    tracker,
                    ctx.time,
                    beam,
                    &measurements,
                    covered,
                    &mut track_obs,
                );
            });
        }

        out.extend(measurements.into_iter().map(Observation::Measurement));
        #[cfg(feature = "tracking")]
        out.append(&mut track_obs);
        if sweep.completed {
            out.push(Observation::ScanCompleted {
                sensor: ctx.sensor,
                time: ctx.time,
            });
        }
        Ok(())
    }
}
