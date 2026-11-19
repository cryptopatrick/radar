//! The radar as a [`SensorModel`]: scan, detect, measure, identify,
//! report jamming strobes, track.

#[cfg(feature = "diagnostics")]
use crate::diagnostics::{DiagnosticOutcome, DiagnosticRecord};
use crate::error::RadarError;
#[cfg(feature = "ew")]
use crate::ew::{jammer_jnr, strobe_sigma};
use crate::geometry::{Rotation3, Vector3, azimuth_of, direction, elevation_of};
#[cfg(feature = "ew")]
use crate::geometry::{wrap_pi, wrap_two_pi};
use crate::ids::{PlatformId, SensorId, TransponderCode};
use crate::radar::config::{DetectionConfig, Dimension, RadarConfig, ScanFrame};
use crate::radar::detection::{
    DetectionContext, DetectionModel, DetectionResult, RadiusDetector, RangeEquationDetector,
};
use crate::radar::false_alarm::{FalseAlarm, generate_false_alarms, generate_radius_false_alarms};
use crate::radar::measurement::{Measurement, measurement_sigma, noisy_measurement};
#[cfg(feature = "tracking")]
use crate::radar::scan::ScanConfig;
#[cfg(feature = "tracking")]
use crate::radar::scan::confidently_illuminated;
use crate::radar::scan::{ScanState, Sweep, advance, half_extent, illumination, initial_state};
use crate::random::RandomSource;
#[cfg(feature = "ew")]
use crate::random::sample::standard_normal;
use crate::random::sample::{bernoulli, uniform01};
use crate::sensor::{Observation, SensorContext, SensorModel};
#[cfg(feature = "tracking")]
use crate::tracking::{BeamMotion, Coverage, Track, TrackManager, TrackerConfig, with_tracker};
use crate::units::linear_to_db;
#[cfg(feature = "ew")]
use crate::world::EmitterState;
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

/// A jammer in line of sight during this update.
#[cfg(feature = "ew")]
#[derive(Debug, Clone)]
struct Jammer {
    emitter: EmitterState,
    /// Range from the sensor, m.
    range: f64,
    /// Scan-frame azimuth.
    azimuth: f64,
    /// Scan-frame elevation.
    elevation: f64,
}

/// A real detection awaiting identification.
#[derive(Debug)]
struct Pending {
    measurement: Measurement,
    transponder: Option<TransponderCode>,
    true_range: f64,
    /// Index of this detection's record in the update's diagnostics.
    #[cfg(feature = "diagnostics")]
    diag: usize,
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
    /// Ground-truth records awaiting [`Self::drain_diagnostics`].
    #[cfg(feature = "diagnostics")]
    diagnostics: Vec<DiagnosticRecord>,
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
                cfg.scan.nominal_revisit(cfg.update_period),
            )
        });
        Self {
            cfg,
            state,
            snr_constant,
            #[cfg(feature = "tracking")]
            tracks,
            #[cfg(feature = "diagnostics")]
            diagnostics: Vec::new(),
        }
    }

    /// Moves the queued diagnostic records to `out`.
    #[cfg(feature = "diagnostics")]
    pub(crate) fn drain_diagnostics(&mut self, out: &mut Vec<DiagnosticRecord>) {
        out.append(&mut self.diagnostics);
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
        #[cfg(feature = "diagnostics")]
        self.diagnostics.clear();
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

    /// Every emitter in line of sight, in `EmitterId` order, with its
    /// scan-frame geometry. Empty without a receiver band (immune radar).
    /// Emitters on the radar's own platform are ignored.
    #[cfg(feature = "ew")]
    fn visible_jammers(&self, ctx: &SensorContext<'_>, frame: Frame) -> Vec<Jammer> {
        if self.cfg.receiver_band.is_none() {
            return Vec::new();
        }
        ctx.world
            .emitters
            .iter()
            .filter(|e| e.platform != Some(ctx.platform.id))
            .filter(|e| {
                !ctx.environment
                    .line_of_sight_blocked(frame.position, e.position)
            })
            .map(|e| {
                let (range, azimuth, elevation) = frame.look_at(e.position);
                Jammer {
                    emitter: e.clone(),
                    range,
                    azimuth,
                    elevation,
                }
            })
            .collect()
    }

    /// Total J/N (linear) toward a target at scan-frame `(az, el)`: each
    /// jammer contributes with the antenna gain at its angular offset from
    /// the target's line of sight (full gain when it rides on the target's
    /// platform: self-protection).
    #[cfg(feature = "ew")]
    fn jamming_to_noise(
        &self,
        jammers: &[Jammer],
        target_platform: Option<PlatformId>,
        az: f64,
        el: f64,
    ) -> f64 {
        let (antenna, dim) = (&self.cfg.antenna, self.cfg.dimension);
        jammers
            .iter()
            .map(|j| {
                let gain = if target_platform.is_some() && j.emitter.platform == target_platform {
                    antenna.gain(0.0, 0.0, dim)
                } else {
                    antenna.gain(wrap_pi(j.azimuth - az), j.elevation - el, dim)
                };
                jammer_jnr(&self.cfg, &j.emitter, j.range, gain)
            })
            .sum()
    }

    /// A strobe for every jammer the sweep illuminates whose main-beam J/N
    /// reaches `strobe_threshold_db`. Noise is drawn in `EmitterId` order;
    /// the result is sorted by azimuth.
    #[cfg(feature = "ew")]
    fn strobes(
        &self,
        ctx: &mut SensorContext<'_>,
        frame: Frame,
        sweep: &Sweep,
        jammers: &[Jammer],
    ) -> Vec<Observation> {
        let (antenna, dim) = (&self.cfg.antenna, self.cfg.dimension);
        let mut out = Vec::new();
        for j in jammers {
            let Some((off_az, off_el)) =
                illumination(sweep, j.azimuth, j.elevation, 0.0, antenna, dim)
            else {
                continue;
            };
            let jnr = jammer_jnr(
                &self.cfg,
                &j.emitter,
                j.range,
                antenna.gain(off_az, off_el, dim),
            );
            let j_over_n_db = linear_to_db(jnr);
            if !(jnr > 0.0 && j_over_n_db >= self.cfg.strobe_threshold_db) {
                continue;
            }
            let az = wrap_two_pi(
                j.azimuth
                    + strobe_sigma(antenna.beamwidth_az, jnr) * standard_normal(&mut *ctx.rng),
            );
            let el = match dim {
                Dimension::TwoD => 0.0,
                Dimension::ThreeD => {
                    j.elevation
                        + strobe_sigma(antenna.beamwidth_el, jnr) * standard_normal(&mut *ctx.rng)
                }
            };
            let d_world = frame.rotation.rotate(direction(az, el));
            out.push(Observation::JammingStrobe {
                sensor: ctx.sensor,
                time: ctx.time,
                azimuth: azimuth_of(d_world),
                elevation: (dim == Dimension::ThreeD)
                    .then(|| elevation_of(d_world).clamp(-FRAC_PI_2, FRAC_PI_2)),
                j_over_n_db,
            });
        }
        let az_of = |o: &Observation| match o {
            Observation::JammingStrobe { azimuth, .. } => *azimuth,
            _ => 0.0,
        };
        out.sort_by(|a, b| az_of(a).total_cmp(&az_of(b)));
        out
    }

    /// Looks at every target in id order and measures the detected ones.
    /// `jamming(platform, az, el)` gives the J/N toward a target.
    fn detect_targets(
        &self,
        ctx: &mut SensorContext<'_>,
        frame: Frame,
        sweep: &Sweep,
        jamming: impl Fn(Option<PlatformId>, f64, f64) -> f64,
        pending: &mut Vec<Pending>,
        #[cfg(feature = "diagnostics")] diag: &mut Vec<DiagnosticRecord>,
    ) {
        #[cfg(feature = "diagnostics")]
        let (diag_sensor, diag_time) = (ctx.sensor, ctx.time);
        #[cfg(feature = "diagnostics")]
        let mut record = |target: &crate::world::TargetState,
                          range: f64,
                          outcome: DiagnosticOutcome,
                          result: Option<&DetectionResult>| {
            diag.push(DiagnosticRecord {
                sensor: diag_sensor,
                time: diag_time,
                target: Some(target.id),
                true_range: Some(range),
                snr_db: result
                    .and_then(|r| r.measured_snr.or(r.mean_snr))
                    .map(linear_to_db),
                pd: result.map(|r| r.probability),
                outcome,
                measurement: None,
            });
            diag.len() - 1
        };
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
                #[cfg(feature = "diagnostics")]
                record(target, range, DiagnosticOutcome::OutOfRange, None);
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
                #[cfg(feature = "diagnostics")]
                record(target, range, DiagnosticOutcome::OutOfBeam, None);
                continue;
            };
            if ctx
                .environment
                .line_of_sight_blocked(frame.position, p_target)
            {
                #[cfg(feature = "diagnostics")]
                record(target, range, DiagnosticOutcome::Occluded, None);
                continue;
            }
            let look = DetectionContext {
                range,
                rcs_m2: target.signature.rcs_m2,
                swerling: target.signature.swerling,
                pattern_gain: self.cfg.antenna.gain(off_az, off_el, self.cfg.dimension),
                jamming_to_noise: jamming(target.platform, az_s, el_s),
            };
            let result = self.detect(&look, &mut *ctx.rng);
            if !result.detected {
                #[cfg(feature = "diagnostics")]
                record(target, range, DiagnosticOutcome::Missed, Some(&result));
                continue;
            }
            #[cfg(feature = "diagnostics")]
            let diag_index = record(target, range, DiagnosticOutcome::Detected, Some(&result));
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
                #[cfg(feature = "diagnostics")]
                diag: diag_index,
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

        #[cfg(feature = "ew")]
        let jammers = self.visible_jammers(ctx, frame);
        #[cfg(feature = "ew")]
        let jamming = |p, az, el| self.jamming_to_noise(&jammers, p, az, el);
        #[cfg(not(feature = "ew"))]
        let jamming = |_: Option<PlatformId>, _: f64, _: f64| 0.0;

        // Draw order: targets, false alarms, identification, strobes, tracking.
        let mut pending = Vec::new();
        #[cfg(feature = "diagnostics")]
        let mut diag = Vec::new();
        self.detect_targets(
            ctx,
            frame,
            &sweep,
            jamming,
            &mut pending,
            #[cfg(feature = "diagnostics")]
            &mut diag,
        );
        let mut false_alarms = Vec::new();
        self.false_alarms(ctx, frame, &sweep, &mut false_alarms);
        self.identify(ctx, &mut pending);
        // Truth is linked before the sort below, which erases the order.
        #[cfg(feature = "diagnostics")]
        {
            for p in &pending {
                diag[p.diag].measurement = Some(p.measurement.clone());
            }
            diag.extend(false_alarms.iter().map(|m| DiagnosticRecord {
                sensor: ctx.sensor,
                time: ctx.time,
                target: None,
                true_range: None,
                snr_db: m.snr_db,
                pd: None,
                outcome: DiagnosticOutcome::FalseAlarm,
                measurement: Some(m.clone()),
            }));
            self.diagnostics.append(&mut diag);
        }
        #[cfg(feature = "ew")]
        let mut strobes = self.strobes(ctx, frame, &sweep, &jammers);

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
            let covered = |p: Vector3, position_sigma: f64| {
                let (range, az, el) = frame.look_at(p);
                Coverage {
                    in_beam: illumination(&sweep, az, el, 0.0, antenna, dim).is_some(),
                    confident: confidently_illuminated(
                        &sweep,
                        az,
                        el,
                        range,
                        position_sigma,
                        antenna,
                        dim,
                    ),
                }
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
        #[cfg(feature = "ew")]
        out.append(&mut strobes);
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
