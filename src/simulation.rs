//! The simulation: sensors, the world snapshot, scheduling and observation
//! queues.

#[cfg(feature = "diagnostics")]
use crate::diagnostics::DiagnosticRecord;
use crate::environment::{Environment, OpenSky};
use crate::error::RadarError;
use crate::geometry::{Pose3, wrap_two_pi};
use crate::ids::{PlatformId, SensorId};
use crate::radar::config::RadarConfig;
use crate::radar::scan::ScanConfig;
use crate::radar::sensor::RadarSensor;
use crate::random::{ChaChaStream, RandomSource, RandomSourceFactory, Seed, StreamId};
use crate::sensor::{Observation, SensorContext, SensorModel};
use crate::time::{SimDuration, SimTime};
#[cfg(feature = "tracking")]
use crate::tracking::Track;
use crate::world::{PlatformState, WorldSnapshot};
use std::collections::BTreeMap;
use std::f64::consts::FRAC_PI_2;
use std::fmt;

/// How [`Simulation::step`] splits a time step.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StepPolicy {
    /// Advance the clock in one sub-step (default).
    #[default]
    SingleStep,
    /// Split the step into sub-steps of at most `max_step` (must be > 0).
    Subdivide {
        /// Longest sub-step.
        max_step: SimDuration,
    },
}

/// Simulation settings: master seed (default `Seed(0)`) and step policy
/// (default [`StepPolicy::SingleStep`]).
pub struct SimulationConfig {
    seed: Seed,
    step_policy: StepPolicy,
    factory: Option<RandomSourceFactory>,
}

impl fmt::Debug for SimulationConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SimulationConfig")
            .field("seed", &self.seed)
            .field("step_policy", &self.step_policy)
            .field("random_source_factory", &self.factory.is_some())
            .finish()
    }
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            seed: Seed(0),
            step_policy: StepPolicy::SingleStep,
            factory: None,
        }
    }
}

impl SimulationConfig {
    /// Sets the master seed.
    pub fn seed(mut self, seed: Seed) -> Self {
        self.seed = seed;
        self
    }

    /// Supplies the random source for every per-sensor stream, replacing
    /// the seeded ChaCha8 streams. The factory is called with
    /// `StreamId::Sensor(id)` when a sensor is added and again on
    /// [`Simulation::reset`]. Use it for hardware or other external noise.
    ///
    /// This is non-deterministic: unless the factory itself is replayable,
    /// runs cannot be reproduced, `reset` does not reproduce the run, and
    /// [`Simulation::is_deterministic`] returns `false`. The seed is then
    /// not used for sensor streams.
    pub fn random_source_factory(mut self, f: RandomSourceFactory) -> Self {
        self.factory = Some(f);
        self
    }

    /// Sets the step policy.
    pub fn step_policy(mut self, policy: StepPolicy) -> Self {
        self.step_policy = policy;
        self
    }
}

/// A snapshot of one sensor's scheduling and beam state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorStatus {
    /// Boresight azimuth in the scan frame, rad (0 for non-radar sensors).
    pub boresight_azimuth: f64,
    /// Boresight elevation in the scan frame, rad (0 for non-radar sensors).
    pub boresight_elevation: f64,
    /// When the sensor next updates.
    pub next_due: SimTime,
    /// Whether the sensor is enabled.
    pub enabled: bool,
}

/// The model behind a sensor entry.
#[derive(Debug)]
enum SensorKind {
    Radar(Box<RadarSensor>),
    Custom(Box<dyn SensorModel>),
}

impl SensorKind {
    fn model_mut(&mut self) -> &mut dyn SensorModel {
        match self {
            SensorKind::Radar(r) => r.as_mut(),
            SensorKind::Custom(m) => m.as_mut(),
        }
    }
}

/// One registered sensor.
struct SensorEntry {
    platform: PlatformId,
    mount: Pose3,
    period: SimDuration,
    next_due: SimTime,
    enabled: bool,
    kind: SensorKind,
    rng: Box<dyn RandomSource + Send>,
    queue: Vec<Observation>,
}

/// A deterministic radar simulation.
///
/// Sensors are updated in ascending [`SensorId`] order on their own
/// periods; each owns a random stream derived from the seed and its id, so
/// a run is reproducible from the seed and the sequence of calls.
pub struct Simulation {
    seed: Seed,
    step_policy: StepPolicy,
    factory: Option<RandomSourceFactory>,
    now: SimTime,
    world: WorldSnapshot,
    sensors: BTreeMap<SensorId, SensorEntry>,
}

impl fmt::Debug for Simulation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Simulation")
            .field("seed", &self.seed)
            .field("step_policy", &self.step_policy)
            .field("now", &self.now)
            .field("sensors", &self.sensors.keys().collect::<Vec<_>>())
            .finish_non_exhaustive()
    }
}

fn make_stream(
    seed: Seed,
    factory: Option<&RandomSourceFactory>,
    id: SensorId,
) -> Box<dyn RandomSource + Send> {
    match factory {
        Some(f) => f(StreamId::Sensor(id)),
        None => Box::new(ChaChaStream::new(seed, StreamId::Sensor(id))),
    }
}

fn invalid(field: &'static str, reason: &str) -> RadarError {
    RadarError::InvalidConfig {
        field,
        reason: reason.into(),
    }
}

impl Simulation {
    /// Creates an empty simulation at time zero with an empty world.
    pub fn new(config: SimulationConfig) -> Self {
        Self {
            seed: config.seed,
            step_policy: config.step_policy,
            factory: config.factory,
            now: SimTime::ZERO,
            world: WorldSnapshot::new(SimTime::ZERO),
            sensors: BTreeMap::new(),
        }
    }

    /// Adds a radar on `platform` at `mount` (relative to the platform
    /// body). Its first update is one `update_period` from now.
    ///
    /// # Errors
    /// `DuplicateSensor` if `id` exists; `InvalidConfig` if the config or
    /// mount is invalid.
    pub fn add_radar(
        &mut self,
        id: SensorId,
        platform: PlatformId,
        mount: Pose3,
        cfg: RadarConfig,
    ) -> Result<(), RadarError> {
        cfg.validate()?;
        let period = cfg.update_period;
        let kind = SensorKind::Radar(Box::new(RadarSensor::new(id, cfg)));
        self.insert(id, platform, mount, period, kind)
    }

    /// Adds a user-supplied sensor updated every `update_period`, first one
    /// period from now.
    ///
    /// # Errors
    /// `DuplicateSensor` if `id` exists; `InvalidConfig` if the period is
    /// zero or the mount is invalid.
    pub fn add_sensor(
        &mut self,
        id: SensorId,
        platform: PlatformId,
        mount: Pose3,
        update_period: SimDuration,
        model: Box<dyn SensorModel>,
    ) -> Result<(), RadarError> {
        if update_period.is_zero() {
            return Err(invalid("update_period", "must be > 0"));
        }
        self.insert(
            id,
            platform,
            mount,
            update_period,
            SensorKind::Custom(model),
        )
    }

    fn insert(
        &mut self,
        id: SensorId,
        platform: PlatformId,
        mount: Pose3,
        period: SimDuration,
        kind: SensorKind,
    ) -> Result<(), RadarError> {
        if !(mount.position.is_finite() && mount.rotation.is_valid()) {
            return Err(invalid(
                "mount",
                "position must be finite, rotation unit-norm",
            ));
        }
        if self.sensors.contains_key(&id) {
            return Err(RadarError::DuplicateSensor(id));
        }
        let entry = SensorEntry {
            platform,
            mount,
            period,
            next_due: self.now + period,
            enabled: true,
            kind,
            rng: make_stream(self.seed, self.factory.as_ref(), id),
            queue: Vec::new(),
        };
        self.sensors.insert(id, entry);
        Ok(())
    }

    fn entry(&mut self, id: SensorId) -> Result<&mut SensorEntry, RadarError> {
        self.sensors
            .get_mut(&id)
            .ok_or(RadarError::UnknownSensor(id))
    }

    fn radar(&mut self, id: SensorId) -> Result<&mut RadarSensor, RadarError> {
        match &mut self.entry(id)?.kind {
            SensorKind::Radar(r) => Ok(r),
            SensorKind::Custom(_) => Err(invalid("scan", "sensor is not a radar")),
        }
    }

    /// Removes a sensor and its queued observations.
    ///
    /// # Errors
    /// `UnknownSensor` if `id` does not exist.
    pub fn remove_sensor(&mut self, id: SensorId) -> Result<(), RadarError> {
        self.sensors
            .remove(&id)
            .map(|_| ())
            .ok_or(RadarError::UnknownSensor(id))
    }

    /// Enables or disables a sensor. A disabled sensor is not updated; when
    /// re-enabled its next update is one period from now.
    ///
    /// # Errors
    /// `UnknownSensor` if `id` does not exist.
    pub fn set_enabled(&mut self, id: SensorId, enabled: bool) -> Result<(), RadarError> {
        let now = self.now;
        let e = self.entry(id)?;
        if enabled && !e.enabled {
            e.next_due = now + e.period;
        }
        e.enabled = enabled;
        Ok(())
    }

    /// Commands a [`ScanConfig::Steered`] radar to slew toward
    /// (`azimuth`, `elevation`) in its scan frame.
    ///
    /// # Errors
    /// `UnknownSensor`; `InvalidConfig { field: "scan" }` if the sensor is
    /// not a steered radar; `InvalidConfig { field: "beam" }` for
    /// non-finite angles or an elevation outside `[−π/2, π/2]`.
    pub fn command_beam(
        &mut self,
        id: SensorId,
        azimuth: f64,
        elevation: f64,
    ) -> Result<(), RadarError> {
        let radar = self.radar(id)?;
        if !matches!(radar.config().scan, ScanConfig::Steered { .. }) {
            return Err(invalid("scan", "command_beam requires a Steered scan"));
        }
        let (az, el) = beam_angles(azimuth, elevation)?;
        radar.scan_state_mut().command = Some((az, el));
        Ok(())
    }

    /// Points a radar's boresight at (`azimuth`, `elevation`) in its scan
    /// frame immediately, with no sweep, clearing any pending command.
    ///
    /// On a [`ScanConfig::Fixed`] scan this has no lasting effect: the next
    /// update re-applies the configured pointing.
    ///
    /// # Errors
    /// `UnknownSensor`; `InvalidConfig { field: "scan" }` if the sensor is
    /// not a radar; `InvalidConfig { field: "beam" }` for bad angles.
    pub fn point_beam(
        &mut self,
        id: SensorId,
        azimuth: f64,
        elevation: f64,
    ) -> Result<(), RadarError> {
        let radar = self.radar(id)?;
        let (az, el) = beam_angles(azimuth, elevation)?;
        let state = radar.scan_state_mut();
        state.azimuth = az;
        state.elevation = el;
        state.command = None;
        Ok(())
    }

    /// Replaces the world snapshot (targets and emitters are stored sorted
    /// by id).
    ///
    /// # Errors
    /// `InvalidWorld` if the snapshot fails validation;
    /// `TimeWentBackwards` if it is older than the previous snapshot.
    pub fn update_world(&mut self, mut snapshot: WorldSnapshot) -> Result<(), RadarError> {
        snapshot.validate()?;
        if snapshot.time < self.world.time {
            return Err(RadarError::TimeWentBackwards {
                previous: self.world.time,
                given: snapshot.time,
            });
        }
        snapshot.targets.sort_by_key(|t| t.id);
        snapshot.emitters.sort_by_key(|e| e.id);
        self.world = snapshot;
        Ok(())
    }

    /// Advances the clock by `dt`, updating every due sensor. `dt == 0`
    /// does nothing.
    ///
    /// # Errors
    /// `InvalidConfig` for a zero `Subdivide` step or a clock overflow; any
    /// error a sensor model returns (the step stops there). After an error
    /// the simulation state (clock, scan states, queues, random streams) is
    /// unspecified until [`reset`](Self::reset).
    pub fn step(&mut self, dt: SimDuration, env: &dyn Environment) -> Result<(), RadarError> {
        if dt.is_zero() {
            return Ok(());
        }
        let end = self
            .now
            .checked_add(dt)
            .ok_or_else(|| invalid("dt", "simulation time overflow"))?;
        let max_step = match self.step_policy {
            StepPolicy::SingleStep => dt,
            StepPolicy::Subdivide { max_step } if max_step.is_zero() => {
                return Err(invalid("step_policy", "max_step must be > 0"));
            }
            StepPolicy::Subdivide { max_step } => max_step,
        };
        while self.now < end {
            let remaining = end.duration_since(self.now).unwrap_or(SimDuration::ZERO);
            let t_end = self.now + remaining.min(max_step);
            self.run_due(t_end, env)?;
            self.now = t_end;
        }
        Ok(())
    }

    /// Runs every enabled sensor, in ascending id order, once per due time
    /// up to and including `t_end`.
    fn run_due(&mut self, t_end: SimTime, env: &dyn Environment) -> Result<(), RadarError> {
        let world = &self.world;
        for (&id, entry) in self.sensors.iter_mut() {
            if !entry.enabled {
                continue;
            }
            while entry.next_due <= t_end {
                let t_due = entry.next_due;
                run_sensor(id, entry, t_due, world, env)?;
                match t_due.checked_add(entry.period) {
                    Some(next) => entry.next_due = next,
                    None => {
                        // The clock saturates at u64::MAX nanoseconds.
                        entry.next_due = SimTime::from_nanos(u64::MAX);
                        break;
                    }
                }
            }
        }
        Ok(())
    }

    /// [`step`](Self::step) with an [`OpenSky`] environment.
    ///
    /// # Errors
    /// As for [`step`](Self::step).
    pub fn step_open(&mut self, dt: SimDuration) -> Result<(), RadarError> {
        self.step(dt, &OpenSky)
    }

    /// The current simulation time.
    pub fn now(&self) -> SimTime {
        self.now
    }

    /// Appends the sensor's queued observations to `out` and clears the
    /// queue.
    ///
    /// # Errors
    /// `UnknownSensor` if `id` does not exist.
    pub fn drain(&mut self, id: SensorId, out: &mut Vec<Observation>) -> Result<(), RadarError> {
        out.append(&mut self.entry(id)?.queue);
        Ok(())
    }

    /// Appends the ground-truth records of every radar to `out`, in sensor
    /// id order, and clears them: one per (update, target) in target order
    /// plus one per false alarm. Records accumulate until drained.
    #[cfg(feature = "diagnostics")]
    pub fn drain_diagnostics(&mut self, out: &mut Vec<DiagnosticRecord>) {
        for e in self.sensors.values_mut() {
            if let SensorKind::Radar(r) = &mut e.kind {
                r.drain_diagnostics(out);
            }
        }
    }

    /// The sensor's beam and scheduling state.
    ///
    /// # Errors
    /// `UnknownSensor` if `id` does not exist.
    pub fn sensor_status(&self, id: SensorId) -> Result<SensorStatus, RadarError> {
        let e = self.sensors.get(&id).ok_or(RadarError::UnknownSensor(id))?;
        let (az, el) = match &e.kind {
            SensorKind::Radar(r) => (r.scan_state().azimuth, r.scan_state().elevation),
            SensorKind::Custom(_) => (0.0, 0.0),
        };
        Ok(SensorStatus {
            boresight_azimuth: az,
            boresight_elevation: el,
            next_due: e.next_due,
            enabled: e.enabled,
        })
    }

    /// Snapshot copies of a sensor's live tracks in id order (deleted
    /// tracks are excluded). Empty for a radar without a tracker or a
    /// custom sensor.
    ///
    /// # Errors
    /// `UnknownSensor` if `id` does not exist.
    #[cfg(feature = "tracking")]
    pub fn tracks(&self, id: SensorId) -> Result<Vec<Track>, RadarError> {
        let e = self.sensors.get(&id).ok_or(RadarError::UnknownSensor(id))?;
        Ok(match &e.kind {
            SensorKind::Radar(r) => r.tracks(),
            SensorKind::Custom(_) => Vec::new(),
        })
    }

    /// Restores the state right after construction and sensor
    /// registration: time 0, the world an empty snapshot at time 0, each
    /// sensor due one period later, radar scans at their initial state,
    /// queues cleared, tracks dropped (numbering restarts at 1) and random
    /// streams rebuilt from the seed (with a random source factory, the
    /// factory is invoked again and the run is not reproduced). Sensors, their configs and enabled
    /// flags are kept.
    ///
    /// The host must feed the world again with
    /// [`update_world`](Self::update_world); replaying the same calls then
    /// reproduces the same observations. Custom sensor models and custom
    /// scan patterns keep any internal state of their own.
    pub fn reset(&mut self) {
        self.now = SimTime::ZERO;
        self.world = WorldSnapshot::new(SimTime::ZERO);
        for (&id, e) in self.sensors.iter_mut() {
            e.next_due = SimTime::ZERO + e.period;
            e.queue.clear();
            e.rng = make_stream(self.seed, self.factory.as_ref(), id);
            if let SensorKind::Radar(r) = &mut e.kind {
                r.reset();
            }
        }
    }

    /// Whether runs are reproducible from the seed: `false` when a random
    /// source factory is set.
    pub fn is_deterministic(&self) -> bool {
        self.factory.is_none()
    }

    /// The master seed.
    pub fn seed(&self) -> Seed {
        self.seed
    }
}

/// Checks beam angles, returning the azimuth wrapped to `[0, 2π)`.
fn beam_angles(azimuth: f64, elevation: f64) -> Result<(f64, f64), RadarError> {
    if !(azimuth.is_finite() && elevation.is_finite() && elevation.abs() <= FRAC_PI_2) {
        return Err(invalid(
            "beam",
            "angles must be finite with elevation in [-π/2, π/2]",
        ));
    }
    Ok((wrap_two_pi(azimuth), elevation))
}

/// Updates one sensor at `time`, queueing its observations. A sensor whose
/// platform is not in the snapshot is skipped.
fn run_sensor(
    id: SensorId,
    entry: &mut SensorEntry,
    time: SimTime,
    world: &WorldSnapshot,
    env: &dyn Environment,
) -> Result<(), RadarError> {
    let Some(platform) = world.platforms.iter().find(|p| p.id == entry.platform) else {
        return Ok(());
    };
    let platform = PlatformState {
        id: platform.id,
        pose: platform.pose_at(world.time, time),
        velocity: platform.velocity,
    };
    let mut ctx = SensorContext {
        sensor: id,
        time,
        interval: entry.period,
        platform: &platform,
        mount: entry.mount,
        world,
        environment: env,
        rng: &mut *entry.rng,
    };
    entry.kind.model_mut().update(&mut ctx, &mut entry.queue)
}
