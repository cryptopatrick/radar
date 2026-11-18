//! The sensor framework: the [`SensorModel`] trait, the context it runs in
//! and the [`Observation`]s it produces.

use crate::environment::Environment;
use crate::error::RadarError;
use crate::geometry::Pose3;
use crate::ids::SensorId;
use crate::radar::Measurement;
use crate::random::RandomSource;
use crate::time::{SimDuration, SimTime};
use crate::world::{PlatformState, WorldSnapshot};

/// Everything a sensor sees during one update.
pub struct SensorContext<'a> {
    /// The sensor being updated.
    pub sensor: SensorId,
    /// The time of this update.
    pub time: SimTime,
    /// The sensor's update period (the time covered by this update).
    pub interval: SimDuration,
    /// The carrying platform, already extrapolated to `time`.
    pub platform: &'a PlatformState,
    /// The sensor's mount pose relative to the platform body.
    pub mount: Pose3,
    /// The current world snapshot (targets sorted by id).
    pub world: &'a WorldSnapshot,
    /// Line-of-sight occlusion.
    pub environment: &'a dyn Environment,
    /// The sensor's own random stream.
    pub rng: &'a mut dyn RandomSource,
}

/// A sensor that the simulation updates once per period.
pub trait SensorModel: Send + std::fmt::Debug {
    /// Runs one update, appending what the sensor reports to `out`.
    ///
    /// # Errors
    /// An error aborts the current [`Simulation::step`](crate::simulation::Simulation::step).
    fn update(
        &mut self,
        ctx: &mut SensorContext<'_>,
        out: &mut Vec<Observation>,
    ) -> Result<(), RadarError>;
}

/// Something a sensor reports.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Observation {
    /// A detection (real or false alarm).
    Measurement(Measurement),
    /// The scan pattern completed a full scan during the update.
    ScanCompleted {
        /// The reporting sensor.
        sensor: SensorId,
        /// Time of the update in which the scan completed.
        time: SimTime,
    },
}
