//! Ground-truth diagnostics (feature `diagnostics`).
//!
//! Observations never reveal ground truth; this channel exposes it for
//! debugging and tests. Recording never draws random numbers, so enabling
//! the feature does not change any observation.
//!
//! Records accumulate in each radar until
//! [`Simulation::drain_diagnostics`](crate::simulation::Simulation::drain_diagnostics)
//! is called, so drain regularly in long runs. Targets carried by the
//! radar's own platform are never looked at, so they produce no record.

use crate::ids::{SensorId, TargetId};
use crate::radar::Measurement;
use crate::time::SimTime;

/// What happened to a target (or false alarm) during one sensor update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum DiagnosticOutcome {
    /// The target was detected and measured.
    Detected,
    /// The target was illuminated and unobstructed but not detected.
    Missed,
    /// The line of sight to the target was blocked.
    Occluded,
    /// The target was outside the beam this update.
    OutOfBeam,
    /// The target was outside the minimum or instrumented range.
    OutOfRange,
    /// The measurement is a false alarm.
    FalseAlarm,
}

/// Ground truth for one target in one sensor update, or one false alarm.
#[derive(Debug, Clone, PartialEq)]
pub struct DiagnosticRecord {
    /// The reporting sensor.
    pub sensor: SensorId,
    /// Time of the update.
    pub time: SimTime,
    /// The target (`None` for a false alarm).
    pub target: Option<TargetId>,
    /// True range to the target, m (`None` for a false alarm).
    pub true_range: Option<f64>,
    /// SNR in dB for a look: the measured SNR when the detector provides
    /// one (detections, misses and false alarms alike), otherwise the mean
    /// SNR; `None` if the detector computes neither.
    pub snr_db: Option<f64>,
    /// Analytic probability of detection for the look, if one was made.
    pub pd: Option<f64>,
    /// What happened.
    pub outcome: DiagnosticOutcome,
    /// The emitted measurement (detections and false alarms only), equal to
    /// the one in the observation stream.
    pub measurement: Option<Measurement>,
}
