//! Realistic radar and sensor simulation for games and simulations.
#![warn(missing_docs)]

#[cfg(feature = "diagnostics")]
pub mod diagnostics;
pub mod environment;
pub mod error;
#[cfg(feature = "ew")]
pub(crate) mod ew;
pub mod geometry;
pub mod ids;
pub mod radar;
pub mod random;
pub mod sensor;
pub mod simulation;
pub mod time;
#[cfg(feature = "tracking")]
pub mod tracking;
pub mod units;
pub mod world;

/// Convenient re-exports of the most common types.
pub mod prelude {
    #[cfg(feature = "diagnostics")]
    pub use crate::diagnostics::{DiagnosticOutcome, DiagnosticRecord};
    pub use crate::environment::{Environment, OpenSky};
    pub use crate::error::RadarError;
    pub use crate::geometry::{Pose3, Rotation3, Vector2, Vector3};
    pub use crate::ids::{EmitterId, PlatformId, SensorId, TargetId, TrackId, TransponderCode};
    pub use crate::radar::{
        AntennaConfig, BeamPattern, Calibration, DetectionConfig, DetectionContext, DetectionModel,
        DetectionResult, Dimension, IdentificationConfig, JammingReference, Measurement,
        MeasurementConfig, MeasurementSigma, RadarConfig, RangeEquationConfig,
        RangeEquationDetector, RangeLimits, ScanConfig, ScanFrame, analytic_pd, resolution_cells,
    };
    pub use crate::random::{ChaChaStream, RandomSource, RandomSourceFactory, Seed, StreamId};
    pub use crate::sensor::{Observation, SensorContext, SensorModel};
    pub use crate::simulation::{SensorStatus, Simulation, SimulationConfig, StepPolicy};
    pub use crate::time::{SimDuration, SimTime};
    #[cfg(feature = "tracking")]
    pub use crate::tracking::{
        AlphaBeta, Covariance6, KalmanCv, Track, TrackManagementConfig, TrackState, TrackStatus,
        Tracker, TrackerConfig,
    };
    pub use crate::units::WorldScale;
    pub use crate::world::{
        Band, EmitterKind, EmitterState, PlatformState, RadarSignature, Swerling, TargetState,
        WorldSnapshot,
    };
}
