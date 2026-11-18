//! Realistic radar and sensor simulation for games and simulations.
#![warn(missing_docs)]

pub mod environment;
pub mod error;
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
    pub use crate::environment::{Environment, OpenSky};
    pub use crate::error::RadarError;
    pub use crate::geometry::{Pose3, Rotation3, Vector2, Vector3};
    pub use crate::ids::{EmitterId, PlatformId, SensorId, TargetId, TrackId, TransponderCode};
    pub use crate::radar::{
        BeamPattern, Calibration, DetectionConfig, DetectionContext, DetectionModel,
        DetectionResult, Dimension, Measurement, MeasurementConfig, MeasurementSigma, RadarConfig,
        RangeLimits, ScanConfig, ScanFrame,
    };
    pub use crate::random::{RandomSource, Seed, StreamId};
    pub use crate::sensor::{Observation, SensorContext, SensorModel};
    pub use crate::simulation::{SensorStatus, Simulation, SimulationConfig, StepPolicy};
    pub use crate::time::{SimDuration, SimTime};
    #[cfg(feature = "tracking")]
    pub use crate::tracking::TrackerConfig;
    pub use crate::units::WorldScale;
    pub use crate::world::{PlatformState, RadarSignature, Swerling, TargetState, WorldSnapshot};
}
