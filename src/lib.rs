//! Realistic radar and sensor simulation for games and simulations.
#![warn(missing_docs)]

pub mod environment;
pub mod error;
pub mod geometry;
pub mod ids;
pub mod random;
pub mod time;
pub mod units;
pub mod world;

/// Convenient re-exports of the most common types.
pub mod prelude {
    pub use crate::environment::{Environment, OpenSky};
    pub use crate::error::RadarError;
    pub use crate::geometry::{Pose3, Rotation3, Vector2, Vector3};
    pub use crate::ids::{EmitterId, PlatformId, SensorId, TargetId, TrackId, TransponderCode};
    pub use crate::random::{RandomSource, Seed, StreamId};
    pub use crate::time::{SimDuration, SimTime};
    pub use crate::units::WorldScale;
    pub use crate::world::{PlatformState, RadarSignature, Swerling, TargetState, WorldSnapshot};
}
