//! Realistic radar and sensor simulation for games and simulations.
#![warn(missing_docs)]

pub mod error;
pub mod geometry;
pub mod ids;
pub mod time;
pub mod units;

/// Convenient re-exports of the most common types.
pub mod prelude {
    pub use crate::error::RadarError;
    pub use crate::geometry::{Pose3, Rotation3, Vector2, Vector3};
    pub use crate::ids::{EmitterId, PlatformId, SensorId, TargetId, TrackId, TransponderCode};
    pub use crate::time::{SimDuration, SimTime};
    pub use crate::units::WorldScale;
}
