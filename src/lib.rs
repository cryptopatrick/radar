//! Realistic radar and sensor simulation for games and simulations.
#![warn(missing_docs)]

pub mod error;
pub mod ids;
pub mod time;
pub mod units;

/// Convenient re-exports of the most common types.
pub mod prelude {
    pub use crate::error::RadarError;
    pub use crate::ids::{EmitterId, PlatformId, SensorId, TargetId, TrackId, TransponderCode};
    pub use crate::time::{SimDuration, SimTime};
    pub use crate::units::WorldScale;
}
