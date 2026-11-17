//! Error type for the crate.

use crate::ids::SensorId;
use crate::time::SimTime;

/// Errors returned by validation and simulation operations.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum RadarError {
    /// A configuration field holds an invalid value.
    #[error("invalid configuration: {field}: {reason}")]
    InvalidConfig {
        /// Name of the offending field.
        field: &'static str,
        /// Why the value is invalid.
        reason: String,
    },
    /// The world description is invalid.
    #[error("invalid world: {0}")]
    InvalidWorld(String),
    /// No sensor with this id exists.
    #[error("unknown sensor {0:?}")]
    UnknownSensor(SensorId),
    /// A sensor with this id already exists.
    #[error("duplicate sensor {0:?}")]
    DuplicateSensor(SensorId),
    /// Entropy could not be obtained from the operating system.
    #[error("entropy unavailable: {0}")]
    EntropyUnavailable(String),
    /// Time was advanced to an instant before the current one.
    #[error("time went backwards: {previous:?} -> {given:?}")]
    TimeWentBackwards {
        /// The current simulation time.
        previous: SimTime,
        /// The earlier time that was supplied.
        given: SimTime,
    },
    /// A model reported an error.
    #[error("model error: {0}")]
    Model(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn invalid_config_displays_field() {
        assert!(
            RadarError::InvalidConfig {
                field: "beamwidth_az",
                reason: "must be > 0".into()
            }
            .to_string()
            .contains("beamwidth_az")
        );
    }
}
