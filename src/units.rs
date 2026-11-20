//! Physical constants, unit conversions and world scaling.

use crate::error::RadarError;
use crate::time::SimDuration;

/// Speed of light in vacuum, m/s.
pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;
/// Boltzmann constant, J/K.
pub const BOLTZMANN: f64 = 1.380_649e-23;
/// Standard reference noise temperature, K.
pub const T0_KELVIN: f64 = 290.0;

/// Converts degrees to radians.
pub fn deg(d: f64) -> f64 {
    d.to_radians()
}

/// Converts radians to degrees.
pub fn to_deg(r: f64) -> f64 {
    r.to_degrees()
}

/// Converts decibels to a linear power ratio.
pub fn db_to_linear(db: f64) -> f64 {
    10f64.powf(db / 10.0)
}

/// Converts a linear power ratio to decibels.
pub fn linear_to_db(x: f64) -> f64 {
    10.0 * x.log10()
}

/// Maps game-world units and ticks to SI metres and seconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldScale {
    /// Metres per world unit.
    pub metres_per_unit: f64,
    /// Seconds per world tick.
    pub seconds_per_tick: f64,
}

impl Default for WorldScale {
    fn default() -> Self {
        WorldScale {
            metres_per_unit: 1.0,
            seconds_per_tick: 1.0,
        }
    }
}

impl WorldScale {
    /// Creates a scale; both factors must be finite and positive.
    ///
    /// # Errors
    /// Returns `InvalidConfig` if either factor is not finite and > 0.
    pub fn new(metres_per_unit: f64, seconds_per_tick: f64) -> Result<Self, RadarError> {
        for (field, v) in [
            ("metres_per_unit", metres_per_unit),
            ("seconds_per_tick", seconds_per_tick),
        ] {
            if !v.is_finite() || v <= 0.0 {
                return Err(RadarError::InvalidConfig {
                    field,
                    reason: format!("must be finite and > 0, got {v}"),
                });
            }
        }
        Ok(WorldScale {
            metres_per_unit,
            seconds_per_tick,
        })
    }

    /// World units to metres.
    pub fn to_metres(&self, units: f64) -> f64 {
        units * self.metres_per_unit
    }

    /// Metres to world units.
    pub fn to_units(&self, metres: f64) -> f64 {
        metres / self.metres_per_unit
    }

    /// Ticks to seconds.
    pub fn to_seconds(&self, ticks: f64) -> f64 {
        ticks * self.seconds_per_tick
    }

    /// Seconds to ticks.
    pub fn to_ticks(&self, seconds: f64) -> f64 {
        seconds / self.seconds_per_tick
    }

    /// Units per tick to metres per second.
    pub fn speed_to_si(&self, units_per_tick: f64) -> f64 {
        units_per_tick * self.metres_per_unit / self.seconds_per_tick
    }

    /// Metres per second to units per tick.
    pub fn speed_from_si(&self, mps: f64) -> f64 {
        mps * self.seconds_per_tick / self.metres_per_unit
    }

    /// Radians per tick to radians per second.
    pub fn rate_to_si(&self, rad_per_tick: f64) -> f64 {
        rad_per_tick / self.seconds_per_tick
    }

    /// Units per tick squared to metres per second squared.
    pub fn accel_to_si(&self, units_per_tick2: f64) -> f64 {
        units_per_tick2 * self.metres_per_unit / (self.seconds_per_tick * self.seconds_per_tick)
    }

    /// A tick count as a simulation duration, rounded to the nearest nanosecond.
    ///
    /// The result is only meaningful for a scale whose `seconds_per_tick` is
    /// finite and positive, as [`WorldScale::new`] guarantees. The fields are
    /// public, so a scale assembled by hand is not checked: a NaN or negative
    /// `seconds_per_tick` yields a zero duration and `+∞` saturates at the
    /// largest representable duration. Build scales with
    /// [`WorldScale::new`] to rule this out.
    pub fn duration(&self, ticks: u64) -> SimDuration {
        // Float-to-int `as` saturates for out-of-range values.
        SimDuration::from_nanos((ticks as f64 * self.seconds_per_tick * 1e9).round() as u64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn db() {
        assert!((db_to_linear(10.0) - 10.0).abs() < 1e-12);
        assert!((linear_to_db(100.0) - 20.0).abs() < 1e-12);
    }

    #[test]
    fn world_scale() {
        let s = WorldScale::new(10.0, 0.5).unwrap();
        assert_eq!(s.to_metres(3.0), 30.0);
        assert_eq!(s.speed_to_si(8.0), 160.0);
        assert_eq!(s.rate_to_si(1.0), 2.0);
        assert_eq!(s.accel_to_si(1.0), 40.0);
        assert!(WorldScale::new(0.0, 1.0).is_err());
        assert!(WorldScale::new(1.0, f64::NAN).is_err());
        assert_eq!(s.duration(3).as_nanos(), 1_500_000_000);
    }

    #[test]
    fn deg_roundtrip() {
        assert!((to_deg(deg(90.0)) - 90.0).abs() < 1e-12);
    }
}
