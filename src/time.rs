//! Simulation time.

use crate::error::RadarError;
use std::ops::{Add, Sub};

/// An instant in simulation time, in nanoseconds since the start.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SimTime(u64);

/// A span of simulation time, in nanoseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SimDuration(u64);

impl SimTime {
    /// The start of simulation time.
    pub const ZERO: SimTime = SimTime(0);

    /// Creates a time from nanoseconds.
    pub const fn from_nanos(n: u64) -> Self {
        SimTime(n)
    }

    /// Returns the time in nanoseconds.
    pub const fn as_nanos(self) -> u64 {
        self.0
    }

    /// Returns the time in seconds.
    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 * 1e-9
    }

    /// Adds a duration, returning `None` on overflow.
    pub fn checked_add(self, d: SimDuration) -> Option<SimTime> {
        self.0.checked_add(d.0).map(SimTime)
    }

    /// Returns the duration since `earlier`, or `None` if `earlier` is later.
    pub fn duration_since(self, earlier: SimTime) -> Option<SimDuration> {
        self.0.checked_sub(earlier.0).map(SimDuration)
    }
}

impl SimDuration {
    /// A zero-length duration.
    pub const ZERO: SimDuration = SimDuration(0);

    /// Creates a duration from nanoseconds.
    pub const fn from_nanos(n: u64) -> Self {
        SimDuration(n)
    }

    /// Creates a duration from milliseconds (saturating).
    pub const fn from_millis(ms: u64) -> Self {
        SimDuration(ms.saturating_mul(1_000_000))
    }

    /// Creates a duration from whole seconds (saturating).
    pub const fn from_secs(s: u64) -> Self {
        SimDuration(s.saturating_mul(1_000_000_000))
    }

    /// Creates a duration from seconds, rounding to the nearest nanosecond.
    ///
    /// # Errors
    /// Returns `InvalidConfig` if `s` is negative or not finite.
    pub fn from_secs_f64(s: f64) -> Result<Self, RadarError> {
        if !s.is_finite() || s < 0.0 {
            return Err(RadarError::InvalidConfig {
                field: "duration",
                reason: format!("must be finite and >= 0, got {s}"),
            });
        }
        // Float-to-int `as` saturates, so huge values clamp to u64::MAX.
        Ok(SimDuration((s * 1e9).round() as u64))
    }

    /// Returns the duration in seconds.
    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 * 1e-9
    }

    /// Returns the duration in nanoseconds.
    pub const fn as_nanos(self) -> u64 {
        self.0
    }

    /// Returns `true` if the duration is zero.
    pub const fn is_zero(self) -> bool {
        self.0 == 0
    }
}

impl Add<SimDuration> for SimTime {
    type Output = SimTime;
    fn add(self, d: SimDuration) -> SimTime {
        SimTime(self.0.saturating_add(d.0))
    }
}

impl Add for SimDuration {
    type Output = SimDuration;
    fn add(self, o: SimDuration) -> SimDuration {
        SimDuration(self.0.saturating_add(o.0))
    }
}

impl Sub for SimDuration {
    type Output = SimDuration;
    fn sub(self, o: SimDuration) -> SimDuration {
        SimDuration(self.0.saturating_sub(o.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_roundtrip() {
        assert_eq!(
            SimDuration::from_secs_f64(0.1).unwrap().as_nanos(),
            100_000_000
        );
    }

    #[test]
    fn negative_duration_rejected() {
        assert!(SimDuration::from_secs_f64(-1.0).is_err());
        assert!(SimDuration::from_secs_f64(f64::NAN).is_err());
    }

    #[test]
    fn time_ordering() {
        assert!(SimTime::ZERO + SimDuration::from_millis(5) > SimTime::ZERO);
    }

    #[test]
    fn no_drift() {
        let mut t = SimTime::ZERO;
        for _ in 0..10_000 {
            t = t + SimDuration::from_millis(100);
        }
        assert_eq!(t, SimTime::ZERO + SimDuration::from_secs(1000));
    }

    #[test]
    fn duration_since_and_checked_add() {
        let a = SimTime::from_nanos(5);
        let b = SimTime::from_nanos(9);
        assert_eq!(b.duration_since(a), Some(SimDuration::from_nanos(4)));
        assert_eq!(a.duration_since(b), None);
        assert_eq!(
            SimTime::from_nanos(u64::MAX).checked_add(SimDuration::from_nanos(1)),
            None
        );
    }
}
