//! The environment the simulation runs in (line-of-sight occlusion).

use crate::geometry::Vector3;

/// Describes occlusion of the world. Implement this to add terrain or buildings.
pub trait Environment {
    /// Returns true if the straight line from `from` to `to` is blocked.
    fn line_of_sight_blocked(&self, _from: Vector3, _to: Vector3) -> bool {
        false
    }
}

/// An environment with nothing in the way.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct OpenSky;

impl Environment for OpenSky {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn open_sky_never_blocks() {
        assert!(!OpenSky.line_of_sight_blocked(Vector3::ZERO, Vector3::new(1e6, 1e6, 1e3)));
    }
}
