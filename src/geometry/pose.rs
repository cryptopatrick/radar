//! Position plus orientation.

use super::rotation::Rotation3;
use super::vector::Vector3;

/// A position and orientation (body-to-world rotation).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose3 {
    /// Position in the parent frame.
    pub position: Vector3,
    /// Orientation (body to parent).
    pub rotation: Rotation3,
}

impl Pose3 {
    /// The identity pose.
    pub const IDENTITY: Pose3 = Pose3 {
        position: Vector3::ZERO,
        rotation: Rotation3::IDENTITY,
    };

    /// Creates a pose.
    pub fn new(position: Vector3, rotation: Rotation3) -> Self {
        Pose3 { position, rotation }
    }

    /// Creates a level pose at `p` with a compass heading.
    pub fn from_position_heading(p: Vector3, heading: f64) -> Self {
        Pose3::new(p, Rotation3::from_heading(heading))
    }

    /// Transforms a local point into the parent (world) frame.
    pub fn to_world(self, local: Vector3) -> Vector3 {
        self.rotation.rotate(local) + self.position
    }

    /// Transforms a parent (world) point into the local frame.
    pub fn to_local(self, world: Vector3) -> Vector3 {
        self.rotation.inverse().rotate(world - self.position)
    }

    /// Composes a child pose expressed in this pose's frame.
    pub fn compose(self, child: Pose3) -> Pose3 {
        Pose3::new(
            self.to_world(child.position),
            child.rotation.then(self.rotation),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::deg;

    #[test]
    fn pose_roundtrip() {
        let p = Pose3::new(
            Vector3::new(10.0, 20.0, 30.0),
            Rotation3::from_heading_pitch_roll(deg(30.0), deg(10.0), deg(5.0)),
        );
        let v = Vector3::new(1.0, -2.0, 3.5);
        assert!(p.to_local(p.to_world(v)).distance(v) < 1e-9);
    }

    #[test]
    fn compose_matches_chained_transform() {
        let a = Pose3::from_position_heading(Vector3::new(1.0, 2.0, 3.0), deg(40.0));
        let b = Pose3::new(
            Vector3::new(0.5, 1.0, 0.0),
            Rotation3::from_heading_pitch_roll(deg(10.0), deg(5.0), 0.0),
        );
        let v = Vector3::new(2.0, 1.0, -1.0);
        assert!(a.compose(b).to_world(v).distance(a.to_world(b.to_world(v))) < 1e-9);
    }
}
