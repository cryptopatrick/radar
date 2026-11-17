//! Unit-quaternion rotations.

use super::vector::Vector3;

/// A 3D rotation stored as a unit quaternion.
///
/// Body frame: forward = +y, right = +x, up = +z. World frame: x = east,
/// y = north, z = up.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Rotation3 {
    w: f64,
    x: f64,
    y: f64,
    z: f64,
}

impl Rotation3 {
    /// The identity rotation.
    pub const IDENTITY: Rotation3 = Rotation3 {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    fn from_axis_angle(axis: Vector3, angle: f64) -> Rotation3 {
        let (s, c) = (angle / 2.0).sin_cos();
        Rotation3 {
            w: c,
            x: axis.x * s,
            y: axis.y * s,
            z: axis.z * s,
        }
    }

    /// Builds a body-to-world rotation from a compass heading (clockwise
    /// about +z), nose-up pitch and right-wing-down roll, all in radians.
    /// Roll is applied first, then pitch, then heading.
    pub fn from_heading_pitch_roll(heading: f64, pitch: f64, roll: f64) -> Rotation3 {
        let roll_r = Self::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), roll);
        let pitch_r = Self::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), pitch);
        let head_r = Self::from_heading(heading);
        roll_r.then(pitch_r).then(head_r)
    }

    /// Rotation by a compass heading only (clockwise about +z).
    pub fn from_heading(h: f64) -> Rotation3 {
        Self::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), -h)
    }

    /// Rotates a vector.
    pub fn rotate(self, v: Vector3) -> Vector3 {
        let q = Vector3::new(self.x, self.y, self.z);
        let t = q.cross(v) * 2.0;
        v + t * self.w + q.cross(t)
    }

    /// The inverse rotation.
    pub fn inverse(self) -> Rotation3 {
        Rotation3 {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// Applies `self` first, then `other`.
    pub fn then(self, other: Rotation3) -> Rotation3 {
        let (a, b) = (other, self);
        let r = Rotation3 {
            w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        };
        let n = (r.w * r.w + r.x * r.x + r.y * r.y + r.z * r.z).sqrt();
        if n > 0.0 && n.is_finite() {
            Rotation3 {
                w: r.w / n,
                x: r.x / n,
                y: r.y / n,
                z: r.z / n,
            }
        } else {
            Rotation3::IDENTITY
        }
    }

    /// Extracts `(heading, pitch, roll)`; the inverse of
    /// [`from_heading_pitch_roll`](Self::from_heading_pitch_roll) for pitch
    /// in `(-π/2, π/2)`. Heading is in `[0, 2π)`.
    pub fn heading_pitch_roll(self) -> (f64, f64, f64) {
        let fwd = self.rotate(Vector3::new(0.0, 1.0, 0.0));
        let right = self.rotate(Vector3::new(1.0, 0.0, 0.0));
        let up = self.rotate(Vector3::new(0.0, 0.0, 1.0));
        let heading = super::angle::wrap_two_pi(fwd.x.atan2(fwd.y));
        let pitch = fwd.z.clamp(-1.0, 1.0).asin();
        let roll = (-right.z).atan2(up.z);
        (heading, pitch, roll)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::angle::wrap_pi;
    use crate::units::deg;
    use proptest::prelude::*;

    fn close(a: Vector3, b: Vector3) -> bool {
        a.distance(b) < 1e-9
    }

    #[test]
    fn heading_rotation() {
        let r = Rotation3::from_heading(deg(90.0));
        assert!(close(
            r.rotate(Vector3::new(0., 1., 0.)),
            Vector3::new(1., 0., 0.)
        ));
    }

    #[test]
    fn pitch_up() {
        let r = Rotation3::from_heading_pitch_roll(0.0, deg(90.0), 0.0);
        assert!(close(
            r.rotate(Vector3::new(0., 1., 0.)),
            Vector3::new(0., 0., 1.)
        ));
    }

    #[test]
    fn roll_lowers_right_wing() {
        let r = Rotation3::from_heading_pitch_roll(0.0, 0.0, deg(90.0));
        assert!(close(
            r.rotate(Vector3::new(1., 0., 0.)),
            Vector3::new(0., 0., -1.)
        ));
    }

    #[test]
    fn hpr_roundtrip() {
        let (h, p, r) = (deg(30.0), deg(10.0), deg(-5.0));
        let (h2, p2, r2) = Rotation3::from_heading_pitch_roll(h, p, r).heading_pitch_roll();
        assert!((h - h2).abs() < 1e-9 && (p - p2).abs() < 1e-9 && (r - r2).abs() < 1e-9);
    }

    proptest! {
        #[test]
        fn prop_rotation_preserves_length(
            h in -7.0f64..7.0, p in -1.5f64..1.5, r in -3.0f64..3.0,
            x in -1e3f64..1e3, y in -1e3f64..1e3, z in -1e3f64..1e3,
        ) {
            let v = Vector3::new(x, y, z);
            let w = Rotation3::from_heading_pitch_roll(h, p, r).rotate(v);
            prop_assert!((w.length() - v.length()).abs() < 1e-8);
        }

        #[test]
        fn prop_hpr_roundtrip(h in 0.0f64..6.0, p in -1.4f64..1.4, r in -3.1f64..3.1) {
            let (h2, p2, r2) = Rotation3::from_heading_pitch_roll(h, p, r).heading_pitch_roll();
            prop_assert!(wrap_pi(h - h2).abs() < 1e-8);
            prop_assert!((p - p2).abs() < 1e-8);
            prop_assert!(wrap_pi(r - r2).abs() < 1e-8);
        }
    }
}
