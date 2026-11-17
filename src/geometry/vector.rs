//! 2D and 3D vectors.

use std::ops::{Add, Mul, Neg, Sub};

/// A 3D vector (x = east, y = north, z = up in the world frame).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vector3 {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
    /// Z component.
    pub z: f64,
}

impl Vector3 {
    /// The zero vector.
    pub const ZERO: Vector3 = Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// Creates a vector from components.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Vector3 { x, y, z }
    }

    /// Dot product.
    pub fn dot(self, o: Vector3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    /// Cross product.
    pub fn cross(self, o: Vector3) -> Vector3 {
        Vector3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    /// Squared Euclidean length.
    pub fn length_squared(self) -> f64 {
        self.dot(self)
    }

    /// Euclidean length.
    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }

    /// Unit vector in the same direction, or `None` when the length is
    /// below 1e-12 or not finite.
    pub fn normalized(self) -> Option<Vector3> {
        let l = self.length();
        if l.is_finite() && l >= 1e-12 {
            Some(Vector3::new(self.x / l, self.y / l, self.z / l))
        } else {
            None
        }
    }

    /// Distance to another point.
    pub fn distance(self, o: Vector3) -> f64 {
        (self - o).length()
    }

    /// True when all components are finite.
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl Add for Vector3 {
    type Output = Vector3;
    fn add(self, o: Vector3) -> Vector3 {
        Vector3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for Vector3 {
    type Output = Vector3;
    fn sub(self, o: Vector3) -> Vector3 {
        Vector3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Neg for Vector3 {
    type Output = Vector3;
    fn neg(self) -> Vector3 {
        Vector3::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f64> for Vector3 {
    type Output = Vector3;
    fn mul(self, k: f64) -> Vector3 {
        Vector3::new(self.x * k, self.y * k, self.z * k)
    }
}

/// A 2D vector (x = east, y = north).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vector2 {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
}

impl Vector2 {
    /// The zero vector.
    pub const ZERO: Vector2 = Vector2 { x: 0.0, y: 0.0 };

    /// Creates a vector from components.
    pub const fn new(x: f64, y: f64) -> Self {
        Vector2 { x, y }
    }

    /// Dot product.
    pub fn dot(self, o: Vector2) -> f64 {
        self.x * o.x + self.y * o.y
    }

    /// Euclidean length.
    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// Lifts to 3D with `z = 0`.
    pub fn to_3d(self) -> Vector3 {
        Vector3::new(self.x, self.y, 0.0)
    }
}

impl Add for Vector2 {
    type Output = Vector2;
    fn add(self, o: Vector2) -> Vector2 {
        Vector2::new(self.x + o.x, self.y + o.y)
    }
}

impl Sub for Vector2 {
    type Output = Vector2;
    fn sub(self, o: Vector2) -> Vector2 {
        Vector2::new(self.x - o.x, self.y - o.y)
    }
}

impl Neg for Vector2 {
    type Output = Vector2;
    fn neg(self) -> Vector2 {
        Vector2::new(-self.x, -self.y)
    }
}

impl Mul<f64> for Vector2 {
    type Output = Vector2;
    fn mul(self, k: f64) -> Vector2 {
        Vector2::new(self.x * k, self.y * k)
    }
}

impl From<Vector2> for Vector3 {
    fn from(v: Vector2) -> Vector3 {
        v.to_3d()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalized_zero_is_none() {
        assert!(Vector3::ZERO.normalized().is_none());
        assert!(Vector3::new(f64::NAN, 0.0, 0.0).normalized().is_none());
    }

    #[test]
    fn basic_ops() {
        let a = Vector3::new(1.0, 2.0, 3.0);
        let b = Vector3::new(4.0, 5.0, 6.0);
        assert_eq!(a + b, Vector3::new(5.0, 7.0, 9.0));
        assert_eq!(a.cross(b), Vector3::new(-3.0, 6.0, -3.0));
        assert_eq!(Vector2::new(1.0, 2.0).to_3d(), Vector3::new(1.0, 2.0, 0.0));
        assert!((Vector3::new(3.0, 4.0, 0.0).length() - 5.0).abs() < 1e-12);
    }
}
