//! Compass-angle helpers. Azimuth is a bearing from +y, clockwise.

use super::vector::Vector3;
use std::f64::consts::{PI, TAU};

/// Wraps an angle into `[0, 2π)`; `-0.0` maps to `+0.0`.
pub fn wrap_two_pi(a: f64) -> f64 {
    // `+ 0.0` turns a `-0.0` remainder into `+0.0`.
    let r = a.rem_euclid(TAU) + 0.0;
    if r >= TAU { 0.0 } else { r }
}

/// Wraps an angle into `(-π, π]`.
pub fn wrap_pi(a: f64) -> f64 {
    let r = wrap_two_pi(a);
    if r > PI { r - TAU } else { r }
}

/// Compass azimuth of a vector, in `[0, 2π)`.
pub fn azimuth_of(v: Vector3) -> f64 {
    wrap_two_pi(v.x.atan2(v.y))
}

/// Elevation of a vector above the horizontal plane.
pub fn elevation_of(v: Vector3) -> f64 {
    v.z.atan2(v.x.hypot(v.y))
}

/// Unit vector for a compass azimuth and elevation.
pub fn direction(az: f64, el: f64) -> Vector3 {
    Vector3::new(az.sin() * el.cos(), az.cos() * el.cos(), el.sin())
}

/// Angle between two vectors in `[0, π]`; 0 if either is zero.
pub fn angle_between(a: Vector3, b: Vector3) -> f64 {
    let (la, lb) = (a.length(), b.length());
    if la == 0.0 || lb == 0.0 {
        return 0.0;
    }
    (a.dot(b) / (la * lb)).clamp(-1.0, 1.0).acos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    #[test]
    fn azimuth_compass() {
        assert!((azimuth_of(Vector3::new(1., 0., 0.)) - PI / 2.0).abs() < 1e-12);
        assert!(azimuth_of(Vector3::new(0., 1., 0.)).abs() < 1e-12);
        assert!((azimuth_of(Vector3::new(-1., 0., 0.)) - 3.0 * PI / 2.0).abs() < 1e-12);
    }

    #[test]
    fn wrap() {
        assert!((wrap_two_pi(-0.1) - (TAU - 0.1)).abs() < 1e-12);
        assert!((wrap_pi(3.0 * PI / 2.0) + PI / 2.0).abs() < 1e-12);
        assert_eq!(wrap_pi(PI), PI);
        assert!(wrap_two_pi(-0.0).is_sign_positive());
        assert!(azimuth_of(Vector3::new(-0.0, 1.0, 0.0)).is_sign_positive());
    }

    #[test]
    fn angle_between_zero_vector() {
        assert_eq!(angle_between(Vector3::ZERO, Vector3::new(1., 0., 0.)), 0.0);
        let a = angle_between(Vector3::new(1., 0., 0.), Vector3::new(0., 1., 0.));
        assert!((a - PI / 2.0).abs() < 1e-12);
    }

    proptest! {
        #[test]
        fn prop_direction_roundtrip(az in 0.0..TAU, el in -1.5f64..1.5) {
            let d = direction(az, el);
            prop_assert!(wrap_pi(azimuth_of(d) - az).abs() < 1e-9);
            prop_assert!((elevation_of(d) - el).abs() < 1e-9);
        }
    }
}
