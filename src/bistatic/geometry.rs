//! Bistatic geometry: transmitter, receiver and target positions.

use crate::geometry::Vector3;

/// Positions of the transmitter, the receiver and the target, in the world
/// frame (m).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BistaticGeometry {
    /// Transmitter (illuminator) position.
    pub transmitter: Vector3,
    /// Receiver position.
    pub receiver: Vector3,
    /// Target position.
    pub target: Vector3,
}

impl BistaticGeometry {
    /// Transmitter-to-target range `R_T`, m.
    pub fn tx_range(&self) -> f64 {
        self.transmitter.distance(self.target)
    }

    /// Target-to-receiver range `R_R`, m.
    pub fn rx_range(&self) -> f64 {
        self.target.distance(self.receiver)
    }

    /// Transmitter-to-receiver distance `L` (the baseline), m.
    pub fn baseline(&self) -> f64 {
        self.transmitter.distance(self.receiver)
    }

    /// Bistatic range `R_T + R_R − L`, m: the extra path the echo travels
    /// compared with the direct signal. Zero on the baseline.
    pub fn bistatic_range(&self) -> f64 {
        (self.tx_range() + self.rx_range() - self.baseline()).max(0.0)
    }

    /// Bistatic angle `β`, the angle at the target between the directions to
    /// the transmitter and to the receiver, in `[0, π]`. Zero is the
    /// monostatic case; `π` is forward scatter (the target on the baseline).
    /// `None` if the target coincides with the transmitter or the receiver.
    pub fn bistatic_angle(&self) -> Option<f64> {
        let to_tx = (self.transmitter - self.target).normalized()?;
        let to_rx = (self.receiver - self.target).normalized()?;
        Some(to_tx.dot(to_rx).clamp(-1.0, 1.0).acos())
    }

    /// Unit bisector of the bistatic angle, pointing from the target
    /// between the transmitter and the receiver. `None` when the angle is
    /// undefined or is forward scatter (`β = π`, no unique bisector).
    pub fn bisector(&self) -> Option<Vector3> {
        let to_tx = (self.transmitter - self.target).normalized()?;
        let to_rx = (self.receiver - self.target).normalized()?;
        (to_tx + to_rx).normalized()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn geo(t: Vector3, r: Vector3, x: Vector3) -> BistaticGeometry {
        BistaticGeometry {
            transmitter: t,
            receiver: r,
            target: x,
        }
    }

    #[test]
    fn ranges_and_baseline() {
        let g = geo(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(8.0, 0.0, 0.0),
            Vector3::new(4.0, 3.0, 0.0),
        );
        assert_relative_eq!(g.tx_range(), 5.0);
        assert_relative_eq!(g.rx_range(), 5.0);
        assert_relative_eq!(g.baseline(), 8.0);
        assert_relative_eq!(g.bistatic_range(), 2.0);
        // Isosceles 3-4-5 triangles: β = 2·atan(4/3).
        assert_relative_eq!(g.bistatic_angle().unwrap(), 2.0 * (4.0_f64 / 3.0).atan());
        let b = g.bisector().unwrap();
        assert_relative_eq!(b.x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(b.y, -1.0, epsilon = 1e-12);
    }

    #[test]
    fn monostatic_is_zero_angle() {
        let site = Vector3::new(1.0, 2.0, 3.0);
        let g = geo(site, site, Vector3::new(100.0, 2.0, 3.0));
        assert_relative_eq!(g.bistatic_angle().unwrap(), 0.0);
        assert_relative_eq!(g.bistatic_range(), 2.0 * 99.0);
        assert_relative_eq!(g.bisector().unwrap().x, -1.0);
    }

    #[test]
    fn right_angle_and_forward_scatter() {
        let g = geo(
            Vector3::new(0.0, 10.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::ZERO,
        );
        assert_relative_eq!(g.bistatic_angle().unwrap(), FRAC_PI_2);
        let g = geo(
            Vector3::new(-5.0, 0.0, 0.0),
            Vector3::new(5.0, 0.0, 0.0),
            Vector3::ZERO,
        );
        assert_relative_eq!(g.bistatic_angle().unwrap(), PI);
        assert_eq!(g.bisector(), None);
        assert_eq!(g.bistatic_range(), 0.0);
    }

    #[test]
    fn degenerate_target_has_no_angle() {
        let g = geo(Vector3::ZERO, Vector3::new(1.0, 0.0, 0.0), Vector3::ZERO);
        assert_eq!(g.bistatic_angle(), None);
        assert_eq!(g.bisector(), None);
    }
}
