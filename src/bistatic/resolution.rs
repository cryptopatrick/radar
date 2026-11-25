//! Bistatic range, Doppler and velocity resolution.

use super::BistaticGeometry;
use crate::geometry::Vector3;
use crate::units::SPEED_OF_LIGHT;

/// Below this `cos`, the geometry is treated as forward scatter and the
/// resolution as unbounded.
const FORWARD_SCATTER_COS: f64 = 1e-9;

/// Bistatic range resolution, eq. (2) of Gomez-del-Hoyo et al. (IRS 2022),
/// after Willis, *Bistatic Radar*:
///
/// `ΔR = c / (2·B·cos(β/2)·cos ψ)`
///
/// for signal bandwidth `B` (Hz), bistatic angle `β` and aspect angle `ψ`
/// between the bistatic bisector and the direction in which two targets are
/// separated (radians). Returns `f64::INFINITY` when either cosine is zero
/// or negative (forward scatter, or separation across the bisector).
pub fn range_resolution(bandwidth_hz: f64, bistatic_angle: f64, aspect_angle: f64) -> f64 {
    let c = (0.5 * bistatic_angle).cos() * aspect_angle.cos().abs();
    if c <= FORWARD_SCATTER_COS {
        f64::INFINITY
    } else {
        SPEED_OF_LIGHT / (2.0 * bandwidth_hz * c)
    }
}

/// Bistatic velocity resolution `λ / (2·T_int·cos(β/2))`, m/s: the
/// Doppler resolution `1/T_int` converted to target speed along the
/// bisector. `f64::INFINITY` at forward scatter.
pub fn velocity_resolution(wavelength_m: f64, integration_time_s: f64, bistatic_angle: f64) -> f64 {
    let c = (0.5 * bistatic_angle).cos();
    if c <= FORWARD_SCATTER_COS {
        f64::INFINITY
    } else {
        wavelength_m / (2.0 * integration_time_s * c)
    }
}

impl BistaticGeometry {
    /// Range resolution along the bisector (`ψ = 0`), m, for a signal of
    /// `bandwidth_hz`. `None` when the bistatic angle is undefined;
    /// `f64::INFINITY` at forward scatter.
    pub fn range_resolution(&self, bandwidth_hz: f64) -> Option<f64> {
        Some(range_resolution(bandwidth_hz, self.bistatic_angle()?, 0.0))
    }

    /// Range resolution between two targets separated along `direction`, m:
    /// eq. (2) with `ψ` the angle between `direction` and the bisector.
    /// `None` when the bisector or `direction` is undefined (forward
    /// scatter, a degenerate target, or a zero direction).
    pub fn range_resolution_along(&self, bandwidth_hz: f64, direction: Vector3) -> Option<f64> {
        let bisector = self.bisector()?;
        let d = direction.normalized()?;
        let psi = bisector.dot(d).clamp(-1.0, 1.0).acos();
        Some(range_resolution(bandwidth_hz, self.bistatic_angle()?, psi))
    }

    /// Velocity resolution along the bisector, m/s. `None` when the
    /// bistatic angle is undefined.
    pub fn velocity_resolution(&self, wavelength_m: f64, integration_time_s: f64) -> Option<f64> {
        Some(velocity_resolution(
            wavelength_m,
            integration_time_s,
            self.bistatic_angle()?,
        ))
    }

    /// Rate of change of the echo path `R_T + R_R`, m/s, for a moving
    /// transmitter and target and a stationary receiver. Negative when the
    /// path is shortening.
    pub fn path_rate(
        &self,
        transmitter_velocity: Vector3,
        target_velocity: Vector3,
    ) -> Option<f64> {
        let tx_to_target = (self.target - self.transmitter).normalized()?;
        let rx_to_target = (self.target - self.receiver).normalized()?;
        Some(
            (target_velocity - transmitter_velocity).dot(tx_to_target)
                + target_velocity.dot(rx_to_target),
        )
    }

    /// Doppler shift of the echo, Hz: `−(1/λ)·d(R_T + R_R)/dt`, for a
    /// moving transmitter and target and a stationary receiver. For a
    /// stationary transmitter this is `(2v/λ)·cos δ·cos(β/2)`, with `δ` the
    /// angle between the target velocity and the bisector.
    ///
    /// A moving illuminator (a low-orbit satellite) shifts the echo even
    /// from a stationary target; a passive radar removes that shift with
    /// the reference (direct) signal, whose Doppler is
    /// [`BistaticGeometry::direct_doppler_hz`].
    pub fn doppler_hz(
        &self,
        wavelength_m: f64,
        transmitter_velocity: Vector3,
        target_velocity: Vector3,
    ) -> Option<f64> {
        Some(-self.path_rate(transmitter_velocity, target_velocity)? / wavelength_m)
    }

    /// Doppler shift of the direct (reference) signal at the receiver, Hz.
    /// `None` if transmitter and receiver coincide.
    pub fn direct_doppler_hz(
        &self,
        wavelength_m: f64,
        transmitter_velocity: Vector3,
    ) -> Option<f64> {
        let rx_to_tx = (self.transmitter - self.receiver).normalized()?;
        Some(-transmitter_velocity.dot(rx_to_tx) / wavelength_m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bistatic::Illuminator;
    use approx::assert_relative_eq;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI};

    fn geo(t: Vector3, r: Vector3, x: Vector3) -> BistaticGeometry {
        BistaticGeometry {
            transmitter: t,
            receiver: r,
            target: x,
        }
    }

    #[test]
    fn monostatic_limit_is_c_over_2b() {
        assert_relative_eq!(range_resolution(1e6, 0.0, 0.0), SPEED_OF_LIGHT / 2e6);
        // Starlink's 250 MHz channel gives 0.6 m monostatic.
        let starlink = Illuminator::starlink();
        assert_relative_eq!(
            range_resolution(starlink.bandwidth_hz, 0.0, 0.0),
            0.599_584_916,
            epsilon = 1e-9
        );
    }

    #[test]
    fn resolution_degrades_with_angles() {
        let b = 10e6;
        let mono = range_resolution(b, 0.0, 0.0);
        // β = 120° halves cos(β/2); ψ = 60° halves cos ψ.
        assert_relative_eq!(
            range_resolution(b, 2.0 * FRAC_PI_3, 0.0),
            2.0 * mono,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            range_resolution(b, 0.0, FRAC_PI_3),
            2.0 * mono,
            max_relative = 1e-12
        );
        assert_eq!(range_resolution(b, PI, 0.0), f64::INFINITY);
        assert_eq!(range_resolution(b, 0.0, FRAC_PI_2), f64::INFINITY);
        // Separation in the opposite direction resolves the same.
        assert_relative_eq!(
            range_resolution(b, 1.0, PI - 0.3),
            range_resolution(b, 1.0, 0.3)
        );
    }

    #[test]
    fn velocity_resolution_values() {
        assert_relative_eq!(velocity_resolution(0.03, 0.5, 0.0), 0.03);
        assert_relative_eq!(
            velocity_resolution(0.03, 0.5, 2.0 * FRAC_PI_3),
            0.06,
            max_relative = 1e-12
        );
        assert_eq!(velocity_resolution(0.03, 0.5, PI), f64::INFINITY);
    }

    #[test]
    fn geometry_methods_use_the_bisector() {
        // β = 90°: transmitter north, receiver east of the target.
        let g = geo(
            Vector3::new(0.0, 1e3, 0.0),
            Vector3::new(1e3, 0.0, 0.0),
            Vector3::ZERO,
        );
        let along = g.range_resolution(1e6).unwrap();
        assert_relative_eq!(
            along,
            SPEED_OF_LIGHT / (2e6 * (PI / 4.0).cos()),
            max_relative = 1e-12
        );
        let bisector = g.bisector().unwrap();
        assert_relative_eq!(
            g.range_resolution_along(1e6, bisector).unwrap(),
            along,
            max_relative = 1e-12
        );
        // Across the bisector, range cannot separate the targets.
        let across = Vector3::new(1.0, -1.0, 0.0);
        assert_eq!(g.range_resolution_along(1e6, across), Some(f64::INFINITY));
        assert_eq!(g.range_resolution_along(1e6, Vector3::ZERO), None);
    }

    #[test]
    fn starlink_resolution_stays_below_3_m() {
        // Section V and Fig. 6: over ±10 km around the receiver, at 50° and
        // 80° satellite elevation, the resolution stays below 3 m and is
        // worst towards the satellite.
        let il = Illuminator::starlink();
        for el in [50.0_f64, 80.0] {
            let e = el.to_radians();
            let slant = il.altitude_m / e.sin();
            let sat = Vector3::new(0.0, slant * e.cos(), il.altitude_m);
            let mut worst = (0.0, Vector3::ZERO);
            for x in -10..=10 {
                for y in -10..=10 {
                    if (x, y) == (0, 0) {
                        continue;
                    }
                    let target = Vector3::new(x as f64 * 1e3, y as f64 * 1e3, 0.0);
                    let r = geo(sat, Vector3::ZERO, target)
                        .range_resolution(il.bandwidth_hz)
                        .unwrap();
                    assert!(r < 3.0, "{el}°: {r} m at {target:?}");
                    if r > worst.0 {
                        worst = (r, target);
                    }
                }
            }
            assert!(
                worst.1.y > 0.0,
                "{el}°: worst cell {worst:?} not towards the satellite"
            );
        }
    }

    #[test]
    fn doppler_of_a_stationary_transmitter() {
        // Monostatic-like: target moving straight at a co-located pair.
        let site = Vector3::ZERO;
        let g = geo(site, site, Vector3::new(1e3, 0.0, 0.0));
        let v = Vector3::new(-10.0, 0.0, 0.0);
        assert_relative_eq!(
            g.doppler_hz(0.03, Vector3::ZERO, v).unwrap(),
            2.0 * 10.0 / 0.03
        );
        // β = 90°, velocity along the bisector: (2v/λ)·cos(45°).
        let g = geo(
            Vector3::new(0.0, 1e3, 0.0),
            Vector3::new(1e3, 0.0, 0.0),
            Vector3::ZERO,
        );
        let v = g.bisector().unwrap() * 10.0;
        assert_relative_eq!(
            g.doppler_hz(0.03, Vector3::ZERO, v).unwrap(),
            2.0 * 10.0 / 0.03 * (PI / 4.0).cos(),
            max_relative = 1e-12
        );
        // Moving across the bisector gives no Doppler.
        let across = Vector3::new(1.0, -1.0, 0.0) * 10.0;
        assert_relative_eq!(
            g.doppler_hz(0.03, Vector3::ZERO, across).unwrap(),
            0.0,
            epsilon = 1e-9
        );
    }

    #[test]
    fn moving_satellite_shifts_a_stationary_target() {
        // A satellite overhead moving towards the target's side shortens
        // R_T: positive Doppler. The direct signal sees the radial part too.
        let g = geo(
            Vector3::new(0.0, 0.0, 550e3),
            Vector3::ZERO,
            Vector3::new(10e3, 0.0, 0.0),
        );
        let v_sat = Vector3::new(7_600.0, 0.0, 0.0);
        let echo = g.doppler_hz(0.0256, v_sat, Vector3::ZERO).unwrap();
        assert!(echo > 0.0);
        let expected = v_sat.dot((g.target - g.transmitter).normalized().unwrap()) / 0.0256;
        assert_relative_eq!(echo, expected, max_relative = 1e-12);
        // Directly overhead, the direct signal has no radial velocity.
        assert_relative_eq!(
            g.direct_doppler_hz(0.0256, v_sat).unwrap(),
            0.0,
            epsilon = 1e-9
        );
    }
}
