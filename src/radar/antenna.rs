//! Antenna beam patterns.

use crate::radar::config::Dimension;
use crate::units::{db_to_linear, deg};

/// Shape of the antenna's main beam.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BeamPattern {
    /// Gaussian main beam whose half-power points lie at half a beamwidth.
    Gaussian,
    /// Flat main beam: full gain inside half a beamwidth, sidelobe level outside.
    Uniform,
}

/// Antenna beamwidths and pattern.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AntennaConfig {
    /// Full azimuth beamwidth (between half-power points) in radians, in `(0, π]`.
    pub beamwidth_az: f64,
    /// Full elevation beamwidth in radians, in `(0, π]`. Ignored in 2D.
    pub beamwidth_el: f64,
    /// Main-beam shape.
    pub pattern: BeamPattern,
    /// Sidelobe level relative to the main-beam peak, in dB (default −30).
    pub sidelobe_level_db: f64,
}

impl Default for AntennaConfig {
    /// A 3° × 3° Gaussian beam with −30 dB sidelobes.
    fn default() -> Self {
        Self {
            beamwidth_az: deg(3.0),
            beamwidth_el: deg(3.0),
            pattern: BeamPattern::Gaussian,
            sidelobe_level_db: -30.0,
        }
    }
}

impl AntennaConfig {
    /// Normalised one-way power gain at the given off-boresight angles
    /// (radians), in `[sidelobe, 1]`. Square it for the two-way gain.
    ///
    /// Gaussian: `exp(-4 ln2 ((off_az/bw_az)² + (off_el/bw_el)²))`; Uniform:
    /// 1 inside both half-beamwidths, else the sidelobe level. In
    /// [`Dimension::TwoD`] the elevation offset is ignored. The result never
    /// falls below `db_to_linear(sidelobe_level_db)`.
    pub fn gain(&self, off_az: f64, off_el: f64, dim: Dimension) -> f64 {
        let three_d = dim == Dimension::ThreeD;
        let main = match self.pattern {
            BeamPattern::Gaussian => {
                let a = off_az / self.beamwidth_az;
                let e = if three_d {
                    off_el / self.beamwidth_el
                } else {
                    0.0
                };
                (-4.0 * std::f64::consts::LN_2 * (a * a + e * e)).exp()
            }
            BeamPattern::Uniform => {
                let inside_az = off_az.abs() <= self.beamwidth_az / 2.0;
                let inside_el = !three_d || off_el.abs() <= self.beamwidth_el / 2.0;
                if inside_az && inside_el { 1.0 } else { 0.0 }
            }
        };
        main.max(db_to_linear(self.sidelobe_level_db))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radar::config::Dimension;
    use crate::units::deg;
    use approx::assert_relative_eq;

    fn antenna(pattern: BeamPattern) -> AntennaConfig {
        AntennaConfig {
            beamwidth_az: deg(3.0),
            beamwidth_el: deg(2.0),
            pattern,
            sidelobe_level_db: -30.0,
        }
    }

    #[test]
    fn gaussian_gain_half_power() {
        let a = antenna(BeamPattern::Gaussian);
        assert_relative_eq!(
            a.gain(a.beamwidth_az / 2.0, 0.0, Dimension::TwoD),
            0.5,
            epsilon = 1e-12
        );
        assert_relative_eq!(a.gain(0.0, 0.0, Dimension::ThreeD), 1.0, epsilon = 1e-12);
        assert_relative_eq!(
            a.gain(0.0, a.beamwidth_el / 2.0, Dimension::ThreeD),
            0.5,
            epsilon = 1e-12
        );
        // 2D ignores the elevation offset.
        assert_relative_eq!(
            a.gain(0.0, a.beamwidth_el / 2.0, Dimension::TwoD),
            1.0,
            epsilon = 1e-12
        );
    }

    #[test]
    fn sidelobe_floor() {
        let a = antenna(BeamPattern::Gaussian);
        assert_relative_eq!(
            a.gain(std::f64::consts::PI, 0.0, Dimension::TwoD),
            1e-3,
            epsilon = 1e-15
        );
    }

    #[test]
    fn uniform_gain() {
        let a = antenna(BeamPattern::Uniform);
        assert_eq!(a.gain(0.0, 0.0, Dimension::TwoD), 1.0);
        assert_eq!(
            a.gain(a.beamwidth_az / 2.0 * 0.99, 0.0, Dimension::TwoD),
            1.0
        );
        assert_eq!(
            a.gain(-a.beamwidth_az / 2.0 * 0.99, 0.0, Dimension::TwoD),
            1.0
        );
        assert_relative_eq!(
            a.gain(a.beamwidth_az, 0.0, Dimension::TwoD),
            1e-3,
            epsilon = 1e-15
        );
        assert_eq!(a.gain(0.0, a.beamwidth_el, Dimension::TwoD), 1.0);
        assert_relative_eq!(
            a.gain(0.0, a.beamwidth_el, Dimension::ThreeD),
            1e-3,
            epsilon = 1e-15
        );
    }
}
