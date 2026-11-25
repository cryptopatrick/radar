//! Range-equation calibration and analytic detection probability.

use crate::error::RadarError;
use crate::units::{BOLTZMANN, SPEED_OF_LIGHT, T0_KELVIN, db_to_linear};
use crate::world::Swerling;
use std::f64::consts::PI;

/// How the range-equation constant `K` is obtained, where
/// `SNR = K·σ·g²/R⁴` (σ in m², R in m, g the normalised one-way pattern gain).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Calibration {
    /// Derived from the radar's physical parameters.
    Physical {
        /// Peak transmitted power, W (> 0).
        peak_power_w: f64,
        /// Antenna peak gain, dB.
        gain_db: f64,
        /// Carrier frequency, Hz (> 0).
        frequency_hz: f64,
        /// Receiver noise figure, dB.
        noise_figure_db: f64,
        /// Receiver noise bandwidth, Hz (> 0).
        bandwidth_hz: f64,
        /// Total system losses, dB.
        losses_db: f64,
    },
    /// "A target of `rcs_m2` at `range_m` gives `snr_db` on boresight."
    Reference {
        /// Reference cross-section, m² (> 0).
        rcs_m2: f64,
        /// Reference range, m (> 0).
        range_m: f64,
        /// Mean single-look SNR at the reference point, dB.
        snr_db: f64,
    },
}

impl Calibration {
    /// The constant `K` such that `SNR = K·σ·g²/R⁴` (linear SNR).
    ///
    /// Physical: `Pt·G²·λ²/((4π)³·k·T0·F·B·L)`. Reference:
    /// `db_to_linear(snr_db)·range_m⁴/rcs_m2`.
    pub fn snr_constant(&self) -> f64 {
        match *self {
            Calibration::Physical {
                peak_power_w,
                gain_db,
                frequency_hz,
                noise_figure_db,
                bandwidth_hz,
                losses_db,
            } => {
                let g = db_to_linear(gain_db);
                let lambda = SPEED_OF_LIGHT / frequency_hz;
                peak_power_w * g * g * lambda * lambda
                    / ((4.0 * PI).powi(3)
                        * BOLTZMANN
                        * T0_KELVIN
                        * db_to_linear(noise_figure_db)
                        * bandwidth_hz
                        * db_to_linear(losses_db))
            }
            Calibration::Reference {
                rcs_m2,
                range_m,
                snr_db,
            } => db_to_linear(snr_db) * range_m.powi(4) / rcs_m2,
        }
    }

    /// Jamming-to-noise ratio per watt of jammer ERP at 1 m with unit
    /// pattern gain: `G·λ²/((4π)²·k·T0·F·B)` for [`Calibration::Physical`].
    /// [`Calibration::Reference`] has no receiver model and returns `None`
    /// (use a [`JammingReference`] instead).
    pub fn jnr_constant(&self) -> Option<f64> {
        match *self {
            Calibration::Physical {
                gain_db,
                frequency_hz,
                noise_figure_db,
                bandwidth_hz,
                ..
            } => {
                let lambda = SPEED_OF_LIGHT / frequency_hz;
                Some(
                    db_to_linear(gain_db) * lambda * lambda
                        / ((4.0 * PI).powi(2)
                            * BOLTZMANN
                            * T0_KELVIN
                            * db_to_linear(noise_figure_db)
                            * bandwidth_hz),
                )
            }
            Calibration::Reference { .. } => None,
        }
    }

    /// Builds a [`Calibration::Reference`] such that a target of `rcs_m2` at
    /// `range_m` is detected with probability `pd` at false-alarm probability
    /// `pfa` for the given fluctuation model. The SNR is found by bisection
    /// on [`analytic_pd`] over `[-20, 60]` dB to 1e-6 dB.
    ///
    /// Returns [`RadarError::InvalidConfig`] for non-positive or non-finite
    /// `rcs_m2`/`range_m`, `pd` outside `(0, 1)`, `pfa` outside `(0, 1)`, or
    /// a `pd` not reachable within the search interval.
    pub fn reference_for_pd(
        rcs_m2: f64,
        range_m: f64,
        pd: f64,
        pfa: f64,
        swerling: Swerling,
    ) -> Result<Calibration, RadarError> {
        let invalid = |field: &'static str, reason: &str| RadarError::InvalidConfig {
            field,
            reason: reason.into(),
        };
        if !(rcs_m2.is_finite() && rcs_m2 > 0.0) {
            return Err(invalid("rcs_m2", "must be finite and > 0"));
        }
        if !(range_m.is_finite() && range_m > 0.0) {
            return Err(invalid("range_m", "must be finite and > 0"));
        }
        let snr_db = required_snr_db(pd, pfa, swerling)?;
        Ok(Calibration::Reference {
            rcs_m2,
            range_m,
            snr_db,
        })
    }
}

/// Mean single-look SNR, dB, at which [`analytic_pd`] reaches `pd` for
/// false-alarm probability `pfa` and fluctuation model `swerling`. Found by
/// bisection over `[-20, 60]` dB to 1e-6 dB.
///
/// Returns [`RadarError::InvalidConfig`] for `pd` or `pfa` outside `(0, 1)`,
/// or a `pd` not reachable within the search interval.
pub fn required_snr_db(pd: f64, pfa: f64, swerling: Swerling) -> Result<f64, RadarError> {
    let invalid = |field: &'static str, reason: &str| RadarError::InvalidConfig {
        field,
        reason: reason.into(),
    };
    if !(pd > 0.0 && pd < 1.0) {
        return Err(invalid("pd", "must be in (0, 1)"));
    }
    if !(pfa > 0.0 && pfa < 1.0) {
        return Err(invalid("pfa", "must be in (0, 1)"));
    }
    let f = |db: f64| analytic_pd(db_to_linear(db), pfa, swerling) - pd;
    let (mut lo, mut hi) = (-20.0_f64, 60.0_f64);
    if f(lo) > 0.0 || f(hi) < 0.0 {
        return Err(invalid("pd", "not reachable for SNR in [-20, 60] dB"));
    }
    while hi - lo > 1e-6 {
        let mid = 0.5 * (lo + hi);
        if f(mid) < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Ok(0.5 * (lo + hi))
}

/// Reference point for noise jamming when the calibration is
/// [`Calibration::Reference`]: "a jammer of `erp_w` at `range_m`, on
/// boresight and fully in band, gives `jnr_db`".
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JammingReference {
    /// Reference effective radiated power, W (> 0).
    pub erp_w: f64,
    /// Reference range, m (> 0).
    pub range_m: f64,
    /// Jamming-to-noise ratio at the reference point, dB.
    pub jnr_db: f64,
}

/// Analytic single-look probability of detection for a square-law detector
/// at linear mean SNR `snr` and false-alarm probability `pfa`, with
/// `Y = -ln(pfa)`:
///
/// - Swerling 1: `pfa^(1/(1+snr))`
/// - Swerling 3: `(1 + 2·snr·Y/(snr+2)²)·exp(-2Y/(2+snr))`
/// - Swerling 0: North's approximation `0.5·erfc(√Y − √(snr + 0.5))`
///
/// Used for diagnostics and calibration only; detection itself samples the
/// envelope.
pub fn analytic_pd(snr: f64, pfa: f64, swerling: Swerling) -> f64 {
    let y = -pfa.ln();
    match swerling {
        Swerling::One => pfa.powf(1.0 / (1.0 + snr)),
        Swerling::Three => {
            (1.0 + 2.0 * snr * y / ((snr + 2.0) * (snr + 2.0))) * (-2.0 * y / (2.0 + snr)).exp()
        }
        Swerling::Zero => 0.5 * erfc(y.sqrt() - (snr + 0.5).sqrt()),
    }
}

/// Complementary error function (Numerical Recipes `erfcc`, |error| < 1.2e-7).
pub(crate) fn erfc(x: f64) -> f64 {
    let z = x.abs();
    let t = 1.0 / (1.0 + 0.5 * z);
    let ans = t
        * (-z * z - 1.265_512_23
            + t * (1.000_023_68
                + t * (0.374_091_96
                    + t * (0.096_784_18
                        + t * (-0.186_288_06
                            + t * (0.278_868_07
                                + t * (-1.135_203_98
                                    + t * (1.488_515_87
                                        + t * (-0.822_152_23 + t * 0.170_872_77)))))))))
            .exp();
    if x >= 0.0 { ans } else { 2.0 - ans }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::db_to_linear;
    use approx::assert_relative_eq;

    const CASES: [Swerling; 3] = [Swerling::Zero, Swerling::One, Swerling::Three];

    #[test]
    fn analytic_pd_values() {
        let snr = db_to_linear(13.0);
        assert!(
            (analytic_pd(snr, 1e-6, Swerling::One) - 1e-6_f64.powf(1.0 / (1.0 + snr))).abs()
                < 1e-12
        );
        for case in CASES {
            let mut prev = analytic_pd(0.0, 1e-6, case);
            // North's approximation (Swerling 0) is coarse at zero SNR: the
            // formula itself gives 1.038e-5, so allow a slightly looser bound.
            let bound = if case == Swerling::Zero { 1.1e-5 } else { 1e-5 };
            assert!(prev < bound, "{case:?}: pd(0) = {prev}");
            for snr in [1.0, 3.0, 10.0, 30.0, 100.0] {
                let pd = analytic_pd(snr, 1e-6, case);
                assert!(pd > prev, "{case:?}: pd({snr}) = {pd} <= {prev}");
                assert!(pd <= 1.0);
                prev = pd;
            }
        }
    }

    #[test]
    fn erfc_matches_known_values() {
        assert!((erfc(0.0) - 1.0).abs() < 1.2e-7);
        assert!((erfc(1.0) - 0.157_299_207_050_285_1).abs() < 1.2e-7);
        assert!((erfc(-1.0) - 1.842_700_792_949_715).abs() < 1.2e-7);
        assert!((erfc(3.0) - 2.209_049_699_858_544e-5).abs() < 1.2e-7);
    }

    #[test]
    fn reference_for_pd_roundtrip() {
        for case in CASES {
            let cal = Calibration::reference_for_pd(1.0, 50_000.0, 0.9, 1e-6, case).unwrap();
            let Calibration::Reference {
                rcs_m2,
                range_m,
                snr_db,
            } = cal
            else {
                panic!("expected Reference");
            };
            assert_eq!(rcs_m2, 1.0);
            assert_eq!(range_m, 50_000.0);
            let pd = analytic_pd(db_to_linear(snr_db), 1e-6, case);
            assert!((pd - 0.9).abs() < 1e-4, "{case:?}: pd {pd} at {snr_db} dB");
        }
    }

    #[test]
    fn required_snr_db_inverts_analytic_pd() {
        for case in CASES {
            let db = required_snr_db(0.5, 1e-4, case).unwrap();
            assert!((analytic_pd(db_to_linear(db), 1e-4, case) - 0.5).abs() < 1e-5);
        }
        // Swerling 1 has a closed form: snr = ln(pfa)/ln(pd) − 1.
        let exact = 1e-6_f64.ln() / 0.9_f64.ln() - 1.0;
        let db = required_snr_db(0.9, 1e-6, Swerling::One).unwrap();
        assert!((db_to_linear(db) - exact).abs() / exact < 1e-5);
        assert!(required_snr_db(0.0, 1e-6, Swerling::One).is_err());
        assert!(required_snr_db(0.9, 1.0, Swerling::One).is_err());
    }

    #[test]
    fn reference_for_pd_rejects_bad_input() {
        assert!(Calibration::reference_for_pd(0.0, 1.0, 0.9, 1e-6, Swerling::One).is_err());
        assert!(Calibration::reference_for_pd(1.0, -1.0, 0.9, 1e-6, Swerling::One).is_err());
        assert!(Calibration::reference_for_pd(1.0, 1.0, 1.5, 1e-6, Swerling::One).is_err());
        assert!(Calibration::reference_for_pd(1.0, 1.0, 0.9, 0.0, Swerling::One).is_err());
        assert!(Calibration::reference_for_pd(1.0, 1.0, f64::NAN, 1e-6, Swerling::One).is_err());
    }

    #[test]
    fn reference_constant() {
        let cal = Calibration::Reference {
            rcs_m2: 2.0,
            range_m: 1000.0,
            snr_db: 10.0,
        };
        assert_relative_eq!(cal.snr_constant(), 10.0 * 1e12 / 2.0, max_relative = 1e-12);
        assert_eq!(cal.jnr_constant(), None);
    }

    #[test]
    fn physical_constant() {
        let cal = Calibration::Physical {
            peak_power_w: 1_000.0,
            gain_db: 30.0,
            frequency_hz: 10e9,
            noise_figure_db: 3.0,
            bandwidth_hz: 1e6,
            losses_db: 0.0,
        };
        let pt = 1_000.0;
        let g = 10f64.powf(30.0 / 10.0);
        let lambda = 299_792_458.0 / 10e9;
        let k_b = 1.380_649e-23;
        let t0 = 290.0;
        let f = 10f64.powf(3.0 / 10.0);
        let b = 1e6;
        let l = 1.0;
        let four_pi = 4.0 * std::f64::consts::PI;
        let expected = pt * g * g * lambda * lambda / (four_pi.powi(3) * k_b * t0 * f * b * l);
        assert_relative_eq!(cal.snr_constant(), expected, max_relative = 1e-12);
        let expected_jnr = g * lambda * lambda / (four_pi.powi(2) * k_b * t0 * f * b);
        assert_relative_eq!(
            cal.jnr_constant().unwrap(),
            expected_jnr,
            max_relative = 1e-12
        );
    }
}
