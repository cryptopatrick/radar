//! The bistatic radar range equation for a passive receiver.

use super::{BistaticGeometry, Illuminator};
use crate::error::RadarError;
use crate::units::{BOLTZMANN, T0_KELVIN, db_to_linear, linear_to_db};
use std::f64::consts::PI;

/// A passive radar's surveillance receiver.
///
/// [`Default`] is the receiver assumed by Gomez-del-Hoyo et al. (IRS 2022):
/// a 40 dBi antenna, 0.5 s coherent integration, noise at `T0` over the
/// signal bandwidth and no losses.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PassiveReceiver {
    /// Surveillance antenna peak gain, dBi.
    pub gain_db: f64,
    /// Receiver noise figure, dB (≥ 0).
    pub noise_figure_db: f64,
    /// Antenna noise temperature, K (≥ 0).
    pub antenna_temperature_k: f64,
    /// Total system losses, dB (≥ 0).
    pub losses_db: f64,
    /// Coherent integration time, s (> 0).
    pub integration_time_s: f64,
    /// Receiver noise bandwidth, Hz (> 0); `None` uses the illuminator's
    /// signal bandwidth.
    pub noise_bandwidth_hz: Option<f64>,
}

impl Default for PassiveReceiver {
    fn default() -> Self {
        Self {
            gain_db: 40.0,
            noise_figure_db: 0.0,
            antenna_temperature_k: T0_KELVIN,
            losses_db: 0.0,
            integration_time_s: 0.5,
            noise_bandwidth_hz: None,
        }
    }
}

impl PassiveReceiver {
    /// Checks every field is finite and in range.
    pub fn validate(&self) -> Result<(), RadarError> {
        let invalid = |field: &'static str, reason: &str| {
            Err(RadarError::InvalidConfig {
                field,
                reason: reason.into(),
            })
        };
        if !self.gain_db.is_finite() {
            return invalid("gain_db", "must be finite");
        }
        for (field, v) in [
            ("noise_figure_db", self.noise_figure_db),
            ("antenna_temperature_k", self.antenna_temperature_k),
            ("losses_db", self.losses_db),
        ] {
            if !(v.is_finite() && v >= 0.0) {
                return invalid(field, "must be finite and >= 0");
            }
        }
        if !(self.integration_time_s.is_finite() && self.integration_time_s > 0.0) {
            return invalid("integration_time_s", "must be finite and > 0");
        }
        if self
            .noise_bandwidth_hz
            .is_some_and(|bn| !(bn.is_finite() && bn > 0.0))
        {
            return invalid("noise_bandwidth_hz", "must be finite and > 0");
        }
        if self.system_temperature_k() <= 0.0 {
            return invalid("antenna_temperature_k", "system temperature must be > 0");
        }
        Ok(())
    }

    /// System noise temperature `T_a + T0·(F − 1)`, K. With the default
    /// `T_a = T0` and `F = 0 dB` this is `T0`, as in the paper.
    pub fn system_temperature_k(&self) -> f64 {
        self.antenna_temperature_k + T0_KELVIN * (db_to_linear(self.noise_figure_db) - 1.0)
    }

    /// Noise bandwidth used with `illuminator`, Hz.
    pub fn noise_bandwidth(&self, illuminator: &Illuminator) -> f64 {
        self.noise_bandwidth_hz.unwrap_or(illuminator.bandwidth_hz)
    }

    /// Receiver noise power `k·T_sys·B_n`, W.
    pub fn noise_power_w(&self, illuminator: &Illuminator) -> f64 {
        BOLTZMANN * self.system_temperature_k() * self.noise_bandwidth(illuminator)
    }

    /// Coherent processing gain `B·T_int`, dB.
    pub fn processing_gain_db(&self, illuminator: &Illuminator) -> f64 {
        linear_to_db(illuminator.bandwidth_hz * self.integration_time_s)
    }

    /// The constant `K` such that `SNR = K·σ/(R_T²·R_R²)` (linear SNR after
    /// integration, σ in m², ranges in m). Eq. (1) of the paper, with the
    /// noise temperature generalised:
    ///
    /// `K = EIRP·G_R·λ²·B·T_int / ((4π)³·L_S·k·T_sys·B_n)`
    pub fn snr_constant(&self, illuminator: &Illuminator) -> f64 {
        let lambda = illuminator.wavelength();
        illuminator.eirp_w()
            * db_to_linear(self.gain_db)
            * lambda
            * lambda
            * illuminator.bandwidth_hz
            * self.integration_time_s
            / ((4.0 * PI).powi(3) * db_to_linear(self.losses_db) * self.noise_power_w(illuminator))
    }

    /// Linear SNR of a target of `rcs_m2` (bistatic cross-section) at
    /// `tx_range_m` from the transmitter and `rx_range_m` from the receiver,
    /// on the receiver's boresight.
    pub fn snr(
        &self,
        illuminator: &Illuminator,
        rcs_m2: f64,
        tx_range_m: f64,
        rx_range_m: f64,
    ) -> f64 {
        self.snr_constant(illuminator) * rcs_m2
            / (tx_range_m * tx_range_m * rx_range_m * rx_range_m)
    }

    /// [`PassiveReceiver::snr`] in dB.
    pub fn snr_db(
        &self,
        illuminator: &Illuminator,
        rcs_m2: f64,
        tx_range_m: f64,
        rx_range_m: f64,
    ) -> f64 {
        linear_to_db(self.snr(illuminator, rcs_m2, tx_range_m, rx_range_m))
    }

    /// Linear SNR for the ranges of `geometry`.
    pub fn snr_at(
        &self,
        illuminator: &Illuminator,
        rcs_m2: f64,
        geometry: &BistaticGeometry,
    ) -> f64 {
        self.snr(
            illuminator,
            rcs_m2,
            geometry.tx_range(),
            geometry.rx_range(),
        )
    }

    /// [`PassiveReceiver::snr_at`] in dB.
    pub fn snr_db_at(
        &self,
        illuminator: &Illuminator,
        rcs_m2: f64,
        geometry: &BistaticGeometry,
    ) -> f64 {
        linear_to_db(self.snr_at(illuminator, rcs_m2, geometry))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::SPEED_OF_LIGHT;
    use approx::assert_relative_eq;

    /// The paper does not state its losses; 4.74 dB reproduces its Starlink
    /// detection ranges (Section IV).
    pub(crate) const PAPER_LOSSES_DB: f64 = 4.74;

    /// Starlink with Table II's rounded 0.03 m wavelength, as the paper used.
    fn starlink_table() -> Illuminator {
        Illuminator {
            frequency_hz: SPEED_OF_LIGHT / 0.03,
            ..Illuminator::starlink()
        }
    }

    /// Receiver range at which `snr_db` falls to `threshold_db`, by
    /// bisection on the SNR itself (independent of the closed form).
    fn range_at(f: impl Fn(f64) -> f64, threshold_db: f64) -> f64 {
        let (mut lo, mut hi) = (1.0_f64, 1e7_f64);
        for _ in 0..200 {
            let mid = (lo * hi).sqrt();
            if f(mid) > threshold_db {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }

    #[test]
    fn matches_eq_1_term_by_term() {
        let il = starlink_table();
        let rx = PassiveReceiver::default();
        let (rt, rr, sigma) = (550e3, 10e3, 40.0);
        let eirp = 10f64.powf(27.6 / 10.0);
        let g = 1e4;
        let lambda = 0.03;
        let b = 250e6;
        let expected = eirp * g * sigma * lambda * lambda * b * 0.5
            / ((4.0 * PI).powi(3) * rt * rt * rr * rr)
            / (1.380_649e-23 * 290.0 * b);
        assert_relative_eq!(rx.snr(&il, sigma, rt, rr), expected, max_relative = 1e-12);
    }

    #[test]
    fn reproduces_the_papers_detection_ranges() {
        // Section IV: with a 16.5 dB threshold, a 40 m² target is detected to
        // about 18 km with Starlink, below 7 km with Galileo E5, and from
        // 2.8 km (1 m²) upward with Starlink.
        let rx = PassiveReceiver {
            losses_db: PAPER_LOSSES_DB,
            ..PassiveReceiver::default()
        };
        let il = starlink_table();
        let alt = il.altitude_m;
        let starlink_40 = range_at(|r| rx.snr_db(&il, 40.0, alt, r), 16.5);
        assert!((starlink_40 - 18e3).abs() < 0.5e3, "{starlink_40}");
        let starlink_1 = range_at(|r| rx.snr_db(&il, 1.0, alt, r), 16.5);
        assert!((starlink_1 - 2.8e3).abs() < 0.1e3, "{starlink_1}");

        let e5 = Illuminator::galileo_e5();
        let e5_40 = range_at(|r| rx.snr_db(&e5, 40.0, e5.altitude_m, r), 16.5);
        assert!(e5_40 < 7e3 && e5_40 > 5e3, "{e5_40}");
    }

    #[test]
    fn starlink_beats_the_other_illuminators() {
        // Fig. 4(a): Starlink gives the highest SNR, then Galileo.
        let rx = PassiveReceiver::default();
        let snr = |il: Illuminator| rx.snr_db(&il, 40.0, il.altitude_m, 1e3);
        let starlink = snr(Illuminator::starlink());
        let e5 = snr(Illuminator::galileo_e5());
        for other in [
            Illuminator::galileo_e1(),
            Illuminator::gps_l1_ca(),
            Illuminator::gps_l5(),
            Illuminator::dvb_s2(),
        ] {
            assert!(starlink > snr(other) && e5 > snr(other), "{other:?}");
        }
        assert!(starlink > e5);
    }

    #[test]
    fn snr_falls_with_both_ranges_squared() {
        let il = Illuminator::starlink();
        let rx = PassiveReceiver::default();
        let base = rx.snr(&il, 1.0, 1e5, 1e3);
        assert_relative_eq!(rx.snr(&il, 1.0, 2e5, 1e3), base / 4.0, max_relative = 1e-12);
        assert_relative_eq!(rx.snr(&il, 1.0, 1e5, 2e3), base / 4.0, max_relative = 1e-12);
        assert_relative_eq!(rx.snr(&il, 3.0, 1e5, 1e3), base * 3.0, max_relative = 1e-12);
    }

    #[test]
    fn noise_temperature_and_bandwidth() {
        let il = Illuminator::starlink();
        let rx = PassiveReceiver::default();
        assert_relative_eq!(rx.system_temperature_k(), T0_KELVIN);
        assert_relative_eq!(rx.processing_gain_db(&il), linear_to_db(125e6));
        // A 3 dB noise figure roughly doubles the noise.
        let noisy = PassiveReceiver {
            noise_figure_db: 3.0,
            ..rx
        };
        assert_relative_eq!(
            noisy.snr_db(&il, 1.0, 1e5, 1e3),
            rx.snr_db(&il, 1.0, 1e5, 1e3) - 3.0,
            epsilon = 0.02
        );
        // Halving the noise bandwidth halves the noise.
        let narrow = PassiveReceiver {
            noise_bandwidth_hz: Some(125e6),
            ..rx
        };
        assert_relative_eq!(
            narrow.snr(&il, 1.0, 1e5, 1e3),
            2.0 * rx.snr(&il, 1.0, 1e5, 1e3),
            max_relative = 1e-12
        );
    }

    #[test]
    fn snr_at_uses_the_geometry() {
        use crate::geometry::Vector3;
        let il = Illuminator::starlink();
        let rx = PassiveReceiver::default();
        let g = BistaticGeometry {
            transmitter: Vector3::new(0.0, 0.0, 550e3),
            receiver: Vector3::ZERO,
            target: Vector3::new(3e3, 4e3, 0.0),
        };
        assert_relative_eq!(
            rx.snr_at(&il, 2.0, &g),
            rx.snr(&il, 2.0, g.tx_range(), 5e3),
            max_relative = 1e-12
        );
    }

    #[test]
    fn validate_rejects_bad_fields() {
        let ok = PassiveReceiver::default();
        ok.validate().unwrap();
        for bad in [
            PassiveReceiver {
                gain_db: f64::NAN,
                ..ok
            },
            PassiveReceiver {
                noise_figure_db: -1.0,
                ..ok
            },
            PassiveReceiver {
                losses_db: -0.5,
                ..ok
            },
            PassiveReceiver {
                integration_time_s: 0.0,
                ..ok
            },
            PassiveReceiver {
                noise_bandwidth_hz: Some(0.0),
                ..ok
            },
            PassiveReceiver {
                antenna_temperature_k: 0.0,
                ..ok
            },
        ] {
            assert!(bad.validate().is_err(), "{bad:?}");
        }
    }
}
