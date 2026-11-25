//! Illuminators of opportunity.

use crate::error::RadarError;
use crate::units::{SPEED_OF_LIGHT, db_to_linear};
use std::f64::consts::PI;

/// A transmitter a passive radar listens to.
///
/// The presets use the parameters in Table II of Gomez-del-Hoyo et al.
/// (IRS 2022). Where the table gives a band, the carrier is its midpoint.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Illuminator {
    /// Equivalent isotropically radiated power over the whole signal
    /// bandwidth, dBW.
    pub eirp_dbw: f64,
    /// Carrier frequency, Hz (> 0).
    pub frequency_hz: f64,
    /// Signal bandwidth, Hz (> 0).
    pub bandwidth_hz: f64,
    /// Nominal orbit altitude, m (> 0). Used by the presets' examples and by
    /// [`Illuminator::nadir_flux_density_dbw_m2`].
    pub altitude_m: f64,
}

impl Illuminator {
    /// Starlink Ku-band user downlink: 27.6 dBW over one 250 MHz channel,
    /// 11.7 GHz (band 10.825–12.575 GHz), 550 km.
    pub fn starlink() -> Self {
        Self::new(27.6, 11.7e9, 250e6, 550e3)
    }

    /// GPS L1 C/A: 26.5 dBW, 1.57542 GHz, 2 MHz, 20 200 km.
    pub fn gps_l1_ca() -> Self {
        Self::new(26.5, 1.575_42e9, 2e6, 20_200e3)
    }

    /// GPS L5: 25 dBW, 1.17645 GHz, 24 MHz, 20 200 km.
    pub fn gps_l5() -> Self {
        Self::new(25.0, 1.176_45e9, 24e6, 20_200e3)
    }

    /// Galileo E1: 35 dBW, 1.57542 GHz, 25 MHz, 23 222 km.
    pub fn galileo_e1() -> Self {
        Self::new(35.0, 1.575_42e9, 25e6, 23_222e3)
    }

    /// Galileo E5: 33 dBW, 1.191795 GHz, 51 MHz, 23 222 km.
    pub fn galileo_e5() -> Self {
        Self::new(33.0, 1.191_795e9, 51e6, 23_222e3)
    }

    /// DVB-S2 satellite television: 53.7 dBW (the top of the 51–53.7 dBW
    /// range), 11.2 GHz (band 10.2–12.2 GHz), 36 MHz, geostationary
    /// (35 786 km).
    pub fn dvb_s2() -> Self {
        Self::new(53.7, 11.2e9, 36e6, 35_786e3)
    }

    const fn new(eirp_dbw: f64, frequency_hz: f64, bandwidth_hz: f64, altitude_m: f64) -> Self {
        Self {
            eirp_dbw,
            frequency_hz,
            bandwidth_hz,
            altitude_m,
        }
    }

    /// Checks every field is finite and the frequency, bandwidth and altitude
    /// are positive.
    pub fn validate(&self) -> Result<(), RadarError> {
        let invalid = |field: &'static str, reason: &str| {
            Err(RadarError::InvalidConfig {
                field,
                reason: reason.into(),
            })
        };
        if !self.eirp_dbw.is_finite() {
            return invalid("eirp_dbw", "must be finite");
        }
        for (field, v) in [
            ("frequency_hz", self.frequency_hz),
            ("bandwidth_hz", self.bandwidth_hz),
            ("altitude_m", self.altitude_m),
        ] {
            if !(v.is_finite() && v > 0.0) {
                return invalid(field, "must be finite and > 0");
            }
        }
        Ok(())
    }

    /// EIRP, W.
    pub fn eirp_w(&self) -> f64 {
        db_to_linear(self.eirp_dbw)
    }

    /// Carrier wavelength, m.
    pub fn wavelength(&self) -> f64 {
        SPEED_OF_LIGHT / self.frequency_hz
    }

    /// Power flux density at `range_m` from the transmitter, dBW/m²:
    /// `EIRP / (4π·R²)`.
    pub fn flux_density_dbw_m2(&self, range_m: f64) -> f64 {
        self.eirp_dbw - 10.0 * (4.0 * PI * range_m * range_m).log10()
    }

    /// Power flux density directly below the transmitter, dBW/m².
    pub fn nadir_flux_density_dbw_m2(&self) -> f64 {
        self.flux_density_dbw_m2(self.altitude_m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_match_table_ii_flux_density() {
        // Table II, "PFD on Earth surface", dBW/m² (rounded in the paper).
        for (il, pfd) in [
            (Illuminator::gps_l1_ca(), -131.0),
            (Illuminator::gps_l5(), -132.0),
            (Illuminator::galileo_e1(), -123.0),
            (Illuminator::galileo_e5(), -126.0),
            (Illuminator::dvb_s2(), -108.0),
            (Illuminator::starlink(), -98.0),
        ] {
            il.validate().unwrap();
            let got = il.nadir_flux_density_dbw_m2();
            assert!((got - pfd).abs() < 1.0, "{il:?}: {got} dBW/m², table {pfd}");
        }
    }

    #[test]
    fn wavelengths_match_table_ii() {
        // Table II wavelengths are rounded to two decimals.
        for (il, lambda) in [
            (Illuminator::gps_l1_ca(), 0.19),
            (Illuminator::gps_l5(), 0.26),
            (Illuminator::galileo_e5(), 0.25),
            (Illuminator::starlink(), 0.03),
            (Illuminator::dvb_s2(), 0.03),
        ] {
            assert!((il.wavelength() - lambda).abs() < 0.006, "{il:?}");
        }
    }

    #[test]
    fn validate_rejects_bad_fields() {
        let ok = Illuminator::starlink();
        assert!(
            Illuminator {
                eirp_dbw: f64::NAN,
                ..ok
            }
            .validate()
            .is_err()
        );
        assert!(
            Illuminator {
                frequency_hz: 0.0,
                ..ok
            }
            .validate()
            .is_err()
        );
        assert!(
            Illuminator {
                bandwidth_hz: -1.0,
                ..ok
            }
            .validate()
            .is_err()
        );
        assert!(
            Illuminator {
                altitude_m: f64::INFINITY,
                ..ok
            }
            .validate()
            .is_err()
        );
    }
}
