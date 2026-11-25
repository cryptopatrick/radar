//! Noise and interference in a passive receiver.

use super::{BistaticGeometry, Illuminator, PassiveReceiver};
use crate::error::RadarError;
use crate::units::{db_to_linear, linear_to_db};
use std::f64::consts::PI;

/// Direct-signal interference: the illuminator's own signal leaking into
/// the surveillance channel along the baseline, partly removed by
/// cancellation (for example an adaptive filter fed by the reference
/// channel).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DirectSignal {
    /// Surveillance antenna gain towards the transmitter, dBi (usually a
    /// sidelobe).
    pub gain_towards_transmitter_db: f64,
    /// How much cancellation removes, dB (≥ 0).
    pub cancellation_db: f64,
}

/// Everything besides thermal noise that competes with the echo. It is
/// treated as noise-like: coherent integration suppresses it exactly as it
/// suppresses noise.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Interference {
    /// Direct-signal interference, if modelled.
    pub direct_signal: Option<DirectSignal>,
    /// Other in-band interference at the receiver input (other satellites,
    /// terrestrial emitters), W (≥ 0).
    pub other_w: f64,
}

impl Interference {
    /// Checks the cancellation and other power are finite and non-negative,
    /// and the gain is finite.
    pub fn validate(&self) -> Result<(), RadarError> {
        let invalid = |field: &'static str, reason: &str| {
            Err(RadarError::InvalidConfig {
                field,
                reason: reason.into(),
            })
        };
        if let Some(d) = self.direct_signal {
            if !d.gain_towards_transmitter_db.is_finite() {
                return invalid("gain_towards_transmitter_db", "must be finite");
            }
            if !(d.cancellation_db.is_finite() && d.cancellation_db >= 0.0) {
                return invalid("cancellation_db", "must be finite and >= 0");
            }
        }
        if !(self.other_w.is_finite() && self.other_w >= 0.0) {
            return invalid("other_w", "must be finite and >= 0");
        }
        Ok(())
    }
}

impl PassiveReceiver {
    /// Receiver noise power, dBW.
    pub fn noise_power_dbw(&self, illuminator: &Illuminator) -> f64 {
        linear_to_db(self.noise_power_w(illuminator))
    }

    /// Direct-signal power at the surveillance channel input before
    /// cancellation, W: `EIRP·G·λ² / ((4π·L)²)` for baseline `L` and gain
    /// `G` towards the transmitter.
    pub fn direct_signal_power_w(
        &self,
        illuminator: &Illuminator,
        baseline_m: f64,
        gain_towards_transmitter_db: f64,
    ) -> f64 {
        let lambda = illuminator.wavelength();
        illuminator.eirp_w() * db_to_linear(gain_towards_transmitter_db) * lambda * lambda
            / (4.0 * PI * baseline_m).powi(2)
    }

    /// Interference-to-noise ratio (linear) at the receiver input, after
    /// cancellation, for a transmitter `baseline_m` away.
    pub fn interference_to_noise(
        &self,
        illuminator: &Illuminator,
        baseline_m: f64,
        interference: &Interference,
    ) -> f64 {
        let direct = interference.direct_signal.map_or(0.0, |d| {
            self.direct_signal_power_w(illuminator, baseline_m, d.gain_towards_transmitter_db)
                / db_to_linear(d.cancellation_db)
        });
        (direct + interference.other_w) / self.noise_power_w(illuminator)
    }

    /// Linear signal-to-interference-plus-noise ratio after integration,
    /// `SNR / (1 + INR)`, for a target of `rcs_m2` at `geometry`.
    pub fn sinr_at(
        &self,
        illuminator: &Illuminator,
        rcs_m2: f64,
        geometry: &BistaticGeometry,
        interference: &Interference,
    ) -> f64 {
        self.snr_at(illuminator, rcs_m2, geometry)
            / (1.0 + self.interference_to_noise(illuminator, geometry.baseline(), interference))
    }

    /// [`PassiveReceiver::sinr_at`] in dB.
    pub fn sinr_db_at(
        &self,
        illuminator: &Illuminator,
        rcs_m2: f64,
        geometry: &BistaticGeometry,
        interference: &Interference,
    ) -> f64 {
        linear_to_db(self.sinr_at(illuminator, rcs_m2, geometry, interference))
    }

    /// Cancellation, dB, that brings direct-signal interference down to
    /// `target_inr_db` relative to noise; 0 if it is already there.
    pub fn required_cancellation_db(
        &self,
        illuminator: &Illuminator,
        baseline_m: f64,
        gain_towards_transmitter_db: f64,
        target_inr_db: f64,
    ) -> f64 {
        let inr = self.direct_signal_power_w(illuminator, baseline_m, gain_towards_transmitter_db)
            / self.noise_power_w(illuminator);
        (linear_to_db(inr) - target_inr_db).max(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Vector3;
    use crate::units::{BOLTZMANN, T0_KELVIN};
    use approx::assert_relative_eq;

    fn setup() -> (Illuminator, PassiveReceiver, BistaticGeometry) {
        let il = Illuminator::starlink();
        let g = BistaticGeometry {
            transmitter: Vector3::new(0.0, 0.0, il.altitude_m),
            receiver: Vector3::ZERO,
            target: Vector3::new(0.0, 5e3, 0.0),
        };
        (il, PassiveReceiver::default(), g)
    }

    #[test]
    fn noise_floor_is_kt0b() {
        let (il, rx, _) = setup();
        assert_relative_eq!(
            rx.noise_power_dbw(&il),
            linear_to_db(BOLTZMANN * T0_KELVIN * 250e6),
            epsilon = 1e-12
        );
        // About −120 dBW over Starlink's 250 MHz.
        assert!((rx.noise_power_dbw(&il) + 120.0).abs() < 0.1);
    }

    #[test]
    fn direct_signal_follows_the_free_space_link() {
        let (il, rx, g) = setup();
        let lambda = il.wavelength();
        let expected = 10f64.powf(2.76) * 10.0 * lambda * lambda / (4.0 * PI * 550e3).powi(2);
        assert_relative_eq!(
            rx.direct_signal_power_w(&il, g.baseline(), 10.0),
            expected,
            max_relative = 1e-12
        );
        // Through a 10 dBi sidelobe it sits about 11 dB under the noise;
        // through the 40 dBi main beam, about 19 dB over it.
        let inr = |gain| {
            linear_to_db(rx.interference_to_noise(
                &il,
                g.baseline(),
                &Interference {
                    direct_signal: Some(DirectSignal {
                        gain_towards_transmitter_db: gain,
                        cancellation_db: 0.0,
                    }),
                    other_w: 0.0,
                },
            ))
        };
        assert!((inr(10.0) + 11.0).abs() < 0.2, "{}", inr(10.0));
        assert!((inr(40.0) - 19.0).abs() < 0.2, "{}", inr(40.0));
    }

    #[test]
    fn sinr_combines_noise_and_interference() {
        let (il, rx, g) = setup();
        let snr = rx.snr_at(&il, 40.0, &g);
        assert_relative_eq!(
            rx.sinr_at(&il, 40.0, &g, &Interference::default()),
            snr,
            max_relative = 1e-12
        );
        // Interference equal to the noise halves the SINR.
        let equal = Interference {
            direct_signal: None,
            other_w: rx.noise_power_w(&il),
        };
        assert_relative_eq!(
            rx.sinr_at(&il, 40.0, &g, &equal),
            snr / 2.0,
            max_relative = 1e-12
        );
        // A main-beam direct signal costs about 19 dB until it is cancelled.
        let mut dsi = Interference {
            direct_signal: Some(DirectSignal {
                gain_towards_transmitter_db: 40.0,
                cancellation_db: 0.0,
            }),
            other_w: 0.0,
        };
        let loss = linear_to_db(snr) - rx.sinr_db_at(&il, 40.0, &g, &dsi);
        assert!((loss - 19.05).abs() < 0.2, "{loss}");
        dsi.direct_signal.as_mut().unwrap().cancellation_db = 60.0;
        let loss = linear_to_db(snr) - rx.sinr_db_at(&il, 40.0, &g, &dsi);
        assert!(loss < 0.01, "{loss}");
    }

    #[test]
    fn required_cancellation() {
        let (il, rx, g) = setup();
        let need = rx.required_cancellation_db(&il, g.baseline(), 40.0, -10.0);
        let inr = rx.interference_to_noise(
            &il,
            g.baseline(),
            &Interference {
                direct_signal: Some(DirectSignal {
                    gain_towards_transmitter_db: 40.0,
                    cancellation_db: need,
                }),
                other_w: 0.0,
            },
        );
        assert_relative_eq!(linear_to_db(inr), -10.0, epsilon = 1e-9);
        // A sidelobe already under −10 dB needs nothing.
        assert_eq!(
            rx.required_cancellation_db(&il, g.baseline(), -10.0, -10.0),
            0.0
        );
    }

    #[test]
    fn validate_rejects_bad_interference() {
        Interference::default().validate().unwrap();
        let ds = |gain, cancel| Interference {
            direct_signal: Some(DirectSignal {
                gain_towards_transmitter_db: gain,
                cancellation_db: cancel,
            }),
            other_w: 0.0,
        };
        assert!(ds(f64::NAN, 0.0).validate().is_err());
        assert!(ds(0.0, -1.0).validate().is_err());
        assert!(
            Interference {
                direct_signal: None,
                other_w: -1.0
            }
            .validate()
            .is_err()
        );
    }
}
