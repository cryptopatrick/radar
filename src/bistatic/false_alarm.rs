//! False alarms in a passive radar's range-Doppler map, and cell-averaging
//! CFAR.

use super::{Illuminator, PassiveReceiver};
use crate::error::RadarError;
use crate::units::{SPEED_OF_LIGHT, linear_to_db};

/// The range-Doppler map a passive radar searches each coherent processing
/// interval (CPI).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RangeDopplerMap {
    /// Largest bistatic range searched, m (> 0).
    pub max_bistatic_range_m: f64,
    /// Range bin size, m (> 0).
    pub range_resolution_m: f64,
    /// Largest Doppler searched either side of zero, Hz (> 0).
    pub max_doppler_hz: f64,
    /// Coherent processing interval, s (> 0); the Doppler bin is `1/T`.
    pub integration_time_s: f64,
}

impl RangeDopplerMap {
    /// The map for `receiver` listening to `illuminator`: range bins of
    /// `c/(2B)` out to `max_bistatic_range_m`, Doppler bins of `1/T_int` out
    /// to `±2·v/λ` for targets up to `max_target_speed_mps`.
    pub fn for_receiver(
        illuminator: &Illuminator,
        receiver: &PassiveReceiver,
        max_bistatic_range_m: f64,
        max_target_speed_mps: f64,
    ) -> Self {
        Self {
            max_bistatic_range_m,
            range_resolution_m: SPEED_OF_LIGHT / (2.0 * illuminator.bandwidth_hz),
            max_doppler_hz: 2.0 * max_target_speed_mps / illuminator.wavelength(),
            integration_time_s: receiver.integration_time_s,
        }
    }

    /// Checks every field is finite and positive.
    pub fn validate(&self) -> Result<(), RadarError> {
        for (field, v) in [
            ("max_bistatic_range_m", self.max_bistatic_range_m),
            ("range_resolution_m", self.range_resolution_m),
            ("max_doppler_hz", self.max_doppler_hz),
            ("integration_time_s", self.integration_time_s),
        ] {
            if !(v.is_finite() && v > 0.0) {
                return Err(RadarError::InvalidConfig {
                    field,
                    reason: "must be finite and > 0".into(),
                });
            }
        }
        Ok(())
    }

    /// Range bins: `⌈R_max / ΔR⌉`.
    pub fn range_cells(&self) -> f64 {
        (self.max_bistatic_range_m / self.range_resolution_m).ceil()
    }

    /// Doppler bins: `⌈2·f_max·T⌉`.
    pub fn doppler_cells(&self) -> f64 {
        (2.0 * self.max_doppler_hz * self.integration_time_s).ceil()
    }

    /// Cells searched per CPI.
    pub fn cells(&self) -> f64 {
        self.range_cells() * self.doppler_cells()
    }

    /// Expected false alarms per CPI at per-cell probability `pfa`.
    pub fn false_alarms_per_cpi(&self, pfa: f64) -> f64 {
        pfa * self.cells()
    }

    /// Expected false alarms per second at per-cell probability `pfa`.
    pub fn false_alarm_rate_hz(&self, pfa: f64) -> f64 {
        self.false_alarms_per_cpi(pfa) / self.integration_time_s
    }

    /// Per-cell `pfa` giving `rate_hz` false alarms per second.
    pub fn pfa_for_rate(&self, rate_hz: f64) -> f64 {
        rate_hz * self.integration_time_s / self.cells()
    }
}

/// Cell-averaging CFAR threshold multiplier for `reference_cells` (N)
/// cells and per-cell `pfa` in exponential (square-law) noise:
/// `α = N·(pfa^(−1/N) − 1)`.
///
/// Returns [`RadarError::InvalidConfig`] for `pfa` outside `(0, 1)` or no
/// reference cells.
pub fn ca_cfar_multiplier(pfa: f64, reference_cells: u32) -> Result<f64, RadarError> {
    check(pfa, reference_cells)?;
    let n = f64::from(reference_cells);
    Ok(n * (pfa.powf(-1.0 / n) - 1.0))
}

/// Probability of detecting a Swerling 1 target of linear mean SNR `snr`
/// with cell-averaging CFAR: `(1 + α/(N·(1 + snr)))^(−N)`.
pub fn ca_cfar_pd(snr: f64, pfa: f64, reference_cells: u32) -> Result<f64, RadarError> {
    let alpha = ca_cfar_multiplier(pfa, reference_cells)?;
    let n = f64::from(reference_cells);
    Ok((1.0 + alpha / (n * (1.0 + snr))).powf(-n))
}

/// Extra mean SNR, dB, a Swerling 1 target needs to reach `pd` with
/// cell-averaging CFAR over `reference_cells` cells, compared with a
/// detector that knows the noise level exactly. It falls towards zero as
/// the number of reference cells grows.
pub fn ca_cfar_loss_db(pd: f64, pfa: f64, reference_cells: u32) -> Result<f64, RadarError> {
    check(pfa, reference_cells)?;
    if !(pd > pfa && pd < 1.0) {
        return Err(RadarError::InvalidConfig {
            field: "pd",
            reason: "must be in (pfa, 1)".into(),
        });
    }
    let n = f64::from(reference_cells);
    let cfar = (pfa.powf(-1.0 / n) - 1.0) / (pd.powf(-1.0 / n) - 1.0) - 1.0;
    let ideal = pfa.ln() / pd.ln() - 1.0;
    Ok(linear_to_db(cfar / ideal))
}

fn check(pfa: f64, reference_cells: u32) -> Result<(), RadarError> {
    if !(pfa > 0.0 && pfa < 1.0) {
        return Err(RadarError::InvalidConfig {
            field: "pfa",
            reason: "must be in (0, 1)".into(),
        });
    }
    if reference_cells == 0 {
        return Err(RadarError::InvalidConfig {
            field: "reference_cells",
            reason: "must be > 0".into(),
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radar::analytic_pd;
    use crate::units::db_to_linear;
    use crate::world::Swerling;
    use approx::assert_relative_eq;

    #[test]
    fn starlink_map_is_huge() {
        // 0.6 m range bins out to 30 km and 2 Hz Doppler bins for ±50 m/s
        // targets: about 2·10⁸ cells, so a Pfa of 1e-6 means ~195 false
        // alarms every CPI.
        let il = Illuminator::starlink();
        let rx = PassiveReceiver::default();
        let map = RangeDopplerMap::for_receiver(&il, &rx, 30e3, 50.0);
        map.validate().unwrap();
        assert_eq!(map.range_cells(), 50_035.0);
        assert_eq!(map.doppler_cells(), 3_903.0);
        assert!((map.false_alarms_per_cpi(1e-6) - 195.3).abs() < 0.1);
        assert_relative_eq!(
            map.false_alarm_rate_hz(1e-6),
            2.0 * map.false_alarms_per_cpi(1e-6)
        );
        // One false alarm a minute needs a per-cell Pfa near 4e-11.
        let pfa = map.pfa_for_rate(1.0 / 60.0);
        assert_relative_eq!(
            map.false_alarm_rate_hz(pfa),
            1.0 / 60.0,
            max_relative = 1e-12
        );
        assert!(pfa > 4e-11 && pfa < 5e-11, "{pfa}");
    }

    #[test]
    fn cfar_multiplier_gives_the_requested_pfa() {
        for n in [4, 16, 64] {
            let alpha = ca_cfar_multiplier(1e-4, n).unwrap();
            // Pd at zero SNR is the false-alarm probability.
            assert_relative_eq!(ca_cfar_pd(0.0, 1e-4, n).unwrap(), 1e-4, max_relative = 1e-9);
            assert!(alpha > -(1e-4_f64.ln()));
        }
    }

    #[test]
    fn cfar_approaches_the_ideal_detector() {
        let snr = db_to_linear(20.0);
        let ideal = analytic_pd(snr, 1e-6, Swerling::One);
        let mut prev = 0.0;
        for n in [8, 16, 32, 128, 10_000] {
            let pd = ca_cfar_pd(snr, 1e-6, n).unwrap();
            assert!(pd > prev && pd < ideal, "{n}: {pd}");
            prev = pd;
        }
        assert_relative_eq!(prev, ideal, epsilon = 1e-3);
    }

    #[test]
    fn cfar_loss_values() {
        // Swerling 1, Pd 0.9, Pfa 1e-6: about 4.3, 2.0 and 1.0 dB for 8, 16
        // and 32 reference cells.
        for (n, loss) in [(8, 4.27), (16, 2.01), (32, 0.97)] {
            let got = ca_cfar_loss_db(0.9, 1e-6, n).unwrap();
            assert!((got - loss).abs() < 0.01, "{n}: {got}");
        }
        assert!(ca_cfar_loss_db(0.9, 1e-6, 100_000).unwrap() < 0.01);
        // The loss is the SNR gap at the same Pd.
        let need = ca_cfar_loss_db(0.9, 1e-6, 16).unwrap();
        let ideal_snr = 1e-6_f64.ln() / 0.9_f64.ln() - 1.0;
        let pd = ca_cfar_pd(ideal_snr * db_to_linear(need), 1e-6, 16).unwrap();
        assert_relative_eq!(pd, 0.9, epsilon = 1e-9);
    }

    #[test]
    fn bad_input_is_rejected() {
        assert!(ca_cfar_multiplier(0.0, 16).is_err());
        assert!(ca_cfar_multiplier(1e-6, 0).is_err());
        assert!(ca_cfar_loss_db(1e-7, 1e-6, 16).is_err());
        assert!(ca_cfar_loss_db(1.0, 1e-6, 16).is_err());
        let map = RangeDopplerMap {
            max_bistatic_range_m: 1.0,
            range_resolution_m: 0.0,
            max_doppler_hz: 1.0,
            integration_time_s: 1.0,
        };
        assert!(map.validate().is_err());
    }
}
