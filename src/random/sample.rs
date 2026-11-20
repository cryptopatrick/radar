//! Distribution samplers implemented in-crate so output never depends on
//! third-party sampling code.

use super::RandomSource;

/// Largest mean [`poisson`] samples with; larger (or infinite) means are
/// clamped to it so a pathological rate cannot exhaust memory.
pub const MAX_POISSON_LAMBDA: f64 = 1e4;

/// Uniform sample in `[0, 1)` with 53 bits of resolution.
pub fn uniform01(r: &mut dyn RandomSource) -> f64 {
    (r.next_u64() >> 11) as f64 * (1.0 / 9_007_199_254_740_992.0)
}

/// Standard normal sample (Marsaglia polar method). The spare value is
/// discarded so the number of draws depends only on rejections.
pub fn standard_normal(r: &mut dyn RandomSource) -> f64 {
    loop {
        let x = 2.0 * uniform01(r) - 1.0;
        let y = 2.0 * uniform01(r) - 1.0;
        let s = x * x + y * y;
        if s > 0.0 && s < 1.0 {
            return x * (-2.0 * s.ln() / s).sqrt();
        }
    }
}

/// Exponential sample with the given `mean`.
pub fn exponential(r: &mut dyn RandomSource, mean: f64) -> f64 {
    -mean * (1.0 - uniform01(r)).ln()
}

/// Sum of two exponentials of mean `theta` (Gamma with shape 2, scale `theta`).
pub fn gamma2(r: &mut dyn RandomSource, theta: f64) -> f64 {
    exponential(r, theta) + exponential(r, theta)
}

/// Poisson sample with mean `lambda`; NaN or non-positive `lambda` yields 0
/// and `lambda` above [`MAX_POISSON_LAMBDA`] (including `+∞`) is clamped to it.
///
/// Uses Knuth multiplication below 30 and a rounded normal approximation above.
pub fn poisson(r: &mut dyn RandomSource, lambda: f64) -> u32 {
    if lambda.is_nan() || lambda <= 0.0 {
        return 0;
    }
    let lambda = lambda.min(MAX_POISSON_LAMBDA);
    if lambda < 30.0 {
        let limit = (-lambda).exp();
        let mut k = 0u32;
        let mut p = uniform01(r);
        while p > limit {
            k += 1;
            p *= uniform01(r);
        }
        k
    } else {
        (lambda + lambda.sqrt() * standard_normal(r))
            .round()
            .max(0.0) as u32
    }
}

/// Returns `true` with probability `p`.
pub fn bernoulli(r: &mut dyn RandomSource, p: f64) -> bool {
    uniform01(r) < p
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::SensorId;
    use crate::random::{ChaChaStream, Seed, StreamId};

    #[test]
    fn poisson_clamps_huge_and_non_finite_lambda() {
        let mut r = ChaChaStream::new(Seed(3), StreamId::Sensor(SensorId(1)));
        for lambda in [f64::INFINITY, 1e300] {
            let k = poisson(&mut r, lambda);
            assert!(f64::from(k) < 2.0 * MAX_POISSON_LAMBDA, "{lambda} -> {k}");
        }
        assert_eq!(poisson(&mut r, f64::NAN), 0);
        assert_eq!(poisson(&mut r, f64::NEG_INFINITY), 0);
    }
}
