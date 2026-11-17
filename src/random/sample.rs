//! Distribution samplers implemented in-crate so output never depends on
//! third-party sampling code.

use super::RandomSource;

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

/// Poisson sample with mean `lambda`; non-positive `lambda` yields 0.
///
/// Uses Knuth multiplication below 30 and a rounded normal approximation above.
pub fn poisson(r: &mut dyn RandomSource, lambda: f64) -> u32 {
    if lambda.is_nan() || lambda <= 0.0 {
        return 0;
    }
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
