//! Deterministic, seeded randomness with independent per-stream generators.

pub mod sample;

use crate::ids::SensorId;
use rand_chacha::ChaCha8Rng;
use rand_core::{Rng, SeedableRng};

/// Version of the stream derivation. Bump when the output of any generator or
/// sampler changes, since that breaks replay compatibility.
pub const STREAM_VERSION: u64 = 1;

/// A source of uniformly distributed 64-bit random words.
pub trait RandomSource {
    /// Returns the next uniformly distributed `u64`.
    fn next_u64(&mut self) -> u64;
}

/// The master seed of a simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Seed(pub u64);

impl Seed {
    /// Draws a fresh seed from operating-system entropy.
    ///
    /// The seed is an ordinary [`Seed`], so recording it (see
    /// `Simulation::seed`) allows the run to be replayed exactly.
    ///
    /// # Errors
    /// `EntropyUnavailable` if the operating system cannot supply entropy.
    #[cfg(feature = "entropy")]
    pub fn from_entropy() -> Result<Seed, crate::error::RadarError> {
        getrandom::u64()
            .map(Seed)
            .map_err(|e| crate::error::RadarError::EntropyUnavailable(e.to_string()))
    }
}

/// Builds the random source for one stream of a simulation; see
/// `SimulationConfig::random_source_factory`.
pub type RandomSourceFactory = Box<dyn Fn(StreamId) -> Box<dyn RandomSource + Send> + Send>;

/// Identifies an independent random stream derived from a [`Seed`].
///
/// Sensor ids of `2^62` or more would overflow the stream index and collide
/// with other streams, so sensor ids must stay below `2^62`; `Simulation`
/// rejects larger ids with `InvalidConfig`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum StreamId {
    /// The stream owned by one sensor.
    Sensor(SensorId),
    /// The stream used for environment effects.
    Environment,
    /// The stream used for scenario setup.
    Scenario,
}

impl StreamId {
    /// Maps the stream to its ChaCha stream number.
    pub(crate) fn index(self) -> u64 {
        match self {
            StreamId::Sensor(id) => id.0 << 2,
            StreamId::Environment => 1,
            StreamId::Scenario => 2,
        }
    }
}

/// A ChaCha8-based [`RandomSource`] bound to one stream of one seed.
#[derive(Debug, Clone)]
pub struct ChaChaStream(ChaCha8Rng);

impl ChaChaStream {
    /// Creates the generator for `stream` under the master `seed`.
    pub fn new(seed: Seed, stream: StreamId) -> Self {
        let mut rng =
            ChaCha8Rng::seed_from_u64(seed.0 ^ STREAM_VERSION.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        rng.set_stream(stream.index());
        Self(rng)
    }
}

impl RandomSource for ChaChaStream {
    fn next_u64(&mut self) -> u64 {
        self.0.next_u64()
    }
}

#[cfg(test)]
mod tests {
    use super::sample::*;
    use super::*;

    #[cfg(feature = "serde")]
    #[test]
    fn replay_types_are_serde() {
        fn serde<T: serde::Serialize + serde::de::DeserializeOwned>() {}
        serde::<Seed>();
        serde::<StreamId>();
        #[cfg(feature = "diagnostics")]
        {
            serde::<crate::diagnostics::DiagnosticRecord>();
            serde::<crate::diagnostics::DiagnosticOutcome>();
        }
    }

    fn stream(seed: u64, id: u64) -> ChaChaStream {
        ChaChaStream::new(Seed(seed), StreamId::Sensor(SensorId(id)))
    }

    #[cfg(feature = "entropy")]
    #[test]
    fn from_entropy_differs() {
        let a = Seed::from_entropy().unwrap();
        let b = Seed::from_entropy().unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn same_seed_same_stream() {
        let mut a = stream(7, 1);
        let mut b = stream(7, 1);
        for _ in 0..100 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn streams_independent() {
        let first_of_2 = stream(7, 2).next_u64();
        let mut s1 = stream(7, 1);
        assert_ne!(s1.next_u64(), first_of_2);
        for _ in 0..1000 {
            s1.next_u64();
        }
        assert_eq!(stream(7, 2).next_u64(), first_of_2);
    }

    fn mean_var(n: usize, mut f: impl FnMut() -> f64) -> (f64, f64) {
        let xs: Vec<f64> = (0..n).map(|_| f()).collect();
        let m = xs.iter().sum::<f64>() / n as f64;
        let v = xs.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / n as f64;
        (m, v)
    }

    #[test]
    fn sampler_moments() {
        const N: usize = 200_000;
        let mut r = stream(12345, 1);
        let (m, v) = mean_var(N, || standard_normal(&mut r));
        assert!(m.abs() < 0.01, "normal mean {m}");
        assert!((v - 1.0).abs() < 0.02, "normal var {v}");

        let (m, _) = mean_var(N, || exponential(&mut r, 2.0));
        assert!((m - 2.0).abs() < 0.03, "exp mean {m}");

        let (m, v) = mean_var(N, || gamma2(&mut r, 0.5));
        assert!((m - 1.0).abs() < 0.01, "gamma2 mean {m}");
        assert!((v - 0.5).abs() < 0.02, "gamma2 var {v}");

        let (m, _) = mean_var(N, || f64::from(poisson(&mut r, 4.0)));
        assert!((m - 4.0).abs() < 0.03, "poisson(4) mean {m}");
        let (m, _) = mean_var(N, || f64::from(poisson(&mut r, 100.0)));
        assert!((m - 100.0).abs() < 0.2, "poisson(100) mean {m}");
    }

    #[test]
    fn sampler_edge_cases() {
        let mut r = stream(1, 1);
        assert_eq!(poisson(&mut r, 0.0), 0);
        assert_eq!(poisson(&mut r, -3.0), 0);
        assert!(!bernoulli(&mut r, 0.0));
        assert!(bernoulli(&mut r, 1.0));
        for _ in 0..1000 {
            let u = uniform01(&mut r);
            assert!((0.0..1.0).contains(&u));
        }
    }

    #[test]
    fn golden_values() {
        // changing these breaks replay compatibility; bump STREAM_VERSION
        let mut r = stream(42, 1);
        let u: Vec<f64> = (0..3).map(|_| uniform01(&mut r)).collect();
        let mut r = stream(42, 1);
        let n: Vec<f64> = (0..3).map(|_| standard_normal(&mut r)).collect();
        assert_eq!(
            u,
            [0.28455179678289744, 0.3111129799157669, 0.7516088725926959]
        );
        assert_eq!(
            n,
            [-1.122159978379963, 1.6562392891324251, -1.310108401463145]
        );
    }
}
