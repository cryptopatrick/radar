//! Realistic radar and sensor simulation for games and simulations.
//!
//! `radar` simulates what a radar operator would actually see: a scanning
//! beam, probabilistic detection from the radar range equation, noisy
//! measurements, false alarms, tracks that form, coast and drop, transponder
//! identification and noise jamming. The host owns the world and the clock;
//! the library never sees host objects and never hands back ground truth.
//!
//! # Quick start
//!
//! ```
//! use radar::prelude::*;
//!
//! # fn main() -> Result<(), RadarError> {
//! let (radar, site) = (SensorId(1), PlatformId(1));
//! let mut sim = Simulation::new(SimulationConfig::default().seed(Seed(7)));
//! // Build configs with the builder or a preset (see `RadarConfig`).
//! sim.add_radar(radar, site, Pose3::IDENTITY, RadarConfig::simple_2d(5_000.0)?)?;
//!
//! let mut world = WorldSnapshot::new(SimTime::ZERO);
//! world.platforms.push(PlatformState {
//!     id: site,
//!     pose: Pose3::IDENTITY,
//!     velocity: Vector3::ZERO,
//! });
//! world.targets.push(TargetState {
//!     id: TargetId(1),
//!     pose: Pose3::from_position_heading(Vector3::new(1_000.0, 2_000.0, 0.0), 0.0),
//!     velocity: Vector3::new(50.0, 0.0, 0.0),
//!     extent_radius: 10.0,
//!     signature: RadarSignature { rcs_m2: 5.0, swerling: Swerling::One },
//!     transponder: None,
//!     platform: None,
//! });
//! sim.update_world(world)?;
//!
//! sim.step_open(SimDuration::from_secs(1))?; // one antenna revolution
//! let mut out = Vec::new();
//! sim.drain(radar, &mut out)?;
//! let plots = out
//!     .iter()
//!     .filter(|o| matches!(o, Observation::Measurement(_)))
//!     .count();
//! assert_eq!(plots, 1);
//! # Ok(())
//! # }
//! ```
//!
//! `examples/` holds fuller programs (`basic_2d`, `basic_3d`,
//! `multi_sensor`).
//!
//! # Conventions
//!
//! - **Frame:** right-handed world frame, **x = east, y = north, z = up**.
//! - **Azimuth** is a compass bearing: from +y (north), clockwise, in
//!   `[0, 2π)`. **Elevation** is up from the horizontal plane, in
//!   `[−π/2, π/2]`.
//! - **Units:** SI everywhere (metres, seconds, radians, hertz, watts,
//!   square metres). Time is integer nanoseconds ([`time::SimTime`],
//!   [`time::SimDuration`]). [`units::WorldScale`] converts host units at
//!   the boundary.
//! - A sensor's mount is relative to its platform; scan angles are
//!   platform-relative or world-stabilised ([`radar::ScanFrame`]).
//! - A [`radar::Dimension::TwoD`] radar reports slant range and no
//!   elevation, so its plots placed in the horizontal plane sit outward of
//!   the ground range of targets at altitude.
//! - Between world snapshots, platform, target and emitter positions are
//!   extrapolated linearly from their velocities.
//!
//! # Features
//!
//! | Feature       | Default | What it adds                                                   |
//! |---------------|---------|----------------------------------------------------------------|
//! | `tracking`    | yes     | Association, alpha-beta and Kalman filters, M-of-N lifecycle   |
//! | `ew`          | yes     | Noise jammers (J/N in the SNR) and jamming strobes             |
//! | `serde`       | no      | `Serialize`/`Deserialize` on configs, observations and seeds   |
//! | `entropy`     | no      | `Seed::from_entropy()` from the operating system               |
//! | `diagnostics` | no      | A ground-truth debug channel (`Simulation::drain_diagnostics`) |
//!
//! Features add fields and enum variants: [`sensor::Observation`],
//! [`error::RadarError`] and the other public enums are `#[non_exhaustive]`,
//! and `RadarConfig` should be built through its builder rather than a
//! struct literal.
//!
//! # Ground truth
//!
//! No observation type ([`radar::Measurement`], tracks,
//! [`sensor::Observation`]) carries a [`ids::TargetId`]: measurements are
//! sorted by range and azimuth so list order reveals nothing, and false
//! alarms carry the same fields as real returns. Truth is available only
//! through the opt-in `diagnostics` feature, whose recording never draws
//! random numbers and so never changes the observation stream.
//!
//! # Determinism
//!
//! - Each sensor draws from its own ChaCha8 stream derived from the master
//!   [`random::Seed`] and its [`ids::SensorId`] (ids must be below `2^62`),
//!   in a fixed order: targets by ascending `TargetId`, then false alarms,
//!   identification replies, jamming strobes and tracking. Samplers are
//!   implemented in-crate.
//! - The same seed and inputs give an identical observation stream, and
//!   adding, removing or disabling one sensor never changes another's.
//! - **Limits:** bit-exact replay is promised only for the same binary on
//!   the same target platform (`sin`, `cos`, `ln`, `exp` may differ across
//!   platforms and libm versions). A custom random source factory makes
//!   runs non-deterministic, and custom models keep their own state across
//!   `Simulation::reset`.
#![warn(missing_docs)]

#[cfg(feature = "diagnostics")]
pub mod diagnostics;
pub mod environment;
pub mod error;
#[cfg(feature = "ew")]
pub(crate) mod ew;
pub mod geometry;
pub mod ids;
pub mod radar;
pub mod random;
pub mod sensor;
pub mod simulation;
pub mod time;
#[cfg(feature = "tracking")]
pub mod tracking;
pub mod units;
pub mod world;

/// Convenient re-exports of the most common types.
pub mod prelude {
    #[cfg(feature = "diagnostics")]
    pub use crate::diagnostics::{DiagnosticOutcome, DiagnosticRecord};
    pub use crate::environment::{Environment, OpenSky};
    pub use crate::error::RadarError;
    pub use crate::geometry::{Pose3, Rotation3, Vector2, Vector3};
    pub use crate::ids::{EmitterId, PlatformId, SensorId, TargetId, TrackId, TransponderCode};
    pub use crate::radar::{
        AntennaConfig, BeamPattern, Calibration, DetectionConfig, DetectionContext, DetectionModel,
        DetectionResult, Dimension, IdentificationConfig, JammingReference, Measurement,
        MeasurementConfig, MeasurementSigma, RadarConfig, RangeEquationConfig,
        RangeEquationDetector, RangeLimits, ScanConfig, ScanFrame, analytic_pd, resolution_cells,
    };
    pub use crate::random::{ChaChaStream, RandomSource, RandomSourceFactory, Seed, StreamId};
    pub use crate::sensor::{Observation, SensorContext, SensorModel};
    pub use crate::simulation::{SensorStatus, Simulation, SimulationConfig, StepPolicy};
    pub use crate::time::{SimDuration, SimTime};
    #[cfg(feature = "tracking")]
    pub use crate::tracking::{
        AlphaBeta, Covariance6, KalmanCv, Track, TrackManagementConfig, TrackState, TrackStatus,
        Tracker, TrackerConfig,
    };
    pub use crate::units::WorldScale;
    pub use crate::world::{
        Band, EmitterKind, EmitterState, PlatformState, RadarSignature, Swerling, TargetState,
        WorldSnapshot,
    };
}
