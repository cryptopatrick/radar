//! Bistatic and passive radar link budgets.
//!
//! A passive radar has no transmitter of its own: it listens to an
//! *illuminator of opportunity* (a broadcast, navigation or communication
//! transmitter) and detects the echoes that targets scatter towards the
//! receiver. Transmitter and receiver sit in different places, so the
//! geometry is bistatic.
//!
//! This module is analytic. It computes link budgets for a given geometry;
//! it does not take part in [`Simulation`](crate::simulation::Simulation),
//! whose radars stay monostatic.
//!
//! The model follows P. Gomez-del-Hoyo, K. Gronowski and P. Samczynski,
//! "The STARLINK-based passive radar: preliminary study and first
//! illuminator signal measurements", *IRS 2022*, which compares Starlink,
//! GPS, Galileo and DVB-S2 satellites as illuminators.
//!
//! [`BistaticGeometry`] also gives the bistatic range and velocity
//! resolution and the Doppler shift of the echo, including the shift a
//! moving satellite puts on a stationary target.
//!
//! ```
//! use radar::bistatic::{BistaticGeometry, Illuminator, PassiveReceiver};
//! use radar::geometry::Vector3;
//!
//! let starlink = Illuminator::starlink();
//! let receiver = PassiveReceiver::default(); // 40 dBi, 0.5 s integration
//! let geometry = BistaticGeometry {
//!     transmitter: Vector3::new(0.0, 0.0, starlink.altitude_m),
//!     receiver: Vector3::ZERO,
//!     target: Vector3::new(0.0, 10_000.0, 0.0),
//! };
//! let snr_db = receiver.snr_db_at(&starlink, 40.0, &geometry);
//! assert!(snr_db > 16.5); // a 40 m² truck at 10 km
//! ```

mod budget;
mod geometry;
mod illuminator;
mod resolution;

pub use budget::PassiveReceiver;
pub use geometry::BistaticGeometry;
pub use illuminator::Illuminator;
pub use resolution::{range_resolution, velocity_resolution};
