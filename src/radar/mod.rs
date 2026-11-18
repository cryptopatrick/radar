//! Radar configuration and models.

pub mod antenna;
pub mod calibration;
pub mod config;
pub mod detection;
pub mod false_alarm;
pub mod measurement;
pub mod scan;

pub use antenna::{AntennaConfig, BeamPattern};
pub use calibration::{Calibration, JammingReference, analytic_pd};
pub use config::{
    DetectionConfig, Dimension, IdentificationConfig, RadarConfig, RadarConfigBuilder,
    RangeEquationConfig, RangeLimits, ScanFrame,
};
pub use detection::{
    DetectionContext, DetectionModel, DetectionResult, RadiusDetector, RangeEquationDetector,
};
pub use false_alarm::resolution_cells;
pub use measurement::{Measurement, MeasurementConfig, MeasurementSigma, measurement_sigma};
pub use scan::{
    ScanConfig, ScanPattern, ScanState, Sweep, half_extent, illumination, max_sweep_per_update,
};
