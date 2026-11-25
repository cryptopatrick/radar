# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the crate
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **`bistatic` module:** link budgets for passive radars that listen to an
  illuminator of opportunity. `Illuminator` with presets for Starlink, GPS
  L1 C/A and L5, Galileo E1 and E5, and DVB-S2 (from Gomez-del-Hoyo et al.,
  IRS 2022); `BistaticGeometry` (transmitter, receiver and target ranges,
  baseline, bistatic range, bistatic angle and bisector); and
  `PassiveReceiver`, which evaluates the bistatic radar range equation with
  coherent processing gain.
- **Bistatic resolution and Doppler:** `range_resolution` (eq. (2) of the
  paper, with bistatic and aspect angles), `velocity_resolution`, and
  `BistaticGeometry` methods for both, plus the echo and direct-signal
  Doppler shifts with a moving transmitter.

## [0.2.0] - 2026-11-20

A complete rewrite. `radar` is now a realistic radar and sensor simulation
library for games and simulations. Nothing from 0.1.x carries over: the 0.1.x
releases were a placeholder ("radar data formats") with no usable API.

### Added

- **Core:** SI units and a `WorldScale` for converting host units and ticks;
  integer-nanosecond `SimTime`/`SimDuration`; geometry (vectors, compass
  angles, rotations, poses) in a right-handed east-north-up frame.
- **World:** host-owned `WorldSnapshot` of platforms, targets and emitters,
  with validation and linear extrapolation between snapshots, plus an
  `Environment` trait for line-of-sight occlusion.
- **Radar:** configuration presets (for example `RadarConfig::surveillance_2d`),
  antenna patterns and calibration; scan patterns (fixed, circular, sector,
  raster, volume, steered and custom) in platform or world-stabilised frames;
  detection from a
  fixed radius or from the radar range equation; measurement noise; false
  alarms; transponder identification.
- **Simulation:** `Simulation` with per-sensor scheduling, an observation
  stream that never contains ground truth, and `reset()` for replay.
- **Determinism:** a per-sensor ChaCha8 stream derived from the master seed
  and `SensorId`, with in-crate samplers, so the same seed and inputs give an
  identical observation stream. Pluggable random sources are available for
  non-deterministic noise.
- **`tracking` feature (default):** alpha-beta and constant-velocity Kalman
  filters, gated association, and an M-of-N track lifecycle (tentative,
  confirmed, coasting, deleted).
- **`ew` feature (default):** noise jammers that add J/N into the SNR, and
  jamming strobes.
- **`serde` feature:** `Serialize`/`Deserialize` on configs, observations,
  seeds and diagnostics records.
- **`entropy` feature:** `Seed::from_entropy()` from the operating system.
- **`diagnostics` feature:** a truth-annotated debug channel
  (`Simulation::drain_diagnostics`).
- A `prelude`, examples (`basic_2d`, `basic_3d`, `multi_sensor`) and a
  criterion benchmark (`radar_2d`).

### Changed

- Licence is AGPL-3.0-only.
- Minimum supported Rust version is 1.86, with edition 2024.

## [0.1.1] - 2026-10-15

- Placeholder release: added crates.io categories.

## [0.1.0] - 2026-10-09

- Placeholder release: minimal manifest and library.

[0.2.0]: https://github.com/cryptopatrick/radar/compare/0345e00...v0.2.0
[0.1.1]: https://github.com/cryptopatrick/radar/commit/0345e00
[0.1.0]: https://github.com/cryptopatrick/radar/commit/7ed73c0
