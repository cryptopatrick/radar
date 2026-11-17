# robocode × radar Integration Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give robocode a `ScanModel` seam. Keep today's exact scan as the default `ClassicScan`, and add an optional realistic `RadarScan` backed by the `radar` crate, where robots receive anonymous track events.

**Architecture:** The scan step of `BattleState::step` moves behind `Box<dyn ScanModel>`. `RadarScan` (cargo feature `radar`) owns a `radar::Simulation`. Each turn it converts robots to a `WorldSnapshot` (y flipped, scaled by `WorldScale`), steers each robot's beam to its radar heading, and steps one turn with buildings as line-of-sight blockers. It reports tracks in `TurnEvents::radar_tracks`. The runner turns these into new `RadarTrack` robot events.

**Tech Stack:** Rust 2024, robocode 0.5.0 (`/Users/cp/wer/crt/robocode`), `radar` via a path dependency.

**Spec:** `/Users/cp/wer/crt/radar/docs/superpowers/specs/2026-11-17-radar-0.1-design.md` §12.

**Prerequisite:** Task 1 has none. Tasks 2–5 need radar plan Tasks 1–12 complete (Simulation, `point_beam`, `command_beam`, tracking, `WorldScale`, `Environment`).

## Global Constraints

- All work happens in `/Users/cp/wer/crt/robocode` on a feature branch (`radar-seam`).
- With the default scan model, **parity output and all existing tests are unchanged**, byte for byte.
- `radar` is an optional dependency: `radar = { path = "../radar", optional = true }`, with feature `radar = ["dep:radar"]`. The default build pulls in nothing new. (Publishing later needs `version = ...` beside `path`; that's out of scope.)
- Existing public API stays source-compatible. New `Robot` trait methods have default no-op bodies.
- In realistic mode, robots receive **no** `ScannedRobotEvent`.
- Units: `radar::WorldScale` defaults to 1 px = 1 m and 1 turn = 1 s. robocode's frame is y-down, so `radar.y = -robocode.y`. Robocode compass degrees map to radar azimuth radians with no offset.
- `RadarRules { identify_robots, reveal_energy, reveal_true_heading }` all default to `false`.
- `cargo fmt --check`, `cargo clippy --all-targets --all-features -- -D warnings`, and `cargo test --all-features` pass at every commit. The Verus feature build is not required to change.

## Review Focus

1. **Multi-round battles** (`Battle::run_rounds` rebuilds `BattleState` at `src/runner.rs:877`). Expected: the realistic model survives round changes, gets a fresh `RadarScan` each round, and seeds deterministically per round. The test is in Task 4.
2. **Radar sweep greater than 45° in one turn.** Body (10°) plus gun (20°) plus radar (45°) can drag the radar 75°/turn. Expected: the steered beam slews the full engine-reported arc, never lagging robocode's own `radar_heading`. The test is in Task 3.
3. **Robots dying mid-battle.** Expected: a dead robot's sensor stops and the robot leaves later snapshots. Other robots' tracks on it coast and are deleted, and no events are delivered to the dead robot. The test is in Task 3.
4. **Robots touching or overlapping**, at distance below `ROBOT_RADIUS`. Expected: no NaN in events. Detection treats the target as within the beam. The test is in Task 3.
5. **Snapshot JSON in classic mode.** Expected: no `radar_tracks` key appears when the list is empty, so parity logs don't change. The test is in Task 5.

---

## File Structure

```
src/physics/scan.rs              ScanModel, ScanWorld, ClassicScan (moved from engine.rs)
src/physics/engine.rs            BattleState holds Box<dyn ScanModel>
src/physics/events.rs            RadarTrackReport, RadarTrackStatus, TurnEvents::radar_tracks
src/physics/mod.rs               re-exports
src/radar_scan/mod.rs            [radar] RadarScan, RadarScanConfig
src/radar_scan/convert.rs        [radar] coordinate/unit conversion
src/radar_scan/environment.rs    [radar] BuildingLineOfSight
src/radar_scan/preset.rs         [radar] robocode_radar(scale)
src/api.rs                       RadarTrackEvent, RadarTrackLostEvent, Event variants, Robot defaults
src/runner.rs                    scan-model factory on Battle; event dispatch; StrategyRobot adoption
src/physics/snapshot.rs          optional "radar_tracks" key
tests/radar_scan.rs              [radar] integration tests
examples/radar_battle.rs         [radar] example
```

---

### Task 1: ScanModel seam with ClassicScan (no radar dependency)

**Files:**
- Create: `src/physics/scan.rs`
- Modify: `src/physics/engine.rs:44-60, 110, 334-370`, `src/physics/mod.rs`
- Test: `tests/scanning.rs` (add one test)

**Interfaces:**
- Produces:
  - `pub trait ScanModel: Send { fn scan(&mut self, world: &ScanWorld<'_>, events: &mut TurnEvents); }`
  - `pub struct ScanWorld<'a> { pub turn: u64, pub robots: &'a [RobotState], pub terrain: &'a Terrain, pub battlefield: Battlefield }`
  - `pub struct ClassicScan;` (`Default`), whose `scan` body is today's `BattleState::scan` moved verbatim. It reads `world.robots` and pushes and sorts `events.scans`.
  - `BattleState` gets a private field `scan_model: Box<dyn ScanModel>`, set to `Box::new(ClassicScan)` in `new`. It also gets `pub fn with_scan_model(mut self, m: Box<dyn ScanModel>) -> Self` and `pub fn set_scan_model(&mut self, m: Box<dyn ScanModel>)`.
  - `step()` calls `self.scan_model.scan(&ScanWorld { turn: self.turn, robots: &self.robots, terrain: &self.terrain, battlefield: self.config.battlefield }, &mut events)` at the same point as before. The borrows are of disjoint fields. (Check the battlefield field name in `BattleConfig` and adapt the name only.)

- [ ] **Step 1: Capture the parity baseline before any change:** `cargo run --release --bin parity -- --battle-id base --turns 2000 --output target/parity-before.log`. Expected: the file exists and is non-empty.
- [ ] **Step 2: Write the failing test** `tests/scanning.rs::custom_scan_model_replaces_classic`: a `struct NoScan; impl ScanModel for NoScan { fn scan(&mut self, _: &ScanWorld<'_>, _: &mut TurnEvents) {} }`. Two robots facing each other at 300 px with spinning radars, using `BattleState::new(..).with_scan_model(Box::new(NoScan))`, produce `events.scans.is_empty()` for 20 turns. The same setup with the default model produces at least one scan.
- [ ] **Step 3:** Run `cargo test --test scanning`. Expected: FAIL (`ScanModel` is not defined).
- [ ] **Step 4:** Implement the seam as specified.
- [ ] **Step 5:** Run `cargo test` (all suites). Expected: PASS, with no existing test changed.
- [ ] **Step 6: Parity check:** run `cargo run --release --bin parity -- --battle-id base --turns 2000 --output target/parity-after.log && cmp target/parity-before.log target/parity-after.log`. Expected: no output, exit 0.
- [ ] **Step 7:** Commit: `refactor: move radar scan behind a ScanModel seam (ClassicScan default)`

---

### Task 2: Radar track report types and the adapter foundations

**Files:**
- Modify: `Cargo.toml`, `src/lib.rs`, `src/physics/events.rs`
- Create: `src/radar_scan/{mod,convert,environment,preset}.rs`

**Interfaces:**
- Produces:
  - In `events.rs` (always compiled; plain robocode types):
    - `pub enum RadarTrackStatus { Tentative, Confirmed, Coasting, Deleted }`
    - `pub struct RadarTrackReport { pub scanner: RobotId, pub track_id: u64, pub status: RadarTrackStatus, pub position: Point, pub velocity: Point /* px per turn, robocode frame */, pub heading_estimate: f64 /* deg [0,360) */, pub speed_estimate: f64 /* px/turn */, pub position_sigma: f64 /* px */, pub identity: Option<RobotId>, pub energy: Option<f64>, pub true_heading: Option<f64> }`
    - `TurnEvents` gets `pub radar_tracks: Vec<RadarTrackReport>`.
  - In `convert.rs` (`#[cfg(feature = "radar")]`):
    - `pub fn to_radar_position(p: Point, s: &WorldScale) -> radar::geometry::Vector3`, which returns `(s.to_metres(p.x), -s.to_metres(p.y), 0)`
    - `pub fn from_radar_position(v: Vector3, s) -> Point`
    - `pub fn heading_to_azimuth(deg: f64) -> f64` (`radar::units::deg` then `wrap_two_pi`)
    - `pub fn azimuth_to_heading(rad: f64) -> f64`
    - `pub fn velocity_to_radar(heading_deg: f64, speed_px_per_turn: f64, s) -> Vector3` (= `direction(az, 0) · s.speed_to_si(speed)`)
    - `pub fn velocity_from_radar(v: Vector3, s) -> Point` (px/turn, y flipped back)
  - In `environment.rs`: `pub struct BuildingLineOfSight<'a> { pub buildings: &'a [Rect], pub scale: WorldScale }`, implementing `radar::environment::Environment`. It converts the endpoints back to px and runs a slab segment-vs-AABB test. A segment that only touches an edge counts as blocked. A segment whose endpoint lies inside a building counts as blocked.
  - In `preset.rs`: `pub fn robocode_radar(scale: &WorldScale) -> Result<RadarConfig, radar::RadarError>`. Values are in game units, converted through `scale`:
    - TwoD; update period `scale.duration(1)`
    - antenna 3° Gaussian
    - `ScanFrame::WorldStabilised`
    - `Steered { max_slew_rate: scale.rate_to_si(deg(75.0)) }`, where 75 = `MAX_RADAR_TURN_RATE + MAX_GUN_TURN_RATE + MAX_TURN_RATE`, written as that sum of the `rules` constants
    - `RangeEquation { Calibration::reference_for_pd(1.0, scale.to_metres(1000.0), 0.9, 1e-6, Swerling::One)?, pfa 1e-6, range_resolution: scale.to_metres(5.0) }`
    - limits `0 .. scale.to_metres(RADAR_SCAN_RADIUS)`
    - `SnrScaled { radial_velocity_resolution: scale.speed_to_si(0.5), floor: (scale.to_metres(0.5), deg(0.1), deg(0.1), scale.speed_to_si(0.05)) }`
    - doppler on
    - `Kalman { process_accel_sigma: scale.accel_to_si(1.0) }`
    - default `TrackManagementConfig`
    - identification None (`RadarScan` sets it when `identify_robots`)
  - `Cargo.toml`: optional `radar` dependency and the `radar` feature. `lib.rs`: `#[cfg(feature = "radar")] pub mod radar_scan;`.

- [ ] **Step 1: Write the failing tests** (unit tests in `radar_scan`):
  - `y_flip_roundtrip`: `from_radar_position(to_radar_position(Point::new(100., 200.), &s), &s) == Point::new(100., 200.)`.
  - `north_is_up_on_screen`: robocode heading 0 at speed 8 → radar velocity ≈ (0, 8, 0), and `velocity_from_radar` gives (0, −8) px/turn (moving to smaller y on screen).
  - `heading_east`: 90° → azimuth π/2.
  - `building_blocks`: a building `Rect::new(100,100,50,50)`; the segment (0,125)→(300,125) is blocked; the segment (0,0)→(300,0) is not.
  - `preset_builds_and_scales`: `robocode_radar(&WorldScale::default())` is `Ok`; with `WorldScale::new(10.0, 1.0)` the instrumented range is 12 000 m.
- [ ] **Step 2:** Run `cargo test --features radar radar_scan`. Expected: FAIL.
- [ ] **Step 3:** Implement. Also add `radar_tracks: Vec::new()` wherever `TurnEvents` is built explicitly, if anywhere (it derives `Default`).
- [ ] **Step 4:** Run `cargo test` and `cargo test --features radar`. Expected: PASS. Re-run the Task 1 Step 6 parity `cmp`. Expected: identical.
- [ ] **Step 5:** Commit: `feat(radar): track report types, unit conversion, building LOS, preset`

---

### Task 3: RadarScan model

**Files:**
- Modify: `src/radar_scan/mod.rs`
- Test: `tests/radar_scan.rs` (`#![cfg(feature = "radar")]`)

**Interfaces:**
- Consumes: Task 2; `radar::Simulation` (`add_radar`, `point_beam`, `command_beam`, `set_enabled`, `update_world`, `step`, `drain`, `tracks`).
- Produces:
  - `pub struct RadarRules { pub identify_robots: bool, pub reveal_energy: bool, pub reveal_true_heading: bool }` (`Default` = all false).
  - `pub struct RadarScanConfig { pub seed: u64, pub scale: WorldScale, pub rules: RadarRules, pub radar: Option<RadarConfig> /* None ⇒ robocode_radar(scale) */ }` with `Default` (seed 0).
  - `pub struct RadarScan` with `pub fn new(cfg: RadarScanConfig) -> Result<RadarScan, radar::RadarError>`, implementing `ScanModel`.
  - The radar config is built once and copied per robot with `RadarConfig::try_clone()`. If it returns `None` (a `Custom` stage), `RadarScan::new` returns `RadarError::InvalidConfig { field: "radar", .. }`.
  - Behaviour per `scan(world, events)`:
    1. **First call:** for every robot, call `add_radar(SensorId(id), PlatformId(id), Pose3::IDENTITY, cfg)` and `point_beam(heading_to_azimuth(robot.last_radar_heading), 0)`. If `identify_robots`, set `identification = Some(IdentificationConfig { reply_probability: 1.0, max_range: instrumented })`.
    2. **Dead robots:** `set_enabled(SensorId(id), false)` once.
    3. **Snapshot:** time = `scale.duration(world.turn)`. Platforms are the living robots (pose = position plus `Rotation3::from_heading(heading_to_azimuth(heading))`, velocity). Targets are the living robots: `extent_radius = scale.to_metres(robot.radius)`, `RadarSignature { rcs_m2: 1.0, swerling: One }`, transponder `Some(TransponderCode(id as u32))` iff `identify_robots`, `platform: Some(PlatformId(id))`.
    4. `update_world`; then `command_beam(SensorId(id), heading_to_azimuth(robot.radar_heading), 0)` for each living robot.
    5. `step(scale.duration(1), &BuildingLineOfSight { .. })`.
    6. **Drain** each living robot's sensor, in id order. For each `TrackUpdated` push a `RadarTrackReport` (converted). For each `TrackDeleted` push a report with status `Deleted` and its last known values. Measurements, `ScanCompleted` and strobes are ignored.
    7. **Extras:** when `reveal_energy` / `reveal_true_heading` is set, find the living robot (other than the scanner) nearest the track position. If it is within `3 × position_sigma + ROBOT_RADIUS`, fill `energy` / `true_heading` from it, else leave them `None`. This is a deliberate game rule that uses ground truth.
    8. `identity = track.identity.map(|c| RobotId from c.0)`.
    9. **Never** push to `events.scans`.

- [ ] **Step 1: Write the failing tests** (`tests/radar_scan.rs`):
  - `realistic_mode_no_scanned_robot_events`: two robots at 400 px apart, both spinning their radar 45°/turn, 100 turns. `events.scans` is always empty, and at least one `Confirmed` report exists for each scanner by turn 100.
  - `fast_sweep_covers_engine_arc` (Review Focus 2): robot A turns body +10, gun +20 and radar +45 each turn. Over 50 turns, `radar::Simulation::sensor_status(SensorId(A)).boresight_azimuth` equals `heading_to_azimuth(A.radar_heading)` within 1e-9 after every turn. (Expose a `pub fn simulation(&self) -> &radar::Simulation` accessor for tests.)
  - `building_blocks_detection`: the urban terrain, with two stationary robots on opposite sides of a building such that the segment between them crosses it. 200 turns of spinning radar → neither robot ever gets a Tentative or Confirmed report.
  - `dead_robot_tracks_deleted` (Review Focus 3): kill robot B at turn 50 (`energy = 0`, so the engine marks it dead). Within 50 more turns A receives a `Deleted` report for its track. No reports have `scanner == B` after turn 50.
  - `overlapping_robots_no_nan` (Review Focus 4): two robots at the same position for 10 turns. Every report field is finite.
  - `same_seed_replays`: two battles with identical spawns and seed → identical `Vec<RadarTrackReport>` per turn (requires `PartialEq` derives).
  - `identify_and_reveal_rules`: with all rules on, confirmed reports carry `identity == Some(other id)` and `energy.is_some()`. With the defaults, both are `None`.
- [ ] **Step 2:** Run `cargo test --features radar --test radar_scan`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run `cargo test --all-features`. Expected: PASS.
- [ ] **Step 5:** Commit: `feat(radar): RadarScan realistic scan model`

---

### Task 4: Robot-facing events, runner wiring, multi-round support

**Files:**
- Modify: `src/api.rs` (event structs, `Event` enum, `Robot` trait), `src/runner.rs` (dispatch near lines 385 and 429; `Battle` factory; `StrategyRobot`)

**Interfaces:**
- Produces:
  - `pub struct RadarTrackEvent { pub track_id: u64, pub status: RadarTrackStatus, pub bearing: f64 /* relative to own heading, (-180,180] */, pub absolute_bearing: f64, pub distance: f64, pub heading_estimate: f64, pub velocity_estimate: f64, pub position_sigma: f64, pub identity: Option<String>, pub energy: Option<f64>, pub true_heading: Option<f64> }`. Bearing and distance are computed by the runner from the scanner's true position to the track's estimated position.
  - `pub struct RadarTrackLostEvent { pub track_id: u64 }`.
  - `Event::RadarTrack(RadarTrackEvent)` and `Event::RadarTrackLost(RadarTrackLostEvent)`.
  - `Robot` trait: `fn on_radar_track(&mut self, _e: &RadarTrackEvent, _s: &Status, _cmd: &mut Commands) {}` and `fn on_radar_track_lost(&mut self, _e: &RadarTrackLostEvent, _s: &Status, _cmd: &mut Commands) {}`. Use the same parameter shape as `on_scanned_robot`.
  - Runner: where scans become pending events (around line 429), also map this robot's `radar_tracks`. Status `Deleted` becomes `RadarTrackLost`; any other status becomes `RadarTrack`. Identity maps to the robot name via `self.names`. Dispatch both variants in the match near line 385.
  - `Battle::set_scan_model_factory(&mut self, f: Box<dyn Fn(u32 /*round*/) -> Box<dyn ScanModel>>)`. It is stored on `Battle` and applied to the new state whenever `BattleState` is (re)built: in `new` once the factory is set (apply immediately to the current state), and at the round rebuild at `runner.rs:877`. Seed convention: callers derive a per-round seed (the example does `seed + round`).
  - `StrategyRobot::on_radar_track`: when status is `Confirmed` or `Coasting`, and the track is nearer than the current one or has the same `track_id` as the held target, convert it to a `ScannedRobotEvent`:
    - `name` is the identity, else `format!("track-{id}")`
    - `bearing`, `absolute_bearing`, `distance` are copied
    - `energy` is the revealed energy, else the last revealed energy for that track id, else `100.0` (classic starting energy; documented convention)
    - `heading` is `heading_estimate`
    - `velocity` is `velocity_estimate`
  - It then reuses the existing target path. `on_radar_track_lost` with the held track's id clears the target.

- [ ] **Step 1: Write the failing tests** (`tests/radar_scan.rs`):
  - `robot_receives_track_events`: a test robot records the events it receives. In a realistic battle it gets `RadarTrack` events and never `ScannedRobot` events.
  - `strategy_robot_targets_tracks`: a sample strategy robot (`robots::circler` or any existing one) in realistic mode fires at least once in 300 turns. This shows `TargetInfo` is fed from tracks.
  - `multi_round_uses_factory` (Review Focus 1): with `run_rounds(3, ..)` and the factory set, every round produces radar tracks and no scans.
  - `classic_robot_api_unchanged`: existing `tests/api.rs` and `tests/robots.rs` pass untouched.
- [ ] **Step 2:** Run `cargo test --features radar`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run `cargo test --all-features` and the parity `cmp`. Expected: PASS and identical.
- [ ] **Step 5:** Commit: `feat(radar): robot track events, runner dispatch, per-round scan models`

---

### Task 5: Snapshot key, example, docs

**Files:**
- Modify: `src/physics/snapshot.rs`, `README.md`, `CHANGELOG.md`
- Create: `examples/radar_battle.rs` (with `required-features = ["radar"]` in `Cargo.toml`)

**Interfaces:**
- Produces: `TurnSnapshot` JSON gets an `"radar_tracks": [...]` key inside `events`, written **only when non-empty** (Review Focus 5). Each entry holds `scanner, track_id, status, x, y, vx, vy, heading, speed, sigma` plus `identity`/`energy`/`true_heading` only when Some. Use the existing `append_f64_field` helpers.

- [ ] **Step 1: Write the failing tests:**
  - `classic_snapshot_has_no_radar_key`: a classic battle's snapshot JSON contains no `radar_tracks`.
  - `radar_snapshot_has_key` (feature radar): a realistic battle with confirmed tracks has a snapshot containing `"radar_tracks"`.
- [ ] **Step 2:** Run. Expected: FAIL.
- [ ] **Step 3:** Implement. Write `examples/radar_battle.rs`: the urban map, two sample robots, `RadarScanConfig { seed: 1, ..Default::default() }` via `set_scan_model_factory(move |round| ... seed + round as u64)`, run 1 000 turns, and print per-turn confirmed-track counts and the outcome.
- [ ] **Step 4:** Update the README with a "Realistic radar (feature `radar`)" section covering what changes for robots (anonymous tracks, `RadarRules`, `WorldScale`) and the conventions. Add a CHANGELOG entry.
- [ ] **Step 5:** Run `cargo fmt --check && cargo clippy --all-targets --all-features -- -D warnings && cargo test --all-features && cargo run --features radar --example radar_battle` and the parity `cmp`. Expected: all PASS, the example runs, parity is identical.
- [ ] **Step 6:** Commit: `feat(radar): snapshot radar_tracks key, example and docs`
