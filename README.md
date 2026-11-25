# radar
Realistic radar and sensor simulation for games and simulations.

`radar` simulates what a radar operator would actually see: a scanning beam,
probabilistic detection from the radar range equation, noisy measurements,
false alarms, tracks that form, coast and drop, transponder identification and
noise jamming. The host owns the world and the clock; the library never sees
your game objects and never hands back ground truth. Every run is
reproducible from a seed.

## Quick start

```rust
use radar::prelude::*;

fn main() -> Result<(), RadarError> {
    let (radar, site) = (SensorId(1), PlatformId(1));
    let mut sim = Simulation::new(SimulationConfig::default());
    let cfg = RadarConfig::surveillance_2d()?;
    sim.add_radar(radar, site, Pose3::IDENTITY, cfg)?;
    let mut world = WorldSnapshot::new(SimTime::ZERO);
    world.platforms.push(PlatformState {
        id: site,
        pose: Pose3::IDENTITY,
        velocity: Vector3::ZERO,
    });
    world.targets.push(TargetState {
        id: TargetId(1),
        pose: Pose3::from_position_heading(Vector3::new(0.0, 30_000.0, 0.0), 0.0),
        velocity: Vector3::new(150.0, 0.0, 0.0),
        extent_radius: 15.0,
        signature: RadarSignature {
            rcs_m2: 5.0,
            swerling: Swerling::One,
        },
        transponder: None,
        platform: None,
    });
    sim.update_world(world)?;
    for _ in 0..10 {
        // One 4 s antenna revolution: 40 radar updates.
        sim.step_open(SimDuration::from_secs(4))?;
        sim.drain(radar, &mut Vec::new())?;
        println!("t = {:.0} s", sim.now().as_secs_f64());
        for t in sim.tracks(radar)? {
            let (id, status, course) = (t.id, t.status, t.course().to_degrees());
            println!("  {id:?} {status:?} course {course:.0}°");
        }
    }
    Ok(())
}
```

This is `examples/basic_2d.rs` (`cargo run --example basic_2d`). See also
`basic_3d` (volume scan with elevation) and `multi_sensor` (two independent
radars). Each sensor's output arrives as `Observation`s: `Measurement`,
`ScanCompleted`, `TrackUpdated`, `TrackDeleted` and `JammingStrobe`.

## Feature flags

| Feature       | Default | What it adds                                                        |
|---------------|---------|---------------------------------------------------------------------|
| `tracking`    | yes     | Association, alpha-beta and Kalman filters, M-of-N track lifecycle |
| `ew`          | yes     | Noise jammers (J/N in the SNR) and jamming strobes                  |
| `serde`       | no      | `Serialize`/`Deserialize` on configs and observations               |
| `entropy`     | no      | `Seed::from_entropy()` from the operating system                    |
| `diagnostics` | no      | A truth-annotated debug channel (`Simulation::drain_diagnostics`)   |

The core (geometry, time, world, sensors, radar, simulation) is always on.

## Conventions

- **Frame:** right-handed world frame, **x = east, y = north, z = up**.
- **Azimuth** is a compass bearing: from +y (north), clockwise, in `[0, 2π)`.
  **Elevation** is up from the horizontal plane, in `[−π/2, π/2]`.
- **Units:** SI everywhere — metres, seconds, radians, hertz, watts, square
  metres. Time is integer nanoseconds (`SimTime`, `SimDuration`), so there is
  no float drift.
- **Poses:** a sensor's mount is relative to its platform; scan angles are
  platform-relative by default or world-stabilised (`ScanFrame`).
- **2D** is `z = 0` with elevation ignored (`Dimension::TwoD`).
- **`WorldScale`** converts host units and ticks (positions, speeds,
  durations, angles per tick) to SI at the boundary; the core never sees host
  units.
- Between world snapshots, platform, target and emitter positions are
  extrapolated linearly from their velocities.

## Determinism

- Each sensor draws from its own ChaCha8 stream derived from the master seed
  and its `SensorId`, in a fixed order (targets by ascending `TargetId`, then
  false alarms, then identification replies, then jamming strobes, then
  tracking). Samplers are
  implemented in-crate, so dependency updates do not change streams.
- The same seed and the same inputs give an identical observation stream.
  Adding, removing or disabling one sensor never changes another sensor's
  output.
- `Simulation::reset()` restores time zero, scan states, tracks and random
  streams; it also empties the world, so feed it again with `update_world`
  before replaying.
- **Limits:** bit-exact replay holds for the same binary on the same target
  platform. Transcendental functions (`sin`, `cos`, `ln`, `exp`) may differ
  across platforms and libm versions, so cross-platform bit-exactness is not
  promised. A custom `random_source_factory` (for example live hardware
  noise) makes runs non-deterministic, and custom sensor models or scan
  patterns keep their own state across `reset()`.

## Passive (bistatic) radar

The `bistatic` module computes link budgets for a passive radar: a receiver
with no transmitter of its own that detects echoes of someone else's signal,
such as a Starlink, GPS, Galileo or DVB-S2 satellite.

```rust
use radar::bistatic::{BistaticGeometry, Illuminator, PassiveReceiver};
use radar::geometry::Vector3;

let starlink = Illuminator::starlink();
let receiver = PassiveReceiver::default(); // 40 dBi, 0.5 s integration
let geometry = BistaticGeometry {
    transmitter: Vector3::new(0.0, 0.0, starlink.altitude_m),
    receiver: Vector3::ZERO,
    target: Vector3::new(0.0, 10_000.0, 0.0),
};
let snr_db = receiver.snr_db_at(&starlink, 40.0, &geometry);
```

The illuminator presets and the receiver defaults follow P. Gomez-del-Hoyo,
K. Gronowski and P. Samczynski, "The STARLINK-based passive radar:
preliminary study and first illuminator signal measurements", IRS 2022.

## Model limitations

- **Bistatic is analytic only:** the `bistatic` module evaluates link
  budgets for a given geometry. `Simulation` radars stay monostatic, so a
  passive radar cannot yet be stepped, tracked or jammed there.

- **2D radars report slant range:** a `Dimension::TwoD` radar measures the
  full 3D slant range but no elevation, so placing its plots in the
  horizontal plane biases the ground range outward for targets at altitude.
- **Single-look detection:** each look is an independent square-law decision
  on one pulse; there is no pulse integration (coherent or non-coherent).
- **No clutter, multipath, atmosphere or refraction:** the only propagation
  effect is host-supplied line-of-sight occlusion (`Environment`).
- **Alpha-beta covariance is approximate:** the alpha-beta filter reports a
  covariance for gating, but it is not a statistically consistent estimate.
- **Per-sensor tracking without fusion:** each radar runs its own tracker;
  there is no multi-sensor track fusion.
- **Plot-per-dwell association:** a sweeping beam yields at most one
  track-updating measurement per target per dwell; extra looks in the same
  dwell are absorbed rather than used.
- **Tentative age-out:** tentative tracks are deleted after
  `(confirm_n + 1) ×` the nominal revisit. A steered beam that revisits more
  slowly than its slew limit allows needs `tentative_max_age` set
  explicitly.
- **Confident-coverage miss counting:** a pass of the beam counts as a
  miss only if the track's predicted direction was inside the beam by the
  track's own angular uncertainty (capped at half the half-beamwidth). A
  track that is only grazed at a beam edge gets no outcome, so an uncertain
  track near an elevation-bar or sector edge is not driven into coasting,
  at the cost of detecting real fades there more slowly.

## Performance

`cargo bench` (`benches/radar_2d.rs`): one `step(100 ms)` of a
`surveillance_2d` radar against 1 000 targets spread uniformly within 90 km
takes about **430 µs** (≈ 2 300 updates/s, against a target of 60) on an
Apple M2 with rustc 1.99, release profile.

## License
This project is licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only). See [LICENSE](LICENSE) for details.
