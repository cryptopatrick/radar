//! Two independent radars, with different update periods, on two platforms.
use radar::prelude::*;

fn main() -> Result<(), RadarError> {
    let mut sim = Simulation::new(SimulationConfig::default().seed(Seed(7)));
    let mut world = WorldSnapshot::new(SimTime::ZERO);
    for (id, x, period_ms) in [(1, 0.0, 100), (2, 40_000.0, 1_000)] {
        let mut cfg = RadarConfig::surveillance_2d()?;
        cfg.update_period = SimDuration::from_millis(period_ms);
        sim.add_radar(SensorId(id), PlatformId(id), Pose3::IDENTITY, cfg)?;
        world.platforms.push(PlatformState {
            id: PlatformId(id),
            pose: Pose3::from_position_heading(Vector3::new(x, 0.0, 0.0), 0.0),
            velocity: Vector3::ZERO,
        });
    }
    world.targets.push(TargetState {
        id: TargetId(1),
        pose: Pose3::from_position_heading(Vector3::new(20_000.0, 25_000.0, 0.0), 0.0),
        velocity: Vector3::new(0.0, -100.0, 0.0),
        extent_radius: 15.0,
        signature: RadarSignature {
            rcs_m2: 5.0,
            swerling: Swerling::One,
        },
        transponder: None,
        platform: None,
    });
    sim.update_world(world)?;
    sim.step_open(SimDuration::from_secs(60))?;
    for id in [SensorId(1), SensorId(2)] {
        let mut out = Vec::new();
        sim.drain(id, &mut out)?;
        out.retain(|o| matches!(o, Observation::Measurement(_)));
        let tracks = sim.tracks(id)?.len();
        println!("{id:?}: {} measurements, {tracks} tracks", out.len());
    }
    Ok(())
}
