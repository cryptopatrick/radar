//! A 2D surveillance radar tracking an aircraft; prints the tracks every scan.
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
