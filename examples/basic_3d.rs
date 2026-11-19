//! A 3D volume-scan radar measuring range, bearing and elevation of a jet.
use radar::prelude::*;

fn main() -> Result<(), RadarError> {
    let (radar, site) = (SensorId(1), PlatformId(1));
    let mut sim = Simulation::new(SimulationConfig::default());
    let cfg = RadarConfig::surveillance_3d()?;
    sim.add_radar(radar, site, Pose3::IDENTITY, cfg)?;
    let mut world = WorldSnapshot::new(SimTime::ZERO);
    world.platforms.push(PlatformState {
        id: site,
        pose: Pose3::IDENTITY,
        velocity: Vector3::ZERO,
    });
    world.targets.push(TargetState {
        id: TargetId(1),
        pose: Pose3::from_position_heading(Vector3::new(20_000.0, 30_000.0, 2_000.0), 0.0),
        velocity: Vector3::new(0.0, -200.0, 0.0),
        extent_radius: 15.0,
        signature: RadarSignature {
            rcs_m2: 5.0,
            swerling: Swerling::One,
        },
        transponder: None,
        platform: None,
    });
    sim.update_world(world)?;
    sim.step_open(SimDuration::from_secs(40))?; // five volumes of four 2 s bars
    let mut out = Vec::new();
    sim.drain(radar, &mut out)?;
    for o in &out {
        if let Observation::Measurement(m) = o {
            let el = m.elevation.unwrap_or(0.0).to_degrees();
            let (t, az, r) = (m.time.as_secs_f64(), m.azimuth.to_degrees(), m.range);
            println!("t = {t:4.1} s  range {r:7.0} m  az {az:5.1}°  el {el:4.1}°");
        }
    }
    Ok(())
}
