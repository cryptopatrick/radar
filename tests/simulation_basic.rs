//! Integration tests for the simulation, scheduling and radar pipeline.

use radar::prelude::*;
use radar::radar::IdentificationConfig;
use radar::sensor::{SensorContext, SensorModel};
use std::f64::consts::FRAC_PI_2;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

const RADAR: SensorId = SensorId(1);
const OWN: PlatformId = PlatformId(7);

fn platform_at(pos: Vector3, heading: f64) -> PlatformState {
    PlatformState {
        id: OWN,
        pose: Pose3::from_position_heading(pos, heading),
        velocity: Vector3::ZERO,
    }
}

fn target(id: u64, pos: Vector3) -> TargetState {
    TargetState {
        id: TargetId(id),
        pose: Pose3::from_position_heading(pos, 0.0),
        velocity: Vector3::ZERO,
        extent_radius: 1.0,
        signature: RadarSignature {
            rcs_m2: 1.0,
            swerling: Swerling::Zero,
        },
        transponder: None,
        platform: None,
    }
}

fn world(platforms: Vec<PlatformState>, targets: Vec<TargetState>) -> WorldSnapshot {
    let mut w = WorldSnapshot::new(SimTime::ZERO);
    w.platforms = platforms;
    w.targets = targets;
    w
}

/// A `simple_2d(1200)` radar on a platform at the origin, given the world.
fn sim_with(cfg: RadarConfig, w: WorldSnapshot) -> Simulation {
    let mut sim = Simulation::new(SimulationConfig::default());
    sim.add_radar(RADAR, OWN, Pose3::IDENTITY, cfg).unwrap();
    sim.update_world(w).unwrap();
    sim
}

/// Runs one second in 100 ms steps and drains the radar.
fn run_one_second(sim: &mut Simulation, env: &dyn Environment) -> Vec<Observation> {
    for _ in 0..10 {
        sim.step(SimDuration::from_millis(100), env).unwrap();
    }
    let mut out = Vec::new();
    sim.drain(RADAR, &mut out).unwrap();
    out
}

fn measurements(obs: &[Observation]) -> Vec<&Measurement> {
    obs.iter()
        .filter_map(|o| match o {
            Observation::Measurement(m) => Some(m),
            _ => None,
        })
        .collect()
}

fn scans_completed(obs: &[Observation]) -> usize {
    obs.iter()
        .filter(|o| matches!(o, Observation::ScanCompleted { .. }))
        .count()
}

#[test]
fn simple_2d_detects_target_on_sweep() {
    let w = world(
        vec![platform_at(Vector3::ZERO, 0.0)],
        vec![target(1, Vector3::new(500.0, 0.0, 0.0))],
    );
    let mut sim = sim_with(RadarConfig::simple_2d(1200.0).unwrap(), w);
    let obs = run_one_second(&mut sim, &OpenSky);
    let ms = measurements(&obs);
    assert_eq!(ms.len(), 1, "{obs:?}");
    assert!((ms[0].range - 500.0).abs() < 1e-9);
    assert!((ms[0].azimuth - FRAC_PI_2).abs() < 1e-9);
    assert_eq!(ms[0].elevation, None);
    assert_eq!(ms[0].identity, None);
    assert_eq!(ms[0].sensor, RADAR);
    assert_eq!(scans_completed(&obs), 1);
}

#[test]
fn out_of_range_not_detected() {
    let w = world(
        vec![platform_at(Vector3::ZERO, 0.0)],
        vec![target(1, Vector3::new(0.0, 1500.0, 0.0))],
    );
    let mut sim = sim_with(RadarConfig::simple_2d(1200.0).unwrap(), w);
    assert!(measurements(&run_one_second(&mut sim, &OpenSky)).is_empty());
}

#[test]
fn own_platform_excluded() {
    let mut own = target(1, Vector3::ZERO);
    own.platform = Some(OWN);
    let w = world(vec![platform_at(Vector3::ZERO, 0.0)], vec![own]);
    let mut sim = sim_with(RadarConfig::simple_2d(1200.0).unwrap(), w);
    assert!(measurements(&run_one_second(&mut sim, &OpenSky)).is_empty());
}

struct Wall;
impl Environment for Wall {
    fn line_of_sight_blocked(&self, _from: Vector3, _to: Vector3) -> bool {
        true
    }
}

#[test]
fn occluded_not_detected() {
    let w = world(
        vec![platform_at(Vector3::ZERO, 0.0)],
        vec![target(1, Vector3::new(500.0, 0.0, 0.0))],
    );
    let mut sim = sim_with(RadarConfig::simple_2d(1200.0).unwrap(), w);
    assert!(measurements(&run_one_second(&mut sim, &Wall)).is_empty());
}

#[test]
fn missing_platform_skips() {
    let w = world(vec![], vec![target(1, Vector3::new(500.0, 0.0, 0.0))]);
    let mut sim = sim_with(RadarConfig::simple_2d(1200.0).unwrap(), w);
    assert!(run_one_second(&mut sim, &OpenSky).is_empty());
    assert_eq!(sim.now(), SimTime::ZERO + SimDuration::from_secs(1));
}

#[test]
fn time_went_backwards() {
    let mut sim = Simulation::new(SimulationConfig::default());
    let t2 = SimTime::ZERO + SimDuration::from_secs(2);
    let t1 = SimTime::ZERO + SimDuration::from_secs(1);
    sim.update_world(WorldSnapshot::new(t2)).unwrap();
    assert!(matches!(
        sim.update_world(WorldSnapshot::new(t1)),
        Err(RadarError::TimeWentBackwards { .. })
    ));
    let before = sim.now();
    assert_eq!(sim.step(SimDuration::ZERO, &OpenSky), Ok(()));
    assert_eq!(sim.now(), before);
}

#[derive(Debug)]
struct Counter(Arc<AtomicUsize>);
impl SensorModel for Counter {
    fn update(
        &mut self,
        _ctx: &mut SensorContext<'_>,
        _out: &mut Vec<Observation>,
    ) -> Result<(), RadarError> {
        self.0.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
}

fn counts_for(policy: StepPolicy) -> (usize, usize) {
    let mut sim = Simulation::new(SimulationConfig::default().step_policy(policy));
    let a = Arc::new(AtomicUsize::new(0));
    let b = Arc::new(AtomicUsize::new(0));
    sim.add_sensor(
        SensorId(1),
        OWN,
        Pose3::IDENTITY,
        SimDuration::from_millis(100),
        Box::new(Counter(a.clone())),
    )
    .unwrap();
    sim.add_sensor(
        SensorId(2),
        OWN,
        Pose3::IDENTITY,
        SimDuration::from_millis(250),
        Box::new(Counter(b.clone())),
    )
    .unwrap();
    sim.update_world(world(vec![platform_at(Vector3::ZERO, 0.0)], vec![]))
        .unwrap();
    sim.step_open(SimDuration::from_secs(1)).unwrap();
    (a.load(Ordering::SeqCst), b.load(Ordering::SeqCst))
}

#[test]
fn scheduling_periods() {
    assert_eq!(counts_for(StepPolicy::SingleStep), (10, 4));
    assert_eq!(
        counts_for(StepPolicy::Subdivide {
            max_step: SimDuration::from_millis(50)
        }),
        (10, 4)
    );
}

#[test]
fn duplicate_sensor_rejected() {
    let mut sim = Simulation::new(SimulationConfig::default());
    let cfg = || RadarConfig::simple_2d(1200.0).unwrap();
    sim.add_radar(RADAR, OWN, Pose3::IDENTITY, cfg()).unwrap();
    assert_eq!(
        sim.add_radar(RADAR, OWN, Pose3::IDENTITY, cfg()),
        Err(RadarError::DuplicateSensor(RADAR))
    );
}

#[test]
fn unknown_sensor_errors() {
    let mut sim = Simulation::new(SimulationConfig::default());
    let unknown = Err(RadarError::UnknownSensor(SensorId(9)));
    let mut out = Vec::new();
    assert_eq!(sim.drain(SensorId(9), &mut out), unknown);
    assert_eq!(sim.remove_sensor(SensorId(9)), unknown);
    assert_eq!(sim.set_enabled(SensorId(9), false), unknown);
    assert_eq!(sim.point_beam(SensorId(9), 0.0, 0.0), unknown);
    assert_eq!(sim.command_beam(SensorId(9), 0.0, 0.0), unknown);
    assert_eq!(
        sim.sensor_status(SensorId(9)),
        Err(RadarError::UnknownSensor(SensorId(9)))
    );
}

#[test]
fn steered_requires_steered_scan() {
    let mut sim = Simulation::new(SimulationConfig::default());
    sim.add_radar(
        RADAR,
        OWN,
        Pose3::IDENTITY,
        RadarConfig::simple_2d(1200.0).unwrap(),
    )
    .unwrap();
    assert!(matches!(
        sim.command_beam(RADAR, 1.0, 0.0),
        Err(RadarError::InvalidConfig { field: "scan", .. })
    ));
}

#[test]
fn point_beam_sets_immediately() {
    let mut sim = Simulation::new(SimulationConfig::default());
    sim.add_radar(
        RADAR,
        OWN,
        Pose3::IDENTITY,
        RadarConfig::simple_2d(1200.0).unwrap(),
    )
    .unwrap();
    sim.point_beam(RADAR, 1.0, 0.0).unwrap();
    let status = sim.sensor_status(RADAR).unwrap();
    assert!((status.boresight_azimuth - 1.0).abs() < 1e-12);
    assert!(status.enabled);
    assert_eq!(
        status.next_due,
        SimTime::ZERO + SimDuration::from_millis(100)
    );
}

#[test]
fn platform_rotation_reports_world_azimuth() {
    // Nose east: the north target sits at scan azimuth 270° but is
    // reported at world azimuth 0.
    let w = world(
        vec![platform_at(Vector3::ZERO, FRAC_PI_2)],
        vec![target(1, Vector3::new(0.0, 500.0, 0.0))],
    );
    let mut sim = sim_with(RadarConfig::simple_2d(1200.0).unwrap(), w);
    let obs = run_one_second(&mut sim, &OpenSky);
    let ms = measurements(&obs);
    assert_eq!(ms.len(), 1, "{obs:?}");
    let az = ms[0].azimuth;
    assert!(az < 1e-9 || (std::f64::consts::TAU - az) < 1e-9, "{az}");
}

#[test]
fn doppler_and_identification() {
    let mut t = target(1, Vector3::new(500.0, 0.0, 0.0));
    t.velocity = Vector3::new(10.0, 0.0, 0.0);
    t.transponder = Some(TransponderCode(7700));
    let w = world(vec![platform_at(Vector3::ZERO, 0.0)], vec![t]);
    let cfg = RadarConfig::builder()
        .detection(DetectionConfig::Radius {
            range: 1200.0,
            pd: 1.0,
            false_alarms_per_scan: 0.0,
        })
        .limits(RangeLimits {
            min_range: 0.0,
            instrumented_range: 1200.0,
        })
        .doppler(true)
        .identification(Some(IdentificationConfig {
            reply_probability: 1.0,
            max_range: 1000.0,
        }))
        .build()
        .unwrap();
    let mut sim = sim_with(cfg, w);
    let obs = run_one_second(&mut sim, &OpenSky);
    let ms = measurements(&obs);
    assert_eq!(ms.len(), 1, "{obs:?}");
    assert!((ms[0].radial_velocity.unwrap() - 10.0).abs() < 1e-9);
    assert_eq!(ms[0].identity, Some(TransponderCode(7700)));
}

#[test]
fn reset_replays_identically() {
    let cfg = RadarConfig::surveillance_2d().unwrap();
    let w = world(
        vec![platform_at(Vector3::ZERO, 0.0)],
        vec![target(1, Vector3::new(20_000.0, 0.0, 0.0))],
    );
    let mut sim = sim_with(cfg, w);
    let run = |sim: &mut Simulation| {
        let mut out = Vec::new();
        for _ in 0..40 {
            sim.step_open(SimDuration::from_millis(100)).unwrap();
        }
        sim.drain(RADAR, &mut out).unwrap();
        out
    };
    let first = run(&mut sim);
    sim.reset();
    assert_eq!(sim.now(), SimTime::ZERO);
    let second = run(&mut sim);
    assert!(!measurements(&first).is_empty());
    assert_eq!(first, second);
}

#[test]
fn three_d_reports_world_elevation() {
    let el = 100.0_f64.atan2(500.0);
    let cfg = RadarConfig::builder()
        .dimension(Dimension::ThreeD)
        .scan(ScanConfig::Fixed {
            azimuth: FRAC_PI_2,
            elevation: el,
        })
        .limits(RangeLimits {
            min_range: 0.0,
            instrumented_range: 1000.0,
        })
        .build()
        .unwrap();
    let w = world(
        vec![platform_at(Vector3::ZERO, 0.0)],
        vec![target(1, Vector3::new(500.0, 0.0, 100.0))],
    );
    let mut sim = sim_with(cfg, w);
    sim.step_open(SimDuration::from_millis(100)).unwrap();
    let mut obs = Vec::new();
    sim.drain(RADAR, &mut obs).unwrap();
    let ms = measurements(&obs);
    assert_eq!(ms.len(), 1, "{obs:?}");
    assert!((ms[0].range - 260_000.0_f64.sqrt()).abs() < 1e-9);
    assert!((ms[0].azimuth - FRAC_PI_2).abs() < 1e-9);
    assert!((ms[0].elevation.unwrap() - el).abs() < 1e-9);
}
