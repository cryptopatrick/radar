//! Integration tests for noise jamming and jamming strobes (feature `ew`).
#![cfg(feature = "ew")]

use radar::prelude::*;
use radar::units::deg;
use std::f64::consts::{FRAC_PI_2, TAU};

const RADAR: SensorId = SensorId(1);
const OWN: PlatformId = PlatformId(7);
const TARGET_PLATFORM: PlatformId = PlatformId(20);
const JAMMER_PLATFORM: PlatformId = PlatformId(30);
/// Target range, also the calibration point (Pd 0.9 at 1 m²).
const R_TARGET: f64 = 10_000.0;

fn band() -> Band {
    Band {
        centre_hz: 10e9,
        bandwidth_hz: 10e6,
    }
}

/// A 2D range-equation radar rotating once per update (1 s), Pd 0.9 on a
/// 1 m² Swerling 1 target at 10 km. A 1 W jammer at 10 km on boresight
/// gives J/N 20 dB.
fn config(dim: Dimension, receiver_band: Option<Band>) -> RadarConfig {
    let calibration =
        Calibration::reference_for_pd(1.0, R_TARGET, 0.9, 1e-6, Swerling::One).unwrap();
    RadarConfig::builder()
        .dimension(dim)
        .update_period(SimDuration::from_secs(1))
        .scan(ScanConfig::Circular {
            rate: TAU,
            elevation: 0.0,
        })
        .detection(DetectionConfig::RangeEquation(RangeEquationConfig {
            calibration,
            pfa: 1e-6,
            range_resolution: 100.0,
        }))
        .limits(RangeLimits {
            min_range: 0.0,
            instrumented_range: 30_000.0,
        })
        .receiver_band(receiver_band)
        .jamming_reference(Some(JammingReference {
            erp_w: 1.0,
            range_m: R_TARGET,
            jnr_db: 20.0,
        }))
        .build()
        .unwrap()
}

fn target_east() -> TargetState {
    TargetState {
        id: TargetId(1),
        pose: Pose3::from_position_heading(Vector3::new(R_TARGET, 0.0, 0.0), 0.0),
        velocity: Vector3::ZERO,
        extent_radius: 1.0,
        signature: RadarSignature {
            rcs_m2: 1.0,
            swerling: Swerling::One,
        },
        transponder: None,
        platform: Some(TARGET_PLATFORM),
    }
}

fn jammer(platform: PlatformId, position: Vector3) -> EmitterState {
    EmitterState {
        id: EmitterId(1),
        platform: Some(platform),
        position,
        velocity: Vector3::ZERO,
        band: band(),
        erp_w: 1.0,
        kind: EmitterKind::NoiseJammer,
    }
}

fn world(targets: Vec<TargetState>, emitters: Vec<EmitterState>) -> WorldSnapshot {
    let mut w = WorldSnapshot::new(SimTime::ZERO);
    w.platforms = vec![PlatformState {
        id: OWN,
        pose: Pose3::IDENTITY,
        velocity: Vector3::ZERO,
    }];
    w.targets = targets;
    w.emitters = emitters;
    w
}

/// Runs `revolutions` one-second updates and returns every observation.
fn run(
    cfg: RadarConfig,
    w: WorldSnapshot,
    env: &dyn Environment,
    revolutions: u64,
) -> Vec<Observation> {
    let mut sim = Simulation::new(SimulationConfig::default().seed(Seed(42)));
    sim.add_radar(RADAR, OWN, Pose3::IDENTITY, cfg).unwrap();
    sim.update_world(w).unwrap();
    for _ in 0..revolutions {
        sim.step(SimDuration::from_secs(1), env).unwrap();
    }
    let mut out = Vec::new();
    sim.drain(RADAR, &mut out).unwrap();
    out
}

/// Fraction of revolutions with a measurement at the target (10 km due east).
fn target_rate(obs: &[Observation], revolutions: u64) -> f64 {
    let hits = obs
        .iter()
        .filter(|o| match o {
            Observation::Measurement(m) => {
                (m.range - R_TARGET).abs() < 300.0 && (m.azimuth - FRAC_PI_2).abs() < deg(3.0)
            }
            _ => false,
        })
        .count();
    hits as f64 / revolutions as f64
}

/// (azimuth, elevation, J/N dB) of every strobe.
fn strobes(obs: &[Observation]) -> Vec<(f64, Option<f64>, f64)> {
    obs.iter()
        .filter_map(|o| match o {
            Observation::JammingStrobe {
                azimuth,
                elevation,
                j_over_n_db,
                ..
            } => Some((*azimuth, *elevation, *j_over_n_db)),
            _ => None,
        })
        .collect()
}

#[test]
fn self_protect_jammer_reduces_pd() {
    let n = 2_000;
    let rx = Some(band());
    let clean = run(
        config(Dimension::TwoD, rx),
        world(vec![target_east()], vec![]),
        &OpenSky,
        n,
    );
    let jammed = run(
        config(Dimension::TwoD, rx),
        world(
            vec![target_east()],
            vec![jammer(TARGET_PLATFORM, Vector3::new(R_TARGET, 0.0, 0.0))],
        ),
        &OpenSky,
        n,
    );
    let (p_clean, p_jammed) = (target_rate(&clean, n), target_rate(&jammed, n));
    assert!(p_clean > 0.85, "clean rate {p_clean}");
    assert!(
        p_jammed < 0.5 * p_clean,
        "jammed {p_jammed} vs clean {p_clean}"
    );
    // The self-protection jammer also shows up as strobes on the target bearing.
    assert!(!strobes(&jammed).is_empty());
}

#[test]
fn offaxis_jammer_weaker_than_mainbeam() {
    let n = 1_000;
    let rx = Some(band());
    // Same bearing as the target, farther out (J/N ≈ 16.5 dB in the main beam).
    let mainbeam = run(
        config(Dimension::TwoD, rx),
        world(
            vec![target_east()],
            vec![jammer(JAMMER_PLATFORM, Vector3::new(15_000.0, 0.0, 0.0))],
        ),
        &OpenSky,
        n,
    );
    // Due north, 90° off the target: only the −30 dB sidelobe sees it.
    let offaxis = run(
        config(Dimension::TwoD, rx),
        world(
            vec![target_east()],
            vec![jammer(JAMMER_PLATFORM, Vector3::new(0.0, 15_000.0, 0.0))],
        ),
        &OpenSky,
        n,
    );
    let (p_main, p_off) = (target_rate(&mainbeam, n), target_rate(&offaxis, n));
    assert!(p_off > 0.85, "off-axis rate {p_off}");
    assert!(p_main < 0.5 * p_off, "main {p_main} vs off-axis {p_off}");
}

#[test]
fn strobe_emitted_bearing_only() {
    let n = 200;
    let pos = Vector3::new(15_000.0, 0.0, 0.0);
    let obs = run(
        config(Dimension::TwoD, Some(band())),
        world(vec![], vec![jammer(JAMMER_PLATFORM, pos)]),
        &OpenSky,
        n,
    );
    let s = strobes(&obs);
    assert_eq!(s.len() as u64, n, "one strobe per revolution");
    // J/N = 100·(10/15)² on boresight.
    let jnr: f64 = 100.0 * (10.0_f64 / 15.0).powi(2);
    let sigma = deg(3.0) / (1.6 * (2.0 * jnr).sqrt());
    for &(az, el, jn_db) in &s {
        assert!((az - FRAC_PI_2).abs() < 3.0 * sigma, "az {az}, σ {sigma}");
        assert_eq!(el, None);
        assert!((jn_db - 10.0 * jnr.log10()).abs() < 1e-9, "{jn_db}");
    }
    // Bearing only: nothing is ranged at the jammer's position.
    let ranged = obs.iter().any(|o| match o {
        Observation::Measurement(m) => {
            (m.range - 15_000.0).abs() < 300.0 && (m.azimuth - FRAC_PI_2).abs() < deg(2.0)
        }
        _ => false,
    });
    assert!(!ranged, "{obs:?}");
    // Only measurements, strobes and scan completions are reported.
    for o in &obs {
        assert!(matches!(
            o,
            Observation::JammingStrobe { sensor: RADAR, .. }
                | Observation::Measurement(_)
                | Observation::ScanCompleted { .. }
        ));
    }
}

#[test]
fn strobe_has_elevation_in_3d() {
    let obs = run(
        config(Dimension::ThreeD, Some(band())),
        world(
            vec![],
            vec![jammer(JAMMER_PLATFORM, Vector3::new(15_000.0, 0.0, 0.0))],
        ),
        &OpenSky,
        5,
    );
    let s = strobes(&obs);
    assert_eq!(s.len(), 5);
    for (_, el, _) in s {
        let el = el.expect("3D strobe has elevation");
        assert!(el.abs() < deg(1.0), "{el}");
    }
}

#[test]
fn no_receiver_band_immune() {
    let n = 300;
    let clean = run(
        config(Dimension::TwoD, None),
        world(vec![target_east()], vec![]),
        &OpenSky,
        n,
    );
    let jammed = run(
        config(Dimension::TwoD, None),
        world(
            vec![target_east()],
            vec![jammer(TARGET_PLATFORM, Vector3::new(R_TARGET, 0.0, 0.0))],
        ),
        &OpenSky,
        n,
    );
    assert!(strobes(&jammed).is_empty());
    assert_eq!(clean, jammed);
}

/// Blocks every line of sight to the jammer position.
struct JammerShadow(Vector3);
impl Environment for JammerShadow {
    fn line_of_sight_blocked(&self, _from: Vector3, to: Vector3) -> bool {
        to == self.0
    }
}

#[test]
fn blocked_jammer_no_effect() {
    let n = 300;
    let pos = Vector3::new(15_000.0, 0.0, 0.0);
    let env = JammerShadow(pos);
    let clean = run(
        config(Dimension::TwoD, Some(band())),
        world(vec![target_east()], vec![]),
        &env,
        n,
    );
    let jammed = run(
        config(Dimension::TwoD, Some(band())),
        world(vec![target_east()], vec![jammer(JAMMER_PLATFORM, pos)]),
        &env,
        n,
    );
    assert!(strobes(&jammed).is_empty());
    assert_eq!(clean, jammed);
}

#[test]
fn moving_jammer_strobe_tracks_extrapolated_position() {
    // A self-screening jammer flying north at 300 m/s; the world is given
    // once, at t = 0, so every later update must extrapolate it.
    let velocity = Vector3::new(0.0, 300.0, 0.0);
    let mut target = target_east();
    target.velocity = velocity;
    let mut j = jammer(TARGET_PLATFORM, Vector3::new(R_TARGET, 0.0, 0.0));
    j.velocity = velocity;
    let obs = run(
        config(Dimension::TwoD, Some(band())),
        world(vec![target], vec![j]),
        &OpenSky,
        20,
    );
    let mut seen = 0;
    for o in &obs {
        if let Observation::JammingStrobe { time, azimuth, .. } = o {
            let north = 300.0 * time.as_secs_f64();
            let expected = R_TARGET.atan2(north);
            assert!(
                (azimuth - expected).abs() < deg(1.0),
                "t {time:?}: azimuth {azimuth}, expected {expected}"
            );
            seen += 1;
        }
    }
    assert_eq!(seen, 20, "one strobe per revolution");
}
