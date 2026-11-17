//! The world snapshot supplied to the simulation each step.

use crate::error::RadarError;
use crate::geometry::{Pose3, Vector3};
use crate::ids::{EmitterId, PlatformId, TargetId, TransponderCode};
use crate::time::SimTime;
use std::collections::HashSet;
use std::hash::Hash;

/// Fluctuation model for a target's radar cross-section.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Swerling {
    /// Non-fluctuating.
    Zero,
    /// Swerling case 1 (slow fluctuation, exponential power).
    One,
    /// Swerling case 3 (slow fluctuation, chi-square with four degrees).
    Three,
}

/// Radar reflectivity of a target.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RadarSignature {
    /// Mean radar cross-section in square metres.
    pub rcs_m2: f64,
    /// Fluctuation model.
    pub swerling: Swerling,
}

/// A platform (a sensor carrier) in the world.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PlatformState {
    /// Platform identifier.
    pub id: PlatformId,
    /// Pose at the snapshot time.
    pub pose: Pose3,
    /// Velocity in metres per second.
    pub velocity: Vector3,
}

impl PlatformState {
    /// Pose at time `t`, extrapolating the position linearly from
    /// `snapshot_time`; the rotation is unchanged.
    pub fn pose_at(&self, snapshot_time: SimTime, t: SimTime) -> Pose3 {
        Pose3::new(
            extrapolate(self.pose.position, self.velocity, snapshot_time, t),
            self.pose.rotation,
        )
    }
}

/// A target in the world.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TargetState {
    /// Target identifier.
    pub id: TargetId,
    /// Pose at the snapshot time.
    pub pose: Pose3,
    /// Velocity in metres per second.
    pub velocity: Vector3,
    /// Radius of the target's extent in metres.
    pub extent_radius: f64,
    /// Radar signature.
    pub signature: RadarSignature,
    /// Transponder code, if the target carries a transponder.
    pub transponder: Option<TransponderCode>,
    /// Platform this target is attached to, if any.
    pub platform: Option<PlatformId>,
}

impl TargetState {
    /// Position at time `t`, extrapolated linearly from `snapshot_time`.
    pub fn position_at(&self, snapshot_time: SimTime, t: SimTime) -> Vector3 {
        extrapolate(self.pose.position, self.velocity, snapshot_time, t)
    }
}

/// A frequency band.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Band {
    /// Centre frequency in hertz.
    pub centre_hz: f64,
    /// Bandwidth in hertz.
    pub bandwidth_hz: f64,
}

impl Band {
    /// Fraction of this band's width that overlaps `receiver`, in `[0, 1]`.
    pub fn overlap_fraction(&self, receiver: &Band) -> f64 {
        if self.bandwidth_hz.is_nan() || self.bandwidth_hz <= 0.0 {
            return 0.0;
        }
        let lo = (self.centre_hz - self.bandwidth_hz / 2.0)
            .max(receiver.centre_hz - receiver.bandwidth_hz / 2.0);
        let hi = (self.centre_hz + self.bandwidth_hz / 2.0)
            .min(receiver.centre_hz + receiver.bandwidth_hz / 2.0);
        ((hi - lo) / self.bandwidth_hz).clamp(0.0, 1.0)
    }
}

/// The kind of an emitter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum EmitterKind {
    /// A noise jammer.
    NoiseJammer,
}

/// An electromagnetic emitter (for example a jammer).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EmitterState {
    /// Emitter identifier.
    pub id: EmitterId,
    /// Platform carrying the emitter, if any.
    pub platform: Option<PlatformId>,
    /// World position in metres.
    pub position: Vector3,
    /// Emitted band.
    pub band: Band,
    /// Effective radiated power in watts.
    pub erp_w: f64,
    /// Emitter kind.
    pub kind: EmitterKind,
}

/// The state of the world at one instant.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WorldSnapshot {
    /// Snapshot time.
    pub time: SimTime,
    /// Platforms.
    pub platforms: Vec<PlatformState>,
    /// Targets.
    pub targets: Vec<TargetState>,
    /// Emitters.
    pub emitters: Vec<EmitterState>,
}

impl WorldSnapshot {
    /// Creates an empty snapshot at `time`.
    pub fn new(time: SimTime) -> Self {
        WorldSnapshot {
            time,
            platforms: Vec::new(),
            targets: Vec::new(),
            emitters: Vec::new(),
        }
    }

    /// Checks finiteness, id uniqueness and value ranges.
    pub fn validate(&self) -> Result<(), RadarError> {
        let mut seen = HashSet::new();
        for p in &self.platforms {
            unique(&mut seen, p.id, "platform")?;
            finite_pose(&p.pose, &p.velocity, &format!("platform {:?}", p.id))?;
        }
        let mut seen = HashSet::new();
        for t in &self.targets {
            unique(&mut seen, t.id, "target")?;
            let who = format!("target {:?}", t.id);
            finite_pose(&t.pose, &t.velocity, &who)?;
            if !(t.signature.rcs_m2.is_finite() && t.signature.rcs_m2 >= 0.0) {
                return bad(&who, "rcs_m2 must be finite and >= 0");
            }
            if !(t.extent_radius.is_finite() && t.extent_radius >= 0.0) {
                return bad(&who, "extent_radius must be finite and >= 0");
            }
        }
        let mut seen = HashSet::new();
        for e in &self.emitters {
            unique(&mut seen, e.id, "emitter")?;
            let who = format!("emitter {:?}", e.id);
            if !e.position.is_finite() {
                return bad(&who, "position must be finite");
            }
            if !(e.erp_w.is_finite() && e.erp_w >= 0.0) {
                return bad(&who, "erp_w must be finite and >= 0");
            }
            if !(e.band.centre_hz.is_finite()
                && e.band.bandwidth_hz.is_finite()
                && e.band.bandwidth_hz > 0.0)
            {
                return bad(&who, "band must be finite with bandwidth_hz > 0");
            }
        }
        Ok(())
    }
}

fn extrapolate(
    position: Vector3,
    velocity: Vector3,
    snapshot_time: SimTime,
    t: SimTime,
) -> Vector3 {
    let dt = t.as_secs_f64() - snapshot_time.as_secs_f64();
    position + velocity * dt
}

fn bad(who: &str, why: &str) -> Result<(), RadarError> {
    Err(RadarError::InvalidWorld(format!("{who}: {why}")))
}

fn unique<I: Hash + Eq + std::fmt::Debug>(
    seen: &mut HashSet<I>,
    id: I,
    kind: &str,
) -> Result<(), RadarError> {
    let label = format!("duplicate {kind} {id:?}");
    if seen.insert(id) {
        Ok(())
    } else {
        Err(RadarError::InvalidWorld(label))
    }
}

fn finite_pose(pose: &Pose3, velocity: &Vector3, who: &str) -> Result<(), RadarError> {
    if !pose.position.is_finite() {
        return bad(who, "position must be finite");
    }
    if !velocity.is_finite() {
        return bad(who, "velocity must be finite");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn target(id: u64) -> TargetState {
        TargetState {
            id: TargetId(id),
            pose: Pose3::IDENTITY,
            velocity: Vector3::new(1.0, 2.0, 0.0),
            extent_radius: 1.0,
            signature: RadarSignature {
                rcs_m2: 1.0,
                swerling: Swerling::Zero,
            },
            transponder: None,
            platform: None,
        }
    }

    fn snap(targets: Vec<TargetState>) -> WorldSnapshot {
        let mut w = WorldSnapshot::new(SimTime::ZERO);
        w.targets = targets;
        w
    }

    #[test]
    fn extrapolation() {
        let t = target(1);
        let p = t.position_at(SimTime::ZERO, SimTime::from_nanos(2_000_000_000));
        assert_eq!(p, Vector3::new(2.0, 4.0, 0.0));
        let back = t.position_at(SimTime::from_nanos(2_000_000_000), SimTime::ZERO);
        assert_eq!(back, Vector3::new(-2.0, -4.0, 0.0));
    }

    #[test]
    fn platform_pose_extrapolates_position_only() {
        let p = PlatformState {
            id: PlatformId(1),
            pose: Pose3::IDENTITY,
            velocity: Vector3::new(0.0, 10.0, 0.0),
        };
        let q = p.pose_at(SimTime::ZERO, SimTime::from_nanos(1_000_000_000));
        assert_eq!(q.position, Vector3::new(0.0, 10.0, 0.0));
        assert_eq!(q.rotation, p.pose.rotation);
    }

    #[test]
    fn valid_world_passes() {
        assert!(snap(vec![target(1), target(2)]).validate().is_ok());
    }

    #[test]
    fn duplicate_target_rejected() {
        let r = snap(vec![target(1), target(1)]).validate();
        assert!(matches!(r, Err(RadarError::InvalidWorld(_))));
    }

    #[test]
    fn nan_rejected() {
        let mut t = target(1);
        t.pose.position = Vector3::new(f64::NAN, 0.0, 0.0);
        assert!(matches!(
            snap(vec![t]).validate(),
            Err(RadarError::InvalidWorld(_))
        ));
        let mut t = target(2);
        t.velocity = Vector3::new(0.0, f64::INFINITY, 0.0);
        assert!(snap(vec![t]).validate().is_err());
    }

    #[test]
    fn negative_rcs_rejected() {
        let mut t = target(1);
        t.signature.rcs_m2 = -1.0;
        assert!(matches!(
            snap(vec![t]).validate(),
            Err(RadarError::InvalidWorld(_))
        ));
        let mut t = target(1);
        t.extent_radius = -1.0;
        assert!(snap(vec![t]).validate().is_err());
    }

    #[test]
    fn emitter_validation() {
        let e = EmitterState {
            id: EmitterId(1),
            platform: None,
            position: Vector3::ZERO,
            band: Band {
                centre_hz: 10e9,
                bandwidth_hz: 100e6,
            },
            erp_w: 10.0,
            kind: EmitterKind::NoiseJammer,
        };
        let mut w = WorldSnapshot::new(SimTime::ZERO);
        w.emitters.push(e.clone());
        assert!(w.validate().is_ok());
        w.emitters[0].erp_w = -1.0;
        assert!(w.validate().is_err());
        w.emitters[0] = e.clone();
        w.emitters[0].band.bandwidth_hz = 0.0;
        assert!(w.validate().is_err());
        w.emitters[0] = e.clone();
        w.emitters.push(e);
        assert!(w.validate().is_err());
    }

    #[test]
    fn band_overlap() {
        let jammer = Band {
            centre_hz: 10e9,
            bandwidth_hz: 100e6,
        };
        let rx = Band {
            centre_hz: 10.025e9,
            bandwidth_hz: 50e6,
        };
        assert!((jammer.overlap_fraction(&rx) - 0.5).abs() < 1e-12);
        let far = Band {
            centre_hz: 20e9,
            bandwidth_hz: 50e6,
        };
        assert_eq!(jammer.overlap_fraction(&far), 0.0);
    }
}
