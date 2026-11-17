//! Opaque identifier types.

macro_rules! id_type {
    ($(#[$doc:meta])* $name:ident($inner:ty)) => {
        $(#[$doc])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        pub struct $name(pub $inner);
    };
}

id_type!(/// Identifies a platform that carries sensors.
    PlatformId(u64));
id_type!(/// Identifies a target in the world.
    TargetId(u64));
id_type!(/// Identifies a sensor.
    SensorId(u64));
id_type!(/// Identifies a track maintained by a tracker.
    TrackId(u64));
id_type!(/// Identifies an emitter (for example a jammer).
    EmitterId(u64));
id_type!(/// A transponder identification code.
    TransponderCode(u32));

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    #[test]
    fn ids_order_and_hash() {
        assert!(TargetId(1) < TargetId(2));
        let s: HashSet<_> = [SensorId(1), SensorId(1)].into();
        assert_eq!(s.len(), 1);
    }
}
