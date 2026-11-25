//! Passive radar with satellite illuminators, after Gomez-del-Hoyo et al.
//! (IRS 2022): compares the illuminators, then maps where a 40 m² truck is
//! detected with a parabolic antenna and with a horn.
//!
//! Run with `cargo run --example passive_starlink`.

use radar::bistatic::{
    CoverageGrid, CoverageMap, CoverageScenario, DetectionThreshold, Illuminator, PassiveReceiver,
    SteeredAntenna, SurveillanceAntenna,
};
use radar::error::RadarError;
use radar::geometry::{Vector2, Vector3};

fn main() -> Result<(), RadarError> {
    let threshold_db = match DetectionThreshold::PAPER {
        DetectionThreshold::MeanSnr { snr_db } => snr_db,
        _ => unreachable!(),
    };
    // The paper does not state its losses; 4.74 dB reproduces its ranges.
    let receiver = PassiveReceiver {
        losses_db: 4.74,
        ..PassiveReceiver::default()
    };

    println!("40 m² target, 40 dBi antenna, 0.5 s integration, {threshold_db} dB threshold");
    println!(
        "(Starlink at its 11.7 GHz carrier; the paper's ~18 km used a rounded 0.03 m wavelength)\n"
    );
    println!(
        "{:<12} {:>12} {:>14} {:>12}",
        "illuminator", "SNR @ 1 km", "max range", "resolution"
    );
    for (name, il) in [
        ("Starlink", Illuminator::starlink()),
        ("Galileo E5", Illuminator::galileo_e5()),
        ("Galileo E1", Illuminator::galileo_e1()),
        ("DVB-S2", Illuminator::dvb_s2()),
        ("GPS L5", Illuminator::gps_l5()),
        ("GPS L1 C/A", Illuminator::gps_l1_ca()),
    ] {
        let snr = receiver.snr_db(&il, 40.0, il.altitude_m, 1e3);
        let range = receiver.max_rx_range(&il, 40.0, il.altitude_m, threshold_db);
        let resolution = radar::bistatic::range_resolution(il.bandwidth_hz, 0.0, 0.0);
        println!(
            "{name:<12} {snr:>9.1} dB {:>11.1} km {resolution:>10.1} m",
            range / 1e3
        );
    }

    let starlink = Illuminator::starlink();
    for (name, antenna, cell) in [
        (
            "parabolic, 40 dBi, 2°",
            SurveillanceAntenna::parabolic(),
            1_000.0,
        ),
        ("horn, 17 dBi, 30°", SurveillanceAntenna::horn(), 100.0),
    ] {
        let scenario = CoverageScenario {
            illuminator: starlink,
            transmitter: Vector3::new(0.0, 0.0, starlink.altitude_m),
            receiver,
            receiver_position: Vector3::ZERO,
            antenna: Some(SteeredAntenna {
                antenna,
                azimuth: 0.0,
                elevation: 0.0,
            }),
            rcs_m2: 40.0,
            threshold_snr_db: threshold_db,
        };
        let grid = CoverageGrid::centred(Vector2::ZERO, 20, cell, 0.0);
        let map = CoverageMap::compute(&scenario, &grid)?;
        println!(
            "\n{name}, pointed north ({} m cells, # = detected, R = receiver):",
            cell
        );
        for row in (0..grid.rows).rev() {
            let line: String = (0..grid.columns)
                .map(|column| {
                    if (column, row) == (20, 20) {
                        'R'
                    } else if map.is_covered(column, row) {
                        '#'
                    } else {
                        '.'
                    }
                })
                .collect();
            println!("  {line}");
        }
        println!(
            "  farthest detection {:.1} km, covered area {:.2} km²",
            map.farthest_covered_m(Vector3::ZERO).unwrap_or(0.0) / 1e3,
            map.covered_area_m2() / 1e6
        );
    }
    Ok(())
}
