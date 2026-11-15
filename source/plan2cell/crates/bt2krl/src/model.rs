//! Static robot model data: software limits and maximum axis speeds.
//!
//! Source: the machine data of KR C4 controllers (`KRC/R1/Mada/$machine.dat` of their robot data
//! exports):
//! - limits: `$SOFTN_END[i]` / `$SOFTP_END[i]` in degrees;
//! - speeds: `$VEL_AXIS_MA[i] / |$RAT_MOT_AX[i]| * 6` in deg/s (motor rpm -> axis deg/s).
//!
//! Controllers read per model: KR240 R2900 ultra four, KR210 R2700 extra one, KR270 R2700 ultra
//! two, KR160 R1570 nano one.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RobotModel {
    pub name: &'static str,
    pub soft_min: [f64; 6],
    pub soft_max: [f64; 6],
    /// deg/s at 100 % `$VEL_AXIS`.
    pub vmax: [f64; 6],
}

pub const MODELS: &[RobotModel] = &[
    RobotModel {
        name: "KR240R2900_ULTRA",
        soft_min: [-185.0, -140.0, -120.0, -350.0, -122.5, -350.0],
        soft_max: [185.0, -5.0, 155.0, 350.0, 122.5, 350.0],
        vmax: [105.1, 101.0, 107.0, 135.7, 128.8, 205.9],
    },
    RobotModel {
        name: "KR210R2700_EXTRA",
        soft_min: [-185.0, -140.0, -120.0, -350.0, -125.0, -350.0],
        soft_max: [185.0, -5.0, 155.0, 350.0, 125.0, 350.0],
        vmax: [123.3, 114.7, 112.0, 178.5, 172.3, 219.4],
    },
    RobotModel {
        name: "KR270R2700_ULTRA",
        soft_min: [-185.0, -140.0, -120.0, -350.0, -122.5, -350.0],
        soft_max: [185.0, -5.0, 155.0, 350.0, 122.5, 350.0],
        vmax: [105.1, 101.0, 107.0, 122.2, 112.7, 175.0],
    },
    RobotModel {
        name: "KR160R1570_NANO",
        soft_min: [-180.0, -145.0, -130.0, -350.0, -120.0, -350.0],
        soft_max: [180.0, 45.0, 145.0, 350.0, 120.0, 350.0],
        vmax: [123.3, 114.2, 88.8, 178.5, 172.3, 219.4],
    },
];

/// Model `"unknown"`: no limit check. Speeds are the per-axis maxima of [`MODELS`], so the
/// computed velocity percentages can only make a real (slower or equal) robot slower than
/// planned, never faster.
pub const UNKNOWN: RobotModel = RobotModel {
    name: "unknown",
    soft_min: [f64::NEG_INFINITY; 6],
    soft_max: [f64::INFINITY; 6],
    vmax: [123.3, 114.7, 112.0, 178.5, 172.3, 219.4],
};

fn norm(s: &str) -> String {
    s.chars()
        .filter(|c| c.is_ascii_alphanumeric())
        .map(|c| c.to_ascii_uppercase())
        .collect()
}

/// `KR240R2900_ULTRA`, `kr240_r2900_ultra` and `KR240 R2900 ULTRA` all match.
pub fn lookup(name: &str) -> Option<&'static RobotModel> {
    if norm(name) == "UNKNOWN" {
        return Some(&UNKNOWN);
    }
    MODELS.iter().find(|m| norm(m.name) == norm(name))
}

impl RobotModel {
    pub fn known(&self) -> bool {
        self.name != "unknown"
    }

    /// First axis outside the software limits: `(axis 1..6, value, min, max)`.
    pub fn violation(&self, q: &[f64; 6]) -> Option<(usize, f64, f64, f64)> {
        (0..6)
            .find(|&i| q[i] < self.soft_min[i] || q[i] > self.soft_max[i])
            .map(|i| (i + 1, q[i], self.soft_min[i], self.soft_max[i]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lookup_normalises_names() {
        assert_eq!(
            lookup("kr240_r2900_ultra").unwrap().name,
            "KR240R2900_ULTRA"
        );
        assert_eq!(lookup("KR160 R1570 NANO").unwrap().name, "KR160R1570_NANO");
        assert!(!lookup("unknown").unwrap().known());
        assert!(lookup("KR6").is_none());
    }

    #[test]
    fn unknown_speeds_are_the_table_maxima() {
        for i in 0..6 {
            let m = MODELS.iter().map(|m| m.vmax[i]).fold(0.0, f64::max);
            assert_eq!(UNKNOWN.vmax[i], m);
        }
    }

    #[test]
    fn home_of_the_examples_is_inside_kr240_limits() {
        let m = lookup("KR240R2900_ULTRA").unwrap();
        assert_eq!(m.violation(&[0.0, -90.0, 90.0, 0.0, 0.0, 0.0]), None);
        assert_eq!(
            m.violation(&[0.0, -180.0, 90.0, 0.0, 0.0, 0.0]),
            Some((2, -180.0, -140.0, -5.0))
        );
    }
}
