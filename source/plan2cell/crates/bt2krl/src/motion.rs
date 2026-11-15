//! Joint-space motion planning for PTP mode (TASK section 6.2): decimation, per-segment
//! velocity percentages and the time estimate.

use crate::config::MotionCfg;
use crate::program::Waypoint;

/// Indices of the waypoints to keep (sorted): time-parametrised Ramer-Douglas-Peucker in
/// joint space (same algorithm as `bt2plc::ir::decimate`, in degrees). Keeps the first and
/// last waypoint and both sides of every phase change; drops a waypoint when linear
/// interpolation in time between the kept neighbours stays within `tol_deg` on every axis.
pub fn decimate(w: &[Waypoint], tol_deg: f64) -> Vec<usize> {
    let n = w.len();
    if tol_deg <= 0.0 || n <= 2 {
        return (0..n).collect();
    }
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;
    for i in 1..n {
        if w[i].phase != w[i - 1].phase {
            keep[i - 1] = true;
            keep[i] = true;
        }
    }
    let anchors: Vec<usize> = (0..n).filter(|&i| keep[i]).collect();
    for a in anchors.windows(2) {
        rdp(w, a[0], a[1], tol_deg, &mut keep);
    }
    (0..n).filter(|&i| keep[i]).collect()
}

/// Largest per-axis deviation of `w[i]` from the time-linear chord `w[a]`..`w[b]`.
pub fn chord_dev(w: &[Waypoint], a: usize, b: usize, i: usize) -> f64 {
    let (wa, wb) = (&w[a], &w[b]);
    let span = wb.t_s - wa.t_s;
    let alpha = if span > 0.0 {
        (w[i].t_s - wa.t_s) / span
    } else {
        0.0
    };
    (0..6)
        .map(|j| (wa.q_deg[j] + alpha * (wb.q_deg[j] - wa.q_deg[j]) - w[i].q_deg[j]).abs())
        .fold(0.0, f64::max)
}

fn rdp(w: &[Waypoint], a: usize, b: usize, tol: f64, keep: &mut [bool]) {
    if b <= a + 1 {
        return;
    }
    let mut worst = (0.0, a);
    for i in a + 1..b {
        let d = chord_dev(w, a, b, i);
        if d > worst.0 {
            worst = (d, i);
        }
    }
    if worst.0 > tol {
        keep[worst.1] = true;
        rdp(w, a, worst.1, tol, keep);
        rdp(w, worst.1, b, tol, keep);
    }
}

/// Fraction of the maximum speed the leading axis needs to cover `dq` in `dt` seconds.
pub fn speed_ratio(dq: &[f64; 6], dt: f64, vmax: &[f64; 6]) -> f64 {
    let need = (0..6).map(|i| dq[i].abs() / vmax[i]).fold(0.0, f64::max);
    if need == 0.0 {
        0.0
    } else if dt <= 0.0 {
        f64::INFINITY
    } else {
        need / dt
    }
}

/// `BAS(#VEL_PTP, v)` for a segment: `ceil(100 * ratio / time_margin)`, clamped to
/// `[min_vel_pct, max_vel_pct]`. `time_margin > 1` slows the robot down.
pub fn vel_pct(dq: &[f64; 6], dt: f64, vmax: &[f64; 6], m: &MotionCfg) -> u8 {
    let r = speed_ratio(dq, dt, vmax);
    let v = (100.0 * r / m.time_margin - 1e-9).ceil();
    let v = if v.is_finite() { v } else { 100.0 };
    v.clamp(f64::from(m.min_vel_pct), f64::from(m.max_vel_pct)) as u8
}

/// Time of one synchronous PTP segment at `vel_pct` (constant speed; the leading axis sets it).
pub fn seg_time(dq: &[f64; 6], vel_pct: u8, vmax: &[f64; 6]) -> f64 {
    let v = f64::from(vel_pct.max(1)) / 100.0;
    (0..6)
        .map(|i| dq[i].abs() / (v * vmax[i]))
        .fold(0.0, f64::max)
}

pub fn delta(a: &[f64; 6], b: &[f64; 6]) -> [f64; 6] {
    std::array::from_fn(|i| b[i] - a[i])
}

pub fn max_abs_diff(a: &[f64; 6], b: &[f64; 6]) -> f64 {
    (0..6).map(|i| (a[i] - b[i]).abs()).fold(0.0, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wp(t: f64, q: f64, phase: &str) -> Waypoint {
        Waypoint {
            t_s: t,
            q_deg: [q, 0.0, 0.0, 0.0, 0.0, 0.0],
            phase: phase.into(),
        }
    }

    #[test]
    fn straight_segments_collapse_and_corners_stay() {
        let w = vec![
            wp(0.0, 0.0, "transit"),
            wp(1.0, 1.0, "transit"),
            wp(2.0, 2.0, "transit"),
            wp(3.0, 0.0, "transit"),
        ];
        assert_eq!(decimate(&w, 0.1), vec![0, 2, 3]);
        assert_eq!(decimate(&w, 0.0), vec![0, 1, 2, 3]);
    }

    #[test]
    fn phase_changes_are_kept() {
        let w = vec![
            wp(0.0, 0.0, "transit"),
            wp(1.0, 1.0, "transit"),
            wp(2.0, 2.0, "approach"),
            wp(3.0, 3.0, "approach"),
        ];
        assert_eq!(decimate(&w, 10.0), vec![0, 1, 2, 3]);
    }

    #[test]
    fn velocity_rounds_up_and_clamps() {
        let m = MotionCfg::default();
        let vmax = [100.0; 6];
        // 20 deg in 1 s at 100 deg/s max -> 20 %.
        assert_eq!(
            vel_pct(&[20.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0, &vmax, &m),
            20
        );
        assert_eq!(
            vel_pct(&[20.5, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0, &vmax, &m),
            21
        );
        assert_eq!(vel_pct(&[0.0; 6], 1.0, &vmax, &m), 1);
        assert_eq!(
            vel_pct(&[500.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0, &vmax, &m),
            100
        );
        let slow = MotionCfg {
            time_margin: 2.0,
            ..MotionCfg::default()
        };
        assert_eq!(
            vel_pct(&[20.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0, &vmax, &slow),
            10
        );
        assert!((seg_time(&[20.0, 0.0, 0.0, 0.0, 0.0, 0.0], 20, &vmax) - 1.0).abs() < 1e-12);
    }
}
