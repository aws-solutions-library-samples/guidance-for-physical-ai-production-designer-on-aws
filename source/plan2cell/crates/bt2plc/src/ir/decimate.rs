//! Optional waypoint decimation for the robot simulator tables: time-parametrised
//! Ramer-Douglas-Peucker in joint space. Keeps the first/last waypoint and every phase change;
//! drops a waypoint when linear interpolation in time between the kept neighbours stays within
//! `tol_deg` on every joint.

use crate::input::Trajectory;

/// Indices of the waypoints to keep (sorted). `tol_deg <= 0` keeps all of them.
pub fn keep(t: &Trajectory, tol_deg: f64) -> Vec<usize> {
    let n = t.waypoints.len();
    if tol_deg <= 0.0 || n <= 2 {
        return (0..n).collect();
    }
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;
    for i in 1..n {
        if t.waypoints[i].phase != t.waypoints[i - 1].phase {
            keep[i - 1] = true;
            keep[i] = true;
        }
    }
    let anchors: Vec<usize> = (0..n).filter(|&i| keep[i]).collect();
    for w in anchors.windows(2) {
        rdp(t, w[0], w[1], tol_deg.to_radians(), &mut keep);
    }
    (0..n).filter(|&i| keep[i]).collect()
}

fn rdp(t: &Trajectory, a: usize, b: usize, tol: f64, keep: &mut [bool]) {
    if b <= a + 1 {
        return;
    }
    let (wa, wb) = (&t.waypoints[a], &t.waypoints[b]);
    let span = wb.t_s - wa.t_s;
    let mut worst = (0.0, a);
    for i in a + 1..b {
        let w = &t.waypoints[i];
        let alpha = if span > 0.0 {
            (w.t_s - wa.t_s) / span
        } else {
            0.0
        };
        let dev = (0..6)
            .map(|j| (wa.q_rad[j] + alpha * (wb.q_rad[j] - wa.q_rad[j]) - w.q_rad[j]).abs())
            .fold(0.0, f64::max);
        if dev > worst.0 {
            worst = (dev, i);
        }
    }
    if worst.0 > tol {
        keep[worst.1] = true;
        rdp(t, a, worst.1, tol, keep);
        rdp(t, worst.1, b, tol, keep);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::{Phase, Waypoint};

    fn wp(t: f64, q: f64) -> Waypoint {
        Waypoint {
            t_s: t,
            q_rad: [q, 0.0, 0.0, 0.0, 0.0, 0.0],
            phase: Phase::Transit,
        }
    }

    #[test]
    fn straight_segments_collapse_and_corners_stay() {
        let t = Trajectory {
            key: None,
            waypoints: vec![wp(0.0, 0.0), wp(1.0, 0.1), wp(2.0, 0.2), wp(3.0, 0.0)],
        };
        assert_eq!(keep(&t, 0.1), vec![0, 2, 3]);
        assert_eq!(keep(&t, 0.0), vec![0, 1, 2, 3]);
    }
}
