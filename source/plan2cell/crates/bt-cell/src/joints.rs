//! Joint-space constants and helpers, verbatim from cell-runner's `trajectory.rs:49-58` and
//! `:343-409`, plus the guards btgui adds for interactive runs.

/// Duration of the pre-move to an interlock's safe intermediate configuration.
pub const HOME_TIME_S: f64 = 2.5;
/// Settle slack appended after a trajectory's own duration elapses.
pub const COMPLETION_MARGIN_S: f64 = 1.0;
/// Duration of the fallback return-to-home when a `MoveHome` node carries no path.
pub const HOME_RETURN_TIME_S: f64 = 7.0;
/// Default tolerance for a joint-space interlock gate.
pub const DEFAULT_INTERLOCK_TOL_RAD: f64 = 0.15;
/// Fallback home configuration when a robot has none of its own.
pub const FALLBACK_HOME_JOINTS: [f64; 6] = [0.0, -1.57, 2.2, 0.0, -0.8, 0.0];

/// Guard against a tree that never settles (cell-runner's `MAX_TICKS`, `btree.rs:46`).
pub const MAX_TICKS: u32 = 1_000_000;
/// A motion that starts further than this from where the robot is makes playback jump
/// (btgui's continuity check; cell-runner has none).
pub const JUMP_TOL_RAD: f64 = 1e-3;
/// Segments a run may place before the cell leaves fail (btgui's guard against a runaway
/// tree, e.g. a `ReactiveSequence` that restarts a motion on every tick).
pub const MAX_SEGMENTS: usize = 20_000;

/// Widen a plan's positions vector to a fixed 6-axis configuration, padding
/// with zeros and dropping any excess axes.
pub fn to_six(positions: &[f64]) -> [f64; 6] {
    let mut q = [0.0f64; 6];
    for (slot, value) in q.iter_mut().zip(positions.iter()) {
        *slot = *value;
    }
    q
}

/// Smallest absolute difference between two joint angles, treating values a full
/// turn apart as the same physical configuration.
///
/// Weld poses put wrist joints near ±π, where the planner and the solver may
/// disagree on the sign; without wrapping that reads as a spurious 2π error and
/// an interlock gate would never open.
pub fn angular_diff(a: f64, b: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    let d = (a - b).abs() % two_pi;
    d.min(two_pi - d)
}

/// Largest per-axis angular difference between two configurations.
pub fn max_joint_error(a: &[f64; 6], b: &[f64; 6]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| angular_diff(*x, *y))
        .fold(0.0f64, f64::max)
}

/// A resolved waypoint: a 6-axis configuration at a segment-local time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    /// Joint angles, rad.
    pub q: [f64; 6],
    /// Seconds from the segment's start.
    pub t: f64,
}

/// Configuration at segment-local time `t`, linearly interpolated and clamped at
/// both ends so a finished segment holds its final pose until the next begins.
pub fn interpolate(traj: &[Waypoint], t: f64) -> [f64; 6] {
    match (traj.first(), traj.last()) {
        (None, _) | (_, None) => FALLBACK_HOME_JOINTS,
        (Some(first), Some(last)) => {
            if t <= first.t {
                return first.q;
            }
            if t >= last.t {
                return last.q;
            }
            for pair in traj.windows(2) {
                let (a, b) = (&pair[0], &pair[1]);
                if a.t <= t && t <= b.t {
                    let span = b.t - a.t;
                    let alpha = if span > 0.0 { (t - a.t) / span } else { 0.0 };
                    let mut q = [0.0f64; 6];
                    for (i, slot) in q.iter_mut().enumerate() {
                        *slot = a.q[i] + alpha * (b.q[i] - a.q[i]);
                    }
                    return q;
                }
            }
            last.q
        }
    }
}
