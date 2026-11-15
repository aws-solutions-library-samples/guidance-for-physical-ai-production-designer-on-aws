//! Running the tree from the panel: a Step is one event, Run to wait and Run to t are virtual
//! runs, and Run follows the wall clock.
//!
//! Like [`Executor::run`], the tree is ticked only at events (a wakeup, new motion, a user
//! change that matters), and otherwise only the clock moves, so a real-time run places the
//! same motion as a virtual one. A tree that is busy without any timer (`Stalled([])`) only
//! moves when ticked: it gets a heartbeat tick at the tick rate instead, and only while it
//! stays that way (a beat that starts a timer hands the tree back to the events).
//!
//! Every run notes why it stopped ([`BtPanel::last_stop`]), and a new stop brings its node into
//! view. A real-time run that halts (a breakpoint, a manual leaf, a stalled gate) stays armed:
//! the clock waits, and the run goes on as soon as the user decides. The other runs go on when
//! a decision in the halt banner asks them to ([`RunMode`]).

use std::time::Duration;

use bt_core::{NodeStatus, Uid};
use bt_exec::{Executor, RunOptions, Stop};

use super::BtPanel;

/// Ticks one real-time frame may take before the run gives up (a runaway tree).
const MAX_TICKS_PER_FRAME: u32 = 10_000;
/// Heartbeat ticks one frame may catch up on.
const MAX_BEATS_PER_FRAME: u32 = 100;
/// The most wall-clock time one frame advances, s (after the window was hidden, say).
const MAX_FRAME_S: f64 = 0.25;
/// How often a halted real-time run looks again.
const HALTED_REPAINT: Duration = Duration::from_millis(250);

/// How the last run went, so that a decision at a halt can go on the same way
/// ([`BtPanel::continue_run`]).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) enum RunMode {
    /// One event at a time.
    #[default]
    Step,
    /// Run to wait.
    Virtual,
    /// Run to t, to this instant.
    UntilTime(f64),
    /// Run in real time.
    RealTime,
}

impl BtPanel {
    pub(crate) fn period(&self) -> f64 {
        1.0 / f64::from(self.tick_hz.max(0.1))
    }

    /// Executor time between two heartbeat ticks.
    fn beat(&self) -> f64 {
        self.period() * f64::from(self.speed)
    }

    /// One event: tick once at the next instant something changes, or at once if something
    /// already asks for a tick. Every paused node is resumed first. With nothing scheduled the
    /// tree is ticked anyway, at `now` (so a Step shows what edits change) or, for a tree busy
    /// without a timer, a heartbeat later.
    pub fn step(&mut self) {
        self.step_once(true);
    }

    /// [`step`](Self::step); `resume: false` leaves the paused nodes to the decisions taken
    /// (going on after a decision in the halt banner about one of them).
    fn step_once(&mut self, resume: bool) {
        let (beat, looping, release) = (self.beat(), self.looping, self.auto_release);
        let Some(ex) = self.executor.as_mut() else {
            return;
        };
        if ex.is_done() && (!looping || ex.has_leaf_handler()) {
            ex.reset();
        }
        if resume {
            ex.resume();
        }
        let report = ex.run(RunOptions {
            max_ticks: 1,
            stop_on_manual: false,
            release_stalls: release,
            ..RunOptions::VIRTUAL
        });
        let status = if report.ticks > 0 {
            report.status
        } else {
            match report.stop {
                Stop::Stalled(blocked) if blocked.is_empty() => ex.tick_at(ex.now() + beat),
                // Nothing would let a paused node pass.
                Stop::Breakpoint(_) => report.status,
                _ => ex.tick(),
            }
        };
        // A Step stops after one tick whatever comes next; what matters is where that left
        // the tree.
        let stop = settled(ex, status);
        self.last_mode = RunMode::Step;
        self.go_live();
        self.note_stop(stop);
    }

    /// Tick in virtual time until the tree finishes, or stops at a breakpoint, a manual leaf or
    /// a stalled gate ([`last_stop`](Self::last_stop) says which). With auto-release on,
    /// stalled gates are released like cell-runner does.
    pub fn run_to_wait(&mut self) {
        self.run_virtual(true);
    }

    /// [`run_to_wait`](Self::run_to_wait); `stop_on_manual: false` runs on past manual leaves
    /// while anything else is scheduled. The ticks it took.
    fn run_virtual(&mut self, stop_on_manual: bool) -> u32 {
        let release = self.auto_release;
        let Some(ex) = self.executor.as_mut() else {
            return 0;
        };
        if ex.is_done() {
            ex.reset();
        }
        let report = ex.run(RunOptions {
            stop_on_manual,
            release_stalls: release,
            ..RunOptions::VIRTUAL
        });
        let mut ticks = report.ticks;
        let stop = if report.ticks == 0 && report.stop == Stop::Stalled(Vec::new()) {
            // A tree busy without a timer only moves when ticked: one tick per call.
            let status = ex.tick();
            ticks = 1;
            settled(ex, status).unwrap_or(report.stop)
        } else {
            report.stop
        };
        self.running = false;
        self.last_mode = RunMode::Virtual;
        self.go_live();
        self.note_stop(Some(stop));
        ticks
    }

    /// Tick in virtual time up to instant `t`, unless the tree finishes or halts first. Manual
    /// leaves do not stop it: they keep waiting while the clock moves.
    pub fn run_to_time(&mut self, t: f64) {
        let release = self.auto_release;
        let Some(ex) = self.executor.as_mut() else {
            return;
        };
        if ex.is_done() {
            ex.reset();
        }
        let report = ex.run(RunOptions {
            until: t,
            stop_on_manual: false,
            release_stalls: release,
            ..RunOptions::VIRTUAL
        });
        self.running = false;
        self.last_mode = RunMode::UntilTime(t);
        self.go_live();
        self.note_stop(Some(report.stop));
    }

    /// Go on the way the last run went, after the user decided something where it stopped:
    ///
    /// - Run to wait runs on. If nothing changed for the manual leaves it stopped for (the user
    ///   left them waiting), it runs on past them while anything else is scheduled.
    /// - Run to t runs on to its instant, and a Step steps once more; only the paused nodes the
    ///   decision let go start (the toolbar's Step resumes them all).
    /// - A real-time run is still armed and goes on by itself. One that is not armed any more
    ///   (it hit the tick limit, the user paused it, or looking back paused it) is armed again:
    ///   the decision was to go on.
    ///
    /// A finished tree is left alone.
    pub(crate) fn continue_run(&mut self) {
        let Some(ex) = &self.executor else {
            return;
        };
        if ex.is_done() {
            return;
        }
        match self.last_mode {
            RunMode::Step => self.step_once(false),
            RunMode::UntilTime(t) => self.run_to_time(t),
            RunMode::RealTime => {
                if !self.running {
                    self.set_running(true);
                }
            }
            RunMode::Virtual => {
                let waiting = ex.waiting_on();
                let ticks = self.run_virtual(true);
                if ticks == 0
                    && !waiting.is_empty()
                    && self.last_stop == Some(Stop::WaitingManual(waiting))
                {
                    self.run_virtual(false);
                }
            }
        }
    }

    /// Run in real time (see [`advance_real_time`](Self::advance_real_time)), or pause.
    pub fn set_running(&mut self, running: bool) {
        if running {
            let looping = self.looping;
            if let Some(ex) = &mut self.executor
                && ex.is_done()
                && (!looping || ex.has_leaf_handler())
            {
                ex.reset();
            }
            self.last_stop = None;
            self.last_mode = RunMode::RealTime;
            self.accumulator = 0.0;
            self.go_live();
        }
        self.running = running;
    }

    /// Whether a real-time run is on (it may be halted, waiting for the user).
    pub fn is_running(&self) -> bool {
        self.running
    }

    /// Start a finished run over instead of stopping. With the cell leaves the run is reset, so
    /// the schedule starts again at 0 (cell-runner's looping playback); a plain tree is ticked
    /// again on the heartbeat, which starts it over like BT.CPP.
    pub fn set_looping(&mut self, on: bool) {
        self.looping = on;
    }

    /// Back to the start (policies, overrides that stay, breakpoints and edits are kept).
    pub fn reset(&mut self) {
        if let Some(ex) = &mut self.executor {
            ex.reset();
        }
        self.running = false;
        self.last_stop = None;
        self.go_live();
    }

    /// Move a real-time run on by `wall_dt_s` seconds of wall-clock time, `× speed` of executor
    /// time, ticking at the events on the way. [`BtPanel::ui`] calls it every frame while
    /// running.
    ///
    /// A tree busy without a timer, and a finished plain tree that loops (ticking it again
    /// starts it over), are ticked on the heartbeat, one tick per period of the tick rate; each
    /// beat is an event like any other, so whatever it starts runs from there. A halt (a
    /// breakpoint, a manual leaf with nothing else scheduled, a stalled gate) keeps the clock
    /// where it is and the run armed; finishing ends the run, or with Loop starts it over. A
    /// finished tree does not start over unless looping.
    pub fn advance_real_time(&mut self, wall_dt_s: f64) {
        let (period, beat, looping) = (self.period(), self.beat(), self.looping);
        let (speed, release) = (f64::from(self.speed), self.auto_release);
        let Some(ex) = self.executor.as_mut() else {
            self.running = false;
            return;
        };
        // Looping, a finished plain tree starts over when ticked again (on the heartbeat, like
        // BT.CPP ticking it at its rate); with the cell leaves the run is reset instead, so the
        // schedule starts again at 0.
        let restarts = looping && !ex.has_leaf_handler();
        if ex.is_done() {
            if !looping {
                self.running = false;
                return;
            }
            if !restarts {
                ex.reset();
            }
        }
        // Wall-clock time not spent on the executor's clock yet (part of a heartbeat period).
        let mut budget = std::mem::take(&mut self.accumulator) + wall_dt_s.max(0.0);
        let (mut ticks, mut beats) = (0, 0);
        // Tick at the events up to where the frame's time ends; whenever the tree waits for a
        // heartbeat instead, give it one per period and go on from there.
        let stop = loop {
            if !(restarts && ex.is_done()) {
                let start = ex.now();
                let report = ex.run(RunOptions {
                    max_ticks: MAX_TICKS_PER_FRAME - ticks,
                    until: start + budget * speed,
                    stop_on_manual: false,
                    release_stalls: release,
                });
                ticks += report.ticks;
                budget = (budget - (ex.now() - start) / speed).max(0.0);
                match report.stop {
                    Stop::Stalled(blocked) if blocked.is_empty() => {}
                    Stop::Finished(_) if restarts => {}
                    stop => break Some(stop),
                }
            }
            // Nothing happens unless the tree is ticked, at the tick rate.
            if beats == MAX_BEATS_PER_FRAME {
                // Too far behind to catch up.
                break None;
            }
            if budget < period {
                // The rest of the period, for the next frame.
                self.accumulator = budget;
                break None;
            }
            budget -= period;
            beats += 1;
            let status = ex.tick_at(ex.now() + beat);
            if status != NodeStatus::Running && !restarts {
                break Some(Stop::Finished(status));
            }
        };
        match stop {
            // Only with the cell leaves: a plain tree starts over on the heartbeat above.
            Some(Stop::Finished(_)) if looping => {
                if let Some(ex) = &mut self.executor {
                    ex.reset();
                }
                self.last_stop = None;
            }
            Some(stop @ (Stop::Finished(_) | Stop::TickLimit)) => {
                self.running = false;
                self.note_stop(Some(stop));
            }
            None | Some(Stop::Until) => self.last_stop = None,
            Some(stop) => self.note_stop(Some(stop)),
        }
    }

    /// The real-time run, once per frame.
    pub(super) fn drive(&mut self, ctx: &egui::Context) {
        let now = web_time::Instant::now();
        let dt = self
            .last_frame
            .map_or(0.0, |last| (now - last).as_secs_f64());
        self.last_frame = Some(now);
        if !self.running {
            return;
        }
        self.advance_real_time(dt.min(MAX_FRAME_S));
        if !self.running {
            return;
        }
        if self.executor.as_ref().and_then(Executor::halted).is_some() {
            // Nothing moves until the user decides.
            ctx.request_repaint_after(HALTED_REPAINT);
        } else {
            ctx.request_repaint();
        }
    }

    /// Why the last run or Step stopped: finished, a breakpoint, manual leaves, a stall, the
    /// tick limit, or (Run to t) the instant. `None` while a real-time run moves, after a Step
    /// that leaves more to do, and after a reset.
    pub fn last_stop(&self) -> Option<&Stop> {
        self.last_stop.as_ref()
    }

    /// Remember why a run stopped, and at which tick; a new stop brings its first node into
    /// view.
    fn note_stop(&mut self, stop: Option<Stop>) {
        if stop != self.last_stop
            && let Some(&uid) = stop.as_ref().and_then(|s| stop_nodes(s).first())
        {
            self.focus = Some(uid);
        }
        self.last_stop = stop;
        self.last_stop_tick = self.executor.as_ref().map_or(0, Executor::ticks);
    }

    // ───────────────────────────── display time ─────────────────────────────

    /// The tick the panel shows: the one scrubbed to, else the executor's latest.
    pub fn shown_tick(&self) -> u64 {
        self.scrub
            .unwrap_or_else(|| self.executor.as_ref().map_or(0, Executor::ticks))
    }

    /// The instant the panel shows: the one scrubbed to (or the scrubbed tick's), else the
    /// executor's `now`.
    pub fn shown_time(&self) -> f64 {
        let Some(ex) = &self.executor else {
            return 0.0;
        };
        self.scrub_t
            .or_else(|| self.scrub.and_then(|tick| ex.time_at_tick(tick)))
            .unwrap_or_else(|| ex.now())
    }

    /// Show the run as it was at instant `t`: the last tick at or before it. `None`, or an
    /// instant not before `now`, follows the executor again. Scrubbing pauses a real-time run.
    pub fn scrub_to_time(&mut self, t: Option<f64>) {
        let Some(ex) = &self.executor else {
            return;
        };
        match t {
            Some(t) if t < ex.now() => {
                let t = t.max(0.0);
                self.scrub = Some(ex.tick_at_time(t));
                self.scrub_t = Some(t);
                self.running = false;
            }
            _ => self.go_live(),
        }
    }

    /// Follow the executor again.
    pub(crate) fn go_live(&mut self) {
        self.scrub = None;
        self.scrub_t = None;
    }
}

/// Where `ex` stands after a tick that returned `status`: finished, halted, or neither.
fn settled(ex: &Executor, status: NodeStatus) -> Option<Stop> {
    // An empty tree fails without its root ever changing: the tick says how it ended.
    if status != NodeStatus::Running {
        Some(Stop::Finished(status))
    } else {
        ex.halted()
    }
}

/// The nodes a stop is about.
fn stop_nodes(stop: &Stop) -> &[Uid] {
    match stop {
        Stop::Breakpoint(uids) | Stop::WaitingManual(uids) | Stop::Stalled(uids) => uids,
        Stop::Finished(_) | Stop::TickLimit | Stop::Until => &[],
    }
}
