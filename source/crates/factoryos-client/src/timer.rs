//! One-shot timers for request deadlines ([`crate::with_deadline`]): `setTimeout` in the browser,
//! one thread for all of them natively.

use std::time::Duration;

/// Call `f` once `delay` has passed.
#[cfg(target_arch = "wasm32")]
pub(crate) fn after(delay: Duration, f: impl FnOnce() + 'static) {
    use wasm_bindgen::prelude::*;

    #[wasm_bindgen]
    extern "C" {
        #[wasm_bindgen(js_name = setTimeout)]
        fn set_timeout(handler: &JsValue, ms: i32) -> JsValue;
    }
    let ms = i32::try_from(delay.as_millis()).unwrap_or(i32::MAX);
    set_timeout(&Closure::once_into_js(f), ms);
}

/// Call `f` once `delay` has passed (on the timer thread: keep it short).
#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn after(delay: Duration, f: impl FnOnce() + Send + 'static) {
    native::timers().add(std::time::Instant::now() + delay, Box::new(f));
}

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use std::sync::{Condvar, Mutex, OnceLock};
    use std::time::Instant;

    type Task = Box<dyn FnOnce() + Send>;

    pub(super) struct Timers {
        due: Mutex<Vec<(Instant, Task)>>,
        changed: Condvar,
    }

    pub(super) fn timers() -> &'static Timers {
        static TIMERS: OnceLock<&'static Timers> = OnceLock::new();
        TIMERS.get_or_init(|| {
            let timers: &'static Timers = Box::leak(Box::new(Timers { due: Mutex::default(), changed: Condvar::new() }));
            std::thread::Builder::new()
                .name("factoryos-timers".into())
                .spawn(move || timers.run())
                .expect("failed to spawn the timer thread");
            timers
        })
    }

    impl Timers {
        pub(super) fn add(&self, at: Instant, task: Task) {
            self.due.lock().unwrap().push((at, task));
            self.changed.notify_one();
        }

        fn run(&self) {
            let mut due = self.due.lock().unwrap();
            loop {
                let now = Instant::now();
                let mut fire = Vec::new();
                let mut i = 0;
                while i < due.len() {
                    if due[i].0 <= now {
                        fire.push(due.swap_remove(i).1);
                    } else {
                        i += 1;
                    }
                }
                if !fire.is_empty() {
                    // Without the lock: a task may arm another timer.
                    drop(due);
                    for task in fire {
                        task();
                    }
                    due = self.due.lock().unwrap();
                    continue;
                }
                due = match due.iter().map(|(at, _)| *at).min() {
                    Some(next) => self.changed.wait_timeout(due, next.saturating_duration_since(now)).unwrap().0,
                    None => self.changed.wait(due).unwrap(),
                };
            }
        }
    }
}
