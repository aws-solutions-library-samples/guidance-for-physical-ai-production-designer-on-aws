//! The code page of step 2 (Control Code Export): while it is shown, the main area is the code
//! (Rerun's welcome route, drawn by [`crate::code_export::CodeExport::page_ui`]) instead of the 3D
//! view; the Behaviour Tree pane, the Scene editor / Robo Fitter column and the timeline are gone with
//! it. Leaving it routes Rerun back to the recording it showed before, so the 3D view, its panes
//! and its timeline come back as they were.
//!
//! [`CodeLayout`] decides the route changes from what is wanted (the code page is open) and what
//! Rerun shows (its active recording); `R` is the recording id ([`StoreId`](crate::rr::re_log_types::StoreId)
//! in the app).

/// What the app does about the route this frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Switch<R> {
    None,
    /// Show the code: route to the welcome page (the code page is drawn there).
    ShowCode,
    /// Back to the layout before: route to this recording (`None`: Rerun showed none, e.g. the
    /// start page, and still shows none: nothing to do).
    Restore(Option<R>),
    /// Rerun shows another recording while the code is shown (picked in Sources): that one stays,
    /// and the code page is closed.
    Interrupted,
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum State<R> {
    Off,
    /// Routed to the welcome page; Rerun still shows `saved` until that takes effect.
    Entering { saved: Option<R> },
    On { saved: Option<R> },
}

#[derive(Clone, Debug)]
pub struct CodeLayout<R> {
    state: State<R>,
}

impl<R> Default for CodeLayout<R> {
    fn default() -> Self {
        Self { state: State::Off }
    }
}

impl<R: Clone + PartialEq> CodeLayout<R> {
    /// The code is shown (or about to be): no 3D view, no panes around it.
    pub fn active(&self) -> bool {
        !matches!(self.state, State::Off)
    }

    /// Call once per frame: `wanted`: the code page is open; `shown`: the recording Rerun shows.
    pub fn update(&mut self, wanted: bool, shown: Option<&R>) -> Switch<R> {
        match (std::mem::replace(&mut self.state, State::Off), wanted) {
            (State::Off, false) => Switch::None,
            (State::Off, true) => {
                let saved = shown.cloned();
                self.state = if saved.is_some() { State::Entering { saved } } else { State::On { saved } };
                Switch::ShowCode
            }
            (State::Entering { saved } | State::On { saved }, false) => Switch::Restore(saved),
            (State::Entering { saved }, true) => {
                self.state = if shown.is_none() { State::On { saved } } else { State::Entering { saved } };
                Switch::None
            }
            (State::On { saved }, true) => {
                if shown.is_some() {
                    Switch::Interrupted
                } else {
                    self.state = State::On { saved };
                    Switch::None
                }
            }
        }
    }

    /// A tool replaced the recording to go back to (a new preview of the same tool) while the code
    /// is shown: go back to that one instead of routing to it now.
    pub fn retarget(&mut self, recording: R) {
        if let State::Entering { saved } | State::On { saved } = &mut self.state {
            *saved = Some(recording);
        }
    }

    /// Recording `old` was replaced by `new` (a scene rebuilt into a fresh recording, a restarted
    /// live scene) and is closed: if the code page is to go back to `old`, it goes back to `new`.
    pub fn replace(&mut self, old: &R, new: R) {
        if let State::Entering { saved } | State::On { saved } = &mut self.state
            && saved.as_ref() == Some(old)
        {
            *saved = Some(new);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_the_3d_view_and_back() {
        let mut layout = CodeLayout::<&str>::default();
        assert_eq!(layout.update(false, Some(&"fitter")), Switch::None);
        assert!(!layout.active());
        assert_eq!(layout.update(true, Some(&"fitter")), Switch::ShowCode);
        assert!(layout.active());
        // Rerun shows the fitter for a frame more: not an interruption.
        assert_eq!(layout.update(true, Some(&"fitter")), Switch::None);
        assert_eq!(layout.update(true, None), Switch::None);
        assert_eq!(layout.update(true, None), Switch::None);
        assert!(layout.active());
        assert_eq!(layout.update(false, None), Switch::Restore(Some("fitter")), "the same recording again");
        assert!(!layout.active());
        assert_eq!(layout.update(false, Some(&"fitter")), Switch::None);
    }

    #[test]
    fn from_a_page_without_recording() {
        let mut layout = CodeLayout::<&str>::default();
        assert_eq!(layout.update(true, None), Switch::ShowCode);
        assert_eq!(layout.update(true, None), Switch::None);
        assert_eq!(layout.update(false, None), Switch::Restore(None));
        assert!(!layout.active());
    }

    #[test]
    fn closed_before_the_route_took_effect() {
        let mut layout = CodeLayout::<&str>::default();
        layout.update(true, Some(&"cell"));
        assert_eq!(layout.update(false, Some(&"cell")), Switch::Restore(Some("cell")));
    }

    #[test]
    fn another_recording_picked_while_the_code_is_shown() {
        let mut layout = CodeLayout::<&str>::default();
        layout.update(true, Some(&"fitter"));
        layout.update(true, None);
        assert_eq!(layout.update(true, Some(&"dresser")), Switch::Interrupted);
        assert!(!layout.active());
        // The app closes the code page: nothing more to restore.
        assert_eq!(layout.update(false, Some(&"dresser")), Switch::None);
    }

    #[test]
    fn a_replaced_preview_is_restored_instead() {
        let mut layout = CodeLayout::<&str>::default();
        layout.update(true, Some(&"fitter_1"));
        layout.update(true, None);
        layout.retarget("fitter_2");
        assert_eq!(layout.update(false, None), Switch::Restore(Some("fitter_2")));
        layout.retarget("fitter_3");
        assert_eq!(layout.update(false, None), Switch::None, "off: nothing to retarget");
    }

    #[test]
    fn a_closed_recording_is_restored_as_its_successor() {
        let mut layout = CodeLayout::<&str>::default();
        layout.replace(&"cell_1", "cell_2");
        assert!(!layout.active(), "off: nothing to replace");
        layout.update(true, Some(&"cell_1"));
        layout.update(true, None);
        layout.replace(&"fitter_1", "fitter_2");
        layout.replace(&"cell_1", "cell_2");
        assert_eq!(layout.update(false, None), Switch::Restore(Some("cell_2")));
    }
}
