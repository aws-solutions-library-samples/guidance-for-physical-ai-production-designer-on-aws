//! The layout of a scene's recordings: one 3D view of the whole scene, the time panel collapsed.
//! The view's eye is the one fitted to the scene ([`super::framing`]) until the user moves it;
//! its reset (double-click) goes back to the latest fit.
//!
//! Rerun never shows this blueprint itself: activating it shows a clone made then
//! (`StoreHub::set_cloned_blueprint_active_for_app`), so what is written to it later reaches the
//! view only through the next activation. Hence the view's query is fixed (everything): what a
//! scene doesn't show (e.g. collision geometry turned off) is not in its recording at all, rather
//! than hidden by a query that would have to change.
//!
//! The eye is the exception: it is written here (the default blueprint, what the view's reset
//! goes back to) and, while the user hasn't moved it, into the active clone too ([`EyeSync`],
//! written by the app with Rerun's `SystemCommand::AppendToStore`).
//!
//! Written as blueprint chunks by hand: the SDK's `rerun::blueprint` (as in `backend.rs`) is
//! native-only, this is also for the web viewer.

use crate::rr::re_chunk::{Chunk, EntityPath, RowId, TimePoint, Timeline};
use crate::rr::re_chunk_store::LatestAtQuery;
use crate::rr::re_entity_db::EntityDb;
use crate::rr::re_log;
use crate::rr::re_log_types::{ApplicationId, BlueprintActivationCommand, LogMsg, SetStoreInfo, StoreId, StoreInfo, StoreKind, StoreSource};
use crate::rr::re_sdk_types::AsComponents;
use crate::rr::re_sdk_types::blueprint::archetypes::{
    ContainerBlueprint, EyeControls3D, TimePanelBlueprint, ViewBlueprint, ViewContents, ViewportBlueprint,
};
use crate::rr::re_sdk_types::blueprint::components::{ContainerKind, IncludedContent, PanelState, QueryExpression, RootContainer};
use crate::rr::re_sdk_types::components::{EntityPath as EntityPathComponent, Position3D, Vector3D};
use crate::rr::re_sdk_types::external::glam::Vec3;
use crate::rr::re_viewer_context::{ContainerId, ViewId, blueprint_timeline};

use super::framing::Eye;

/// Rerun's 3D view (`Spatial3DView`).
const VIEW_CLASS: &str = "3D";

/// The view's query: the whole recording.
pub const VIEW_QUERY: &str = "+ /**";

/// The view's name when the scene gives no title.
const DEFAULT_TITLE: &str = "3D scene";

/// One blueprint per [`super::RerunScene`] (its application), kept in step with the scene.
pub struct Blueprint {
    store: StoreId,
    view: ViewId,
    container: ContainerId,
    /// The `blueprint` timeline's time of the next write (a later write wins).
    time: i64,
    /// The title logged last; `None` before the first [`Blueprint::sync`].
    logged: Option<String>,
    /// The view's eye, as fitted last ([`Self::set_eye`]).
    eye: Option<Eye>,
}

impl Blueprint {
    pub fn new(app_id: &str) -> Self {
        Self {
            store: StoreId::random(StoreKind::Blueprint, ApplicationId::new_or_unknown(app_id)),
            view: ViewId::random(),
            container: ContainerId::random(),
            time: 0,
            logged: None,
            eye: None,
        }
    }

    /// Messages that bring the blueprint to `title`: the first call creates and activates it,
    /// later ones log the view again if the title changed (nothing if it didn't), which the view
    /// shows from the next activation on ([`Self::activate`], a new scene).
    pub fn sync(&mut self, title: &str) -> Vec<LogMsg> {
        let title = if title.is_empty() { DEFAULT_TITLE } else { title };
        let first = self.logged.is_none();
        if self.logged.as_deref() == Some(title) {
            return Vec::new();
        }

        let mut msgs = Vec::new();
        if first {
            msgs.push(LogMsg::SetStoreInfo(SetStoreInfo {
                row_id: *RowId::new(),
                info: StoreInfo::new(self.store.clone(), StoreSource::Other("factoryos scene3d".to_owned())),
            }));
        }
        let view_path = self.view.as_entity_path();
        self.log(
            &mut msgs,
            &view_path.to_string(),
            &ViewBlueprint::new(VIEW_CLASS).with_display_name(title).with_space_origin("/"),
        );
        if first {
            // In the first write only: it is what the active blueprint is cloned from, and what the
            // view's reset goes back to. The whole recording, always (see the module docs).
            self.log(&mut msgs, &format!("{view_path}/ViewContents"), &ViewContents::new([QueryExpression::from(VIEW_QUERY)]));
            if let Some(eye) = self.eye {
                self.log(&mut msgs, &self.eye_entity().to_string(), &eye_controls(&eye));
            }
            let container = ContainerBlueprint::new(ContainerKind::Tabs).with_contents([IncludedContent::from(view_path.to_string())]);
            self.log(&mut msgs, &self.container.as_entity_path().to_string(), &container);
            self.log(&mut msgs, "viewport", &ViewportBlueprint::new().with_root_container(RootContainer(self.container.into())));
            // The timeline is rarely needed here: collapsed, shown with the top bar's bottom-panel toggle.
            self.log(&mut msgs, "time_panel", &TimePanelBlueprint::new().with_state(PanelState::Hidden));
            msgs.push(self.activation());
        }
        self.time += 1;
        self.logged = Some(title.to_owned());
        msgs
    }

    /// The view's eye is `eye` from now on: logged (once the blueprint is, else with its first
    /// write), for the view's reset and the next activation. The active blueprint's is
    /// [`EyeSync`]'s.
    pub fn set_eye(&mut self, eye: Eye) -> Vec<LogMsg> {
        if self.eye == Some(eye) {
            return Vec::new();
        }
        self.eye = Some(eye);
        let mut msgs = Vec::new();
        if self.logged.is_some() {
            self.log(&mut msgs, &self.eye_entity().to_string(), &eye_controls(&eye));
            self.time += 1;
        }
        msgs
    }

    /// The eye as fitted last.
    pub fn eye(&self) -> Option<Eye> {
        self.eye
    }

    /// This blueprint (the application's default one, once activated).
    pub fn store(&self) -> &StoreId {
        &self.store
    }

    /// Where the view's `EyeControls3D` is, in this blueprint and its clones.
    pub fn eye_entity(&self) -> EntityPath {
        self.view.as_entity_path() / EntityPath::from_single_string("EyeControls3D")
    }

    /// The chunk that sets the view's eye to `eye` in another blueprint (the active one), at
    /// `timepoint` (after everything in it, so that it wins).
    pub fn eye_chunk(&self, eye: &Eye, timepoint: TimePoint) -> Option<Chunk> {
        Chunk::builder(self.eye_entity())
            .with_archetype(RowId::new(), timepoint, &eye_controls(eye))
            .build()
            .map_err(|err| re_log::warn!("Failed to build the view's eye: {err}"))
            .ok()
    }

    /// Make the blueprint (as logged so far) the active one again: Rerun clones it, so the view
    /// shows the fitted eye again (see [`super::framing`]), and drops what the user changed in
    /// the active one (the camera they moved). Nothing before the first [`Blueprint::sync`].
    ///
    /// Only for an application whose recording is shown: Rerun switches to the application it
    /// activates a blueprint for.
    pub fn activate(&self) -> Option<LogMsg> {
        self.logged.as_ref().map(|_| self.activation())
    }

    fn activation(&self) -> LogMsg {
        LogMsg::BlueprintActivationCommand(BlueprintActivationCommand {
            blueprint_id: self.store.clone(),
            make_active: true,
            make_default: true,
        })
    }

    fn log(&self, msgs: &mut Vec<LogMsg>, entity: &str, archetype: &dyn AsComponents) {
        let timepoint = TimePoint::default().with(Timeline::new_sequence("blueprint"), self.time);
        let chunk = Chunk::builder(entity).with_archetype(RowId::new(), timepoint, archetype).build();
        match chunk.and_then(|chunk| Ok(chunk.to_arrow_msg()?)) {
            Ok(arrow_msg) => msgs.push(LogMsg::ArrowMsg(self.store.clone(), arrow_msg)),
            Err(err) => re_log::warn!("Failed to build a blueprint chunk for {entity}: {err}"),
        }
    }
}

/// The view's eye controls for `eye`: where it is, what it orbits (Rerun's eye is orbital by
/// default: rotating keeps the look target and the distance to it), Z up.
fn eye_controls(eye: &Eye) -> EyeControls3D {
    EyeControls3D::new()
        .with_position(Position3D::from(eye.position))
        .with_look_target(Position3D::from(eye.look_target))
        .with_eye_up(Vector3D::from(Vec3::Z))
}

/// A view's eye as a blueprint has it (`EyeControls3D`): what is set of it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ViewEye {
    pub position: Option<Vec3>,
    pub look_target: Option<Vec3>,
    pub eye_up: Option<Vec3>,
    /// The entity the eye follows, if any (then the eye is that entity's, not the above).
    pub tracking: Option<String>,
}

impl ViewEye {
    /// The eye at `entity` (a view's `EyeControls3D`) in `blueprint`, at its latest.
    pub fn read(blueprint: &EntityDb, entity: &EntityPath) -> Self {
        let position = EyeControls3D::descriptor_position().component;
        let look_target = EyeControls3D::descriptor_look_target().component;
        let eye_up = EyeControls3D::descriptor_eye_up().component;
        let tracking = EyeControls3D::descriptor_tracking_entity().component;
        let query = LatestAtQuery::latest(blueprint_timeline());
        let results = blueprint.latest_at(&query, entity, [position, look_target, eye_up, tracking]);
        Self {
            position: results.component_mono_quiet::<Position3D>(position).map(Vec3::from),
            look_target: results.component_mono_quiet::<Position3D>(look_target).map(Vec3::from),
            eye_up: results.component_mono_quiet::<Vector3D>(eye_up).map(|v| Vec3::from_array(v.0.0)),
            tracking: results
                .component_mono_quiet::<EntityPathComponent>(tracking)
                .map(|path| path.as_str().to_owned())
                .filter(|path| !path.is_empty()),
        }
    }

    /// Whether this is `eye` (as written by [`eye_controls`]).
    fn is(&self, eye: &Eye) -> bool {
        self.tracking.is_none()
            && self.position.is_some_and(|p| near(p, eye.position))
            && self.look_target.is_some_and(|p| near(p, eye.look_target))
            && self.eye_up.is_none_or(|up| near(up, Vec3::Z))
    }

    /// Whether nothing of the eye is set (Rerun's fallbacks).
    fn is_unset(&self) -> bool {
        self.position.is_none() && self.look_target.is_none() && self.eye_up.is_none() && self.tracking.is_none()
    }
}

fn near(a: Vec3, b: Vec3) -> bool {
    (a - b).abs().max_element() <= 1e-5 * a.abs().max_element().max(b.abs().max_element()).max(1.0)
}

/// Keeps the active blueprint's eye (a clone of [`Blueprint`]'s, see the module docs) on the
/// fitted one until the user moves it: then Rerun has written another eye there, and it stays.
///
/// The user has moved it when the active blueprint's eye is neither the one fitted now nor one
/// written to that blueprint before. A new active blueprint (an activation, a new scene) is
/// a clone of ours, so the user hasn't moved its eye yet; the view's reset (double-click) sets it
/// back to the default blueprint's, the one fitted now.
#[derive(Default)]
pub struct EyeSync {
    /// The active blueprint written to last, and the eyes written to it (the last two).
    written: Option<(StoreId, Eye, Option<Eye>)>,
}

impl EyeSync {
    /// Whether to write `wanted` into the active blueprint `store`, whose eye is `active`.
    pub fn should_write(&mut self, wanted: Eye, store: &StoreId, active: &ViewEye) -> bool {
        let ours = match &self.written {
            Some((written_to, last, before)) if written_to == store => {
                active.is_unset() || active.is(&wanted) || active.is(last) || before.as_ref().is_some_and(|eye| active.is(eye))
            }
            // A clone of the default blueprint: ours (whatever it was fitted to then).
            _ => true,
        };
        if !ours || active.is(&wanted) {
            if ours {
                self.note(store, wanted);
            }
            return false;
        }
        self.note(store, wanted);
        true
    }

    fn note(&mut self, store: &StoreId, eye: Eye) {
        let before = match &self.written {
            Some((written_to, last, _)) if written_to == store && *last != eye => Some(*last),
            Some((written_to, _, before)) if written_to == store => *before,
            _ => None,
        };
        self.written = Some((store.clone(), eye, before));
    }
}

/// Whether Rerun's right panel (its selection panel, toggled by the top bar's right-panel button)
/// is expanded in `blueprint`; `default` where the blueprint has not set it (Rerun's own default
/// follows the window's width).
pub fn right_panel_expanded(blueprint: &EntityDb, default: bool) -> bool {
    use crate::rr::re_sdk_types::blueprint::archetypes::PanelBlueprint;
    let query = LatestAtQuery::latest(blueprint_timeline());
    let component = PanelBlueprint::descriptor_state().component;
    blueprint
        .latest_at_component_quiet::<PanelState>(&EntityPath::from("selection_panel"), &query, component)
        .map_or(default, |(_, state)| state.is_expanded())
}
