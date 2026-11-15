//! Editing values from the inspector: a port gets the widget its value calls for (and can be
//! typed instead), a main-blackboard entry is edited as JSON, and big values are summed up in
//! one cheap line.
//!
//! Text being typed lives in the panel's drafts until it is committed (Enter, or leaving the
//! field), so a half-typed value never reaches the executor. Edits never re-tick by themselves
//! ([`Executor::set_port_value`](bt_exec::Executor::set_port_value)): a node sees them the
//! next time it reads the value.

use std::collections::HashMap;
use std::ops::RangeInclusive;

use bt_core::Uid;
use bt_core::model::{Port, blackboard_pointer};
use bt_exec::script::{display, json_num};
use egui::{Id, RichText, Ui};
use serde_json::Value;

use crate::panel::Action;

/// Which text field a draft belongs to.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub(crate) enum DraftKey {
    /// A port's text field.
    Port(Uid, String),
    /// A main-blackboard entry's editor; it is open while its draft exists.
    Blackboard(String),
}

/// Text being typed, by field.
pub(crate) type Drafts = HashMap<DraftKey, String>;

/// Array elements (or object entries) a summary writes out, at most.
const INLINE_ITEMS: usize = 8;
/// Array elements a hover lists, at most.
const HOVER_ITEMS: usize = 5;
/// How wide a port's text field is.
const FIELD_WIDTH: f32 = 150.0;

/// `text` cut to `max` characters, with `…` when cut. Only looks at the first `max` characters.
pub(crate) fn short(text: &str, max: usize) -> String {
    match text.char_indices().nth(max) {
        None => text.to_string(),
        Some((end, _)) => format!("{}…", &text[..end]),
    }
}

/// A number, bool, null or short string.
fn small(value: &Value, max: usize) -> bool {
    match value {
        Value::String(s) => s.len() <= max,
        Value::Array(_) | Value::Object(_) => false,
        _ => true,
    }
}

/// `value` in one line of about `max` characters, without serialising anything big: a long
/// or nested array reads `array[47]`, a big object lists its keys.
pub(crate) fn summary(value: &Value, max: usize) -> String {
    match value {
        Value::String(s) => short(s, max),
        Value::Array(items) => {
            if items.len() <= INLINE_ITEMS && items.iter().all(|v| small(v, max)) {
                let text = value.to_string();
                if text.chars().count() <= max {
                    return text;
                }
            }
            format!("array[{}]", items.len())
        }
        Value::Object(map) => {
            if map.len() <= INLINE_ITEMS && map.values().all(|v| small(v, max)) {
                let text = value.to_string();
                if text.chars().count() <= max {
                    return text;
                }
            }
            // The keys, as many as fit.
            let mut keys = String::new();
            for key in map.keys() {
                if keys.len() > max {
                    break;
                }
                if !keys.is_empty() {
                    keys.push_str(", ");
                }
                keys.push_str(key);
            }
            short(&format!("{{{keys}}}"), max)
        }
        other => other.to_string(),
    }
}

/// A trajectory waypoint (`{positions, time_s, phase}`) as `t = … s · [joints] · phase`.
fn waypoint(value: &Value) -> Option<String> {
    let positions = value.get("positions")?.as_array()?;
    let t = value.get("time_s")?.as_f64()?;
    let joints: Vec<String> = positions
        .iter()
        .map(|q| {
            q.as_f64()
                .map_or_else(|| "?".to_string(), |q| format!("{q:.3}"))
        })
        .collect();
    let phase = value
        .get("phase")
        .and_then(Value::as_str)
        .map(|p| format!(" · {p}"))
        .unwrap_or_default();
    Some(format!("t = {t:.3} s · [{}]{phase}", joints.join(", ")))
}

/// The hover for a value: the first few elements of an array (waypoints as time, joints and
/// phase), or the value itself, cut short. Only built while hovered.
pub(crate) fn hover(ui: &mut Ui, value: &Value) {
    match value {
        Value::Array(items) => {
            ui.label(format!("{} elements", items.len()));
            for item in items.iter().take(HOVER_ITEMS) {
                let line = waypoint(item).unwrap_or_else(|| summary(item, 120));
                ui.label(RichText::new(line).monospace().small());
            }
            if items.len() > HOVER_ITEMS {
                ui.label(format!("… {} more", items.len() - HOVER_ITEMS));
            }
        }
        other => {
            ui.label(RichText::new(short(&display(other), 2000)).monospace());
        }
    }
}

/// What the blackboard editor's text means: JSON, or bare text (a word that is not JSON) as a
/// string. Text that looks like JSON but does not parse is an error, so a typo in an array is
/// not stored as a string.
pub(crate) fn parse_edit(text: &str) -> Result<Value, String> {
    let text = text.trim();
    if text.is_empty() {
        return Err("empty: type a JSON value, or revert with ↺".to_string());
    }
    match serde_json::from_str(text) {
        Ok(value) => Ok(value),
        Err(e) if looks_like_json(text) => Err(format!("invalid JSON: {e}")),
        Err(_) => Ok(Value::String(text.to_string())),
    }
}

/// Starts like a JSON array, object, string or number.
fn looks_like_json(text: &str) -> bool {
    text.starts_with(|c: char| matches!(c, '[' | '{' | '"' | '-' | '+' | '.') || c.is_ascii_digit())
}

/// The editor's starting text for `value`: bare text for a string that reads back as itself,
/// JSON otherwise (an array one element per line).
pub(crate) fn draft_of(value: &Value) -> String {
    match value {
        Value::String(s) if parse_edit(s).as_ref() == Ok(value) => s.clone(),
        Value::Array(items) if !items.is_empty() => {
            let lines: Vec<String> = items.iter().map(Value::to_string).collect();
            format!("[\n  {}\n]", lines.join(",\n  "))
        }
        other => serde_json::to_string_pretty(other).unwrap_or_default(),
    }
}

/// What a port's value looks like, and so which widget edits it.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Kind {
    Bool(bool),
    Integer(i64),
    Number(f64),
    Text,
}

/// Ports whose numbers are fractional even when written as integers.
fn fractional(port: &str) -> bool {
    port == "velocity_scale" || port.ends_with("_s")
}

fn kind(port: &str, value: Option<&str>) -> Kind {
    let Some(value) = value.map(str::trim) else {
        return Kind::Text;
    };
    if blackboard_pointer(value).is_some() {
        return Kind::Text;
    }
    match value {
        "true" | "True" | "TRUE" => Kind::Bool(true),
        "false" | "False" | "FALSE" => Kind::Bool(false),
        _ => match value.parse::<i64>() {
            Ok(i) if !fractional(port) => Kind::Integer(i),
            _ => value
                .parse::<f64>()
                .ok()
                .filter(|x| x.is_finite())
                .map_or(Kind::Text, Kind::Number),
        },
    }
}

/// The range and drag speed for a number port, by its name.
fn number_range(port: &str) -> (RangeInclusive<f64>, f64) {
    if port == "velocity_scale" {
        (0.01..=10.0, 0.01)
    } else if port.ends_with("_s") {
        (0.0..=f64::INFINITY, 0.05)
    } else if port.ends_with("msec") {
        (0.0..=f64::INFINITY, 10.0)
    } else {
        (f64::NEG_INFINITY..=f64::INFINITY, 0.1)
    }
}

/// `v` as port text, keeping the decimal point of the value it replaces (`1.0` stays `1.0`, so
/// dragging back to the file's value is no edit).
fn number_text(v: f64, like: Option<&str>) -> String {
    if v.fract() == 0.0 && v.abs() < 1e15 && like.is_some_and(|l| l.contains('.')) {
        format!("{v:.1}")
    } else {
        display(&json_num(v))
    }
}

/// The widget for one port's value, by what the value looks like: a checkbox for a boolean, a
/// drag value for a number (`velocity_scale` within 0.01..=10, `*_s` and `*msec` not below 0),
/// and a text field otherwise, `{key}` pointers included. A boolean or a number can be typed
/// instead (right-click), to make it a `{key}` pointer, say. The text is committed on Enter or
/// when the field loses focus; Escape drops it. An unset port shows `default`.
pub(crate) fn port_editor(
    ui: &mut Ui,
    uid: Uid,
    port: &Port,
    default: Option<&str>,
    drafts: &mut Drafts,
    actions: &mut Vec<Action>,
) {
    let current = port.value.as_deref();
    let shown = current.or(default);
    let set = |value: String| Action::SetPort {
        uid,
        port: port.name.clone(),
        value: Some(value),
    };
    let key = DraftKey::Port(uid, port.name.clone());
    let id = ui.make_persistent_id(&key);
    // Text being typed keeps its field, whatever it looks like.
    let kind = if drafts.contains_key(&key) {
        Kind::Text
    } else {
        kind(&port.name, shown)
    };
    let widget = match kind {
        Kind::Bool(was) => {
            let mut on = was;
            let response = ui.checkbox(&mut on, "");
            if response.changed() && on != was {
                actions.push(set(on.to_string()));
            }
            response
        }
        Kind::Integer(was) => {
            let (range, speed) = number_range(&port.name);
            let mut v = was;
            let range = (*range.start()).max(i64::MIN as f64) as i64
                ..=(*range.end()).min(i64::MAX as f64) as i64;
            let drag = egui::DragValue::new(&mut v)
                .range(range)
                .speed(speed)
                .clamp_existing_to_range(false);
            let response = ui.add(drag);
            if response.changed() && v != was {
                actions.push(set(v.to_string()));
            }
            response
        }
        Kind::Number(was) => {
            let (range, speed) = number_range(&port.name);
            let mut v = was;
            let drag = egui::DragValue::new(&mut v)
                .range(range)
                .speed(speed)
                .max_decimals(4)
                .clamp_existing_to_range(false);
            let response = ui.add(drag);
            if response.changed() && v != was {
                actions.push(set(number_text(v, shown)));
            }
            response
        }
        Kind::Text => {
            if let Some(text) = text_field(ui, id, &key, current.unwrap_or(""), default, drafts) {
                actions.push(set(text));
            }
            return;
        }
    };
    let mut typing = false;
    widget
        .on_hover_text("Right-click to type a value instead, e.g. a {key} pointer")
        .context_menu(|ui| {
            if ui.button("Type a value…").clicked() {
                typing = true;
                ui.close();
            }
        });
    if typing {
        // The field shows from the next frame on; it keeps the focus asked for until then.
        drafts.insert(key, current.unwrap_or("").to_string());
        ui.memory_mut(|m| m.request_focus(id));
    }
}

/// A one-line text field (widget `id`) showing `current` until it is focused; the text typed
/// while focused stays in `drafts`. The committed text, if it differs from `current`.
fn text_field(
    ui: &mut Ui,
    id: Id,
    key: &DraftKey,
    current: &str,
    hint: Option<&str>,
    drafts: &mut Drafts,
) -> Option<String> {
    let mut text = drafts
        .get(key)
        .cloned()
        .unwrap_or_else(|| current.to_string());
    let mut field = egui::TextEdit::singleline(&mut text)
        .id(id)
        .font(egui::TextStyle::Monospace);
    if let Some(hint) = hint {
        field = field.hint_text(hint.to_string());
    }
    // Its own width: in a grid, the available width is the column's from the last frame.
    let response = ui.add_sized([FIELD_WIDTH, ui.spacing().interact_size.y], field);
    if response.lost_focus() {
        drafts.remove(key);
        let escaped = ui.input(|i| i.key_pressed(egui::Key::Escape));
        return (!escaped && text != current).then_some(text);
    }
    if response.has_focus() {
        drafts.insert(key.clone(), text);
    } else {
        drafts.remove(key);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn summaries_stay_short_and_cheap() {
        assert_eq!(summary(&json!(1.5), 40), "1.5");
        assert_eq!(summary(&json!(true), 40), "true");
        assert_eq!(summary(&json!(null), 40), "null");
        assert_eq!(summary(&json!("robot1"), 40), "robot1");
        assert_eq!(summary(&json!("abcdefgh"), 4), "abcd…");
        assert_eq!(summary(&json!([]), 40), "[]");
        assert_eq!(summary(&json!([1, 2, 3]), 40), "[1,2,3]");
        // Too long to write out, or nested: just how many.
        assert_eq!(summary(&json!([1, 2, 3]), 4), "array[3]");
        let waypoint = json!({"positions": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], "time_s": 0.5});
        let trajectory = Value::Array(vec![waypoint.clone(); 47]);
        assert_eq!(summary(&trajectory, 40), "array[47]");
        assert_eq!(summary(&Value::Array(vec![json!(0); 9]), 40), "array[9]");
        // Small objects are written out, others list their keys.
        assert_eq!(summary(&json!({"a": 1}), 40), r#"{"a":1}"#);
        assert_eq!(summary(&waypoint, 40), "{positions, time_s}");
        assert_eq!(summary(&waypoint, 5), "{posi…");
    }

    #[test]
    fn hovers_list_a_few_elements() {
        use egui::accesskit::Role;
        let positions = json!([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]);
        let waypoint = |t: f64| json!({"positions": positions, "time_s": t, "phase": "weld"});
        let trajectory = Value::Array((0..47).map(|i| waypoint(f64::from(i) * 0.5)).collect());
        let lines: Vec<String> = crate::testing::draw(|ui| hover(ui, &trajectory))
            .into_iter()
            .filter(|(role, _)| *role == Role::Label)
            .map(|(_, text)| text)
            .collect();
        let waypoints: Vec<&String> = lines.iter().filter(|l| l.starts_with("t = ")).collect();
        assert_eq!(waypoints.len(), HOVER_ITEMS, "{lines:?}");
        assert_eq!(
            waypoints[1],
            "t = 0.500 s · [0.100, 0.200, 0.300, 0.400, 0.500, 0.600] · weld"
        );
        assert!(lines.contains(&"47 elements".to_string()), "{lines:?}");
        assert!(lines.contains(&"… 42 more".to_string()), "{lines:?}");

        // Other elements are summed up; a short array lists them all.
        let lines: Vec<String> = crate::testing::draw(|ui| hover(ui, &json!([1, [2, 3], "x"])))
            .into_iter()
            .map(|(_, text)| text)
            .collect();
        for line in ["3 elements", "1", "[2,3]", "x"] {
            assert!(lines.contains(&line.to_string()), "{line}: {lines:?}");
        }
        assert!(!lines.iter().any(|l| l.contains("more")), "{lines:?}");
    }

    #[test]
    fn short_counts_characters() {
        assert_eq!(short("äöü", 3), "äöü");
        assert_eq!(short("äöüß", 3), "äöü…");
        assert_eq!(short("", 0), "");
    }

    #[test]
    fn edits_parse_as_json_or_bare_text() {
        assert_eq!(parse_edit("42"), Ok(json!(42)));
        assert_eq!(parse_edit(" -0.5 "), Ok(json!(-0.5)));
        assert_eq!(parse_edit("true"), Ok(json!(true)));
        assert_eq!(parse_edit("null"), Ok(json!(null)));
        assert_eq!(parse_edit(r#"{"a": [1, 2]}"#), Ok(json!({"a": [1, 2]})));
        assert_eq!(parse_edit(r#""42""#), Ok(json!("42")));
        assert_eq!(parse_edit("estop_clear"), Ok(json!("estop_clear")));
        assert_eq!(parse_edit("two words"), Ok(json!("two words")));
        for broken in ["[1, 2", "{\"a\": }", "\"open", "1.2.3", "-x"] {
            let e = parse_edit(broken).expect_err(broken);
            assert!(e.starts_with("invalid JSON"), "{broken}: {e}");
        }
        assert!(parse_edit("  ").is_err());
    }

    #[test]
    fn drafts_read_back_as_the_value() {
        for value in [
            json!("estop_clear"),
            json!("42"),
            json!(" padded "),
            json!(1.25),
            json!([]),
            json!([{"positions": [0.0, 1.0], "time_s": 0.0}, {"positions": [1.0], "time_s": 2.0}]),
            json!({"joint_positions": [0.1, 0.0]}),
        ] {
            let draft = draft_of(&value);
            let back = parse_edit(&draft).unwrap();
            // Bare text loses surrounding blanks, so a padded string is quoted instead.
            assert_eq!(back, value, "{draft}");
        }
        assert_eq!(draft_of(&json!("estop_clear")), "estop_clear");
        assert_eq!(draft_of(&json!("42")), "\"42\"");
    }

    #[test]
    fn ports_get_the_widget_their_value_calls_for() {
        assert_eq!(kind("has_interlock", Some("false")), Kind::Bool(false));
        assert_eq!(kind("has_interlock", Some("True")), Kind::Bool(true));
        assert_eq!(kind("success_count", Some("-1")), Kind::Integer(-1));
        assert_eq!(kind("velocity_scale", Some("1")), Kind::Number(1.0));
        assert_eq!(kind("hold_duration_s", Some("0.5")), Kind::Number(0.5));
        assert_eq!(kind("trajectory", Some("{traj_W01}")), Kind::Text);
        assert_eq!(kind("robot_ns", Some("robot1")), Kind::Text);
        assert_eq!(kind("x", Some("inf")), Kind::Text);
        assert_eq!(kind("x", None), Kind::Text);
        assert_eq!(number_text(2.0, Some("1.0")), "2.0");
        assert_eq!(number_text(2.0, Some("1")), "2");
        assert_eq!(number_text(0.25, Some("1.0")), "0.25");
    }
}
