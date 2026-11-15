//! An OpenAPI document as tools: one per operation, plus how a call becomes its HTTP request.

use std::sync::Arc;

use http::Method;
use rmcp::model::{Tool, ToolAnnotations};
use serde_json::{json, Map, Value};

use crate::{schema, Error};

/// The longest tool description; clients cut longer ones (Claude Code at about 2 KB).
const MAX_DESCRIPTION: usize = 2000;

/// The operations a path item can hold.
const METHODS: [&str; 8] = [
    "get", "put", "post", "delete", "options", "head", "patch", "trace",
];

/// How a tool call becomes an operation's request.
#[derive(Debug)]
pub(crate) struct Operation {
    pub method: Method,
    /// The path template, e.g. `/api/v1/jobs/{id}`.
    pub path: String,
    /// Every tool argument.
    pub args: Vec<Arg>,
    pub body: BodyKind,
}

/// One tool argument.
#[derive(Debug)]
pub(crate) struct Arg {
    pub name: String,
    pub required: bool,
    pub place: Place,
}

/// Where an argument goes in the request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Place {
    /// Replaces `{name}` in the path.
    Path,
    Query,
    /// One property of the flattened JSON body.
    Field,
    /// The whole body: JSON (`body`) or a string (`yaml`, `text`).
    Body,
    /// A multipart text field.
    Part,
    /// A multipart file field, base64 in the arguments.
    File,
}

/// What the request body is made of.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum BodyKind {
    None,
    /// JSON: the `Body` argument, or the `Field` arguments as one object (`{}` if none is given
    /// and the body is required).
    Json {
        required: bool,
    },
    /// The `Body` argument's string, sent as this media type.
    Text {
        media_type: String,
    },
    /// `multipart/form-data` from the `Part` and `File` arguments.
    Multipart,
}

/// The tool and request plan of every operation in `doc`, sorted by tool name. `read_only`:
/// operationIds to mark read-only although their method isn't GET.
pub(crate) fn operations(
    doc: &Value,
    read_only: &[String],
) -> Result<Vec<(Tool, Operation)>, Error> {
    let paths = doc
        .get("paths")
        .and_then(Value::as_object)
        .ok_or_else(|| Error::Spec("no `paths` object".into()))?;
    let mut out = Vec::new();
    for (path, item) in paths {
        let item = deref(doc, item)?;
        for method in METHODS {
            if let Some(op) = item.get(method) {
                out.push(operation(doc, path, method, item, op, read_only)?);
            }
        }
    }
    out.sort_by(|a, b| a.0.name.cmp(&b.0.name));
    if let Some(pair) = out.windows(2).find(|w| w[0].0.name == w[1].0.name) {
        return Err(Error::Spec(format!(
            "two operations are named `{}`",
            pair[0].0.name
        )));
    }
    if let Some(name) = read_only
        .iter()
        .find(|n| !out.iter().any(|(tool, _)| tool.name == n.as_str()))
    {
        return Err(Error::Spec(format!(
            "read_only names `{name}`, which is no operationId"
        )));
    }
    Ok(out)
}

/// A tool name as the Claude API accepts it.
fn valid_name(name: &str) -> bool {
    (1..=64).contains(&name.len())
        && name
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
}

/// The name of an operation without an operationId: `get_api_v1_items_id` for `GET /api/v1/items/{id}`.
fn synthesized_name(method: &str, path: &str) -> String {
    let mut name = method.to_string();
    for word in path.split(|c: char| !c.is_ascii_alphanumeric()) {
        if !word.is_empty() {
            name.push('_');
            name.push_str(word);
        }
    }
    name.chars().take(64).collect()
}

fn operation(
    doc: &Value,
    path: &str,
    method: &str,
    item: &Value,
    op: &Value,
    read_only: &[String],
) -> Result<(Tool, Operation), Error> {
    let name = match op.get("operationId").and_then(Value::as_str) {
        Some(id) => id.to_string(),
        None => synthesized_name(method, path),
    };
    let upper = method.to_ascii_uppercase();
    let fail = |message: String| Error::Spec(format!("`{name}` ({upper} {path}): {message}"));
    let within = |e: Error| fail(message(e));
    if !valid_name(&name) {
        return Err(fail("a tool name must match ^[A-Za-z0-9_-]{1,64}$".into()));
    }
    let mut inputs = Inputs::default();
    for p in parameters(doc, item, op).map_err(within)? {
        let pname = p
            .get("name")
            .and_then(Value::as_str)
            .ok_or_else(|| fail("a parameter has no name".into()))?;
        let place = match p.get("in").and_then(Value::as_str) {
            Some("path") => Place::Path,
            Some("query") => Place::Query,
            other => {
                return Err(fail(format!(
                    "parameter `{pname}` is in {}; only path and query parameters are supported",
                    other.unwrap_or("nowhere")
                )))
            }
        };
        let mut s = match p.get("schema") {
            Some(s) => schema::convert(doc, s).map_err(within)?,
            None => json!({"type": "string"}),
        };
        if let Some(d) = p.get("description").and_then(Value::as_str) {
            describe_schema(&mut s, d);
        }
        let required =
            place == Place::Path || p.get("required").and_then(Value::as_bool).unwrap_or(false);
        inputs.add(pname, s, required, place).map_err(fail)?;
    }
    let body = match op.get("requestBody") {
        Some(rb) => {
            request_body(doc, deref(doc, rb).map_err(within)?, &mut inputs).map_err(fail)?
        }
        None => BodyKind::None,
    };
    check_template(path, &inputs.args).map_err(fail)?;
    // The request must be buildable; any argument value is percent-encoded into the path.
    if let Err(e) = placeholder_path(path).parse::<http::Uri>() {
        return Err(fail(format!("the path is no valid URI: {e}")));
    }
    let method = Method::from_bytes(upper.as_bytes()).map_err(|e| fail(format!("method: {e}")))?;
    let (title, description) = describe(op, &upper, path);
    // Vendor extensions: `x-mcp-read-only` (a POST that only reads) and `x-mcp-destructive` (a
    // change that is hard to undo or acts on the outside world: a client asks before calling it).
    let flag = |key: &str| op.get(key).and_then(Value::as_bool);
    let reads_only = matches!(method, Method::GET | Method::HEAD)
        || read_only.contains(&name)
        || flag("x-mcp-read-only") == Some(true);
    let mut annotations = ToolAnnotations::new()
        .read_only(reads_only)
        .open_world(false);
    if reads_only {
        annotations = annotations.idempotent(true);
    } else if let Some(destructive) = flag("x-mcp-destructive") {
        annotations = annotations.destructive(destructive);
    }
    let mut tool =
        Tool::new(name, description, Arc::new(inputs.schema())).with_annotations(annotations);
    if let Some(title) = title {
        tool = tool.with_title(title);
    }
    Ok((
        tool,
        Operation {
            method,
            path: path.to_string(),
            args: inputs.args,
            body,
        },
    ))
}

/// The tool's arguments as they are collected.
#[derive(Default)]
struct Inputs {
    properties: Map<String, Value>,
    required: Vec<String>,
    args: Vec<Arg>,
}

impl Inputs {
    fn add(
        &mut self,
        name: &str,
        schema: Value,
        required: bool,
        place: Place,
    ) -> Result<(), String> {
        if self.properties.contains_key(name) {
            return Err(format!("two inputs are named `{name}`"));
        }
        self.properties.insert(name.to_string(), schema);
        if required {
            self.required.push(name.to_string());
        }
        self.args.push(Arg {
            name: name.to_string(),
            required,
            place,
        });
        Ok(())
    }

    /// The first of `names` no argument has yet.
    fn free<'a>(&self, names: &[&'a str]) -> Option<&'a str> {
        names
            .iter()
            .copied()
            .find(|n| !self.properties.contains_key(*n))
    }

    /// The `inputSchema`: always an object, with no other keys allowed.
    fn schema(&self) -> Map<String, Value> {
        let mut s = Map::new();
        s.insert("type".into(), "object".into());
        s.insert("properties".into(), Value::Object(self.properties.clone()));
        if !self.required.is_empty() {
            s.insert("required".into(), json!(self.required));
        }
        s.insert("additionalProperties".into(), false.into());
        s
    }
}

/// An error's message, without the `OpenAPI document:` prefix of [`Error::Spec`].
fn message(e: Error) -> String {
    match e {
        Error::Spec(message) => message,
        other => other.to_string(),
    }
}

/// The kinds of request body, best first.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Media {
    Json,
    Multipart,
    Yaml,
    Text,
}

fn media(media_type: &str) -> Option<Media> {
    let mt = media_type
        .split(';')
        .next()
        .unwrap_or_default()
        .trim()
        .to_ascii_lowercase();
    if mt == "application/json" || mt.ends_with("+json") {
        Some(Media::Json)
    } else if mt == "multipart/form-data" {
        Some(Media::Multipart)
    } else if mt.contains("yaml") {
        Some(Media::Yaml)
    } else if mt.starts_with("text/") {
        Some(Media::Text)
    } else {
        None
    }
}

/// Adds the request body's arguments to `inputs`.
fn request_body(doc: &Value, rb: &Value, inputs: &mut Inputs) -> Result<BodyKind, String> {
    let required = rb.get("required").and_then(Value::as_bool).unwrap_or(false);
    let content = rb
        .get("content")
        .and_then(Value::as_object)
        .ok_or_else(|| "the request body has no content".to_string())?;
    let Some((kind, media_type, entry)) = content
        .iter()
        .filter_map(|(mt, entry)| media(mt).map(|kind| (kind, mt, entry)))
        .min_by_key(|(kind, _, _)| *kind)
    else {
        let types: Vec<&str> = content.keys().map(String::as_str).collect();
        return Err(format!(
            "request body {}: only JSON, multipart/form-data and text bodies are supported",
            types.join(", ")
        ));
    };
    let s = match entry.get("schema") {
        Some(s) => schema::convert(doc, s).map_err(message)?,
        None => json!({}),
    };
    let description = rb.get("description").and_then(Value::as_str);
    match kind {
        Media::Json => {
            if let Some(fields) = flattenable(&s, inputs) {
                for (field, fs) in fields {
                    let needed = required && listed(&s, field);
                    inputs.add(field, fs.clone(), needed, Place::Field)?;
                }
            } else {
                let name = inputs
                    .free(&["body", "request_body"])
                    .ok_or_else(|| "no name is left for the body".to_string())?;
                let mut s = s;
                if let Some(d) = description {
                    describe_schema(&mut s, d);
                }
                inputs.add(name, s, required, Place::Body)?;
            }
            Ok(BodyKind::Json { required })
        }
        Media::Multipart => {
            let fields = flattenable(&s, inputs).ok_or_else(|| {
                "a multipart body must be an object whose fields no parameter shares".to_string()
            })?;
            for (field, fs) in fields {
                let needed = required && listed(&s, field);
                if fs.get("contentMediaType").is_some() {
                    inputs.add(field, file_field(fs.clone()), needed, Place::File)?;
                } else {
                    inputs.add(field, fs.clone(), needed, Place::Part)?;
                }
            }
            Ok(BodyKind::Multipart)
        }
        Media::Yaml | Media::Text => {
            let first = if kind == Media::Yaml { "yaml" } else { "text" };
            let name = inputs
                .free(&[first, "body", "request_body"])
                .ok_or_else(|| "no name is left for the body".to_string())?;
            let d = description.map_or_else(
                || format!("The request body ({media_type})."),
                str::to_string,
            );
            inputs.add(
                name,
                json!({"type": "string", "description": d}),
                required,
                Place::Body,
            )?;
            Ok(BodyKind::Text {
                media_type: media_type.clone(),
            })
        }
    }
}

/// The properties of an object schema whose fields can be top-level arguments: a plain object
/// (no composition, no additional properties), none of whose fields is named like a parameter.
fn flattenable<'a>(s: &'a Value, inputs: &Inputs) -> Option<&'a Map<String, Value>> {
    let s = s.as_object()?;
    let fields = s.get("properties")?.as_object()?;
    let object = s.get("type").is_none_or(|t| t == "object");
    let closed = matches!(
        s.get("additionalProperties"),
        None | Some(Value::Bool(false))
    );
    let plain = ["allOf", "anyOf", "oneOf", "not"]
        .iter()
        .all(|k| !s.contains_key(*k));
    let free = fields.keys().all(|f| !inputs.properties.contains_key(f));
    (object && closed && plain && free).then_some(fields)
}

/// Whether `field` is in the object schema's `required` list.
fn listed(s: &Value, field: &str) -> bool {
    s.get("required")
        .and_then(Value::as_array)
        .is_some_and(|r| r.iter().any(|f| f == field))
}

/// A multipart file field: its content travels base64-encoded in the arguments.
fn file_field(mut s: Value) -> Value {
    if let Value::Object(m) = &mut s {
        m.insert("type".into(), "string".into());
        m.insert("contentEncoding".into(), "base64".into());
        let d = match m
            .get("description")
            .and_then(Value::as_str)
            .map(str::trim_end)
        {
            Some(d) if d.ends_with('.') => format!("{d} Base64-encoded."),
            Some(d) if !d.is_empty() => format!("{d}. Base64-encoded."),
            _ => "The file content, base64-encoded.".to_string(),
        };
        m.insert("description".into(), d.into());
    }
    s
}

/// Sets a schema's description (a `true` schema becomes an object for it).
fn describe_schema(s: &mut Value, description: &str) {
    if !s.is_object() {
        *s = json!({});
    }
    s["description"] = description.into();
}

/// An operation's parameters: the path item's, overridden by the operation's own (same name and
/// location).
fn parameters<'a>(doc: &'a Value, item: &'a Value, op: &'a Value) -> Result<Vec<&'a Value>, Error> {
    let key = |p: &Value| (p.get("name").cloned(), p.get("in").cloned());
    let mut out: Vec<&Value> = Vec::new();
    for list in [item.get("parameters"), op.get("parameters")]
        .into_iter()
        .flatten()
    {
        let list = list
            .as_array()
            .ok_or_else(|| Error::Spec("`parameters` must be a list".into()))?;
        for p in list {
            let p = deref(doc, p)?;
            out.retain(|q| key(q) != key(p));
            out.push(p);
        }
    }
    Ok(out)
}

/// `value`, or what its `$ref` points at.
fn deref<'a>(doc: &'a Value, value: &'a Value) -> Result<&'a Value, Error> {
    let mut v = value;
    for _ in 0..16 {
        match v.get("$ref").and_then(Value::as_str) {
            Some(r) => v = schema::resolve(doc, r)?,
            None => return Ok(v),
        }
    }
    Err(Error::Spec("a $ref chain is too long".into()))
}

/// The `{name}`s of a path template.
fn placeholders(path: &str) -> impl Iterator<Item = &str> {
    path.split('{')
        .skip(1)
        .filter_map(|s| s.split_once('}').map(|(name, _)| name))
}

/// The path template with every `{name}` replaced by `x`.
fn placeholder_path(path: &str) -> String {
    let mut out = path.to_string();
    for name in placeholders(path) {
        out = out.replace(&format!("{{{name}}}"), "x");
    }
    out
}

/// Every `{name}` in the path has a path parameter, and every path parameter a `{name}`.
fn check_template(path: &str, args: &[Arg]) -> Result<(), String> {
    for name in placeholders(path) {
        if !args
            .iter()
            .any(|a| a.place == Place::Path && a.name == name)
        {
            return Err(format!("`{{{name}}}` has no path parameter"));
        }
    }
    for a in args.iter().filter(|a| a.place == Place::Path) {
        if !placeholders(path).any(|n| n == a.name) {
            return Err(format!("path parameter `{}` is not in the path", a.name));
        }
    }
    Ok(())
}

/// The tool's title (the summary) and description: the summary, the description and an
/// `HTTP: METHOD /path` footer, cut to [`MAX_DESCRIPTION`] characters (the footer is kept).
fn describe(op: &Value, method: &str, path: &str) -> (Option<String>, String) {
    let text = |key: &str| {
        op.get(key)
            .and_then(Value::as_str)
            .map(str::trim)
            .filter(|s| !s.is_empty())
    };
    let summary = text("summary");
    let mut head = summary.unwrap_or_default().to_string();
    if let Some(d) = text("description").filter(|d| Some(*d) != summary) {
        if !head.is_empty() {
            head.push_str("\n\n");
        }
        head.push_str(d);
    }
    let footer = format!("HTTP: {method} {path}");
    let room = MAX_DESCRIPTION.saturating_sub(footer.chars().count() + 2);
    if head.chars().count() > room {
        head = head
            .chars()
            .take(room.saturating_sub(1))
            .collect::<String>()
            + "…";
    }
    let description = if head.is_empty() {
        footer
    } else {
        format!("{head}\n\n{footer}")
    };
    (summary.map(str::to_string), description)
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;
    use crate::testing::spec;

    fn tools() -> Vec<(Tool, Operation)> {
        operations(&spec(), &["validate_yaml".to_string()]).unwrap()
    }

    fn tool<'a>(all: &'a [(Tool, Operation)], name: &str) -> &'a (Tool, Operation) {
        all.iter().find(|(t, _)| t.name == name).unwrap()
    }

    fn input(t: &Tool) -> Value {
        Value::Object(t.input_schema.as_ref().clone())
    }

    #[test]
    fn every_operation_is_a_tool_named_by_its_operation_id() {
        let names: Vec<String> = tools().iter().map(|(t, _)| t.name.to_string()).collect();
        assert_eq!(
            names,
            [
                "broken",
                "export_zip",
                "get_item",
                "get_notes",
                "list_items",
                "rename_item",
                "tag_item",
                "upload_file",
                "validate_yaml"
            ]
        );
        for (t, _) in tools() {
            let s = input(&t);
            assert_eq!(s["type"], "object", "{}", t.name);
            assert_eq!(s["additionalProperties"], false, "{}", t.name);
            assert!(!s.to_string().contains("$ref"), "{}: {s}", t.name);
        }
    }

    #[test]
    fn titles_descriptions_and_hints_come_from_the_operation() {
        let all = tools();
        let (t, _) = tool(&all, "list_items");
        assert_eq!(t.title.as_deref(), Some("List the items."));
        assert_eq!(
            t.description.as_deref(),
            Some("List the items.\n\nEvery item, by id.\n\nHTTP: GET /items")
        );
        assert_eq!(
            input(t),
            json!({"type": "object", "properties": {}, "additionalProperties": false})
        );
        let hints = t.annotations.as_ref().unwrap();
        assert_eq!(hints.read_only_hint, Some(true));
        assert_eq!(hints.idempotent_hint, Some(true));
        assert_eq!(hints.open_world_hint, Some(false));
        // A POST only reads when the options say so.
        let (t, _) = tool(&all, "rename_item");
        assert_eq!(t.annotations.as_ref().unwrap().read_only_hint, Some(false));
        assert_eq!(t.annotations.as_ref().unwrap().open_world_hint, Some(false));
        let (t, _) = tool(&all, "validate_yaml");
        assert_eq!(t.annotations.as_ref().unwrap().read_only_hint, Some(true));
        assert_eq!(t.title, None);
        assert_eq!(t.description.as_deref(), Some("HTTP: POST /validate"));
    }

    #[test]
    fn parameters_and_a_json_object_body_become_top_level_arguments() {
        let all = tools();
        let (t, op) = tool(&all, "get_item");
        assert_eq!(
            input(t),
            json!({
                "type": "object",
                "properties": {
                    "id": {"type": "string", "description": "The item id."},
                    "verbose": {"type": "boolean", "default": false},
                    "tag": {"type": "array", "items": {"type": "string"}}
                },
                "required": ["id"],
                "additionalProperties": false
            })
        );
        assert_eq!(op.body, BodyKind::None);
        let (t, op) = tool(&all, "rename_item");
        let s = input(t);
        assert_eq!(s["required"], json!(["id", "name"]));
        assert_eq!(s["properties"]["note"]["type"], json!(["string", "null"]));
        // The referenced Owner is inlined, and nullable.
        assert_eq!(s["properties"]["owner"]["type"], json!(["object", "null"]));
        assert_eq!(s["properties"]["owner"]["required"], json!(["login"]));
        assert_eq!(op.body, BodyKind::Json { required: true });
        let places: Vec<(&str, Place)> =
            op.args.iter().map(|a| (a.name.as_str(), a.place)).collect();
        assert!(places.contains(&("id", Place::Path)));
        assert!(places.contains(&("name", Place::Field)));
    }

    #[test]
    fn a_body_with_a_parameter_s_field_name_goes_under_body() {
        let all = tools();
        let (t, op) = tool(&all, "tag_item");
        let s = input(t);
        assert_eq!(s["required"], json!(["id", "body"]));
        assert_eq!(s["properties"]["id"], json!({"type": "string"}));
        assert_eq!(
            s["properties"]["body"],
            json!({
                "type": "object",
                "description": "The tag to add.",
                "required": ["id"],
                "properties": {"id": {"type": "string", "description": "The tag."}}
            })
        );
        assert_eq!(op.args[1].place, Place::Body);
    }

    #[test]
    fn a_yaml_body_is_one_string() {
        let all = tools();
        let (t, op) = tool(&all, "validate_yaml");
        assert_eq!(
            input(t)["properties"],
            json!({"yaml": {"type": "string", "description": "The YAML text."}})
        );
        assert_eq!(input(t)["required"], json!(["yaml"]));
        assert_eq!(
            op.body,
            BodyKind::Text {
                media_type: "application/x-yaml".into()
            }
        );
    }

    #[test]
    fn a_multipart_body_is_flattened_with_base64_files() {
        let all = tools();
        let (t, op) = tool(&all, "upload_file");
        let s = input(t);
        assert_eq!(
            s["properties"]["file"],
            json!({
                "type": "string",
                "description": "The file. Base64-encoded.",
                "contentEncoding": "base64",
                "contentMediaType": "application/octet-stream"
            })
        );
        assert_eq!(s["properties"]["filename"], json!({"type": "string"}));
        assert_eq!(op.body, BodyKind::Multipart);
        let file = op.args.iter().find(|a| a.name == "file").unwrap();
        assert_eq!(file.place, Place::File);
        assert!(file.required);
    }

    #[test]
    fn bad_names_and_unsupported_inputs_fail_fast() {
        let with = |path: &str, op: Value| json!({"paths": {path: {"post": op}}});
        let fails = |doc: Value, needle: &str| {
            let err = operations(&doc, &[]).err().unwrap().to_string();
            assert!(err.contains(needle), "{needle:?} not in {err:?}");
        };
        fails(with("/a", json!({"operationId": "a b"})), "tool name");
        fails(
            with("/a", json!({"operationId": "x".repeat(65)})),
            "tool name",
        );
        fails(
            json!({"paths": {"/a": {"get": {"operationId": "same"}, "post": {"operationId": "same"}}}}),
            "two operations are named `same`",
        );
        fails(
            with(
                "/a",
                json!({"operationId": "h", "parameters": [{"name": "X-Id", "in": "header"}]}),
            ),
            "only path and query",
        );
        fails(
            with(
                "/a",
                json!({"operationId": "o", "requestBody": {"content": {"application/octet-stream": {}}}}),
            ),
            "application/octet-stream",
        );
        fails(
            with("/a/{id}", json!({"operationId": "p"})),
            "`{id}` has no path parameter",
        );
        fails(
            with(
                "/a",
                json!({"operationId": "r", "requestBody": {"content": {"application/json": {"schema": {"$ref": "#/nope"}}}}}),
            ),
            "does not resolve",
        );
    }

    #[test]
    fn an_operation_without_an_id_is_named_by_method_and_path() {
        let doc = json!({"paths": {"/api/v1/items/{id}": {"get": {
            "parameters": [{"name": "id", "in": "path", "required": true}]
        }}}});
        let all = operations(&doc, &[]).unwrap();
        assert_eq!(all[0].0.name, "get_api_v1_items_id");
    }

    #[test]
    fn long_descriptions_are_cut_but_keep_the_route() {
        let doc = json!({"paths": {"/long": {"get": {
            "operationId": "long", "summary": "Long.", "description": "x".repeat(5000)
        }}}});
        let all = operations(&doc, &[]).unwrap();
        let d = all[0].0.description.as_deref().unwrap();
        assert_eq!(d.chars().count(), MAX_DESCRIPTION);
        assert!(
            d.ends_with("…\n\nHTTP: GET /long"),
            "{}",
            &d[d.len() - 40..]
        );
    }
}
