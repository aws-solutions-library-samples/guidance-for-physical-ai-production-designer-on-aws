//! OpenAPI 3.0/3.1 schemas as self-contained JSON Schema (2020-12, MCP's default dialect).
//!
//! - `$ref`s are inlined, and keywords next to a `$ref` override its target's, so a tool's
//!   `inputSchema` holds no references. A recursive reference ends in an any-value schema.
//! - A one-entry `allOf` (utoipa's wrapper for a documented or optional reference) is flattened.
//! - OpenAPI 3.0's `nullable: true` becomes a `[T, "null"]` type union, and `null` joins an `enum`.
//! - OpenAPI-only keywords are converted or dropped: `example` becomes `examples`, 3.0's boolean
//!   `exclusiveMinimum`/`exclusiveMaximum` become numbers, `format: binary`/`byte` become
//!   `contentEncoding: base64`, and formats JSON Schema doesn't define (`int32`, `double`, …),
//!   `discriminator`, `xml`, `externalDocs` and `x-` extensions are dropped.

use serde_json::{json, Map, Value};

use crate::Error;

/// The formats JSON Schema 2020-12 defines; any other `format` is dropped.
const FORMATS: &[&str] = &[
    "date-time",
    "date",
    "time",
    "duration",
    "email",
    "idn-email",
    "hostname",
    "idn-hostname",
    "ipv4",
    "ipv6",
    "uri",
    "uri-reference",
    "iri",
    "iri-reference",
    "uuid",
    "uri-template",
    "json-pointer",
    "relative-json-pointer",
    "regex",
];

/// Keywords whose value is one schema.
const ONE: &[&str] = &[
    "items",
    "additionalProperties",
    "not",
    "contains",
    "propertyNames",
    "if",
    "then",
    "else",
];

/// Keywords whose value is a list of schemas.
const LIST: &[&str] = &["allOf", "anyOf", "oneOf", "prefixItems"];

/// Keywords whose value maps names to schemas.
const MAP: &[&str] = &["properties", "patternProperties", "dependentSchemas"];

/// `schema`, a schema in the OpenAPI document `doc`, as a self-contained JSON Schema.
pub(crate) fn convert(doc: &Value, schema: &Value) -> Result<Value, Error> {
    walk(doc, schema, &mut Vec::new())
}

/// What a local `$ref` (`#/components/…`) points at in `doc`.
pub(crate) fn resolve<'a>(doc: &'a Value, reference: &str) -> Result<&'a Value, Error> {
    let pointer = reference
        .strip_prefix('#')
        .ok_or_else(|| Error::Spec(format!("external $ref {reference} is not supported")))?;
    doc.pointer(pointer)
        .ok_or_else(|| Error::Spec(format!("$ref {reference} does not resolve")))
}

/// `refs`: the references being inlined, outermost first (a repeat is a cycle).
fn walk(doc: &Value, schema: &Value, refs: &mut Vec<String>) -> Result<Value, Error> {
    let Value::Object(s) = schema else {
        // `true` / `false` (3.1), or not a schema at all.
        return Ok(schema.clone());
    };
    let reference = s.get("$ref").and_then(Value::as_str);
    let only = match s.get("allOf") {
        Some(Value::Array(all)) if reference.is_none() && all.len() == 1 => Some(&all[0]),
        _ => None,
    };
    // The base: the reference's target or the single `allOf` entry. The keywords of `s` win.
    let mut out = match (reference, only) {
        (Some(r), _) => inline(doc, r, refs)?,
        (None, Some(only)) => into_map(walk(doc, only, refs)?),
        (None, None) => Map::new(),
    };
    for (key, value) in s {
        match key.as_str() {
            "$ref" | "nullable" | "discriminator" | "xml" | "externalDocs" => {}
            "allOf" if only.is_some() => {}
            k if k.starts_with("x-") => {}
            "example" => {
                if !s.contains_key("examples") {
                    out.insert("examples".into(), json!([value]));
                }
            }
            "format" => match value.as_str() {
                Some("binary") => {
                    out.insert("contentEncoding".into(), "base64".into());
                    out.entry("contentMediaType")
                        .or_insert_with(|| "application/octet-stream".into());
                }
                Some("byte") => {
                    out.insert("contentEncoding".into(), "base64".into());
                }
                Some(f) if FORMATS.contains(&f) => {
                    out.insert(key.clone(), value.clone());
                }
                _ => {}
            },
            k if ONE.contains(&k) => {
                out.insert(key.clone(), walk(doc, value, refs)?);
            }
            k if LIST.contains(&k) => {
                let converted = match value {
                    Value::Array(list) => Value::Array(
                        list.iter()
                            .map(|v| walk(doc, v, refs))
                            .collect::<Result<_, _>>()?,
                    ),
                    other => other.clone(),
                };
                out.insert(key.clone(), converted);
            }
            k if MAP.contains(&k) => {
                let converted = match value {
                    Value::Object(map) => {
                        let mut out = Map::new();
                        for (name, v) in map {
                            out.insert(name.clone(), walk(doc, v, refs)?);
                        }
                        Value::Object(out)
                    }
                    other => other.clone(),
                };
                out.insert(key.clone(), converted);
            }
            _ => {
                out.insert(key.clone(), value.clone());
            }
        }
    }
    // OpenAPI 3.0: `exclusiveMinimum: true` qualifies `minimum`; JSON Schema takes the number.
    for (flag, bound) in [
        ("exclusiveMinimum", "minimum"),
        ("exclusiveMaximum", "maximum"),
    ] {
        if let Some(Value::Bool(exclusive)) = out.get(flag) {
            let exclusive = *exclusive;
            out.remove(flag);
            if exclusive {
                if let Some(bound) = out.remove(bound) {
                    out.insert(flag.into(), bound);
                }
            }
        }
    }
    if s.get("nullable") == Some(&Value::Bool(true)) {
        out = nullable(out);
    }
    Ok(Value::Object(out))
}

/// The converted target of `reference`; a reference already being inlined is a cycle and becomes
/// an any-value schema.
fn inline(
    doc: &Value,
    reference: &str,
    refs: &mut Vec<String>,
) -> Result<Map<String, Value>, Error> {
    if refs.iter().any(|r| r == reference) {
        let mut any = Map::new();
        any.insert(
            "description".into(),
            format!("(recursive: {reference})").into(),
        );
        return Ok(any);
    }
    let target = resolve(doc, reference)?;
    refs.push(reference.to_string());
    let converted = walk(doc, target, refs);
    refs.pop();
    Ok(into_map(converted?))
}

/// A schema as a keyword map (`true` is `{}`, `false` is `{"not": {}}`).
fn into_map(schema: Value) -> Map<String, Value> {
    match schema {
        Value::Object(map) => map,
        Value::Bool(false) => {
            let mut never = Map::new();
            never.insert("not".into(), json!({}));
            never
        }
        _ => Map::new(),
    }
}

/// OpenAPI 3.0's `nullable: true`: `null` joins the type and any `enum`.
fn nullable(mut s: Map<String, Value>) -> Map<String, Value> {
    if let Some(Value::Array(values)) = s.get_mut("enum") {
        if !values.contains(&Value::Null) {
            values.push(Value::Null);
        }
    }
    match s.get("type").cloned() {
        Some(Value::String(t)) => {
            s.insert("type".into(), json!([t, "null"]));
            s
        }
        Some(Value::Array(mut types)) => {
            if !types.iter().any(|t| t == "null") {
                types.push("null".into());
            }
            s.insert("type".into(), Value::Array(types));
            s
        }
        _ => {
            // No type to extend: the schema or null, with the annotations outside.
            let mut outer = Map::new();
            for k in ["title", "description", "default", "examples", "deprecated"] {
                if let Some(v) = s.remove(k) {
                    outer.insert(k.into(), v);
                }
            }
            outer.insert("anyOf".into(), json!([s, {"type": "null"}]));
            outer
        }
    }
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    fn doc() -> Value {
        json!({"components": {"schemas": {
            "Owner": {
                "type": "object",
                "description": "Who owns it.",
                "required": ["login"],
                "properties": {"login": {"type": "string"}}
            },
            "Item": {
                "type": "object",
                "properties": {
                    "owner": {"$ref": "#/components/schemas/Owner"},
                    "parts": {"type": "array", "items": {"$ref": "#/components/schemas/Part"}}
                }
            },
            "Part": {
                "type": "object",
                "properties": {"of": {"$ref": "#/components/schemas/Item"}}
            }
        }}})
    }

    #[test]
    fn refs_are_inlined_and_their_siblings_win() {
        let doc = doc();
        let s = convert(
            &doc,
            &json!({"$ref": "#/components/schemas/Owner", "description": "The owner."}),
        )
        .unwrap();
        assert_eq!(
            s,
            json!({
                "type": "object",
                "description": "The owner.",
                "required": ["login"],
                "properties": {"login": {"type": "string"}}
            })
        );
        // Nested references, and a cycle (Item → Part → Item) cut short.
        let s = convert(&doc, &json!({"$ref": "#/components/schemas/Item"})).unwrap();
        assert_eq!(s["properties"]["owner"]["required"], json!(["login"]));
        let of = &s["properties"]["parts"]["items"]["properties"]["of"];
        assert_eq!(
            *of,
            json!({"description": "(recursive: #/components/schemas/Item)"})
        );
        assert!(!s.to_string().contains("$ref"), "{s}");
    }

    #[test]
    fn unresolvable_and_external_refs_fail() {
        let doc = doc();
        assert!(convert(&doc, &json!({"$ref": "#/components/schemas/Nope"})).is_err());
        assert!(convert(&doc, &json!({"$ref": "other.json#/Owner"})).is_err());
    }

    #[test]
    fn nullable_becomes_a_type_union() {
        let doc = doc();
        let s = convert(&doc, &json!({"type": "string", "nullable": true})).unwrap();
        assert_eq!(s, json!({"type": ["string", "null"]}));
        let s = convert(
            &doc,
            &json!({"type": "string", "enum": ["a", "b"], "nullable": true}),
        )
        .unwrap();
        assert_eq!(
            s,
            json!({"type": ["string", "null"], "enum": ["a", "b", null]})
        );
        // utoipa's Option<Struct>: a one-entry allOf around the reference.
        let s = convert(
            &doc,
            &json!({"allOf": [{"$ref": "#/components/schemas/Owner"}], "nullable": true}),
        )
        .unwrap();
        assert_eq!(s["type"], json!(["object", "null"]));
        assert_eq!(s["properties"]["login"], json!({"type": "string"}));
        assert!(s.get("allOf").is_none());
        // No type to extend.
        let s = convert(
            &doc,
            &json!({"oneOf": [{"type": "string"}, {"type": "integer"}], "description": "d", "nullable": true}),
        )
        .unwrap();
        assert_eq!(
            s,
            json!({"description": "d", "anyOf": [
                {"oneOf": [{"type": "string"}, {"type": "integer"}]},
                {"type": "null"}
            ]})
        );
    }

    #[test]
    fn openapi_only_keywords_are_converted() {
        let doc = doc();
        let s = convert(
            &doc,
            &json!({
                "type": "object",
                "discriminator": {"propertyName": "kind"},
                "x-internal": true,
                "properties": {
                    "count": {"type": "integer", "format": "int32", "minimum": 0,
                              "exclusiveMinimum": true, "example": 3},
                    "file": {"type": "string", "format": "binary"},
                    "digest": {"type": "string", "format": "byte"},
                    "at": {"type": "string", "format": "date-time", "xml": {"name": "at"}}
                }
            }),
        )
        .unwrap();
        assert_eq!(
            s,
            json!({
                "type": "object",
                "properties": {
                    "count": {"type": "integer", "exclusiveMinimum": 0, "examples": [3]},
                    "file": {"type": "string", "contentEncoding": "base64",
                             "contentMediaType": "application/octet-stream"},
                    "digest": {"type": "string", "contentEncoding": "base64"},
                    "at": {"type": "string", "format": "date-time"}
                }
            })
        );
    }
}
