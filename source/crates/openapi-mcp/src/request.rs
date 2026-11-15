//! Tool arguments as the operation's HTTP request.

use base64::alphabet;
use base64::engine::general_purpose::{GeneralPurpose, GeneralPurposeConfig};
use base64::engine::DecodePaddingMode;
use base64::Engine as _;
use percent_encoding::{utf8_percent_encode, AsciiSet, NON_ALPHANUMERIC};
use serde_json::{Map, Value};

use crate::spec::{Arg, BodyKind, Operation, Place};
use crate::Body;

/// Everything but the unreserved characters is encoded, so a value stays one path segment
/// (`a/b` is `a%2Fb`).
const SEGMENT: &AsciiSet = &NON_ALPHANUMERIC
    .remove(b'-')
    .remove(b'.')
    .remove(b'_')
    .remove(b'~');

/// Base64 with or without padding, in the standard or the URL-safe alphabet.
const BASE64: [GeneralPurpose; 2] = [
    GeneralPurpose::new(&alphabet::STANDARD, LENIENT),
    GeneralPurpose::new(&alphabet::URL_SAFE, LENIENT),
];
const LENIENT: GeneralPurposeConfig =
    GeneralPurposeConfig::new().with_decode_padding_mode(DecodePaddingMode::Indifferent);

/// The request for `op` with `args`; `Err` says what is wrong with the arguments.
pub(crate) fn build(
    op: &Operation,
    args: &Map<String, Value>,
) -> Result<http::Request<Body>, String> {
    check(op, args)?;
    let mut path = op.path.clone();
    let mut query = form_urlencoded::Serializer::new(String::new());
    let mut queried = false;
    let mut fields = Map::new();
    let mut whole = None;
    let mut parts = Vec::new();
    for arg in &op.args {
        let Some(value) = args.get(&arg.name) else {
            continue;
        };
        match arg.place {
            Place::Path => {
                let segment = scalar(value)
                    .ok_or_else(|| format!("`{}` must be a string, number or boolean", arg.name))?;
                if matches!(segment.as_str(), "" | "." | "..") {
                    return Err(format!("`{}` must not be empty, `.` or `..`", arg.name));
                }
                let encoded = utf8_percent_encode(&segment, SEGMENT).to_string();
                path = path.replace(&format!("{{{}}}", arg.name), &encoded);
            }
            Place::Query => {
                let values = match value {
                    Value::Null => continue,
                    Value::Array(items) => items.iter().collect(),
                    one => vec![one],
                };
                for v in values {
                    let v = scalar(v).ok_or_else(|| {
                        format!(
                            "`{}` must be a string, number or boolean, or a list of them",
                            arg.name
                        )
                    })?;
                    query.append_pair(&arg.name, &v);
                    queried = true;
                }
            }
            Place::Field => {
                fields.insert(arg.name.clone(), value.clone());
            }
            Place::Body => whole = Some(value),
            Place::Part if value.is_null() => {}
            Place::Part => parts.push(Part {
                name: &arg.name,
                file: false,
                data: match value {
                    Value::String(s) => s.clone().into_bytes(),
                    other => other.to_string().into_bytes(),
                },
            }),
            Place::File if value.is_null() => {}
            Place::File => parts.push(Part {
                name: &arg.name,
                file: true,
                data: base64(arg, value)?,
            }),
        }
    }
    let (content_type, body) = match &op.body {
        BodyKind::None => (None, Vec::new()),
        BodyKind::Json { required } => match whole {
            Some(v) => (Some("application/json".to_string()), json(v)?),
            None if *required || !fields.is_empty() => (
                Some("application/json".to_string()),
                json(&Value::Object(fields))?,
            ),
            None => (None, Vec::new()),
        },
        BodyKind::Text { media_type } => match whole {
            Some(Value::String(s)) => (Some(media_type.clone()), s.clone().into_bytes()),
            Some(_) => {
                let name = op.args.iter().find(|a| a.place == Place::Body);
                return Err(format!(
                    "`{}` must be a string",
                    name.map_or("body", |a| a.name.as_str())
                ));
            }
            None => (None, Vec::new()),
        },
        BodyKind::Multipart => {
            let (content_type, body) = multipart(&parts);
            (Some(content_type), body)
        }
    };
    let uri = if queried {
        format!("{path}?{}", query.finish())
    } else {
        path
    };
    let mut request = http::Request::builder().method(op.method.clone()).uri(uri);
    if let Some(content_type) = content_type {
        request = request.header(http::header::CONTENT_TYPE, content_type);
    }
    request
        .body(Body::from(body))
        .map_err(|e| format!("the request can't be built: {e}"))
}

/// No unknown argument, and every required one.
fn check(op: &Operation, args: &Map<String, Value>) -> Result<(), String> {
    if let Some(unknown) = args.keys().find(|k| !op.args.iter().any(|a| a.name == **k)) {
        return Err(format!(
            "unknown argument `{unknown}`; {}",
            expected(&op.args)
        ));
    }
    if let Some(missing) = op
        .args
        .iter()
        .find(|a| a.required && !args.contains_key(&a.name))
    {
        return Err(format!(
            "missing required argument `{}`; {}",
            missing.name,
            expected(&op.args)
        ));
    }
    Ok(())
}

fn expected(args: &[Arg]) -> String {
    if args.is_empty() {
        return "this tool takes no arguments".into();
    }
    let names: Vec<String> = args
        .iter()
        .map(|a| {
            if a.required {
                format!("{} (required)", a.name)
            } else {
                a.name.clone()
            }
        })
        .collect();
    format!("the arguments are: {}", names.join(", "))
}

/// A string, number or boolean as text.
fn scalar(value: &Value) -> Option<String> {
    match value {
        Value::String(s) => Some(s.clone()),
        Value::Number(n) => Some(n.to_string()),
        Value::Bool(b) => Some(b.to_string()),
        _ => None,
    }
}

fn json(value: &Value) -> Result<Vec<u8>, String> {
    serde_json::to_vec(value).map_err(|e| format!("the body can't be serialised: {e}"))
}

/// A file argument's bytes: base64 (standard or URL-safe, padded or not, whitespace ignored),
/// optionally as a `data:…;base64,` URL.
fn base64(arg: &Arg, value: &Value) -> Result<Vec<u8>, String> {
    let text = value
        .as_str()
        .ok_or_else(|| format!("`{}` must be a base64 string", arg.name))?;
    let text = match text.split_once(";base64,") {
        Some((head, data)) if head.starts_with("data:") => data,
        _ => text,
    };
    let text: String = text.chars().filter(|c| !c.is_ascii_whitespace()).collect();
    let mut last = None;
    for engine in &BASE64 {
        match engine.decode(&text) {
            Ok(bytes) => return Ok(bytes),
            Err(e) => last = Some(e),
        }
    }
    Err(format!(
        "`{}` is not valid base64: {}",
        arg.name,
        last.map_or_else(String::new, |e| e.to_string())
    ))
}

/// One `multipart/form-data` field.
struct Part<'a> {
    name: &'a str,
    file: bool,
    data: Vec<u8>,
}

/// The `Content-Type` and body of a `multipart/form-data` request. A file part is named after its
/// field and sent as `application/octet-stream`.
fn multipart(parts: &[Part]) -> (String, Vec<u8>) {
    let boundary = (0..)
        .map(|n| format!("openapi-mcp-boundary-{n}"))
        .find(|b| !parts.iter().any(|p| contains(&p.data, b.as_bytes())))
        .unwrap_or_default();
    let mut body = Vec::new();
    for part in parts {
        let name = quoted(part.name);
        body.extend_from_slice(format!("--{boundary}\r\n").as_bytes());
        if part.file {
            body.extend_from_slice(
                format!(
                    "Content-Disposition: form-data; name=\"{name}\"; filename=\"{name}\"\r\n\
                     Content-Type: application/octet-stream\r\n\r\n"
                )
                .as_bytes(),
            );
        } else {
            body.extend_from_slice(
                format!("Content-Disposition: form-data; name=\"{name}\"\r\n\r\n").as_bytes(),
            );
        }
        body.extend_from_slice(&part.data);
        body.extend_from_slice(b"\r\n");
    }
    body.extend_from_slice(format!("--{boundary}--\r\n").as_bytes());
    (format!("multipart/form-data; boundary={boundary}"), body)
}

fn contains(haystack: &[u8], needle: &[u8]) -> bool {
    haystack.windows(needle.len()).any(|w| w == needle)
}

/// A field name inside a quoted header parameter, escaped as browsers do.
fn quoted(name: &str) -> String {
    name.replace('"', "%22")
        .replace('\r', "%0D")
        .replace('\n', "%0A")
}

#[cfg(test)]
mod tests {
    use http_body_util::BodyExt as _;
    use serde_json::json;

    use super::*;
    use crate::spec::operations;
    use crate::testing::spec;

    fn op(name: &str) -> Operation {
        operations(&spec(), &[])
            .unwrap()
            .into_iter()
            .find(|(t, _)| t.name == name)
            .unwrap()
            .1
    }

    fn args(v: Value) -> Map<String, Value> {
        let Value::Object(map) = v else { panic!() };
        map
    }

    async fn body(r: http::Request<Body>) -> Vec<u8> {
        r.into_body().collect().await.unwrap().to_bytes().to_vec()
    }

    #[test]
    fn a_path_parameter_is_one_encoded_segment() {
        let get = op("get_item");
        for (id, path) in [
            ("a/b", "/items/a%2Fb"),
            ("a b?#%", "/items/a%20b%3F%23%25"),
            ("ü", "/items/%C3%BC"),
            ("x-1._~", "/items/x-1._~"),
            ("..a", "/items/..a"),
        ] {
            let r = build(&get, &args(json!({"id": id}))).unwrap();
            assert_eq!(r.uri().path(), path, "{id}");
        }
        let r = build(&get, &args(json!({"id": 7}))).unwrap();
        assert_eq!(r.uri(), "/items/7");
    }

    #[test]
    fn dot_segments_and_empty_values_are_refused() {
        let get = op("get_item");
        for id in [
            json!(""),
            json!("."),
            json!(".."),
            json!(null),
            json!({"a": 1}),
        ] {
            assert!(build(&get, &args(json!({"id": id}))).is_err(), "{id}");
        }
    }

    #[test]
    fn query_parameters_are_form_encoded() {
        let r = build(
            &op("get_item"),
            &args(json!({"id": "1", "verbose": true, "tag": ["a&b=c", "d e+f"]})),
        )
        .unwrap();
        assert_eq!(
            r.uri().query(),
            Some("verbose=true&tag=a%26b%3Dc&tag=d+e%2Bf")
        );
        assert_eq!(r.method(), http::Method::GET);
        assert!(r.headers().is_empty());
        // Null is left out.
        let r = build(&op("get_item"), &args(json!({"id": "1", "verbose": null}))).unwrap();
        assert_eq!(r.uri(), "/items/1");
    }

    #[tokio::test]
    async fn body_fields_are_sent_as_one_json_object() {
        let r = build(
            &op("rename_item"),
            &args(json!({"id": "1", "name": "n", "owner": {"login": "me"}})),
        )
        .unwrap();
        assert_eq!(r.method(), http::Method::POST);
        assert_eq!(r.headers()["content-type"], "application/json");
        let sent: Value = serde_json::from_slice(&body(r).await).unwrap();
        assert_eq!(sent, json!({"name": "n", "owner": {"login": "me"}}));
        // Under `body`, the value is the body.
        let r = build(
            &op("tag_item"),
            &args(json!({"id": "1", "body": {"id": "t"}})),
        )
        .unwrap();
        assert_eq!(body(r).await, br#"{"id":"t"}"#);
    }

    #[tokio::test]
    async fn a_text_body_keeps_its_media_type() {
        let validate = op("validate_yaml");
        let r = build(&validate, &args(json!({"yaml": "a: 1\n"}))).unwrap();
        assert_eq!(r.headers()["content-type"], "application/x-yaml");
        assert_eq!(body(r).await, b"a: 1\n");
        assert!(build(&validate, &args(json!({"yaml": 1}))).is_err());
    }

    #[tokio::test]
    async fn a_multipart_body_has_text_and_file_parts() {
        let r = build(
            &op("upload_file"),
            &args(json!({"filename": "plan.obj", "file": "djEK"})),
        )
        .unwrap();
        assert_eq!(
            r.headers()["content-type"],
            "multipart/form-data; boundary=openapi-mcp-boundary-0"
        );
        let sent = String::from_utf8(body(r).await).unwrap();
        // The parts follow the schema's property order, which depends on serde_json's
        // `preserve_order` feature.
        let file = "--openapi-mcp-boundary-0\r\n\
                    Content-Disposition: form-data; name=\"file\"; filename=\"file\"\r\n\
                    Content-Type: application/octet-stream\r\n\r\nv1\n\r\n";
        let filename = "--openapi-mcp-boundary-0\r\n\
                        Content-Disposition: form-data; name=\"filename\"\r\n\r\nplan.obj\r\n";
        assert!(sent.contains(file), "{sent:?}");
        assert!(sent.contains(filename), "{sent:?}");
        assert!(
            sent.ends_with("\r\n--openapi-mcp-boundary-0--\r\n"),
            "{sent:?}"
        );
        assert_eq!(sent.len(), file.len() + filename.len() + 28);
    }

    #[test]
    fn the_boundary_never_occurs_in_a_part() {
        let parts = [Part {
            name: "a",
            file: false,
            data: b"--openapi-mcp-boundary-0".to_vec(),
        }];
        let (content_type, _) = multipart(&parts);
        assert_eq!(
            content_type,
            "multipart/form-data; boundary=openapi-mcp-boundary-1"
        );
    }

    #[test]
    fn base64_comes_in_several_spellings() {
        let file = Arg {
            name: "file".into(),
            required: true,
            place: Place::File,
        };
        for text in [
            "/+8=",
            "/+8",
            "_-8",
            " /+\n8= ",
            "data:application/octet-stream;base64,/+8=",
        ] {
            assert_eq!(
                base64(&file, &json!(text)).unwrap(),
                [0xff, 0xef],
                "{text:?}"
            );
        }
        assert!(base64(&file, &json!("not base64!")).is_err());
        assert!(base64(&file, &json!(12)).is_err());
    }

    #[test]
    fn unknown_and_missing_arguments_are_named() {
        let err = build(&op("get_item"), &args(json!({"id": "1", "colour": "red"})))
            .err()
            .unwrap();
        assert!(err.starts_with("unknown argument `colour`"), "{err}");
        assert!(err.contains("id (required), verbose, tag"), "{err}");
        let err = build(&op("rename_item"), &args(json!({"id": "1"})))
            .err()
            .unwrap();
        assert!(err.starts_with("missing required argument `name`"), "{err}");
        let err = build(&op("list_items"), &args(json!({"x": 1})))
            .err()
            .unwrap();
        assert!(err.ends_with("this tool takes no arguments"), "{err}");
    }
}
