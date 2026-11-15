//! FNV-1a 64 (plan fingerprint and PLAN_ID).

pub fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// JSON with object keys sorted recursively, independent of serde_json's `preserve_order`
/// feature (which another crate in the dependency graph may enable).
pub fn canonical_json(v: &serde_json::Value) -> String {
    use serde_json::Value;
    match v {
        Value::Object(m) => {
            let mut keys: Vec<&String> = m.keys().collect();
            keys.sort();
            let items: Vec<String> = keys
                .into_iter()
                .map(|k| format!("{}:{}", Value::String(k.clone()), canonical_json(&m[k])))
                .collect();
            format!("{{{}}}", items.join(","))
        }
        Value::Array(a) => format!(
            "[{}]",
            a.iter().map(canonical_json).collect::<Vec<_>>().join(",")
        ),
        other => other.to_string(),
    }
}

/// Fold a 64-bit fingerprint to a non-zero 16-bit plan id.
pub fn fold16(h: u64) -> u16 {
    let v = (h ^ (h >> 16) ^ (h >> 32) ^ (h >> 48)) as u16;
    if v == 0 { 1 } else { v }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_json_sorts_keys() {
        let a: serde_json::Value = serde_json::from_str(r#"{"b":1,"a":[{"y":2,"x":1}]}"#).unwrap();
        assert_eq!(canonical_json(&a), r#"{"a":[{"x":1,"y":2}],"b":1}"#);
    }

    #[test]
    fn known_vectors() {
        assert_eq!(fnv1a64(b""), 0xcbf2_9ce4_8422_2325);
        assert_eq!(fnv1a64(b"a"), 0xaf63_dc4c_8601_ec8c);
        assert_ne!(fold16(0), 0);
    }
}
