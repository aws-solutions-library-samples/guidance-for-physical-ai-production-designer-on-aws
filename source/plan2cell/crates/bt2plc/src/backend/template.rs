//! Tiny template engine for the ST templates (`docs/ST_DIALECT.md` "Template syntax").
//!
//! `{{KEY}}` scalars, `{{#robots}} … {{/robots}}` repeated per robot with `{{r.KEY}}` inside.

use std::collections::BTreeMap;

use crate::error::{Error, Result};

#[derive(Clone, Debug, Default)]
pub struct Vars {
    pub scalars: BTreeMap<String, String>,
    pub robots: Vec<BTreeMap<String, String>>,
}

impl Vars {
    pub fn set(&mut self, k: &str, v: impl ToString) -> &mut Self {
        self.scalars.insert(k.to_string(), v.to_string());
        self
    }
}

pub fn expand(name: &str, text: &str, vars: &Vars) -> Result<String> {
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    const OPEN: &str = "{{#robots}}";
    const CLOSE: &str = "{{/robots}}";
    while let Some(i) = rest.find(OPEN) {
        out.push_str(&scalars(name, &rest[..i], vars)?);
        let after = &rest[i + OPEN.len()..];
        let j = after
            .find(CLOSE)
            .ok_or_else(|| Error::Config(format!("template {name}: unclosed {{{{#robots}}}}")))?;
        let block = &after[..j];
        for r in &vars.robots {
            let mut b = block.to_string();
            for (k, v) in r {
                b = b.replace(&format!("{{{{r.{k}}}}}"), v);
            }
            out.push_str(&scalars(name, &b, vars)?);
        }
        rest = &after[j + CLOSE.len()..];
    }
    out.push_str(&scalars(name, rest, vars)?);
    if out.contains("{{") {
        return Err(Error::Config(format!(
            "template {name}: unexpanded placeholder left"
        )));
    }
    Ok(out)
}

fn scalars(name: &str, text: &str, vars: &Vars) -> Result<String> {
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(i) = rest.find("{{") {
        out.push_str(&rest[..i]);
        let after = &rest[i + 2..];
        let j = after
            .find("}}")
            .ok_or_else(|| Error::Config(format!("template {name}: unclosed placeholder")))?;
        let key = &after[..j];
        if key.starts_with("r.") || key.starts_with('#') || key.starts_with('/') {
            return Err(Error::Config(format!(
                "template {name}: {{{{{key}}}}} outside a robots block"
            )));
        }
        let v = vars
            .scalars
            .get(key)
            .ok_or_else(|| Error::Config(format!("template {name}: unknown placeholder {key}")))?;
        out.push_str(v);
        rest = &after[j + 2..];
    }
    out.push_str(rest);
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expands_scalars_and_robot_blocks() {
        let mut v = Vars::default();
        v.set("N", 2);
        for i in 1..=2 {
            let mut r = BTreeMap::new();
            r.insert("i".to_string(), i.to_string());
            r.insert("tag".to_string(), format!("R{i}"));
            v.robots.push(r);
        }
        let t = "A : ARRAY[1..{{N}}] OF INT;\n{{#robots}}x_{{r.tag}} : BYTE; (* {{r.i}}/{{N}} *)\n{{/robots}}end";
        assert_eq!(
            expand("t", t, &v).unwrap(),
            "A : ARRAY[1..2] OF INT;\nx_R1 : BYTE; (* 1/2 *)\nx_R2 : BYTE; (* 2/2 *)\nend"
        );
        assert!(expand("t", "{{MISSING}}", &v).is_err());
    }
}
