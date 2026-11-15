//! PLC object model between the plan and the files (one model, several renderings).
//!
//! A [`PlcProject`] is one PLC program (the head PLC or one robot simulator) as a list of
//! CODESYS objects: DUTs, GVLs and POUs with separate declaration and implementation text.
//! The CODESYS split files, `project.json`, the trust rendering and the co-simulation all
//! consume this model.

use serde::Serialize;

use crate::ir::RobotIdx;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ObjKind {
    Dut,
    Gvl,
    FunctionBlock,
    Program,
}

impl ObjKind {
    pub fn folder(self) -> &'static str {
        match self {
            ObjKind::Dut => "DUT",
            ObjKind::Gvl => "GVL",
            ObjKind::FunctionBlock | ObjKind::Program => "POU",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Origin {
    Template,
    Generated,
}

#[derive(Clone, Debug, Serialize)]
pub struct PlcObject {
    pub name: String,
    pub kind: ObjKind,
    pub decl: String,
    pub body: Option<String>,
    pub origin: Origin,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ProjectRole {
    HeadPlc,
    RobotSim,
}

#[derive(Clone, Debug, Serialize)]
pub struct PlcProject {
    /// "head" or the robot ns.
    pub key: String,
    pub role: ProjectRole,
    /// CODESYS project name.
    pub name: String,
    pub robot: Option<RobotIdx>,
    /// In creation order (dependencies first).
    pub objects: Vec<PlcObject>,
    /// Enum type names declared in this project (for renderings that need them).
    pub enums: Vec<String>,
}

impl PlcProject {
    pub fn object(&self, name: &str) -> Option<&PlcObject> {
        self.objects.iter().find(|o| o.name == name)
    }

    /// Directory of the CODESYS sources relative to the output root.
    pub fn source_dir(&self) -> String {
        match self.role {
            ProjectRole::HeadPlc => "codesys/head".into(),
            ProjectRole::RobotSim => format!("codesys/robots/{}", self.key),
        }
    }

    pub fn decl_path(&self, o: &PlcObject) -> String {
        format!("{}/{}/{}.st", self.source_dir(), o.kind.folder(), o.name)
    }

    pub fn body_path(&self, o: &PlcObject) -> Option<String> {
        o.body.as_ref().map(|_| {
            format!(
                "{}/{}/{}_Body.st",
                self.source_dir(),
                o.kind.folder(),
                o.name
            )
        })
    }

    /// Sort objects so that every object comes after the objects it references
    /// (identifier scan; ties keep kind order DUT < GVL < FB < PRG, then name).
    pub fn sort_by_dependencies(&mut self) {
        let names: Vec<String> = self.objects.iter().map(|o| o.name.clone()).collect();
        let deps: Vec<Vec<usize>> = self
            .objects
            .iter()
            .map(|o| {
                let text = format!("{}\n{}", o.decl, o.body.as_deref().unwrap_or(""));
                let stripped = strip_comments(&text);
                let idents = identifiers(&stripped);
                names
                    .iter()
                    .enumerate()
                    .filter(|(_, n)| **n != o.name && idents.contains(n.as_str()))
                    .map(|(i, _)| i)
                    .collect()
            })
            .collect();
        let n = self.objects.len();
        let mut placed = vec![false; n];
        let mut order = Vec::with_capacity(n);
        while order.len() < n {
            let mut ready: Vec<usize> = (0..n)
                .filter(|&i| !placed[i] && deps[i].iter().all(|&d| placed[d]))
                .collect();
            if ready.is_empty() {
                // Cycle (should not happen): fall back to the remaining objects in kind/name order.
                ready = (0..n).filter(|&i| !placed[i]).collect();
            }
            ready.sort_by(|&a, &b| {
                (self.objects[a].kind, &self.objects[a].name)
                    .cmp(&(self.objects[b].kind, &self.objects[b].name))
            });
            let i = ready[0];
            placed[i] = true;
            order.push(i);
        }
        let objs = std::mem::take(&mut self.objects);
        let mut slots: Vec<Option<PlcObject>> = objs.into_iter().map(Some).collect();
        self.objects = order
            .into_iter()
            .map(|i| slots[i].take().unwrap())
            .collect();
    }
}

/// Remove `(* ... *)` comments and string literals (keeps line structure).
pub fn strip_comments(text: &str) -> String {
    let b = text.as_bytes();
    let mut out = String::with_capacity(text.len());
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'(' && b.get(i + 1) == Some(&b'*') {
            i += 2;
            while i < b.len() && !(b[i] == b'*' && b.get(i + 1) == Some(&b')')) {
                if b[i] == b'\n' {
                    out.push('\n');
                }
                i += 1;
            }
            i += 2;
        } else if b[i] == b'\'' {
            i += 1;
            while i < b.len() && b[i] != b'\'' {
                i += if b[i] == b'$' { 2 } else { 1 };
            }
            i += 1;
            out.push_str("''");
        } else {
            out.push(b[i] as char);
            i += 1;
        }
    }
    out
}

pub fn identifiers(text: &str) -> std::collections::BTreeSet<&str> {
    text.split(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
        .filter(|s| !s.is_empty())
        .collect()
}
