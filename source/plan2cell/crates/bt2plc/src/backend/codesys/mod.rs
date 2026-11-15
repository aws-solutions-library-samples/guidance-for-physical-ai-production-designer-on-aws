//! CODESYS V3.5 backend: head PLC + one robot simulator project per robot, split
//! declaration/implementation `.st` files, `project.json` build manifest, IO mapping CSVs,
//! PROJECT.md and MCP_PROMPT.md.

pub mod docs;
pub mod gvl;
pub mod io_csv;
pub mod project_json;
pub mod templates;

use crate::backend::model::{
    ObjKind, Origin, PlcObject, PlcProject, ProjectRole, identifiers, strip_comments,
};
use crate::backend::st_fmt::comment;
use crate::backend::template::{Vars, expand};
use crate::backend::{Backend, Flavour, Output};
use crate::config::Config;
use crate::error::{Diagnostics, Result};
use crate::ir::CellPlan;

pub struct CodesysBackend;

pub const GENERATOR: &str = concat!("bt2plc ", env!("CARGO_PKG_VERSION"));

impl Backend for CodesysBackend {
    fn flavour(&self) -> Flavour {
        Flavour::Codesys
    }

    fn projects(&self, plan: &CellPlan, cfg: &Config) -> Result<Vec<PlcProject>> {
        let mut projects = vec![head_project(plan, cfg)?];
        if cfg.robot_sim.enabled {
            // External robots (a KUKA VC, a real controller) have no simulator project.
            for ri in (0..plan.robots.len()).filter(|&ri| !plan.robots[ri].addr.external) {
                projects.push(device_project(plan, cfg, ri)?);
            }
        }
        Ok(projects)
    }

    fn render(
        &self,
        plan: &CellPlan,
        cfg: &Config,
        projects: &[PlcProject],
        out: &mut Output,
        _d: &mut Diagnostics,
    ) -> Result<()> {
        for p in projects {
            for o in &p.objects {
                out.add(p.decl_path(o), ensure_nl(&o.decl))?;
                if let (Some(path), Some(body)) = (p.body_path(o), &o.body) {
                    out.add(path, ensure_nl(body))?;
                }
            }
            out.add(
                format!("{}/io_mapping.csv", p.source_dir()),
                io_csv::io_mapping_csv(plan, cfg, p),
            )?;
        }
        let manifest = project_json::manifest(plan, cfg, projects);
        out.add(
            "project.json",
            serde_json::to_string_pretty(&manifest).expect("manifest serialises") + "\n",
        )?;
        out.add(
            "PROJECT.md",
            docs::project_md(plan, cfg, projects, &manifest),
        )?;
        out.add(
            "MCP_PROMPT.md",
            docs::mcp_prompt_md(plan, cfg, projects, &manifest),
        )?;
        Ok(())
    }
}

fn ensure_nl(s: &str) -> String {
    if s.ends_with('\n') {
        s.to_string()
    } else {
        format!("{s}\n")
    }
}

/// Project file names: `[A-Za-z0-9_-]` only.
fn file_safe(s: &str) -> String {
    s.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' || c == '-' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

pub fn head_project_name(plan: &CellPlan, cfg: &Config) -> String {
    file_safe(&cfg.codesys.head_project.replace("{cell}", &plan.cell_id))
}

pub fn device_project_name(plan: &CellPlan, cfg: &Config, ri: usize) -> String {
    file_safe(
        &cfg.codesys
            .device_project
            .replace("{cell}", &plan.cell_id)
            .replace("{tag}", &plan.robots[ri].addr.tag),
    )
}

fn head_project(plan: &CellPlan, cfg: &Config) -> Result<PlcProject> {
    let mut v = Vars::default();
    let n_tickets: usize = plan.zones.iter().map(|z| z.tickets.len()).sum();
    let n_preds: usize = plan.ops.iter().map(|o| o.permit_preds.len()).sum();
    v.set("N_ROBOTS", plan.robots.len())
        .set("N_ZONES_DIM", plan.zones.len().max(1))
        .set("N_TICKETS_DIM", n_tickets.max(1))
        .set("N_OPS_DIM", plan.ops.len().max(1))
        .set("N_PREDS_DIM", n_preds.max(1))
        .set("GENERATOR", GENERATOR)
        .set("TREE_ID", comment(&plan.tree_id))
        .set("CELL_ID", comment(&plan.cell_id));
    for (i, r) in plan.robots.iter().enumerate() {
        let mut m = std::collections::BTreeMap::new();
        m.insert("i".into(), (i + 1).to_string());
        m.insert("tag".into(), r.addr.tag.clone());
        m.insert("ns".into(), comment(&r.ns));
        m.insert("station".into(), comment(&r.addr.station));
        m.insert("device_name".into(), r.addr.device_name.clone());
        m.insert("ip".into(), r.addr.ip.to_string());
        v.robots.push(m);
    }
    let mut objects = from_templates(templates::HEAD, &v)?;
    objects.push(gvl::gvl_cell(plan, cfg));
    Ok(finish(PlcProject {
        key: "head".into(),
        role: ProjectRole::HeadPlc,
        name: head_project_name(plan, cfg),
        robot: None,
        objects,
        enums: Vec::new(),
    }))
}

fn device_project(plan: &CellPlan, cfg: &Config, ri: usize) -> Result<PlcProject> {
    let r = &plan.robots[ri];
    let mut v = Vars::default();
    v.set("GENERATOR", GENERATOR)
        .set("TREE_ID", comment(&plan.tree_id))
        .set("CELL_ID", comment(&plan.cell_id))
        .set("ROBOT_TAG", comment(&r.addr.tag))
        .set("ROBOT_NS", comment(&r.ns))
        .set("STATION", comment(&r.addr.station));
    let mut objects = from_templates(templates::DEVICE, &v)?;
    objects.push(gvl::gvl_program(plan, cfg, ri)?);
    Ok(finish(PlcProject {
        key: r.ns.clone(),
        role: ProjectRole::RobotSim,
        name: device_project_name(plan, cfg, ri),
        robot: Some(ri),
        objects,
        enums: Vec::new(),
    }))
}

fn finish(mut p: PlcProject) -> PlcProject {
    p.enums = enum_names(&p.objects);
    p.sort_by_dependencies();
    p
}

fn from_templates(set: &[(&str, &str, &str, Option<&str>)], v: &Vars) -> Result<Vec<PlcObject>> {
    set.iter()
        .map(|(folder, name, decl, body)| {
            let decl = expand(name, decl, v)?;
            let body = body.map(|b| expand(name, b, v)).transpose()?;
            let kind = match *folder {
                "DUT" => ObjKind::Dut,
                "GVL" => ObjKind::Gvl,
                _ => pou_kind(&decl),
            };
            Ok(PlcObject {
                name: name.to_string(),
                kind,
                decl,
                body,
                origin: Origin::Template,
            })
        })
        .collect()
}

fn pou_kind(decl: &str) -> ObjKind {
    let text = strip_comments(decl);
    if identifiers(&text).contains("FUNCTION_BLOCK") {
        ObjKind::FunctionBlock
    } else {
        ObjKind::Program
    }
}

/// Enum types declared in DUTs (`TYPE X : (` ...).
pub fn enum_names(objects: &[PlcObject]) -> Vec<String> {
    let mut out = Vec::new();
    for o in objects.iter().filter(|o| o.kind == ObjKind::Dut) {
        let t = strip_comments(&o.decl);
        let mut rest = t.as_str();
        while let Some(i) = rest.find("TYPE") {
            let after = &rest[i + 4..];
            let before_ok = i == 0
                || !rest.as_bytes()[i - 1].is_ascii_alphanumeric()
                    && rest.as_bytes()[i - 1] != b'_';
            if before_ok && let Some(colon) = after.find(':') {
                let name = after[..colon].trim();
                let tail = after[colon + 1..].trim_start();
                if tail.starts_with('(')
                    && !name.is_empty()
                    && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_')
                {
                    out.push(name.to_string());
                }
            }
            rest = after;
        }
    }
    out.sort();
    out.dedup();
    out
}
