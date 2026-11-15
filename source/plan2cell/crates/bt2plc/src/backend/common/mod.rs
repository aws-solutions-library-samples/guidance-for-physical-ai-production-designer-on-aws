//! Flavour-independent outputs: `cell_plan.json`, `ZONES.md`, `robots/<ns>/program.json`,
//! `docker/docker-compose.robots.yml` + `docker/README.md`.

pub mod cell_plan_json;
pub mod docker;
pub mod program_json;
pub mod zones_md;

use crate::backend::Output;
use crate::config::Config;
use crate::error::{Diagnostics, Result};
use crate::ir::CellPlan;

pub fn render(plan: &CellPlan, cfg: &Config, d: &Diagnostics, out: &mut Output) -> Result<()> {
    out.add("cell_plan.json", cell_plan_json::render(plan, cfg, d))?;
    out.add("ZONES.md", zones_md::render(plan, d))?;
    for (ri, r) in plan.robots.iter().enumerate() {
        out.add(
            format!("robots/{}/program.json", r.ns),
            program_json::render(plan, cfg, ri),
        )?;
    }
    let (compose, readme) = docker::render(plan, cfg);
    out.add("docker/docker-compose.robots.yml", compose)?;
    out.add("docker/README.md", readme)?;
    Ok(())
}
