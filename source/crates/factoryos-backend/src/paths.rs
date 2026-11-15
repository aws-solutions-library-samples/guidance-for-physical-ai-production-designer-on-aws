//! Where the backend finds its files at runtime: everything hangs off one root, `FACTORY_OS_HOME`.
//!
//! ```text
//! $FACTORY_OS_HOME/
//!   bin/factoryos-backend, bin/cell-rerun-server
//!   web/                    the wasm viewer (--web-root)
//!   cells/                  sample cells; `assets/cells/` in a checkout
//!   assets/catalogue/       Robo Dresser catalogue / Robo Fitter assets
//!   umarell/                the agentic planner checkout (deploy/provision.sh)
//!   factoryos.config.json   unless --config / FACTORY_OS_CONFIG says otherwise
//! ```
//!
//! Without `FACTORY_OS_HOME` the root is found from the running binary (an installed release),
//! and for `cargo run` in a checkout it is the repo itself, whose `assets/` holds the same layout.
//! Nothing here depends on where the binary was compiled, so a release works on any host.

use std::path::{Path, PathBuf};

use factoryos_protocol::SampleCell;

/// The root all defaults below resolve against.
pub fn home() -> PathBuf {
    if let Some(home) = std::env::var_os("FACTORY_OS_HOME") {
        return PathBuf::from(home);
    }
    if let Some(home) = std::env::current_exe().ok().and_then(|exe| home_of_binary(&exe)) {
        return home;
    }
    repo_dir()
}

/// The release layout the binary sits in, if any: `<home>/bin/<exe>` (installed), or `<home>/<exe>`
/// next to `web/` (the flat bundles of earlier releases).
fn home_of_binary(exe: &Path) -> Option<PathBuf> {
    let dir = exe.parent()?;
    if dir.file_name().is_some_and(|name| name == "bin")
        && let Some(home) = dir.parent()
        && home.join("web").is_dir()
    {
        return Some(home.to_owned());
    }
    dir.join("web").is_dir().then(|| dir.to_owned())
}

/// This checkout, for `cargo run` (the binary then lives in `target/`, outside any release layout).
fn repo_dir() -> PathBuf {
    let repo = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    std::fs::canonicalize(&repo).unwrap_or(repo)
}

/// The sample cells: `<home>/cells` when installed, `assets/cells` in a checkout.
fn cells_dir() -> PathBuf {
    let home = home();
    let installed = home.join("cells");
    if installed.is_dir() { installed } else { home.join("assets/cells") }
}

pub fn default_config_file() -> PathBuf {
    let file = home().join("factoryos.config.json");
    std::fs::canonicalize(&file).unwrap_or(file)
}

/// `cell-rerun-server`: `FACTORY_OS_CELL_RERUN_SERVER`, else `<home>/bin/`, else the cell workspace's
/// own build in a checkout (`cell/target/release/`).
pub fn cell_server_binary() -> PathBuf {
    if let Some(binary) = std::env::var_os("FACTORY_OS_CELL_RERUN_SERVER") {
        return PathBuf::from(binary);
    }
    let installed = home().join("bin/cell-rerun-server");
    if installed.is_file() {
        return installed;
    }
    repo_dir().join("cell/target/release/cell-rerun-server")
}

/// The `plan2cell` CLI (Control Code Export): `FACTORY_OS_PLAN2CELL`, else next to this binary
/// (`bin/plan2cell` in a release), else `<home>/bin/`, else plan2cell's own build in a checkout
/// (`plan2cell/target/{release,debug}/`, before `PATH`: a stale `cargo install`ed one must not win
/// over the worktree's), else on `PATH`. `None` when there is none.
pub fn plan2cell_binary() -> Option<PathBuf> {
    let name = format!("plan2cell{}", std::env::consts::EXE_SUFFIX);
    if let Some(binary) = std::env::var_os("FACTORY_OS_PLAN2CELL").filter(|b| !b.is_empty()) {
        // As set, even if missing: the error then names it.
        return Some(PathBuf::from(binary));
    }
    let sibling = std::env::current_exe().ok().map(|exe| exe.with_file_name(&name));
    let on_path = std::env::var_os("PATH")
        .map(|path| std::env::split_paths(&path).map(|dir| dir.join(&name)).collect::<Vec<_>>())
        .unwrap_or_default();
    let checkout = ["release", "debug"].map(|profile| repo_dir().join("plan2cell/target").join(profile).join(&name));
    sibling
        .into_iter()
        .chain([home().join("bin").join(&name)])
        .chain(checkout)
        .chain(on_path)
        .find(|candidate| candidate.is_file())
}

/// A sample cell: its name, a note, its folder, its environment's (scene and meshes) and its scene file there
/// (`<cells>/<folder>`, `<cells>/<env>/<scene>`).
pub(crate) type Sample = (&'static str, &'static str, &'static str, &'static str, &'static str);

pub(crate) const WELDING_CELL: Sample =
    ("WeldingCell", "self-contained dressed welding cell", "WeldingCell", "WeldingCell", "scene.yaml");
pub(crate) const WELDING_CELL_2_SMALL: Sample = (
    "welding_cell_2_small",
    "two dressed KR240s, the umarell planner's cell (Agentic Planning)",
    "welding_cell_2_small",
    "welding_cell_2_small",
    "scene.factoryos.yaml",
);

/// The sample cells that are present, in the order they are offered (the first is the default cell).
pub fn sample_cells() -> Vec<SampleCell> {
    let cells = cells_dir();
    let cells = std::fs::canonicalize(&cells).unwrap_or(cells);
    let path = |p: &str| cells.join(p).to_string_lossy().into_owned();
    #[cfg(feature = "internal")]
    let samples = crate::internal::sample_cells();
    #[cfg(not(feature = "internal"))]
    let samples = vec![WELDING_CELL, WELDING_CELL_2_SMALL];
    samples
        .into_iter()
        .map(|(name, note, cell_dir, env_dir, scene)| SampleCell {
            name: name.to_owned(),
            note: note.to_owned(),
            cell_dir: path(cell_dir),
            env_dir: path(env_dir),
            scene: scene.to_owned(),
        })
        // Large sample data is fetched separately, so not every cell is always there.
        .filter(|cell| Path::new(&cell.cell_dir).is_dir() && Path::new(&cell.env_dir).is_dir())
        .collect()
}

/// The umarell multi-robot planner (`multi_robots_agents`) checkout: `<home>/umarell`.
pub fn default_agentic_repo() -> Option<String> {
    let repo = home().join("umarell");
    repo.join("Makefile").is_file().then(|| canonical(repo))
}

/// The Robo Dresser catalogue used when the config leaves it empty: `<home>/assets/catalogue`.
pub fn default_catalogue() -> String {
    canonical(home().join("assets/catalogue"))
}

/// The Robo Dresser / Robo Fitter asset directory used when the config leaves it empty.
pub fn default_assets() -> String {
    default_catalogue()
}

fn canonical(path: PathBuf) -> String {
    std::fs::canonicalize(&path)
        .unwrap_or(path)
        .to_string_lossy()
        .into_owned()
}

#[cfg(test)]
mod tests {
    use super::home_of_binary;
    use std::path::Path;

    #[test]
    fn finds_the_release_layout_from_the_binary() {
        let tmp = std::env::temp_dir().join(format!("factoryos-paths-{}", std::process::id()));
        let installed = tmp.join("installed");
        let flat = tmp.join("flat");
        for dir in [installed.join("bin"), installed.join("web"), flat.join("web")] {
            std::fs::create_dir_all(dir).unwrap();
        }
        assert_eq!(
            home_of_binary(&installed.join("bin/factoryos-backend")).as_deref(),
            Some(installed.as_path())
        );
        assert_eq!(home_of_binary(&flat.join("factoryos-backend")).as_deref(), Some(flat.as_path()));
        assert_eq!(home_of_binary(Path::new("/nonexistent/target/release/factoryos-backend")), None);
        std::fs::remove_dir_all(&tmp).ok();
    }
}
