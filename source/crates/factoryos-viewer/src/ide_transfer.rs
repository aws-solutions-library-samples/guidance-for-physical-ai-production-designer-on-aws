//! The Workspace IDE's uploads and downloads, natively and in the browser: picking files, reading the
//! files dropped onto the tree, and where a download goes (natively a save dialog; in the browser the
//! page's own download of the backend's attachment).

use crate::rr::egui;

/// Files to upload into folder `dir` (`""`: the root), by name.
pub struct Picked {
    pub dir: String,
    pub files: Vec<(String, Vec<u8>)>,
    /// Files that could not be read.
    pub errors: Vec<String>,
}

/// The name of a picked or dropped file, without any folder (an upload goes into the folder chosen).
pub fn file_name(path: &std::path::Path) -> Option<String> {
    path.file_name().map(|n| n.to_string_lossy().into_owned()).filter(|n| !n.is_empty() && n != "." && n != "..")
}

/// Let the user pick files to upload into `dir`; `send` gets them (natively at once, after the dialog;
/// in the browser once read).
pub fn pick_files(dir: String, send: impl FnOnce(Picked) + Send + 'static) {
    #[cfg(not(target_arch = "wasm32"))]
    {
        // The blocking dialog, on the UI thread, as the settings' file pickers.
        let Some(paths) = rfd::FileDialog::new().set_title("Upload files").pick_files() else { return };
        let mut picked = Picked { dir, files: Vec::new(), errors: Vec::new() };
        for path in paths {
            match (file_name(&path), std::fs::read(&path)) {
                (Some(name), Ok(bytes)) => picked.files.push((name, bytes)),
                (_, Err(err)) => picked.errors.push(format!("{}: {err}", path.display())),
                (None, _) => {}
            }
        }
        send(picked);
    }
    #[cfg(target_arch = "wasm32")]
    wasm_bindgen_futures::spawn_local(async move {
        let Some(handles) = rfd::AsyncFileDialog::new().set_title("Upload files").pick_files().await else { return };
        let mut picked = Picked { dir, files: Vec::new(), errors: Vec::new() };
        for handle in handles {
            picked.files.push((handle.file_name(), handle.read().await));
        }
        send(picked);
    });
}

/// Let the user pick one zip; `send` gets its name and bytes, or why it could not be read (natively
/// at once, after the dialog; in the browser once read).
pub fn pick_zip(title: &str, send: impl FnOnce(Result<(String, Vec<u8>), String>) + Send + 'static) {
    #[cfg(not(target_arch = "wasm32"))]
    {
        let Some(path) = rfd::FileDialog::new().set_title(title).add_filter("zip", &["zip"]).pick_file() else { return };
        send(std::fs::read(&path).map(|bytes| (file_name(&path).unwrap_or_default(), bytes)).map_err(|err| format!("{}: {err}", path.display())));
    }
    #[cfg(target_arch = "wasm32")]
    {
        let title = title.to_owned();
        wasm_bindgen_futures::spawn_local(async move {
            let Some(handle) = rfd::AsyncFileDialog::new().set_title(&title).add_filter("zip", &["zip"]).pick_file().await else { return };
            send(Ok((handle.file_name(), handle.read().await)));
        });
    }
}

/// Read files dropped onto the tree, for folder `dir`.
pub fn read_dropped(dir: String, dropped: Vec<egui::DroppedFileHandle>, send: impl FnOnce(Picked) + Send + 'static) {
    #[cfg(not(target_arch = "wasm32"))]
    {
        let mut picked = Picked { dir, files: Vec::new(), errors: Vec::new() };
        for file in dropped {
            let Some(name) = file_name(file.path()) else { continue };
            if file.path().is_dir() {
                picked.errors.push(format!("{name}: a folder (drop its files)"));
                continue;
            }
            match file.bytes() {
                Ok(bytes) => picked.files.push((name, bytes)),
                Err(err) => picked.errors.push(format!("{name}: {err}")),
            }
        }
        send(picked);
    }
    #[cfg(target_arch = "wasm32")]
    wasm_bindgen_futures::spawn_local(async move {
        let mut picked = Picked { dir, files: Vec::new(), errors: Vec::new() };
        for file in dropped {
            let Some(name) = file_name(file.path()) else { continue };
            match file.bytes_async().await {
                Ok(bytes) => picked.files.push((name, bytes)),
                Err(err) => picked.errors.push(format!("{name}: {err}")),
            }
        }
        send(picked);
    });
}

/// Where to save a download named `name` (natively; `None`: cancelled).
#[cfg(not(target_arch = "wasm32"))]
pub fn save_as(name: &str) -> Option<std::path::PathBuf> {
    rfd::FileDialog::new().set_title("Download").set_file_name(name).save_file()
}

/// `name` in folder `dir` (`""`: the root).
pub fn join(dir: &str, name: &str) -> String {
    let dir = dir.trim_matches('/');
    if dir.is_empty() { name.to_owned() } else { format!("{dir}/{name}") }
}

/// The folder of `path` (`""`: the root).
pub fn parent(path: &str) -> String {
    path.rsplit_once('/').map_or_else(String::new, |(dir, _)| dir.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paths_in_folders() {
        assert_eq!(join("", "a.st"), "a.st");
        assert_eq!(join("biw/out/", "a.st"), "biw/out/a.st");
        assert_eq!(parent("biw/out/a.st"), "biw/out");
        assert_eq!(parent("a.st"), "");
        assert_eq!(file_name(std::path::Path::new("/tmp/x/weld.src")).as_deref(), Some("weld.src"));
        assert_eq!(file_name(std::path::Path::new("/")), None);
    }
}
