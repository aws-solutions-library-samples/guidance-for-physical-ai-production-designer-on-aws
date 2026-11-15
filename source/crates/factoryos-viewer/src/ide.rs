//! The Workspace IDE: the files of a root the backend allows ([`api::FILES_ROOTS`]: the active cell's
//! folder, and those it was started with), edited in the main area in place of the 3D view, laid
//! out like VS Code and coloured like the Control Code Export page: the folder tree at the left, the
//! open files as tabs at the right, a terminal panel below ([`crate::ide_terminal`]: shells on the backend's host).
//! Opened from the top bar; while it is open only the planner chat stays beside it. Files go up and
//! down as in VS Code's remote explorer ([`crate::ide_transfer`]): a file's context menu or its ⬇ to
//! download, a folder's (or the tree's) context menu or ⬆ to upload into it, or files dropped onto it.
//!
//! A root with a storage scope (the **Shared catalogue**, [`FileRoot::storage`]) goes through the
//! storage routes and a [`DataSource`] instead (S3 directly in a deployed workspace, the backend's
//! local folder otherwise): browsing and opening for everyone; for catalogue admins (`writable`)
//! also saving, uploading, new folders, rename/move, and (Advanced mode) delete. The backend checks
//! every write again.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use factoryos_client::protocol::api;
use factoryos_client::protocol::files::{FileEntry, FileRoot, FileSave, FileSaved, FileText, FileTree};
use factoryos_client::protocol::storage::{FOLDER_MARKER, ObjectList, Scope};
use factoryos_client::{Api, DataSource};

use crate::code_export::{icon_button, search_ui, tree_row};
use crate::ide_terminal::TerminalPanel;
use crate::ide_transfer::{self, Picked};
use crate::rr::egui::{self, RichText};
use crate::{syntax, theme};

enum Msg {
    Roots(Result<Vec<FileRoot>, String>),
    Tree(String, Result<FileTree, String>),
    File { root: String, path: String, file: Result<FileText, String> },
    Saved { root: String, path: String, text: String, saved: Result<FileSaved, String> },
    /// One folder of a root listed a folder at a time (`~`, `/`).
    Folder(String, String, Result<FileTree, String>),
    /// Files picked or dropped, to upload.
    Picked(String, Picked),
    /// An upload's answer; `bytes` kept to replace an existing file when asked.
    Uploaded { root: String, path: String, bytes: Vec<u8>, result: Result<FileEntry, String> },
    /// A download saved (natively): where to, or why not.
    #[cfg_attr(target_arch = "wasm32", expect(dead_code, reason = "the browser saves downloads itself"))]
    Downloaded { path: String, result: Result<String, String> },
    /// A storage root was changed (new folder, move, delete): what happened, to say and re-list.
    Changed { root: String, result: Result<String, String> },
}

/// What a click in the tree asks for.
enum TreeAction {
    Open(String),
    Download(String),
    /// Upload into this folder (`""`: the root).
    Upload(String),
    /// The folder uploads go into (the ⬆ button's, a drop's without a row under the pointer).
    Select(String),
    /// List this folder (of a root listed a folder at a time): it was unfolded.
    Load(String),
    /// Storage roots an admin may change: a new folder in this one (`""`: the root).
    NewFolder(String),
    /// Rename/move this file or folder (`is_dir`).
    Rename(String, bool),
    /// Delete this file or folder (`is_dir`), once confirmed.
    Delete(String, bool),
}

/// A dialog of a storage root's change, open.
enum Prompt {
    NewFolder { parent: String, name: String },
    Rename { from: String, dir: bool, to: String },
    Delete { path: String, dir: bool },
}

/// What a tab's context menu (or ⌘W) asks for.
enum TabAction {
    Close(usize),
    CloseOthers(usize),
    CloseRight(usize),
    CloseSaved,
    CloseAll,
    CopyPath(usize),
    CopyRelative(usize),
}

/// An upload or download in progress or done, for the tree's status lines.
struct Transfer {
    label: String,
    /// `None`: running; else its outcome.
    done: Option<Result<String, String>>,
}

enum Buffer {
    Loading,
    Failed(String),
    Ready(Edit),
}

struct Edit {
    text: String,
    /// What is on disk (as last read or saved), to tell unsaved changes.
    saved_text: String,
    version: String,
    saving: bool,
    /// The file changed on disk since it was opened: reload, or overwrite it.
    conflict: bool,
    error: Option<String>,
    /// The line numbers (for this many lines).
    gutter: (usize, String),
}

impl Edit {
    fn dirty(&self) -> bool {
        self.text != self.saved_text
    }
}

struct Tab {
    root: String,
    path: String,
    buffer: Buffer,
}

impl Tab {
    fn dirty(&self) -> bool {
        matches!(&self.buffer, Buffer::Ready(edit) if edit.dirty())
    }

    fn name(&self) -> &str {
        self.path.rsplit('/').next().unwrap_or(&self.path)
    }
}

pub struct Ide {
    api: Api,
    /// Storage roots' bytes (the shared catalogue): S3 or the backend.
    source: DataSource,
    /// The shared catalogue's changes seen (crate::catalogue_events): a storage root shown lists again.
    catalogue_seen: u64,
    prompt: Option<Prompt>,
    inbox: Arc<Mutex<Vec<Msg>>>,
    /// The IDE is shown (in place of the 3D view, see [`crate::code_layout`]).
    pub open: bool,
    roots: Option<Result<Vec<FileRoot>, String>>,
    root: Option<String>,
    tree: Option<(String, Result<FileTree, String>)>,
    filter: String,
    tabs: Vec<Tab>,
    active: usize,
    /// A tab with unsaved changes the user asked to close.
    closing: Option<usize>,
    /// Close this file (root, path) once it is saved.
    close_when_saved: Option<(String, String)>,
    terminal: TerminalPanel,
    /// The folder uploads go into (`""`: the root), picked in the tree.
    upload_dir: String,
    /// The tree's rows last frame and the folder each stands for (a file's: its folder), to find
    /// where files dragged over it would land.
    drop_rows: Vec<(egui::Rect, String)>,
    /// Where files dragged over the tree land now (`None`: none dragged).
    drop_target: Option<String>,
    transfers: Vec<Transfer>,
    /// Uploads of files that exist: replace them, or not.
    conflicts: Vec<(String, String, Vec<u8>)>,
    /// Of a root listed a folder at a time: the folders listed, and those asked for.
    loaded: std::collections::HashSet<String>,
    loading: std::collections::HashSet<String>,
    /// A short note in the header (e.g. unsaved tabs a Close All kept), until the next action.
    notice: Option<String>,
}

impl Ide {
    pub fn new(api: Api) -> Self {
        Self {
            terminal: TerminalPanel::new(api.clone()),
            source: DataSource::new(api.clone()),
            catalogue_seen: crate::catalogue_events::generation(),
            prompt: None,
            api,
            inbox: Arc::default(),
            open: false,
            roots: None,
            root: None,
            tree: None,
            filter: String::new(),
            tabs: Vec::new(),
            active: 0,
            closing: None,
            close_when_saved: None,
            upload_dir: String::new(),
            drop_rows: Vec::new(),
            drop_target: None,
            transfers: Vec::new(),
            conflicts: Vec::new(),
            loaded: Default::default(),
            loading: Default::default(),
            notice: None,
        }
    }

    /// The shown root, if it is listed a folder at a time.
    fn lazy(&self) -> bool {
        let Some(Ok(roots)) = &self.roots else { return false };
        self.root.as_ref().is_some_and(|id| roots.iter().any(|r| r.id == *id && r.lazy))
    }

    fn root_info(&self, root: &str) -> Option<&FileRoot> {
        let Some(Ok(roots)) = &self.roots else { return None };
        roots.iter().find(|r| r.id == root)
    }

    /// The storage scope `root`'s files go through, if any.
    fn scope(&self, root: &str) -> Option<Scope> {
        self.root_info(root).and_then(|r| r.storage)
    }

    /// Whether `root` may be written (for a storage root: the caller is its admin; checked again by the backend).
    fn writable(&self, root: &str) -> bool {
        self.root_info(root).is_some_and(|r| r.writable)
    }

    /// Its folder on the backend's host.
    fn root_path(&self, root: &str) -> Option<String> {
        let Some(Ok(roots)) = &self.roots else { return None };
        roots.iter().find(|r| r.id == root).map(|r| r.path.clone()).filter(|p| !p.is_empty())
    }

    fn load_folder(&mut self, ctx: &egui::Context, dir: &str) {
        let Some(root) = self.root.clone() else { return };
        if !self.loading.insert(dir.to_owned()) {
            return;
        }
        let send = self.sender(ctx);
        let dir = dir.to_owned();
        self.api.get(&api::files_tree_dir(&root, &dir), move |tree| send(Msg::Folder(root, dir, tree)));
    }

    /// Close the tabs `which` says, but not those with unsaved changes (they stay, said in the header).
    fn close_tabs(&mut self, which: impl Fn(usize, &Tab) -> bool) {
        let before = self.tabs.len();
        let active_path = self.tabs.get(self.active).map(|t| (t.root.clone(), t.path.clone()));
        let mut kept = 0;
        let mut index = 0;
        self.tabs.retain(|t| {
            let close = which(index, t);
            index += 1;
            if close && t.dirty() {
                kept += 1;
            }
            !close || t.dirty()
        });
        self.active = active_path.and_then(|(r, p)| self.tabs.iter().position(|t| t.root == r && t.path == p)).unwrap_or(0);
        self.notice = (kept > 0).then(|| format!("{kept} tab{} with unsaved changes kept open", if kept == 1 { "" } else { "s" }));
        if self.tabs.len() == before && kept == 0 {
            self.notice = None;
        }
    }

    fn tab_action(&mut self, ctx: &egui::Context, action: TabAction) {
        match action {
            TabAction::Close(i) => {
                if self.tabs.get(i).is_some_and(Tab::dirty) {
                    self.closing = Some(i);
                } else if i < self.tabs.len() {
                    self.close_tab(i);
                }
            }
            TabAction::CloseOthers(i) => self.close_tabs(|j, _| j != i),
            TabAction::CloseRight(i) => self.close_tabs(|j, _| j > i),
            TabAction::CloseSaved => self.close_tabs(|_, t| !t.dirty()),
            TabAction::CloseAll => self.close_tabs(|_, _| true),
            TabAction::CopyPath(i) => {
                if let Some(tab) = self.tabs.get(i) {
                    let full = match self.root_path(&tab.root) {
                        Some(base) => format!("{}/{}", base.trim_end_matches('/'), tab.path),
                        None => tab.path.clone(),
                    };
                    ctx.copy_text(full);
                }
            }
            TabAction::CopyRelative(i) => {
                if let Some(tab) = self.tabs.get(i) {
                    ctx.copy_text(tab.path.clone());
                }
            }
        }
    }

    /// Upload `picked` into its folder of the shown root (an existing file only with `overwrite`).
    fn upload(&mut self, ctx: &egui::Context, root: &str, picked: Picked) {
        for err in picked.errors {
            self.transfers.push(Transfer { label: "Upload".to_owned(), done: Some(Err(err)) });
        }
        for (name, bytes) in picked.files {
            self.upload_one(ctx, root, ide_transfer::join(&picked.dir, &name), bytes, false);
        }
    }

    fn upload_one(&mut self, ctx: &egui::Context, root: &str, path: String, bytes: Vec<u8>, overwrite: bool) {
        if !self.writable(root) {
            self.transfers.push(Transfer { label: format!("⬆ {path}"), done: Some(Err("this folder is read-only".to_owned())) });
            return;
        }
        if let Some(scope) = self.scope(root) {
            let exists = matches!(&self.tree, Some((r, Ok(tree))) if r == root && tree.entries.iter().any(|e| e.path == path));
            if exists && !overwrite {
                self.conflicts.push((root.to_owned(), path, bytes));
                return;
            }
            self.transfers.push(Transfer { label: format!("⬆ {path}"), done: None });
            let send = self.sender(ctx);
            let (root, kept, len) = (root.to_owned(), bytes.clone(), bytes.len() as u64);
            let entry = FileEntry { path: path.clone(), bytes: len, dir: false };
            let key = path.clone();
            self.source.put(scope, &key, bytes, Box::new(move |result| send(Msg::Uploaded { root, path, bytes: kept, result: result.map(|()| entry) })));
            return;
        }
        self.transfers.push(Transfer { label: format!("⬆ {path}"), done: None });
        let send = self.sender(ctx);
        let (root, url) = (root.to_owned(), api::files_raw(root, &path, overwrite));
        let kept = bytes.clone();
        self.api.put_bytes(&url, bytes, move |result| send(Msg::Uploaded { root, path, bytes: kept, result }));
    }

    /// Download `path` of the shown root: natively to a file the user picks, in the browser as the
    /// page's download.
    fn download(&mut self, ctx: &egui::Context, path: &str) {
        let Some(root) = self.root.clone() else { return };
        let scope = self.scope(&root);
        let url = api::files_raw(&root, path, false);
        #[cfg(target_arch = "wasm32")]
        {
            let _ = ctx;
            let url = match scope {
                Some(scope) => self.source.download_url(scope, path),
                None => self.api.url_for(&url),
            };
            crate::workspace_ui::open_url(&url);
            self.transfers.push(Transfer { label: format!("⬇ {path}"), done: Some(Ok("in the browser's downloads".to_owned())) });
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let name = path.rsplit('/').next().unwrap_or(path);
            let Some(dest) = ide_transfer::save_as(name) else { return };
            self.transfers.push(Transfer { label: format!("⬇ {path}"), done: None });
            let send = self.sender(ctx);
            let (key, path) = (path.to_owned(), path.to_owned());
            let max = factoryos_client::protocol::files::MAX_TRANSFER_BYTES as usize;
            let done = move |bytes: Result<Vec<u8>, String>| {
                let result = bytes.and_then(|b| std::fs::write(&dest, b).map_err(|e| format!("{}: {e}", dest.display()))).map(|()| dest.display().to_string());
                send(Msg::Downloaded { path, result });
            };
            match scope {
                Some(scope) => self.source.get(scope, &key, max, Box::new(done)),
                None => self.api.get_bytes(&url, max, done),
            }
        }
    }

    fn pick_upload(&mut self, ctx: &egui::Context, dir: String) {
        let Some(root) = self.root.clone() else { return };
        let send = self.sender(ctx);
        ide_transfer::pick_files(dir, move |picked| send(Msg::Picked(root, picked)));
    }

    /// Files dragged over the IDE: where they would land (the row under the pointer, else the folder
    /// picked), and once dropped their upload. Taken before Rerun, which would open them as recordings.
    fn take_drops(&mut self, ctx: &egui::Context) {
        let (hovering, dropped, pointer) = ctx.input_mut(|i| {
            let hovering = !i.raw.hovered_files.is_empty();
            i.raw.hovered_files.clear();
            (hovering, std::mem::take(&mut i.raw.dropped_files), i.pointer.latest_pos())
        });
        let target = || {
            pointer
                .and_then(|pos| self.drop_rows.iter().find(|(rect, _)| rect.contains(pos)).map(|(_, dir)| dir.clone()))
                .unwrap_or_else(|| self.upload_dir.clone())
        };
        if !dropped.is_empty() {
            let dir = self.drop_target.take().unwrap_or_else(target);
            if let Some(root) = self.root.clone() {
                let send = self.sender(ctx);
                ide_transfer::read_dropped(dir, dropped, move |picked| send(Msg::Picked(root, picked)));
            }
        } else if hovering {
            self.drop_target = Some(target());
            ctx.request_repaint();
        } else {
            self.drop_target = None;
        }
    }

    fn sender(&self, ctx: &egui::Context) -> impl Fn(Msg) + Send + Sync + 'static {
        let (inbox, ctx) = (self.inbox.clone(), ctx.clone());
        move |msg| {
            inbox.lock().unwrap().push(msg);
            ctx.request_repaint();
        }
    }

    fn list_roots(&mut self, ctx: &egui::Context) {
        let send = self.sender(ctx);
        self.api.get(api::FILES_ROOTS, move |roots| send(Msg::Roots(roots)));
    }

    fn list_files(&mut self, ctx: &egui::Context, root: &str) {
        let send = self.sender(ctx);
        let id = root.to_owned();
        self.tree = None;
        if let Some(scope) = self.scope(root) {
            self.source.list(scope, "", Box::new(move |list| send(Msg::Tree(id, list.map(|l| tree_of(&l))))));
            return;
        }
        self.api.get(&api::files_tree(root), move |tree| send(Msg::Tree(id, tree)));
    }

    fn fetch(&self, ctx: &egui::Context, root: &str, path: &str) {
        let send = self.sender(ctx);
        let (root, path) = (root.to_owned(), path.to_owned());
        if let Some(scope) = self.scope(&root) {
            let key = path.clone();
            self.source.get(scope, &key, MAX_TEXT_BYTES + 1, Box::new(move |bytes| {
                let file = bytes.and_then(|bytes| {
                    if bytes.len() > MAX_TEXT_BYTES {
                        return Err(format!("larger than {} KiB: download it instead", MAX_TEXT_BYTES / 1024));
                    }
                    let text = String::from_utf8(bytes).map_err(|_| "not a text file: download it instead".to_owned())?;
                    Ok(FileText { text, version: String::new() })
                });
                send(Msg::File { root, path, file });
            }));
            return;
        }
        self.api.get(&api::files_file(&root, &path), move |file| send(Msg::File { root, path, file }));
    }

    fn open_file(&mut self, ctx: &egui::Context, root: &str, path: &str) {
        if let Some(index) = self.tabs.iter().position(|t| t.root == root && t.path == path) {
            self.active = index;
            return;
        }
        self.tabs.push(Tab { root: root.to_owned(), path: path.to_owned(), buffer: Buffer::Loading });
        self.active = self.tabs.len() - 1;
        self.fetch(ctx, root, path);
    }

    /// Save the shown file; `force`: over whatever is on disk now.
    fn save(&mut self, ctx: &egui::Context, force: bool) {
        let send = self.sender(ctx);
        let Some(root) = self.tabs.get(self.active).map(|t| t.root.clone()) else { return };
        let (scope, writable) = (self.scope(&root), self.writable(&root));
        let Some(tab) = self.tabs.get_mut(self.active) else { return };
        let Buffer::Ready(edit) = &mut tab.buffer else { return };
        if edit.saving || (!edit.dirty() && !force) || !writable {
            return;
        }
        edit.saving = true;
        edit.error = None;
        if let Some(scope) = scope {
            // Storage roots keep no versions: the last save wins.
            let (root, path, text) = (tab.root.clone(), tab.path.clone(), edit.text.clone());
            let bytes = text.clone().into_bytes();
            let key = path.clone();
            self.source.put(scope, &key, bytes, Box::new(move |saved| {
                send(Msg::Saved { root, path, text, saved: saved.map(|()| FileSaved { version: String::new() }) })
            }));
            return;
        }
        let body = FileSave { text: edit.text.clone(), base_version: (!force).then(|| edit.version.clone()) };
        let (root, path, text) = (tab.root.clone(), tab.path.clone(), edit.text.clone());
        self.api.put_json(&api::files_file(&root, &path), &body, move |saved| send(Msg::Saved { root, path, text, saved }));
    }

    fn poll(&mut self, ctx: &egui::Context) {
        let messages = std::mem::take(&mut *self.inbox.lock().unwrap());
        for msg in messages {
            match msg {
                Msg::Roots(roots) => {
                    if let Ok(list) = &roots
                        && !self.root.as_ref().is_some_and(|r| list.iter().any(|l| l.id == *r))
                    {
                        self.root = list.first().map(|r| r.id.clone());
                        if let Some(root) = self.root.clone() {
                            self.list_files(ctx, &root);
                        }
                    }
                    self.roots = Some(roots);
                }
                Msg::Tree(root, tree) => {
                    if self.root.as_deref() == Some(root.as_str()) {
                        self.loaded.clear();
                        self.loading.clear();
                        self.loaded.insert(String::new());
                        self.tree = Some((root, tree));
                    }
                }
                Msg::Folder(root, dir, folder) => {
                    self.loading.remove(&dir);
                    if self.root.as_deref() != Some(root.as_str()) {
                        continue;
                    }
                    match folder {
                        Ok(folder) => {
                            self.loaded.insert(dir.clone());
                            if let Some((_, Ok(tree))) = &mut self.tree {
                                let prefix = format!("{dir}/");
                                tree.entries.retain(|e| !(e.path.starts_with(&prefix) && !e.path[prefix.len()..].contains('/')));
                                tree.entries.extend(folder.entries);
                                tree.entries.sort_by(|a, b| a.path.cmp(&b.path));
                                tree.truncated |= folder.truncated;
                            }
                        }
                        Err(err) => self.notice = Some(format!("{dir}: {err}")),
                    }
                }
                Msg::File { root, path, file } => {
                    if let Some(tab) = self.tabs.iter_mut().find(|t| t.root == root && t.path == path) {
                        tab.buffer = match file {
                            Ok(file) => Buffer::Ready(Edit {
                                saved_text: file.text.clone(),
                                text: file.text,
                                version: file.version,
                                saving: false,
                                conflict: false,
                                error: None,
                                gutter: (0, String::new()),
                            }),
                            Err(err) => Buffer::Failed(err),
                        };
                    }
                }
                Msg::Picked(root, picked) => {
                    if self.root.as_deref() == Some(root.as_str()) {
                        self.upload(ctx, &root, picked);
                    }
                }
                Msg::Uploaded { root, path, bytes, result } => {
                    let label = format!("⬆ {path}");
                    let done = match result {
                        Ok(_) => Ok("uploaded".to_owned()),
                        Err(err) if err.starts_with("409") && err.contains("exists") => {
                            self.conflicts.push((root.clone(), path.clone(), bytes));
                            Err("exists".to_owned())
                        }
                        Err(err) => Err(err),
                    };
                    if let Some(i) = self.transfers.iter().rposition(|t| t.label == label && t.done.is_none()) {
                        // One that exists is asked about (conflicts_ui), not a failure.
                        if done.as_ref().is_err_and(|e| e == "exists") {
                            self.transfers.remove(i);
                        } else {
                            self.transfers[i].done = Some(done);
                        }
                    }
                    if self.transfers.iter().all(|t| t.done.is_some()) && self.root.as_deref() == Some(root.as_str()) {
                        self.list_files(ctx, &root);
                        // An open, unchanged tab of an uploaded file shows the new one.
                        let reload: Vec<String> = self.tabs.iter().filter(|t| t.root == root && !t.dirty()).map(|t| t.path.clone()).collect();
                        for p in reload {
                            if let Some(tab) = self.tabs.iter_mut().find(|t| t.path == p) {
                                tab.buffer = Buffer::Loading;
                            }
                            self.fetch(ctx, &root, &p);
                        }
                    }
                }
                Msg::Changed { root, result } => {
                    self.notice = Some(match result {
                        Ok(done) => done,
                        Err(err) => format!("Not done: {err}"),
                    });
                    if self.root.as_deref() == Some(root.as_str()) {
                        self.list_files(ctx, &root);
                    }
                }
                Msg::Downloaded { path, result } => {
                    let label = format!("⬇ {path}");
                    if let Some(t) = self.transfers.iter_mut().rev().find(|t| t.label == label && t.done.is_none()) {
                        t.done = Some(result);
                    }
                }
                Msg::Saved { root, path, text, saved } => {
                    let Some(index) = self.tabs.iter().position(|t| t.root == root && t.path == path) else { continue };
                    let Buffer::Ready(edit) = &mut self.tabs[index].buffer else { continue };
                    edit.saving = false;
                    match saved {
                        Ok(saved) => {
                            edit.saved_text = text;
                            edit.version = saved.version;
                            edit.conflict = false;
                            if self.close_when_saved.as_ref() == Some(&(root, path)) && !self.tabs[index].dirty() {
                                self.close_when_saved = None;
                                self.close_tab(index);
                            }
                        }
                        Err(err) if err.starts_with("409") => edit.conflict = true,
                        Err(err) => edit.error = Some(err),
                    }
                }
            }
        }
    }

    /// The page, in the main area.
    pub fn page_ui(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        self.poll(&ctx);
        self.take_drops(&ctx);
        if crate::catalogue_events::changed_since(&mut self.catalogue_seen)
            && let Some(root) = self.root.clone().filter(|r| self.scope(r).is_some())
        {
            self.list_files(&ctx, &root);
        }
        if self.roots.is_none() {
            self.roots = Some(Ok(Vec::new()));
            self.list_roots(&ctx);
        }
        if ui.input_mut(|i| i.consume_shortcut(&egui::KeyboardShortcut::new(egui::Modifiers::COMMAND, egui::Key::S))) {
            self.save(&ctx, false);
        }
        if !self.tabs.is_empty() && ui.input_mut(|i| i.consume_shortcut(&CLOSE_TAB)) {
            self.tab_action(&ctx, TabAction::Close(self.active));
        }
        let p = theme::palette(ui);
        egui::Frame::new().fill(p.bg).inner_margin(egui::Margin::symmetric(10, 8)).show(ui, |ui| {
            self.header_ui(ui, &ctx);
            ui.add_space(6.0);
            self.terminal.ui(ui, self.root.as_deref());
            let max_width = (ui.available_width() * 0.5).max(220.0);
            egui::Panel::left("workspace_ide_files")
                .resizable(true)
                .default_size(280.0_f32.min(max_width))
                .min_size(180.0_f32.min(max_width))
                .max_size(max_width)
                .frame(egui::Frame::new().inner_margin(egui::Margin { right: 8, ..Default::default() }))
                .show(ui, |ui| self.files_ui(ui, &ctx));
            egui::CentralPanel::no_frame().show(ui, |ui| self.editor_ui(ui, &ctx));
        });
        self.close_dialog_ui(&ctx);
        self.conflicts_ui(&ctx);
        self.prompt_ui(&ctx);
    }

    /// A storage root's change: the dialog asking for a name, or to confirm a delete.
    fn prompt_ui(&mut self, ctx: &egui::Context) {
        let Some(root) = self.root.clone().filter(|_| self.prompt.is_some()) else { return };
        let Some(scope) = self.scope(&root) else {
            self.prompt = None;
            return;
        };
        let Some(prompt) = &mut self.prompt else { return };
        let mut answer: Option<bool> = None;
        egui::Modal::new(egui::Id::new("workspace_ide_storage_prompt")).show(ctx, |ui| {
            theme::with_controls(ui, |ui| {
                ui.set_width(380.0);
                let (title, action) = match prompt {
                    Prompt::NewFolder { parent, name } => {
                        let at = if parent.is_empty() { "the top".to_owned() } else { format!("{parent}/") };
                        ui.label(RichText::new(format!("New folder in {at}")).strong());
                        ui.text_edit_singleline(name).request_focus();
                        ("", "Create")
                    }
                    Prompt::Rename { from, dir, to } => {
                        ui.label(RichText::new(format!("Rename or move {from}{}", if *dir { "/" } else { "" })).strong());
                        ui.label(RichText::new("The new path, from the top of the shared catalogue").small().weak());
                        ui.text_edit_singleline(to).request_focus();
                        ("", "Move")
                    }
                    Prompt::Delete { path, dir } => {
                        let what = if *dir { format!("the folder {path}/ and everything in it") } else { path.clone() };
                        ui.label(RichText::new(format!("Delete {what}?")).strong());
                        ui.label(RichText::new("For everyone using the shared catalogue. Earlier versions are kept for 30 days in S3.").small().weak());
                        ("danger", "Delete")
                    }
                };
                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    let button = if title == "danger" {
                        let p = theme::palette(ui);
                        egui::Button::new(RichText::new(action).strong().color(egui::Color32::WHITE)).fill(p.danger)
                    } else {
                        theme::primary(ui, action)
                    };
                    if ui.add(button).clicked() || (title.is_empty() && ui.input(|i| i.key_pressed(egui::Key::Enter))) {
                        answer = Some(true);
                    }
                    if ui.button("Cancel").clicked() || ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                        answer = Some(false);
                    }
                });
            });
        });
        let Some(go) = answer else { return };
        let Some(prompt) = self.prompt.take() else { return };
        if !go {
            return;
        }
        let send = self.sender(ctx);
        let done = |what: String| {
            let root = root.clone();
            move |result: Result<String, String>| send(Msg::Changed { root, result: result.map(|_| what) })
        };
        match prompt {
            Prompt::NewFolder { parent, name } => {
                let key = ide_transfer::join(&parent, name.trim().trim_matches('/'));
                let done = done(format!("Folder {key}/ created"));
                self.source.make_folder(scope, &key, Box::new(move |r| done(r.map(|()| String::new()))));
            }
            Prompt::Rename { from, dir, to } => {
                let to = to.trim().trim_matches('/').to_owned();
                let (from_key, to_key) = if dir { (format!("{from}/"), format!("{to}/")) } else { (from.clone(), to.clone()) };
                let done = done(format!("Moved {from_key} to {to_key}"));
                self.source.move_keys(scope, &from_key, &to_key, false, Box::new(move |r| done(r.map(|p| p.objects.to_string()))));
            }
            Prompt::Delete { path, dir } => {
                let key = if dir { format!("{path}/") } else { path.clone() };
                // Open, unchanged tabs of what goes, go too.
                self.close_tabs(|_, t| t.root == root && (t.path == path || (dir && t.path.starts_with(&key))));
                let done = done(format!("Deleted {key}"));
                self.source.remove(scope, &key, Box::new(move |r| done(r.map(|p| p.objects.to_string()))));
            }
        }
    }

    /// Uploads of files that exist: replace them all, or keep them.
    fn conflicts_ui(&mut self, ctx: &egui::Context) {
        if self.conflicts.is_empty() {
            return;
        }
        let mut answer = None;
        egui::Modal::new(egui::Id::new("workspace_ide_upload_conflicts")).show(ctx, |ui| {
            crate::theme::controls_style(ui);
            ui.set_width(380.0);
            ui.label(RichText::new("These files exist").strong());
            for (_, path, _) in self.conflicts.iter().take(8) {
                ui.label(RichText::new(path).monospace().small());
            }
            if self.conflicts.len() > 8 {
                ui.label(format!("and {} more", self.conflicts.len() - 8));
            }
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if ui.button("Replace").clicked() {
                    answer = Some(true);
                }
                if ui.button("Keep them").clicked() {
                    answer = Some(false);
                }
            });
        });
        match answer {
            Some(true) => {
                for (root, path, bytes) in std::mem::take(&mut self.conflicts) {
                    self.upload_one(ctx, &root, path, bytes, true);
                }
            }
            Some(false) => self.conflicts.clear(),
            None => {}
        }
    }

    fn header_ui(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let p = theme::palette(ui);
        ui.horizontal(|ui| {
            ui.label(RichText::new("Workspace IDE").size(16.0).strong().color(p.text_strong));
            if let Some(note) = &self.notice {
                ui.label(RichText::new(note).small().color(p.warn));
            }
            crate::catalogue_events::notice_ui(ui);
            ui.add_space(8.0);
            let roots = match &self.roots {
                Some(Ok(roots)) => roots.clone(),
                _ => Vec::new(),
            };
            let shown = self.root.as_ref().and_then(|id| roots.iter().find(|r| r.id == *id)).map_or("No folder", |r| r.label.as_str());
            let mut pick = None;
            egui::ComboBox::from_id_salt("workspace_ide_root").selected_text(shown).width(220.0).truncate().show_ui(ui, |ui| {
                for root in &roots {
                    if ui.selectable_label(self.root.as_deref() == Some(root.id.as_str()), &root.label).clicked() {
                        pick = Some(root.id.clone());
                    }
                }
            })
            .response
            .on_hover_text("The folders this workspace's backend lets the IDE open (FACTORY_OS_IDE_ROOTS)");
            if let Some(id) = pick
                && self.root.as_deref() != Some(id.as_str())
            {
                self.root = Some(id.clone());
                self.filter.clear();
                self.upload_dir.clear();
                self.list_files(ctx, &id);
            }
            if ui.add(icon_button("↻", p)).on_hover_text("Re-list the folders and files").clicked() {
                self.list_roots(ctx);
                if let Some(root) = self.root.clone() {
                    self.list_files(ctx, &root);
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let back = egui::Button::new(RichText::new("✕  Back to the 3D view").color(p.text))
                    .fill(p.card)
                    .stroke(egui::Stroke::new(1.0, p.border))
                    .corner_radius(6);
                if ui.add(back).on_hover_text("Close the IDE (open files stay open)").clicked() {
                    self.open = false;
                }
                let terminal = if self.terminal.open { "Hide terminal" } else { "Terminal" };
                if ui.add(icon_button("⌨", p)).on_hover_text(terminal).clicked() {
                    self.terminal.open = !self.terminal.open;
                }
            });
        });
    }

    fn files_ui(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let p = theme::palette(ui);
        if let Some(Err(err)) = &self.roots {
            ui.colored_label(p.danger, err);
            return;
        }
        let Some(root) = self.root.clone() else {
            ui.label(RichText::new("No folder to open: the backend allows none.").small().color(p.muted));
            return;
        };
        let viewing = self.tabs.get(self.active).filter(|t| t.root == root).map(|t| t.path.clone());
        let mut action = None;
        let writable = self.writable(&root);
        // A storage root an admin may change: new folder, rename/move; delete in Advanced mode only.
        let manage = writable && self.scope(&root).is_some();
        let advanced = crate::ui_mode::is_advanced(ui);
        if self.scope(&root).is_some() {
            let note = if writable { "Shared catalogue · you may change it (everyone sees the changes)" } else { "Shared catalogue · read-only" };
            ui.label(RichText::new(note).small().color(if writable { p.warn } else { p.muted }));
        }
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 2.0;
            let into = if self.upload_dir.is_empty() { "the root".to_owned() } else { format!("{}/", self.upload_dir) };
            ui.label(RichText::new(format!("EXPLORER · {into}")).small().color(p.muted)).on_hover_text("Uploads go into this folder: click a folder to pick another");
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if let Some(path) = &viewing
                    && ui.add(icon_button("⬇", p)).on_hover_text(format!("Download {path}")).clicked()
                {
                    action = Some(TreeAction::Download(path.clone()));
                }
                if writable && ui.add(icon_button("⬆", p)).on_hover_text(format!("Upload files into {into} (or drop them onto a folder)")).clicked() {
                    action = Some(TreeAction::Upload(self.upload_dir.clone()));
                }
                if manage && ui.add(icon_button("+", p)).on_hover_text(format!("New folder in {into}")).clicked() {
                    action = Some(TreeAction::NewFolder(self.upload_dir.clone()));
                }
            });
        });
        search_ui(ui, &mut self.filter, p);
        ui.add_space(4.0);
        if !self.transfers.is_empty() {
            egui::Panel::bottom("workspace_ide_transfers").frame(egui::Frame::new()).resizable(false).show(ui, |ui| self.transfers_ui(ui, p));
        }
        let mut rows = Vec::new();
        match &self.tree {
            None => {
                ui.spinner();
            }
            Some((_, Err(err))) => {
                ui.colored_label(p.danger, err);
            }
            Some((_, Ok(tree))) => {
                let files: Vec<&FileEntry> = tree.entries.iter().filter(|e| !e.dir).collect();
                let filter = self.filter.trim().to_lowercase();
                egui::ScrollArea::vertical().id_salt("workspace_ide_tree").auto_shrink([false, false]).show(ui, |ui| {
                    ui.spacing_mut().item_spacing.y = 0.0;
                    if tree.truncated {
                        ui.label(RichText::new("Only the first files are listed.").small().color(p.warn));
                    }
                    if filter.is_empty() {
                        ui.label(RichText::new(format!("FILES  {}", files.len())).small().color(p.muted));
                        let lazy = self.lazy();
                        let mut t = TreeUi { viewing: viewing.as_deref(), upload_dir: &self.upload_dir, drop_target: self.drop_target.as_deref(), p, action: &mut action, rows: &mut rows, lazy: lazy.then_some(&self.loaded), writable, manage, advanced };
                        show_dir(ui, &dir_tree(&tree.entries), "", 0, &mut t);
                    } else {
                        let matches: Vec<_> = files.iter().filter(|f| f.path.to_lowercase().contains(&filter)).collect();
                        ui.label(RichText::new(format!("FILES  {} of {}", matches.len(), files.len())).small().color(p.muted));
                        let mut t = TreeUi { viewing: viewing.as_deref(), upload_dir: &self.upload_dir, drop_target: self.drop_target.as_deref(), p, action: &mut action, rows: &mut rows, lazy: None, writable, manage, advanced };
                        for file in matches {
                            file_row(ui, file, &file.path, 0, &mut t);
                        }
                    }
                    // The rest of the tree: the root's (a right-click uploads there, a drop lands there).
                    let rest = ui.available_rect_before_wrap();
                    let rest = egui::Rect::from_min_size(rest.min, egui::vec2(rest.width(), rest.height().max(40.0)));
                    let response = ui.allocate_rect(rest, egui::Sense::click());
                    if self.drop_target.as_deref() == Some("") {
                        ui.painter().rect_stroke(rest.shrink(2.0), 4, egui::Stroke::new(1.0, p.accent), egui::StrokeKind::Inside);
                    }
                    rows.push((rest, String::new()));
                    if response.clicked() {
                        action = Some(TreeAction::Select(String::new()));
                    }
                    if writable {
                        response.context_menu(|ui| {
                            if ui.button("⬆  Upload files here…").clicked() {
                                action = Some(TreeAction::Upload(String::new()));
                                ui.close();
                            }
                            if manage && ui.button("+  New folder…").clicked() {
                                action = Some(TreeAction::NewFolder(String::new()));
                                ui.close();
                            }
                        });
                    }
                });
            }
        }
        self.drop_rows = rows;
        if let Some(target) = &self.drop_target {
            let into = if target.is_empty() { "the root".to_owned() } else { format!("{target}/") };
            egui::Area::new(egui::Id::new("workspace_ide_drop_hint")).order(egui::Order::Tooltip).fixed_pos(ui.max_rect().left_bottom() + egui::vec2(8.0, -34.0)).show(ui.ctx(), |ui| {
                egui::Frame::new().fill(p.accent).corner_radius(6).inner_margin(egui::Margin::symmetric(10, 5)).show(ui, |ui| {
                    ui.label(RichText::new(format!("⬆ Drop to upload into {into}")).color(egui::Color32::WHITE));
                });
            });
        }
        match action {
            Some(TreeAction::Open(path)) => self.open_file(ctx, &root, &path),
            Some(TreeAction::Download(path)) => self.download(ctx, &path),
            Some(TreeAction::Upload(dir)) => {
                self.upload_dir = dir.clone();
                self.pick_upload(ctx, dir);
            }
            Some(TreeAction::Select(dir)) => self.upload_dir = dir,
            Some(TreeAction::Load(dir)) => self.load_folder(ctx, &dir),
            Some(TreeAction::NewFolder(parent)) => self.prompt = Some(Prompt::NewFolder { parent, name: String::new() }),
            Some(TreeAction::Rename(from, dir)) => self.prompt = Some(Prompt::Rename { to: from.clone(), from, dir }),
            Some(TreeAction::Delete(path, dir)) => self.prompt = Some(Prompt::Delete { path, dir }),
            None => {}
        }
    }

    /// The uploads and downloads running, and the last ones done (✕ clears those).
    fn transfers_ui(&mut self, ui: &mut egui::Ui, p: &theme::Palette) {
        if self.transfers.is_empty() {
            return;
        }
        let running = self.transfers.iter().filter(|t| t.done.is_none()).count();
        ui.separator();
        ui.horizontal(|ui| {
            let title = if running > 0 { format!("TRANSFERS  {running} running") } else { "TRANSFERS".to_owned() };
            ui.label(RichText::new(title).small().color(p.muted));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.add(icon_button("✕", p)).on_hover_text("Clear the finished ones").clicked() {
                    self.transfers.retain(|t| t.done.is_none());
                }
            });
        });
        for t in self.transfers.iter().rev().take(6) {
            ui.horizontal(|ui| {
                match &t.done {
                    None => {
                        ui.spinner();
                    }
                    Some(Ok(_)) => {
                        ui.label(RichText::new("✔").color(p.ok));
                    }
                    Some(Err(_)) => {
                        ui.label(RichText::new("✖").color(p.danger));
                    }
                }
                let detail = match &t.done {
                    Some(Ok(d)) | Some(Err(d)) => d.clone(),
                    None => String::new(),
                };
                ui.add(egui::Label::new(RichText::new(&t.label).small().monospace()).truncate()).on_hover_text(detail);
            });
        }
    }

    fn editor_ui(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let p = theme::palette(ui);
        if self.tabs.is_empty() {
            ui.centered_and_justified(|ui| ui.label(RichText::new("Pick a file at the left to edit it.").color(p.muted)));
            return;
        }
        self.active = self.active.min(self.tabs.len() - 1);
        if let Some(action) = self.tabs_ui(ui, p) {
            self.tab_action(ctx, action);
            if self.tabs.is_empty() {
                return;
            }
            self.active = self.active.min(self.tabs.len() - 1);
        }
        let mut save = None;
        let mut reload = false;
        let mut download = false;
        let mut crumb = None;
        let writable = self.tabs.get(self.active).is_some_and(|t| self.writable(&t.root));
        let entries: &[FileEntry] = match &self.tree {
            Some((root, Ok(tree))) if Some(root) == self.tabs.get(self.active).map(|t| &t.root) => &tree.entries,
            _ => &[],
        };
        let tab = &mut self.tabs[self.active];
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            ui.spacing_mut().item_spacing.x = 6.0;
            // Breadcrumbs, as VS Code's: each folder a menu of what is in it (folders open as submenus).
            let parts: Vec<&str> = tab.path.split('/').collect();
            for (i, part) in parts.iter().enumerate() {
                if i > 0 {
                    ui.label(RichText::new("›").color(p.muted));
                }
                let dir = parts[..i].join("/");
                if i + 1 == parts.len() {
                    let (icon, color) = file_icon(&tab.path);
                    ui.label(RichText::new(icon).monospace().strong().color(color));
                    ui.menu_button(RichText::new(*part).color(p.text), |ui| crumb_menu(ui, entries, &dir, &mut crumb));
                } else {
                    ui.menu_button(RichText::new(*part).color(p.muted), |ui| crumb_menu(ui, entries, &dir, &mut crumb));
                }
            }
            ui.add_space(8.0);
            if let Buffer::Ready(edit) = &tab.buffer {
                let lines = edit.text.split('\n').count();
                ui.label(RichText::new(format!("{} · {lines} lines", syntax::language_name(&tab.path, &edit.text))).small().color(p.muted));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.add(icon_button("⬇", p)).on_hover_text("Download this file").clicked() {
                        download = true;
                    }
                    if !writable {
                        ui.label(RichText::new("read-only").small().color(p.muted));
                        return;
                    }
                    let label = if edit.saving { "Saving…" } else { "Save" };
                    let button = egui::Button::new(RichText::new(label).color(egui::Color32::WHITE)).fill(p.accent).corner_radius(4);
                    if ui.add_enabled(edit.dirty() && !edit.saving, button).on_hover_text("Save (Cmd/Ctrl+S)").clicked() {
                        save = Some(false);
                    }
                    if edit.dirty() {
                        ui.label(RichText::new("● unsaved").small().color(p.warn));
                    }
                });
            }
        });
        if let Buffer::Ready(edit) = &tab.buffer {
            if edit.conflict {
                notice(ui, p.warn, p.warn_subtle, "This file changed on disk since you opened it.", |ui| {
                    if ui.button("Overwrite").on_hover_text("Save your text over the one on disk").clicked() {
                        save = Some(true);
                    }
                    if ui.button("Reload").on_hover_text("Discard your changes, open the one on disk").clicked() {
                        reload = true;
                    }
                });
            }
            if let Some(err) = &edit.error {
                notice(ui, p.danger, p.danger_subtle, &format!("Not saved: {err}"), |_| {});
            }
        }
        ui.add_space(4.0);
        match &mut tab.buffer {
            Buffer::Loading => {
                ui.spinner();
            }
            Buffer::Failed(err) => {
                ui.colored_label(p.warn, err.as_str());
            }
            Buffer::Ready(edit) => code_editor_ui(ui, &tab.root, &tab.path, edit, !writable, p),
        }
        if reload {
            let (root, path) = (tab.root.clone(), tab.path.clone());
            tab.buffer = Buffer::Loading;
            self.fetch(ctx, &root, &path);
        }
        if let Some(force) = save {
            self.save(ctx, force);
        }
        if download {
            let path = self.tabs[self.active].path.clone();
            self.download(ctx, &path);
        }
        match crumb {
            Some(TreeAction::Open(path)) => {
                let root = self.tabs[self.active].root.clone();
                self.open_file(ctx, &root, &path);
            }
            Some(TreeAction::Load(dir)) => self.load_folder(ctx, &dir),
            _ => {}
        }
    }

    fn close_tab(&mut self, index: usize) {
        self.tabs.remove(index);
        if self.active >= index && self.active > 0 {
            self.active -= 1;
        }
    }

    fn close_dialog_ui(&mut self, ctx: &egui::Context) {
        let Some(index) = self.closing.filter(|i| *i < self.tabs.len()) else {
            self.closing = None;
            return;
        };
        let name = self.tabs[index].name().to_owned();
        let mut answer = None;
        egui::Modal::new(egui::Id::new("workspace_ide_close")).show(ctx, |ui| {
            crate::theme::controls_style(ui);
            ui.set_width(340.0);
            ui.label(RichText::new("Unsaved changes").strong());
            ui.label(format!("{name} has changes that aren't saved."));
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if crate::theme::primary_button(ui, "Save and close").clicked() {
                    answer = Some(true);
                }
                if ui.button("Discard").clicked() {
                    answer = Some(false);
                }
                if ui.button("Cancel").clicked() {
                    self.closing = None;
                }
            });
        });
        match answer {
            Some(true) => {
                self.active = index;
                self.close_when_saved = Some((self.tabs[index].root.clone(), self.tabs[index].path.clone()));
                self.save(ctx, false);
                self.closing = None;
            }
            Some(false) => {
                self.close_tab(index);
                self.closing = None;
            }
            None => {}
        }
    }

    /// The open files as a tab bar, like VS Code's: the bar in the card colour with a border at its
    /// bottom; the shown tab in the editor's colour (open into the code below) with an accent line on
    /// top; a file-type icon, and a close button (`●` while unsaved, `✕` on hover); returns the one closed.
    fn tabs_ui(&mut self, ui: &mut egui::Ui, p: &theme::Palette) -> Option<TabAction> {
        const HEIGHT: f32 = 38.0;
        const PAD: f32 = 14.0;
        const CLOSE: f32 = 20.0;
        let mut close = None;
        let mut pick = None;
        let mut menu = None;
        let count = self.tabs.len();
        let any_dirty = self.tabs.iter().any(Tab::dirty);
        let bar = egui::Rect::from_min_size(ui.cursor().min, egui::vec2(ui.available_width(), HEIGHT));
        ui.painter().rect_filled(bar, 0, p.card);
        let mut active_x = None;
        egui::ScrollArea::horizontal().id_salt("workspace_ide_tabs").auto_shrink([false, true]).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 0.0;
                for (index, tab) in self.tabs.iter().enumerate() {
                    let active = index == self.active;
                    let color = if active { p.text_strong } else { p.muted };
                    let mut job = egui::text::LayoutJob::simple_singleline(tab.name().to_owned(), egui::FontId::proportional(13.5), color);
                    job.wrap = egui::text::TextWrapping::truncate_at_width(200.0);
                    let galley = ui.fonts_mut(|f| f.layout_job(job));
                    let (icon, icon_color) = file_icon(&tab.path);
                    let icon_galley = ui.fonts_mut(|f| f.layout_no_wrap(icon.to_owned(), egui::FontId::monospace(12.0), icon_color));
                    let width = PAD + icon_galley.size().x + 8.0 + galley.size().x + 10.0 + CLOSE + 8.0;
                    let (rect, response) = ui.allocate_exact_size(egui::vec2(width, HEIGHT), egui::Sense::click());
                    let hovered = ui.rect_contains_pointer(rect);
                    let painter = ui.painter_at(rect);
                    painter.rect_filled(rect, 0, if active { p.bg } else if hovered { p.hover } else { p.card });
                    painter.vline(rect.right() - 0.5, rect.y_range(), egui::Stroke::new(1.0, p.border));
                    if active {
                        painter.hline(rect.x_range(), rect.top() + 1.0, egui::Stroke::new(2.0, p.accent));
                        active_x = Some(rect.x_range());
                    }
                    let mid = rect.center().y;
                    let x = rect.left() + PAD;
                    painter.galley(egui::pos2(x, mid - icon_galley.size().y / 2.0), icon_galley.clone(), icon_color);
                    painter.galley(egui::pos2(x + icon_galley.size().x + 8.0, mid - galley.size().y / 2.0), galley, color);
                    let at = egui::Rect::from_center_size(egui::pos2(rect.right() - 8.0 - CLOSE / 2.0, mid), egui::vec2(CLOSE, CLOSE));
                    let dirty = tab.dirty();
                    let over_close = ui.rect_contains_pointer(at);
                    if dirty && !over_close {
                        painter.circle_filled(at.center(), 4.0, if active { p.text } else { p.muted });
                    } else if active || hovered {
                        // Added after the tab, so it is on top and gets its own clicks.
                        let x = egui::Button::new(RichText::new("✕").size(13.0).color(if active { p.text } else { p.muted })).frame(false);
                        if ui.place(at, x).on_hover_text("Close (middle-click)").clicked() {
                            close = Some(index);
                        }
                    }
                    if response.clicked() {
                        pick = Some(index);
                    }
                    if response.middle_clicked() {
                        close = Some(index);
                    }
                    let last = index + 1 == count;
                    let dirty_any = any_dirty;
                    response.context_menu(|ui| {
                        let shortcut = ui.ctx().format_shortcut(&CLOSE_TAB);
                        if ui.add(egui::Button::new("Close").shortcut_text(shortcut)).clicked() {
                            menu = Some(TabAction::Close(index));
                            ui.close();
                        }
                        if ui.add_enabled(count > 1, egui::Button::new("Close Others")).clicked() {
                            menu = Some(TabAction::CloseOthers(index));
                            ui.close();
                        }
                        if ui.add_enabled(!last, egui::Button::new("Close to the Right")).clicked() {
                            menu = Some(TabAction::CloseRight(index));
                            ui.close();
                        }
                        if ui.add_enabled(count > 1 || !dirty_any, egui::Button::new("Close Saved")).clicked() {
                            menu = Some(TabAction::CloseSaved);
                            ui.close();
                        }
                        if ui.button("Close All").clicked() {
                            menu = Some(TabAction::CloseAll);
                            ui.close();
                        }
                        ui.separator();
                        if ui.button("Copy Path").clicked() {
                            menu = Some(TabAction::CopyPath(index));
                            ui.close();
                        }
                        if ui.button("Copy Relative Path").clicked() {
                            menu = Some(TabAction::CopyRelative(index));
                            ui.close();
                        }
                    });
                    response.on_hover_text(format!("{}: {}", tab.root, tab.path));
                }
            });
        });
        // The bar's bottom edge, open under the shown tab.
        let stroke = egui::Stroke::new(1.0, p.border);
        let y = bar.bottom() - 0.5;
        match active_x {
            Some(x) => {
                ui.painter().hline(bar.left()..=x.min, y, stroke);
                ui.painter().hline(x.max..=bar.right(), y, stroke);
            }
            None => {
                ui.painter().hline(bar.x_range(), y, stroke);
            }
        }
        if close.is_none()
            && menu.is_none()
            && let Some(index) = pick
        {
            self.active = index;
        }
        menu.or(close.map(TabAction::Close))
    }
}

/// A file's icon in the tabs and breadcrumb: VS Code's kind of glyph, in a colour by language.
fn file_icon(path: &str) -> (&'static str, egui::Color32) {
    use egui::Color32;
    let ext = path.rsplit_once('.').map(|(_, ext)| ext.to_ascii_lowercase()).unwrap_or_default();
    match ext.as_str() {
        "st" | "exp" | "iecst" | "gvl" | "pou" | "iec" => ("ST", Color32::from_rgb(0x6c, 0xb6, 0xff)),
        "src" | "dat" | "sub" => ("KRL", Color32::from_rgb(0xff, 0x8c, 0x1a)),
        "sh" | "bash" | "zsh" => ("$", Color32::from_rgb(0x8d, 0xc1, 0x49)),
        "py" => ("py", Color32::from_rgb(0x4b, 0x8b, 0xbe)),
        "rs" => ("rs", Color32::from_rgb(0xde, 0xa5, 0x84)),
        "json" => ("{}", Color32::from_rgb(0xcb, 0xcb, 0x41)),
        "yaml" | "yml" => ("!", Color32::from_rgb(0xa0, 0x74, 0xc4)),
        "toml" => ("⚙", Color32::from_rgb(0x9c, 0x9c, 0x9c)),
        "md" => ("M↓", Color32::from_rgb(0x51, 0x9a, 0xba)),
        "xml" | "urdf" | "sdf" | "launch" => ("<>", Color32::from_rgb(0xe3, 0x7e, 0x33)),
        "ts" | "js" | "mjs" => ("JS", Color32::from_rgb(0xf1, 0xe0, 0x5a)),
        _ => ("≡", Color32::from_rgb(0x9c, 0x9c, 0x9c)),
    }
}

/// The text with line numbers beside it, scrolling both ways (lines aren't wrapped).
/// `read_only`: selectable and copyable, not editable (a root the caller may not write).
fn code_editor_ui(ui: &mut egui::Ui, root: &str, path: &str, edit: &mut Edit, read_only: bool, p: &theme::Palette) {
    let font = egui::TextStyle::Monospace.resolve(ui.style());
    let lines = edit.text.split('\n').count();
    if edit.gutter.0 != lines {
        let width = lines.to_string().len();
        edit.gutter = (lines, (1..=lines).map(|n| format!("{n:>width$}")).collect::<Vec<_>>().join("\n"));
    }
    egui::ScrollArea::both().id_salt(("workspace_ide_editor", root, path)).auto_shrink([false, false]).show(ui, |ui| {
        ui.horizontal_top(|ui| {
            ui.spacing_mut().item_spacing.x = 14.0;
            ui.add(egui::Label::new(RichText::new(&edit.gutter.1).font(font.clone()).color(p.muted)).selectable(false).extend());
            let path_id = path;
            let path = path.to_owned();
            let font_for_layout = font.clone();
            let mut layouter = move |ui: &egui::Ui, text: &dyn egui::TextBuffer, _wrap: f32| syntax::galley(ui, text.as_str(), &path, &font_for_layout);
            let mut fixed: &str = &edit.text;
            let text: &mut dyn egui::TextBuffer = if read_only { &mut fixed } else { &mut edit.text };
            ui.add(
                egui::TextEdit::multiline(text)
                    .id(egui::Id::new(("workspace_ide_text", root, path_id)))
                    .font(font)
                    .code_editor()
                    .lock_focus(true)
                    .frame(egui::Frame::NONE)
                    .margin(egui::Margin::ZERO)
                    .desired_width(f32::INFINITY)
                    .desired_rows(lines)
                    .layouter(&mut layouter),
            );
        });
    });
}

fn notice(ui: &mut egui::Ui, color: egui::Color32, fill: egui::Color32, text: &str, buttons: impl FnOnce(&mut egui::Ui)) {
    ui.add_space(4.0);
    egui::Frame::new().fill(fill).corner_radius(6).inner_margin(egui::Margin::symmetric(8, 4)).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.label(RichText::new(text).color(color));
            buttons(ui);
        });
    });
}

#[derive(Default)]
struct Dir<'a> {
    dirs: BTreeMap<&'a str, Dir<'a>>,
    files: Vec<&'a FileEntry>,
}

/// The entries as nested folders (empty folders too).
fn dir_tree(entries: &[FileEntry]) -> Dir<'_> {
    let mut root = Dir::default();
    for entry in entries {
        let mut parts: Vec<&str> = entry.path.split('/').collect();
        let leaf = if entry.dir { None } else { parts.pop() };
        let mut dir = &mut root;
        for part in parts {
            dir = dir.dirs.entry(part).or_default();
        }
        if leaf.is_some() {
            dir.files.push(entry);
        }
    }
    root
}

const CLOSE_TAB: egui::KeyboardShortcut = egui::KeyboardShortcut::new(egui::Modifiers::COMMAND, egui::Key::W);

/// A breadcrumb's menu: what folder `dir` holds (`""`: the root), its folders as submenus, a file a
/// click to open. A folder not listed yet (a root listed a folder at a time) asks for its listing.
fn crumb_menu(ui: &mut egui::Ui, entries: &[FileEntry], dir: &str, action: &mut Option<TreeAction>) {
    let prefix = if dir.is_empty() { String::new() } else { format!("{dir}/") };
    let children: Vec<&FileEntry> = entries.iter().filter(|e| e.path.starts_with(&prefix) && !e.path[prefix.len()..].contains('/') && e.path.len() > prefix.len()).collect();
    if children.is_empty() {
        ui.label(RichText::new("…").weak());
        if action.is_none() {
            *action = Some(TreeAction::Load(dir.to_owned()));
        }
        return;
    }
    egui::ScrollArea::vertical().max_height(420.0).show(ui, |ui| {
        for entry in children.iter().filter(|e| e.dir) {
            let name = &entry.path[prefix.len()..];
            ui.menu_button(format!("▸ {name}"), |ui| crumb_menu(ui, entries, &entry.path, action));
        }
        for entry in children.iter().filter(|e| !e.dir) {
            let name = &entry.path[prefix.len()..];
            let (icon, _) = file_icon(&entry.path);
            if ui.button(format!("{icon}  {name}")).clicked() {
                *action = Some(TreeAction::Open(entry.path.clone()));
                ui.close();
            }
        }
    });
}

/// What the tree's rows draw with, and what they report.
struct TreeUi<'a> {
    viewing: Option<&'a str>,
    upload_dir: &'a str,
    drop_target: Option<&'a str>,
    p: &'a theme::Palette,
    action: &'a mut Option<TreeAction>,
    /// Each row's rect and the folder a drop there lands in.
    rows: &'a mut Vec<(egui::Rect, String)>,
    /// A root listed a folder at a time: the folders listed so far.
    lazy: Option<&'a std::collections::HashSet<String>>,
    /// Uploads allowed.
    writable: bool,
    /// New folder and rename/move allowed (a storage root of an admin); `advanced`: delete too.
    manage: bool,
    advanced: bool,
}

/// A storage root's change actions in a row's context menu.
fn manage_menu(ui: &mut egui::Ui, path: &str, dir: bool, t: &mut TreeUi<'_>) {
    if !t.manage {
        return;
    }
    ui.separator();
    if dir && ui.button("+  New folder…").clicked() {
        *t.action = Some(TreeAction::NewFolder(path.to_owned()));
        ui.close();
    }
    if ui.button("✏  Rename / move…").clicked() {
        *t.action = Some(TreeAction::Rename(path.to_owned(), dir));
        ui.close();
    }
    if t.advanced && ui.button(RichText::new("🗑  Delete…").color(t.p.danger)).clicked() {
        *t.action = Some(TreeAction::Delete(path.to_owned(), dir));
        ui.close();
    }
}

fn show_dir(ui: &mut egui::Ui, dir: &Dir<'_>, prefix: &str, depth: usize, t: &mut TreeUi<'_>) {
    for (name, sub) in &dir.dirs {
        let path = format!("{prefix}{name}");
        let id = egui::Id::new(("workspace_ide_dir", &path));
        let unfolded = ui.data(|d| d.get_temp::<bool>(id)).unwrap_or_else(|| t.viewing.is_some_and(|v| v.starts_with(&format!("{path}/"))));
        let marked = t.drop_target.map_or(t.upload_dir == path, |target| target == path);
        let response = tree_row(ui, depth, Some(unfolded), None, name, marked, t.p);
        t.rows.push((response.rect, path.clone()));
        if response.clicked() {
            ui.data_mut(|d| d.insert_temp(id, !unfolded));
            *t.action = Some(TreeAction::Select(path.clone()));
        }
        response.context_menu(|ui| {
            if t.writable && ui.button("⬆  Upload files here…").clicked() {
                *t.action = Some(TreeAction::Upload(path.clone()));
                ui.close();
            }
            if ui.button("Copy the path").clicked() {
                ui.ctx().copy_text(format!("{path}/"));
                ui.close();
            }
            manage_menu(ui, &path, true, t);
        });
        if unfolded {
            if t.lazy.is_some_and(|loaded| !loaded.contains(&path)) {
                if t.action.is_none() {
                    *t.action = Some(TreeAction::Load(path.clone()));
                }
                ui.horizontal(|ui| {
                    ui.add_space(12.0 * (depth + 2) as f32);
                    ui.spinner();
                });
            }
            show_dir(ui, sub, &format!("{path}/"), depth + 1, t);
        }
    }
    for file in &dir.files {
        let name = file.path.rsplit('/').next().unwrap_or(&file.path);
        file_row(ui, file, name, depth, t);
    }
}

/// A file's row: a click opens it; ⬇ on hover and its context menu download it.
fn file_row(ui: &mut egui::Ui, file: &FileEntry, label: &str, depth: usize, t: &mut TreeUi<'_>) {
    let response = tree_row(ui, depth, None, Some((&file.path, file.bytes)), label, t.viewing == Some(file.path.as_str()), t.p);
    t.rows.push((response.rect, ide_transfer::parent(&file.path)));
    // Over the size, added after the row so it gets its own clicks.
    if ui.rect_contains_pointer(response.rect) {
        let at = egui::Rect::from_center_size(egui::pos2(response.rect.right() - 14.0, response.rect.center().y), egui::vec2(20.0, 20.0));
        ui.painter().rect_filled(at.expand2(egui::vec2(22.0, 0.0)).translate(egui::vec2(-11.0, 0.0)), 4, t.p.hover);
        if ui.place(at, icon_button("⬇", t.p)).on_hover_text("Download").clicked() {
            *t.action = Some(TreeAction::Download(file.path.clone()));
        }
    }
    if response.clicked() {
        *t.action = Some(TreeAction::Open(file.path.clone()));
    }
    response.context_menu(|ui| {
        if ui.button("⬇  Download").clicked() {
            *t.action = Some(TreeAction::Download(file.path.clone()));
            ui.close();
        }
        if ui.button("Copy the path").clicked() {
            ui.ctx().copy_text(file.path.clone());
            ui.close();
        }
        manage_menu(ui, &file.path, false, t);
    });
}

/// Larger text files are not opened (as the files routes' `MAX_TEXT_BYTES`).
const MAX_TEXT_BYTES: usize = 512 * 1024;

/// A storage listing as a tree: its files, and its folders (an empty one by its [`FOLDER_MARKER`]).
fn tree_of(list: &ObjectList) -> FileTree {
    let mut dirs = std::collections::BTreeSet::new();
    let mut entries = Vec::new();
    for object in &list.objects {
        let mut parts: Vec<&str> = object.key.split('/').collect();
        let name = parts.pop().unwrap_or_default();
        for i in 1..=parts.len() {
            dirs.insert(parts[..i].join("/"));
        }
        if name != FOLDER_MARKER {
            entries.push(FileEntry { path: object.key.clone(), bytes: object.bytes, dir: false });
        }
    }
    entries.extend(dirs.into_iter().map(|path| FileEntry { path, bytes: 0, dir: true }));
    entries.sort_by(|a, b| a.path.cmp(&b.path));
    FileTree { entries, truncated: list.truncated }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(path: &str, dir: bool) -> FileEntry {
        FileEntry { path: path.to_owned(), bytes: 1, dir }
    }

    #[test]
    fn folders_nest_and_empty_ones_show() {
        let entries = [entry("biw", true), entry("biw/_ssm.sh", false), entry("biw/out", true), entry("plan2cell.toml", false)];
        let tree = dir_tree(&entries);
        assert_eq!(tree.files.iter().map(|f| f.path.as_str()).collect::<Vec<_>>(), ["plan2cell.toml"]);
        let biw = &tree.dirs["biw"];
        assert_eq!(biw.files.iter().map(|f| f.path.as_str()).collect::<Vec<_>>(), ["biw/_ssm.sh"]);
        assert!(biw.dirs["out"].files.is_empty(), "an empty folder is listed");
    }

    #[test]
    fn storage_listings_become_trees() {
        use factoryos_client::protocol::storage::ObjectInfo;
        let object = |key: &str| ObjectInfo { key: key.to_owned(), bytes: 2, etag: None };
        let list = ObjectList { objects: vec![object("a/b/c.urdf"), object("empty/.keep"), object("top.txt")], truncated: false };
        let tree = tree_of(&list);
        let paths: Vec<(&str, bool)> = tree.entries.iter().map(|e| (e.path.as_str(), e.dir)).collect();
        assert_eq!(paths, [("a", true), ("a/b", true), ("a/b/c.urdf", false), ("empty", true), ("top.txt", false)]);
    }

    #[test]
    fn unsaved_until_saved() {
        let mut edit = Edit {
            text: "a".into(),
            saved_text: "a".into(),
            version: "1".into(),
            saving: false,
            conflict: false,
            error: None,
            gutter: (0, String::new()),
        };
        assert!(!edit.dirty());
        edit.text.push('b');
        assert!(edit.dirty());
        edit.saved_text = edit.text.clone();
        assert!(!edit.dirty());
    }
}
