//! Path confinement: every path a tool accepts is relative to the server's root directory.

use std::path::{Component, Path, PathBuf};

#[derive(Clone, Debug)]
pub struct Sandbox {
    root: PathBuf,
}

impl Sandbox {
    pub fn new(root: &Path) -> std::io::Result<Self> {
        Ok(Self {
            root: root.canonicalize()?,
        })
    }

    pub fn root(&self) -> &Path {
        &self.root
    }

    /// Lexical check: relative, no `..`, no prefix / root components.
    fn relative(rel: &str) -> Result<PathBuf, String> {
        let p = Path::new(rel);
        if rel.is_empty() || p.is_absolute() {
            return Err(format!("path {rel:?} must be relative to the server root"));
        }
        for c in p.components() {
            match c {
                Component::Normal(_) | Component::CurDir => {}
                _ => return Err(format!("path {rel:?} must not leave the server root")),
            }
        }
        Ok(p.to_path_buf())
    }

    /// An existing file or directory inside the root (symlinks resolved, then checked again).
    pub fn existing(&self, rel: &str) -> Result<PathBuf, String> {
        let p = self.root.join(Self::relative(rel)?);
        let c = p.canonicalize().map_err(|e| format!("{rel}: {e}"))?;
        if !c.starts_with(&self.root) {
            return Err(format!("path {rel:?} resolves outside the server root"));
        }
        Ok(c)
    }

    /// A path that may not exist yet (an output directory): its deepest existing ancestor must
    /// lie inside the root.
    pub fn new_path(&self, rel: &str) -> Result<PathBuf, String> {
        let p = self.root.join(Self::relative(rel)?);
        let mut probe = p.as_path();
        loop {
            if probe.exists() {
                let c = probe.canonicalize().map_err(|e| format!("{rel}: {e}"))?;
                if !c.starts_with(&self.root) {
                    return Err(format!("path {rel:?} resolves outside the server root"));
                }
                return Ok(p);
            }
            probe = match probe.parent() {
                Some(x) => x,
                None => return Err(format!("path {rel:?} has no existing ancestor")),
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paths_stay_in_the_root() {
        let d = tempfile::tempdir().unwrap();
        std::fs::create_dir(d.path().join("a")).unwrap();
        std::fs::write(d.path().join("a/plan.json"), "{}").unwrap();
        let s = Sandbox::new(d.path()).unwrap();
        assert!(s.existing("a/plan.json").is_ok());
        assert!(s.existing("./a/plan.json").is_ok());
        for bad in ["/etc/passwd", "../x", "a/../../x", ""] {
            assert!(s.existing(bad).is_err(), "{bad}");
            assert!(s.new_path(bad).is_err(), "{bad}");
        }
        assert!(s.new_path("out/cell").is_ok());
        #[cfg(unix)]
        {
            std::os::unix::fs::symlink("/tmp", d.path().join("escape")).unwrap();
            assert!(s.existing("escape").is_err());
            assert!(s.new_path("escape/out").is_err());
        }
    }
}
