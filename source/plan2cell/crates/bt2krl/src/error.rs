//! Errors and exit codes (same numbering as bt2plc).

use std::path::{Path, PathBuf};

#[derive(thiserror::Error, Debug)]
pub enum Error {
    /// The bt2plc output cannot be turned into a safe robot program.
    #[error("input rejected: {0}")]
    Input(String),
    #[error("usage: {0}")]
    Usage(String),
    #[error("config: {0}")]
    Config(String),
    #[error("{}: {source}", path.display())]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    /// `--strict` and at least one warning.
    #[error("--strict: {0}")]
    Strict(String),
    /// A verification (round trip, hashes) failed.
    #[error("verification failed: {0}")]
    Verify(String),
    /// The generated KRL failed the strict validator.
    #[error("generated KRL rejected by the validator:\n{0}")]
    Validate(String),
}

pub type Result<T> = std::result::Result<T, Error>;

impl Error {
    pub fn io(path: &Path, source: std::io::Error) -> Self {
        Error::Io {
            path: path.to_path_buf(),
            source,
        }
    }

    pub fn exit_code(&self) -> i32 {
        match self {
            Error::Input(_) => 1,
            Error::Usage(_) => 2,
            Error::Config(_) => 3,
            Error::Io { .. } => 4,
            Error::Strict(_) => 6,
            Error::Verify(_) => 7,
            Error::Validate(_) => 8,
        }
    }
}
