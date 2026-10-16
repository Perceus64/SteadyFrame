use std::fmt;

/// Errors returned by the stabilization engine.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// Frame buffers or dimensions that don't match what was configured.
    InvalidInput(String),
    /// A configuration value that is out of range or unparseable.
    Config(String),
    /// Gyro data that is missing, malformed, or could not be synchronised.
    Gyro(String),
    /// A file format (CSV, GPMF, JSON) that could not be parsed.
    Parse(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::InvalidInput(m) => write!(f, "invalid input: {m}"),
            Error::Config(m) => write!(f, "configuration error: {m}"),
            Error::Gyro(m) => write!(f, "gyro error: {m}"),
            Error::Parse(m) => write!(f, "parse error: {m}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<serde_json::Error> for Error {
    fn from(e: serde_json::Error) -> Self {
        Error::Parse(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, Error>;
