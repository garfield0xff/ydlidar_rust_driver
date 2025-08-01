use core::fmt;

/// Error types for LIDAR operations.
#[derive(Debug)]
pub enum LidarError {
    /// Serial port error.
    #[cfg(feature = "std")]
    SerialPort(serialport::Error),
    /// I/O error.
    #[cfg(feature = "std")]
    Io(std::io::Error),
    /// Command timeout.
    Timeout,
    /// Not found device.
    NotFound,
    /// Invalid response from device.
    InvalidResponse,
    /// Command execution failed.
    CommandFailed,
    /// Buffer overflow.
    BufferOverflow,
}

impl fmt::Display for LidarError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            #[cfg(feature = "std")]
            LidarError::SerialPort(e) => write!(f, "Serial port error: {}", e),
            #[cfg(feature = "std")]
            LidarError::Io(e) => write!(f, "IO error: {}", e),
            LidarError::Timeout => write!(f, "Command timeout"),
            LidarError::NotFound => write!(f, "Device not found"),
            LidarError::InvalidResponse => write!(f, "Invalid response from LIDAR"),
            LidarError::CommandFailed => write!(f, "Command execution failed"),
            LidarError::BufferOverflow => write!(f, "Scan buffer overflow"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for LidarError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            LidarError::SerialPort(e) => Some(e),
            LidarError::Io(e) => Some(e),
            _ => None,
        }
    }
}

#[cfg(feature = "std")]
impl From<serialport::Error> for LidarError {
    fn from(e: serialport::Error) -> Self {
        LidarError::SerialPort(e)
    }
}

#[cfg(feature = "std")]
impl From<std::io::Error> for LidarError {
    fn from(e: std::io::Error) -> Self {
        LidarError::Io(e)
    }
}

impl embedded_io::Error for LidarError {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            #[cfg(feature = "std")]
            LidarError::SerialPort(_) => embedded_io::ErrorKind::Other,
            #[cfg(feature = "std")]
            LidarError::Io(_) => embedded_io::ErrorKind::Other,
            LidarError::NotFound => embedded_io::ErrorKind::NotFound,
            LidarError::Timeout => embedded_io::ErrorKind::TimedOut,
            LidarError::InvalidResponse => embedded_io::ErrorKind::InvalidData,
            LidarError::CommandFailed => embedded_io::ErrorKind::Other,
            LidarError::BufferOverflow => embedded_io::ErrorKind::OutOfMemory,
        }
    }
}

/// Type alias for Results with LidarError.
pub type Result<T> = core::result::Result<T, LidarError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = LidarError::Timeout;
        assert_eq!(format!("{}", err), "Command timeout");

        let err = LidarError::InvalidResponse;
        assert_eq!(format!("{}", err), "Invalid response from LIDAR");
    }

    #[test]
    fn test_embedded_io_error_kind() {
        use embedded_io::Error;

        let err = LidarError::Timeout;
        assert_eq!(err.kind(), embedded_io::ErrorKind::TimedOut);

        let err = LidarError::InvalidResponse;
        assert_eq!(err.kind(), embedded_io::ErrorKind::InvalidData);
    }
}
