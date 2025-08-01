use crate::cmd::TMiniPlusCmd;

/// # Example
///
/// ```
/// use ydlidar::types::{LidarCommands, TMiniPlus};
///
/// let start_cmd = TMiniPlus::start_scan_cmd();
/// assert_eq!(start_cmd, &[0xA5, 0x60]);
/// ```
pub trait LidarCommands {
    /// The header structure for this LIDAR model's data packets.
    type Header;

    /// Get the command to start scanning.
    ///
    /// Returns a reference to the command byte array.
    fn start_scan_cmd() -> &'static [u8; 2];

    /// Get the command to stop scanning.
    ///
    /// Returns a reference to the command byte array.
    fn stop_scan_cmd() -> &'static [u8; 2];

    /// Get the command to retrieve device information.
    ///
    /// Returns a reference to the command byte array.
    fn get_info_cmd() -> &'static [u8; 2];
}

/// YDLidar T-mini Plus model implementation.
///
/// This struct represents the T-mini Plus LIDAR model and implements
/// the necessary traits for device communication.
#[derive(Debug, Clone, Copy)]
pub struct TMiniPlus;

impl LidarCommands for TMiniPlus {
    type Header = TMiniHeader;

    fn start_scan_cmd() -> &'static [u8; 2] {
        TMiniPlusCmd::START_SCAN.as_cmd()
    }

    fn stop_scan_cmd() -> &'static [u8; 2] {
        TMiniPlusCmd::STOP_SCAN.as_cmd()
    }

    fn get_info_cmd() -> &'static [u8; 2] {
        TMiniPlusCmd::GET_INFO.as_cmd()
    }
}

/// Data packet header for T-mini Plus.
///
/// # Fields
///
/// - `header`: Start marker (0x55AA)
/// - `ct`: Control byte containing scan frequency and package type
/// - `lsn`: Sample count in this packet
/// - `fsa`: First sample angle (raw value)
/// - `lsa`: Last sample angle (raw value)
/// - `cs`: Checksum for data integrity
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct TMiniHeader {
    /// Start marker (0x55AA).
    pub header: u16,
    /// Control byte: scan frequency and package type.
    pub ct: u8,
    /// Sample count in this packet.
    pub lsn: u8,
    /// First sample angle (raw value).
    pub fsa: u16,
    /// Last sample angle (raw value).
    pub lsa: u16,
    /// Checksum for data integrity.
    pub cs: u16,
}

impl TMiniHeader {
    /// Create a header from raw bytes.
    ///
    /// # Arguments
    ///
    /// * `data` - Raw byte array containing header data
    ///
    /// # Returns
    ///
    /// A new `TMiniHeader` instance parsed from the bytes.
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 10 {
            return None;
        }

        Some(TMiniHeader {
            header: u16::from_le_bytes([data[0], data[1]]),
            ct: data[2],
            lsn: data[3],
            fsa: u16::from_le_bytes([data[4], data[5]]),
            lsa: u16::from_le_bytes([data[6], data[7]]),
            cs: u16::from_le_bytes([data[8], data[9]]),
        })
    }

    /// Check if the header is valid.
    ///
    /// Verifies that the start marker is correct (0x55AA).
    pub fn is_valid(&self) -> bool {
        self.header == 0x55AA // Little endian: bytes are AA 55, value is 0x55AA
    }

    /// Get the scan frequency from the control byte.
    ///
    /// # Returns
    ///
    /// The scan frequency in Hz.
    pub fn scan_frequency(&self) -> f32 {
        match self.ct & 0x03 {
            0 => 5.0,
            1 => 8.0,
            2 => 10.0,
            _ => 0.0,
        }
    }
}

/// Each sample in a scan packet contains distance and intensity information.
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct SampleNode {
    /// Signal intensity (0-255).
    pub intensity: u8,
    /// Low byte of distance value.
    pub distance_low: u8,
    /// High byte of distance and flag bits.
    pub distance_high_and_flag: u8,
}

impl SampleNode {
    /// # Returns
    ///
    /// Distance in millimeters.
    pub fn distance(&self) -> u16 {
        // where s1=intensity, s2=distance_low, s3=distance_high_and_flag
        ((self.distance_high_and_flag as u16) << 6) | ((self.distance_low as u16) >> 2)
    }

    /// # Returns
    ///
    /// True if the sample should be discarded.
    pub fn is_invalid(&self) -> bool {
        (self.distance_high_and_flag & 0x80) != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tmini_header_parsing() {
        let data = [0x55, 0xAA, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        let header = TMiniHeader::from_bytes(&data).unwrap();
        assert!(header.is_valid());
        assert_eq!(header.lsn, 1);
    }

    #[test]
    fn test_sample_node_distance() {
        let sample = SampleNode {
            intensity: 100,
            distance_low: 0x34,
            distance_high_and_flag: 0x12,
        };
        assert_eq!(sample.distance(), 0x1234);
        assert!(!sample.is_invalid());
    }
}
