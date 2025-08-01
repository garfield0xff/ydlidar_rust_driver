/// Commands for YDLidar T-mini Plus model.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum TMiniPlusCmd {
    /// Enable command (0xA5).
    ENABLE,
    /// Start scanning command (0xA5 0x60).
    START_SCAN,
    /// Stop scanning command (0xA5 0x65).
    STOP_SCAN,
    /// Get device information command (0xA5 0x90).
    GET_INFO,
    /// Get device health status command (0xA5 0x92).
    GET_HEALTH,
    /// Increase frequency by 0.1Hz (0xA5 0x09).
    FREQ_UP_0_1HZ,
    /// Decrease frequency by 0.1Hz (0xA5 0x0A).
    FREQ_DOWN_0_1HZ,
    /// Increase frequency by 1Hz (0xA5 0x0B).
    FREQ_UP_1HZ,
    /// Decrease frequency by 1Hz (0xA5 0x0C).
    FREQ_DOWN_1HZ,
    /// Get current scan frequency (0xA5 0x0D).
    GET_FREQ,
    /// Soft restart device (0xA5 0x40).
    SOFT_RESTART,
}

impl TMiniPlusCmd {
    /// Get the command byte without header.
    ///
    /// Returns a reference to the command byte only (without 0xA5 header).
    ///
    /// # Example
    ///
    /// ```
    /// use ydlidar::cmd::TMiniPlusCmd;
    ///
    /// let cmd_byte = TMiniPlusCmd::START_SCAN.as_bytes();
    /// assert_eq!(cmd_byte, &[0x60]);
    /// ```
    pub const fn as_bytes(self) -> &'static [u8] {
        match self {
            Self::ENABLE => &[0xA5],
            Self::START_SCAN => &[0x60],
            Self::STOP_SCAN => &[0x65],
            Self::GET_INFO => &[0x90],
            Self::GET_HEALTH => &[0x92],
            Self::FREQ_UP_0_1HZ => &[0x09],
            Self::FREQ_DOWN_0_1HZ => &[0x0A],
            Self::FREQ_UP_1HZ => &[0x0B],
            Self::FREQ_DOWN_1HZ => &[0x0C],
            Self::GET_FREQ => &[0x0D],
            Self::SOFT_RESTART => &[0x40],
        }
    }

    /// Get the full command with header.
    ///
    /// Returns a reference to a static 2-byte array containing
    /// the header (0xA5) and command byte.
    ///
    /// # Example
    ///
    /// ```
    /// use ydlidar::cmd::TMiniPlusCmd;
    ///
    /// let cmd = TMiniPlusCmd::START_SCAN.as_cmd();
    /// assert_eq!(cmd[0], 0xA5);  // Header
    /// assert_eq!(cmd[1], 0x60);  // Command
    /// ```
    pub const fn as_cmd(self) -> &'static [u8; 2] {
        match self {
            Self::ENABLE => &[0xA5, 0xA5], // Special case: double header
            Self::START_SCAN => &[0xA5, 0x60],
            Self::STOP_SCAN => &[0xA5, 0x65],
            Self::GET_INFO => &[0xA5, 0x90],
            Self::GET_HEALTH => &[0xA5, 0x92],
            Self::FREQ_UP_0_1HZ => &[0xA5, 0x09],
            Self::FREQ_DOWN_0_1HZ => &[0xA5, 0x0A],
            Self::FREQ_UP_1HZ => &[0xA5, 0x0B],
            Self::FREQ_DOWN_1HZ => &[0xA5, 0x0C],
            Self::GET_FREQ => &[0xA5, 0x0D],
            Self::SOFT_RESTART => &[0xA5, 0x40],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command_values() {
        assert_eq!(TMiniPlusCmd::START_SCAN.as_cmd(), &[0xA5, 0x60]);
        assert_eq!(TMiniPlusCmd::STOP_SCAN.as_cmd(), &[0xA5, 0x65]);
        assert_eq!(TMiniPlusCmd::GET_INFO.as_cmd(), &[0xA5, 0x90]);
    }

    #[test]
    fn test_command_bytes() {
        assert_eq!(TMiniPlusCmd::START_SCAN.as_bytes(), &[0x60]);
        assert_eq!(TMiniPlusCmd::STOP_SCAN.as_bytes(), &[0x65]);
        assert_eq!(TMiniPlusCmd::GET_INFO.as_bytes(), &[0x90]);
    }
}