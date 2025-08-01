//! # LIDAR Driver Module

//! ## Example (std)
//!
//! ```no_run
//! use ydlidar::{Lidar, TMiniPlus};
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let mut lidar = Lidar::<TMiniPlus>::new()?;
//! lidar.start_scan()?;
//!
//! loop {
//!     let points = lidar.get_scan_points()?;
//!     for point in points {
//!         println!("Angle: {:.2}°, Distance: {}mm, Quality: {}",
//!                  point.angle, point.distance, point.quality);
//!     }
//! }
//! # }
//! ```
//!
//! ```no_run
//! # #![no_std]
//! use ydlidar::{Lidar, TMiniPlus};
//!
//! # fn example<UART: embedded_io::Read + embedded_io::Write>(uart: UART) -> Result<(), ydlidar::LidarError> {
//! let mut lidar = Lidar::<TMiniPlus, UART>::new(uart);
//! lidar.start_scan()?;
//!
//! loop {
//!     lidar.read_scan_data()?;
//!     let points = lidar.get_scan_points()?;
//!     // Process points...
//! }
//! # }
//! ```


use crate::types::LidarCommands;
use crate::error::LidarError;
use core::marker::PhantomData;


#[cfg(feature = "std")]
use std::time::Duration;
#[cfg(feature = "std")]
use embedded_io;
#[cfg(feature = "std")]
use serialport::{SerialPort, SerialPortType};
#[cfg(feature = "std")]
use anyhow::Result;
#[cfg(feature = "std")]
use std::sync::{Arc, Mutex};
#[cfg(feature = "std")]
use std::thread;
#[cfg(feature = "std")]
use std::collections::VecDeque;



const LIDAR_RESPONSE_HEADER: u8 = 0xA5;
const MAX_RETRY_COUNT: u8 = 3;
#[cfg(feature = "std")]
const COMMAND_TIMEOUT: Duration = Duration::from_millis(500);
const SCAN_BUFFER_SIZE: usize = 4096;
const SCAN_POINT_SIZE: usize = 5;

/// Scan point data from LIDAR.
#[derive(Debug, Clone, Copy)]
pub struct ScanPoint {
    /// Angle in degrees (0.0 to 360.0).
    pub angle: f32,
    /// Distance in millimeters.
    pub distance: u16,
    /// Signal quality (0-63, higher is better).
    pub quality: u8,
    /// Sync flag indicating start of new scan rotation.
    pub sync: bool,
}


/// LIDAR driver for standard environments.
#[cfg(feature = "std")]
pub struct Lidar<T> 
where T: LidarCommands + Send + 'static,
{

    port: Arc<Mutex<Box<dyn SerialPort>>>,
    scan_buffer: Arc<Mutex<VecDeque<u8>>>,
    scan_thread: Option<thread::JoinHandle<()>>,
    is_scanning: Arc<Mutex<bool>>,
    /// Type marker for LIDAR model.
    _phantom: PhantomData<T>,
}

/// LIDAR driver for no_std environments.
///
/// Provides direct UART control suitable for embedded systems
/// without heap allocation.
#[cfg(not(feature = "std"))]
pub struct Lidar<T, UART> 
where 
    T: LidarCommands,
    UART: embedded_io::Read + embedded_io::Write,
{
    /// UART interface for communication.
    uart: UART,
    /// Fixed-size scan data buffer.
    scan_buffer: heapless::Vec<u8, 4096>,
    /// Current scanning state.
    is_scanning: bool,
    /// Type marker for LIDAR model.
    _phantom: PhantomData<T>,
}

#[cfg(feature = "std")]
impl<T> Lidar<T> 
where T: LidarCommands + Send + 'static,
{
    /// Create a new LIDAR instance.
    ///
    /// Automatically detects and connects to the LIDAR device
    /// via USB (CP2102 chip).
    ///
    /// # Returns
    ///
    /// A new `Lidar` instance or an error if connection fails.
    ///
    /// # Errors
    ///
    /// - `LidarError::InvalidResponse` if no compatible device found
    /// - `LidarError::SerialPort` if port opening fails
    pub fn new() -> Result<Self, LidarError> {
        let port = Self::find_and_open_port()?;
        
        Ok(Self { 
            port: Arc::new(Mutex::new(port)),
            scan_buffer: Arc::new(Mutex::new(VecDeque::with_capacity(SCAN_BUFFER_SIZE))),
            scan_thread: None,
            is_scanning: Arc::new(Mutex::new(false)),
            _phantom: PhantomData,
        })
    }

    /// Find and open the LIDAR USB port.
    ///
    /// Searches for CP2102 USB-to-UART devices and opens the port
    /// with appropriate settings.
    fn find_and_open_port() -> Result<Box<dyn SerialPort>, LidarError> {
        let port_name = Self::find_port()?
            .ok_or(LidarError::InvalidResponse)?;

        let port = serialport::new(&port_name, 230400)
            .timeout(Duration::from_millis(100))
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .open()?;
        
        port.clear(serialport::ClearBuffer::All)?;
        
        Ok(port)
    }

    /// Find CP2102-based LIDAR device.
    ///
    /// Searches system ports for devices with CP2102 chip
    /// (VID: 0x10c4, PID: 0xea60).
    ///
    /// # Returns
    ///
    /// Port name if found, None otherwise.
    fn find_port() -> Result<Option<String>, LidarError> {
        let ports = serialport::available_ports()?;

        for port_info in ports {
            match &port_info.port_type {
                SerialPortType::UsbPort(usb_info) => {
                    // YDLidar T-mini Plus uses CP2102 USB-to-UART chip
                    if usb_info.vid == 0x10c4 && usb_info.pid == 0xea60 {
                        return Ok(Some(port_info.port_name));
                    }
                }
                _ => {}
            }
        }
        
        Ok(None)
    }

    /// Send command to LIDAR device.
    ///
    /// Clears input buffer before sending to ensure clean communication.
    fn send_command(&self, cmd: &[u8]) -> Result<(), LidarError> {
        let mut port = self.port.lock().unwrap();
        
        port.clear(serialport::ClearBuffer::Input)?;
        
        port.write_all(cmd)?;
        port.flush()?;
        
        Ok(())
    }
    
    /// Send command with automatic retry.
    ///
    /// Attempts to send command up to MAX_RETRY_COUNT times
    /// with small delays between attempts.
    fn send_command_with_retry(&self, cmd: &[u8]) -> Result<(), LidarError> {
        for _ in 0..MAX_RETRY_COUNT {
            match self.send_command(cmd) {
                Ok(_) => return Ok(()),
                Err(_e) => {
                    thread::sleep(Duration::from_millis(10));
                    continue;
                }
            }
        }
        Err(LidarError::CommandFailed)
    }
    
    /// Wait for specific response from device.
    ///
    /// Reads data until expected response header is found or timeout occurs.
    ///
    /// # Arguments
    ///
    /// * `expected_cmd` - Expected command byte in response
    /// * `timeout` - Maximum time to wait
    fn wait_for_response(&self, expected_cmd: u8, timeout: Duration) -> Result<Vec<u8>, LidarError> {
        let start = std::time::Instant::now();
        let mut response = Vec::new();
        let mut header_found = false;
        
        loop {
            if start.elapsed() > timeout {
                return Err(LidarError::Timeout);
            }
            
            let mut buf = [0u8; 1];
            let mut port = self.port.lock().unwrap();
            
            match port.read(&mut buf) {
                Ok(1) => {
                    if !header_found && buf[0] == LIDAR_RESPONSE_HEADER {
                        header_found = true;
                        response.push(buf[0]);
                    } else if header_found {
                        response.push(buf[0]);
                        if response.len() >= 2 && response[1] == expected_cmd {
                            return Ok(response);
                        }
                    }
                }
                Ok(_) => continue,
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    thread::sleep(Duration::from_millis(1));
                    continue;
                }
                Err(e) => return Err(e.into()),
            }
        }
    }

    /// Start scanning operation.
    ///
    /// Initiates continuous scanning and starts a background thread
    /// to collect data into the buffer.
    ///
    /// # Returns
    ///
    /// Ok(()) if scanning started successfully.
    ///
    /// # Errors
    ///
    /// - `LidarError::InvalidResponse` if device response is invalid
    /// - `LidarError::Timeout` if device doesn't respond
    pub fn start_scan(&mut self) -> Result<(), LidarError> {
        if *self.is_scanning.lock().unwrap() {
            return Ok(());
        }
        let cmd = T::start_scan_cmd();
        self.send_command_with_retry(cmd)?;
        
        let response = self.wait_for_response(cmd[1], COMMAND_TIMEOUT)?;
        if response.len() < 7 {
            return Err(LidarError::InvalidResponse);
        }
        
        *self.is_scanning.lock().unwrap() = true;
        
        let port_clone = Arc::clone(&self.port);
        let buffer_clone = Arc::clone(&self.scan_buffer);
        let is_scanning_clone = Arc::clone(&self.is_scanning);
        
        self.scan_thread = Some(thread::spawn(move || {
            let mut read_buf = [0u8; 512];
            
            while *is_scanning_clone.lock().unwrap() {
                let mut port = port_clone.lock().unwrap();
                
                match port.read(&mut read_buf) {
                    Ok(n) if n > 0 => {
                        let mut buffer = buffer_clone.lock().unwrap();
                        
                        for i in 0..n {
                            if buffer.len() >= SCAN_BUFFER_SIZE {
                                buffer.pop_front();
                            }
                            buffer.push_back(read_buf[i]);
                        }
                    }
                    Ok(_) => thread::sleep(Duration::from_millis(1)),
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                        thread::sleep(Duration::from_millis(1));
                    }
                    Err(_) => break,
                }
            }
        }));
        
        Ok(())
    }

    /// Stop scanning operation.
    ///
    /// Stops the scanning process and terminates the background
    /// data collection thread.
    pub fn stop_scan(&mut self) -> Result<(), LidarError> {
        if !*self.is_scanning.lock().unwrap() {
            return Ok(());
        }
        
        let cmd = T::stop_scan_cmd();
        self.send_command_with_retry(cmd)?;
        
        *self.is_scanning.lock().unwrap() = false;
        
        if let Some(thread) = self.scan_thread.take() {
            let _ = thread.join();
        }
        
        self.scan_buffer.lock().unwrap().clear();
        
        Ok(())
    }
    
    /// Get parsed scan points from buffer.
    ///
    /// Processes raw data in the buffer and returns parsed scan points.
    /// Processed data is removed from the buffer.
    ///
    /// # Returns
    ///
    /// Vector of scan points parsed from available data.
    ///
    /// # Protocol Details
    ///
    /// Each point is 5 bytes:
    /// - Byte 0: sync/quality flags
    /// - Bytes 1-2: angle data
    /// - Bytes 3-4: distance data
    pub fn get_scan_points(&self) -> Result<Vec<ScanPoint>, LidarError> {
        let mut buffer = self.scan_buffer.lock().unwrap();
        let mut points = Vec::new();
        
        while buffer.len() >= SCAN_POINT_SIZE {
            let sync_quality = buffer[0];
            let angle_low = buffer[1];
            let angle_high = buffer[2];
            let distance_low = buffer[3];
            let distance_high = buffer[4];
            
            let sync = (sync_quality & 0x01) != 0;
            let inv_sync = (sync_quality & 0x02) != 0;
            
            if sync == inv_sync {
                buffer.pop_front();
                continue;
            }
            
            let quality = (sync_quality >> 2) & 0x3F;
            let angle = ((angle_high as u16) << 7) | ((angle_low as u16) >> 1);
            let distance = ((distance_high as u16) << 8) | (distance_low as u16);
            
            let angle_deg = (angle as f32) / 64.0;
            
            points.push(ScanPoint {
                angle: angle_deg,
                distance,
                quality,
                sync,
            });
            
            for _ in 0..SCAN_POINT_SIZE {
                buffer.pop_front();
            }
        }
        
        Ok(points)
    }
    
    /// Get device information.
    ///
    /// Retrieves model number, firmware version, and hardware version
    /// from the connected device.
    ///
    /// # Returns
    ///
    /// Tuple of (model_number, firmware_version, hardware_version).
    pub fn get_device_info(&self) -> Result<(u8, u16, u16), LidarError> {
        let cmd = T::get_info_cmd();
        self.send_command_with_retry(cmd)?;
        
        let mut response = vec![0u8; 27];
        let mut port = self.port.lock().unwrap();
        
        let start = std::time::Instant::now();
        let mut total_read = 0;
        
        while total_read < 27 && start.elapsed() < COMMAND_TIMEOUT {
            match port.read(&mut response[total_read..]) {
                Ok(n) => total_read += n,
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    thread::sleep(Duration::from_millis(1));
                }
                Err(e) => return Err(e.into()),
            }
        }
        
        if total_read < 27 {
            return Err(LidarError::InvalidResponse);
        }
        
        if response[0] != LIDAR_RESPONSE_HEADER || response[1] != 0x14 {
            return Err(LidarError::InvalidResponse);
        }
        
        let model = response[7];
        let firmware_major = response[9];
        let firmware_minor = response[8];
        let firmware = ((firmware_major as u16) << 8) | (firmware_minor as u16);
        let hardware = response[10] as u16;
        
        Ok((model, firmware, hardware))
    }
}


#[cfg(feature = "std")]
impl<T> Drop for Lidar<T>
where T: LidarCommands + Send + 'static,
{
    fn drop(&mut self) {
        let _ = self.stop_scan();
    }
}

#[cfg(feature = "std")]
impl<T> embedded_io::ErrorType for Lidar<T>
where T: LidarCommands + Send + 'static,
{
    type Error = LidarError;
}

#[cfg(feature = "std")]
impl<T> embedded_io::Write for Lidar<T>
where T: LidarCommands + Send + 'static,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut port = self.port.lock().unwrap();
        port.write(buf).map_err(|e| e.into())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        let mut port = self.port.lock().unwrap();
        port.flush().map_err(|e| e.into())
    }
}

#[cfg(feature = "std")]
impl<T> embedded_io::Read for Lidar<T>
where T: LidarCommands + Send + 'static,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut port = self.port.lock().unwrap();
        port.read(buf).map_err(|e| e.into())
    }
}

// ==================== no_std implementation ====================

#[cfg(not(feature = "std"))]
impl<T, UART> Lidar<T, UART> 
where 
    T: LidarCommands,
    UART: embedded_io::Read + embedded_io::Write,
{
    /// Create new LIDAR instance for no_std environments.
    ///
    /// # Arguments
    ///
    /// * `uart` - UART interface for communication
    pub fn new(uart: UART) -> Self {
        Self {
            uart,
            scan_buffer: heapless::Vec::new(),
            is_scanning: false,
            _phantom: PhantomData,
        }
    }
    
    /// Send command to LIDAR device.
    ///
    /// Writes command bytes to UART interface.
    pub fn send_command(&mut self, cmd: &[u8]) -> Result<(), LidarError> {
        for &byte in cmd {
            let buf = [byte];
            self.uart.write(&buf).map_err(|_| LidarError::CommandFailed)?;
        }
        self.uart.flush().map_err(|_| LidarError::CommandFailed)?;
        Ok(())
    }
    
    /// Start scanning operation.
    ///
    /// Sends start command and waits for acknowledgment.
    pub fn start_scan(&mut self) -> Result<(), LidarError> {
        if self.is_scanning {
            return Ok(());
        }
        
        let cmd = T::start_scan_cmd();
        self.send_command(cmd)?;
        
        // Wait for response
        let mut response = [0u8; 7];
        let mut total_read = 0;
        while total_read < 7 {
            let mut buf = [0u8];
            match self.uart.read(&mut buf) {
                Ok(1) => {
                    response[total_read] = buf[0];
                    total_read += 1;
                }
                Ok(_) => continue,
                Err(_) => return Err(LidarError::Timeout),
            }
        }
        
        if response[0] != LIDAR_RESPONSE_HEADER || response[1] != cmd[1] {
            return Err(LidarError::InvalidResponse);
        }
        
        self.is_scanning = true;
        Ok(())
    }
    
    /// Stop scanning operation.
    ///
    /// Sends stop command and clears buffer.
    pub fn stop_scan(&mut self) -> Result<(), LidarError> {
        if !self.is_scanning {
            return Ok(());
        }
        
        let cmd = T::stop_scan_cmd();
        self.send_command(cmd)?;
        
        self.is_scanning = false;
        self.scan_buffer.clear();
        Ok(())
    }
    
    /// Read available scan data into buffer.
    ///
    /// Non-blocking read that adds available data to internal buffer.
    /// Call this regularly in your main loop.
    pub fn read_scan_data(&mut self) -> Result<(), LidarError> {
        let mut temp_buf = [0u8; 64];
        
        match self.uart.read(&mut temp_buf) {
            Ok(n) => {
                for i in 0..n {
                    if self.scan_buffer.len() < self.scan_buffer.capacity() {
                        self.scan_buffer.push(temp_buf[i]).map_err(|_| LidarError::BufferOverflow)?;
                    } else {
                        return Err(LidarError::BufferOverflow);
                    }
                }
                Ok(())
            }
            Err(_) => Ok(()), // No data available
        }
    }
    
    /// Get parsed scan points from buffer.
    ///
    /// Processes raw data and returns up to 64 scan points.
    /// Suitable for resource-constrained environments.
    pub fn get_scan_points(&mut self) -> Result<heapless::Vec<ScanPoint, 64>, LidarError> {
        let mut points = heapless::Vec::new();
        
        while self.scan_buffer.len() >= SCAN_POINT_SIZE {
            let sync_quality = self.scan_buffer[0];
            let angle_low = self.scan_buffer[1];
            let angle_high = self.scan_buffer[2];
            let distance_low = self.scan_buffer[3];
            let distance_high = self.scan_buffer[4];
            
            let sync = (sync_quality & 0x01) != 0;
            let inv_sync = (sync_quality & 0x02) != 0;
            
            if sync == inv_sync {
                // Invalid sync, skip one byte
                self.scan_buffer.remove(0);
                continue;
            }
            
            let quality = (sync_quality >> 2) & 0x3F;
            let angle = ((angle_high as u16) << 7) | ((angle_low as u16) >> 1);
            let distance = ((distance_high as u16) << 8) | (distance_low as u16);
            
            let angle_deg = (angle as f32) / 64.0;
            
            if points.push(ScanPoint {
                angle: angle_deg,
                distance,
                quality,
                sync,
            }).is_err() {
                break; // Points buffer full
            }
            
            // Remove processed bytes
            for _ in 0..SCAN_POINT_SIZE {
                self.scan_buffer.remove(0);
            }
        }
        
        Ok(points)
    }
    
    /// Get device information.
    ///
    /// # Returns
    ///
    /// Tuple of (model_number, firmware_version, hardware_version).
    pub fn get_device_info(&mut self) -> Result<(u8, u16, u16), LidarError> {
        let cmd = T::get_info_cmd();
        self.send_command(cmd)?;
        
        let mut response = [0u8; 27];
        let mut total_read = 0;
        while total_read < 27 {
            let mut buf = [0u8];
            match self.uart.read(&mut buf) {
                Ok(1) => {
                    response[total_read] = buf[0];
                    total_read += 1;
                }
                Ok(_) => continue,
                Err(_) => return Err(LidarError::Timeout),
            }
        }
        
        if response[0] != LIDAR_RESPONSE_HEADER || response[1] != 0x14 {
            return Err(LidarError::InvalidResponse);
        }
        
        let model = response[7];
        let firmware_major = response[9];
        let firmware_minor = response[8];
        let firmware = ((firmware_major as u16) << 8) | (firmware_minor as u16);
        let hardware = response[10] as u16;
        
        Ok((model, firmware, hardware))
    }
}

#[cfg(not(feature = "std"))]
impl<T, UART> Drop for Lidar<T, UART>
where 
    T: LidarCommands,
    UART: embedded_io::Read + embedded_io::Write,
{
    fn drop(&mut self) {
        let _ = self.stop_scan();
    }
}

#[cfg(not(feature = "std"))]
impl<T, UART> embedded_io::ErrorType for Lidar<T, UART>
where 
    T: LidarCommands,
    UART: embedded_io::Read + embedded_io::Write,
{
    type Error = LidarError;
}

#[cfg(not(feature = "std"))]
impl<T, UART> embedded_io::Write for Lidar<T, UART>
where 
    T: LidarCommands,
    UART: embedded_io::Read + embedded_io::Write,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.uart.write(buf).map_err(|_| LidarError::CommandFailed)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.uart.flush().map_err(|_| LidarError::CommandFailed)
    }
}

#[cfg(not(feature = "std"))]
impl<T, UART> embedded_io::Read for Lidar<T, UART>
where 
    T: LidarCommands,
    UART: embedded_io::Read + embedded_io::Write,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.uart.read(buf).map_err(|_| LidarError::CommandFailed)
    }
}