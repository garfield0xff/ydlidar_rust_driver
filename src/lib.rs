//! # YDLidar Driver
//! 
//! rust implementation of the YDLidar
//! ## Quick Start
//!
//! ```no_run
//! use ydlidar::{Lidar, TMiniPlus};
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! // Create a new LIDAR instance (auto-detects USB port)
//! let mut lidar = Lidar::<TMiniPlus>::new()?;
//!
//! // Get device information
//! let (model, firmware, hardware) = lidar.get_device_info()?;
//! println!("Model: {}, Firmware: {}, Hardware: {}", model, firmware, hardware);
//!
//! // Start scanning
//! lidar.start_scan()?;
//!
//! // Read scan data
//! loop {
//!     let points = lidar.get_scan_points()?;
//!     for point in points {
//!         println!("Angle: {:.2}°, Distance: {}mm, Quality: {}",
//!                  point.angle, point.distance, point.quality);
//!     }
//! }
//! # }
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![warn(missing_docs)]
#![warn(rust_2018_idioms)]

/// Command definitions for LIDAR models.
pub mod cmd;

/// Error types and implementations.
pub mod error;

/// Main LIDAR driver implementation.
pub mod lidar;

/// Core types and traits.
pub mod types;

// Re-export main types for convenience
pub use crate::error::LidarError;
pub use crate::lidar::{Lidar, ScanPoint};
pub use crate::types::{LidarCommands, TMiniPlus};