# YDLidar Rust Driver

High-performance Rust driver for YDLidar sensors with support for multiple models.

## Supported Models

### Currently Implemented
- **YDLidar T-mini Plus**

### Compatible Models (Can be added via LidarCommands trait)
- YDLidar X2/X2L
- YDLidar X4
- YDLidar G2/G4
- YDLidar S2/S4

## Module Specifications

### T-mini Plus
- **Scanning Range**: 0.1 - 12m
- **Scan Rate**: 5Hz - 10Hz (adjustable)
- **Ranging Frequency**: 3000Hz
- **Angle Resolution**: 0.5° - 1.5°
- **Measurement Resolution**: 1mm
- **Scan Angle**: 360°

### Communication Interface
- **Protocol**: YDLidar Protocol v1.3
- **Interface**: UART
- **Baud Rate**: 230400 bps
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None

### USB Connection
- **USB Chip**: CP2102 (Silicon Labs)
- **VID**: 0x10c4
- **PID**: 0xEA60
- **Auto-detection**: Supported

### Data Format
- **Packet Size**: 5 bytes per point
- **Angle Encoding**: 14-bit (0.0055° resolution)
- **Distance Encoding**: 16-bit (mm)
- **Quality**: 6-bit (0-63)

## Usage

```rust
use yd_driver::{Lidar, TMiniPlus};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Auto-detect USB port
    let mut lidar = Lidar::<TMiniPlus>::new()?;
    
    // Get device info
    let (model, firmware, hardware) = lidar.get_device_info()?;
    println!("Model: {}, FW: {:#x}, HW: {}", model, firmware, hardware);
    
    // Start scanning
    lidar.start_scan()?;
    
    loop {
        let points = lidar.get_scan_points()?;
        for point in points {
            println!("{}°: {}mm (Q:{})", point.angle, point.distance, point.quality);
        }
    }
}
```

## No-std Usage

```rust
use yd_driver::{Lidar, TMiniPlus};

let mut lidar = Lidar::<TMiniPlus, UART>::new(uart);
lidar.start_scan()?;

loop {
    lidar.read_scan_data()?;
    let points = lidar.get_scan_points()?;
    // Process points...
}
```
    fn get_info_cmd() -> &'static [u8; 2] {
        &[0xA5, 0x90]
    }
}
```
