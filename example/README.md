# YDLidar Rust Driver Example

Simple examples showing how to use the `ydlidar-rust-driver` library.

## Quick Start

1. **Connect your YDLidar device** to a USB port

2. **Run the basic example:**
   ```bash
   cd example
   cargo run
   ```

3. **Run tests:**
   ```bash
   cargo test
   ```

## Examples

### Basic Example (`cargo run`)
```rust
use ydlidar_rust_driver::{Lidar, TMiniPlus};

// Connect to device
let mut lidar = Lidar::<TMiniPlus>::new()?;

// Get device info
let (model, firmware, hardware) = lidar.get_device_info()?;

// Start scanning
lidar.start_scan()?;

// Get scan points
let points = lidar.get_scan_points()?;
for point in points {
    println!("Angle: {}°, Distance: {}mm", point.angle, point.distance);
}

// Stop scanning
lidar.stop_scan()?;
```

### Debug Example (`cargo run --bin debug`)
- Check available serial ports
- Test device connection
- Verify device information
- Test basic scanning functionality

## Output Example

```
YDLidar Example
Connecting to device...
Connected!
Device: Model=4, Firmware=1.3, Hardware=1
Starting scan...
Collecting data for 5 seconds...
Angle: 104.5°, Distance: 43529mm, Quality: 2
Angle: 466.2°, Distance: 9586mm, Quality: 43
Angle: 31.1°, Distance: 0mm, Quality: 4
...
Scan stopped. Total points: 3724
```

## Requirements

- **Hardware:** YDLidar T-Mini Plus (or compatible)
- **Connection:** CP2102 USB-to-UART chip (VID: 0x10c4, PID: 0xea60)
- **Permissions:** Serial port access (may require `sudo` or adding user to `dialout` group)

## API Overview

| Function | Description |
|----------|-------------|
| `Lidar::new()` | Connect to YDLidar device |
| `get_device_info()` | Get model, firmware, hardware version |
| `start_scan()` | Begin scanning |
| `get_scan_points()` | Retrieve scan data points |
| `stop_scan()` | Stop scanning |

Each scan point contains:
- `angle`: Angle in degrees (0.0 to 360.0)
- `distance`: Distance in millimeters  
- `quality`: Signal quality (0-63, higher is better)
- `sync`: Sync flag indicating start of new rotation