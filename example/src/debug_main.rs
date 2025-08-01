use std::thread;
use std::time::Duration;
use ydlidar_rust_driver::{Lidar, TMiniPlus};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("YDLidar Debug Example");
    println!("====================");

    // Check available serial ports
    println!("\n1. Checking serial ports...");
    match serialport::available_ports() {
        Ok(ports) => {
            for port in &ports {
                println!("   Port: {}", port.port_name);
                if let serialport::SerialPortType::UsbPort(usb) = &port.port_type {
                    if usb.vid == 0x10c4 && usb.pid == 0xea60 {
                        println!("   -> YDLidar device found!");
                    }
                }
            }
        }
        Err(e) => println!("   Error: {}", e),
    }

    // Connect to device
    println!("\n2. Connecting to device...");
    let mut lidar = match Lidar::<TMiniPlus>::new() {
        Ok(lidar) => {
            println!("   Connected successfully!");
            lidar
        }
        Err(e) => {
            println!("   Failed: {}", e);
            return Err(e.into());
        }
    };

    // Get device info
    println!("\n3. Getting device info...");
    match lidar.get_device_info() {
        Ok((model, firmware, hardware)) => {
            println!("   Model: {}", model);
            println!("   Firmware: {}.{}", firmware >> 8, firmware & 0xFF);
            println!("   Hardware: {}", hardware);
        }
        Err(e) => println!("   Failed: {}", e),
    }

    // Test scan
    println!("\n4. Testing scan...");
    match lidar.start_scan() {
        Ok(()) => {
            println!("   Scan started");

            // Wait and get sample data
            thread::sleep(Duration::from_millis(500));
            match lidar.get_scan_points() {
                Ok(points) => {
                    println!("   Got {} points", points.len());
                    if let Some(point) = points.first() {
                        println!(
                            "   Sample: {:.1}°, {}mm, Q:{}",
                            point.angle, point.distance, point.quality
                        );
                    }
                }
                Err(e) => println!("   Error getting points: {}", e),
            }

            lidar.stop_scan()?;
            println!("   Scan stopped");
        }
        Err(e) => println!("   Failed to start scan: {}", e),
    }

    println!("\nDebug completed!");
    Ok(())
}
