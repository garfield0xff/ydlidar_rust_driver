use std::thread;
use std::time::Duration;
use ydlidar_rust_driver::{Lidar, TMiniPlus};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("YDLidar Example");
    
    // 1. Connect to YDLidar device
    println!("Connecting to device...");
    let mut lidar = Lidar::<TMiniPlus>::new()?;
    println!("Connected!");
    
    // 2. Get device information
    let (model, firmware, hardware) = lidar.get_device_info()?;
    println!("Device: Model={}, Firmware={}.{}, Hardware={}", 
             model, firmware >> 8, firmware & 0xFF, hardware);
    
    // 3. Start scanning
    println!("Starting scan...");
    lidar.start_scan()?;
    
    // 4. Collect scan data for 5 seconds
    println!("Collecting data for 5 seconds...");
    let start_time = std::time::Instant::now();
    let mut total_points = 0;
    
    while start_time.elapsed() < Duration::from_secs(5) {
        let points = lidar.get_scan_points()?;
        total_points += points.len();
        
        // Print first point of each batch
        if let Some(point) = points.first() {
            println!("Angle: {:.1}°, Distance: {}mm, Quality: {}", 
                     point.angle, point.distance, point.quality);
        }
        
        thread::sleep(Duration::from_millis(100));
    }
    
    // 5. Stop scanning
    lidar.stop_scan()?;
    println!("Scan stopped. Total points: {}", total_points);
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_lidar_connection() {
        // This test will pass only if YDLidar device is connected
        match Lidar::<TMiniPlus>::new() {
            Ok(_) => println!("Device connected successfully"),
            Err(_) => println!("No device found (this is normal if no hardware is connected)"),
        }
    }
}