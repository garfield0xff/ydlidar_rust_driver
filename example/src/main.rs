use std::thread;
use std::time::Duration;
use ydlidar_rust_driver::{Lidar, LidarError, TMiniPlus};

fn main() -> Result<(), LidarError> {
    println!("YDLidar Example");

    println!("Connecting to device...");
    let mut lidar = match Lidar::<TMiniPlus>::new() {
        Ok(lidar) => {
            println!("Connected!");
            lidar
        }
        Err(e) => {
            println!("Error: {}", e);
            return Err(e);
        }
    };

    println!("Starting scan...");
    lidar.start_scan()?;

    println!("Collecting data for 5 seconds...");
    let start_time = std::time::Instant::now();
    let mut total_points = 0;
    let mut empty_reads = 0;

    while start_time.elapsed() < Duration::from_secs(5) {
        let points = lidar.get_scan_points()?;

        total_points += points.len();
        println!("Received {} points", points.len());

        for (i, point) in points.iter().take(3).enumerate() {
            println!(
                "  Point {}: Angle: {:.1}°, Distance: {}mm, Quality: {}, Sync: {}",
                i + 1,
                point.angle,
                point.distance,
                point.quality,
                point.sync
            );
        }
        if points.len() > 3 {
            println!("  ... and {} more points", points.len() - 3);
        }

        thread::sleep(Duration::from_millis(100));
    }

    lidar.stop_scan()?;
    println!("Scan stopped. Total points: {}", total_points);

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lidar_connection() {
        match Lidar::<TMiniPlus>::new() {
            Ok(_) => println!("Device connected successfully"),
            Err(_) => println!("No device found (this is normal if no hardware is connected)"),
        }
    }
}
