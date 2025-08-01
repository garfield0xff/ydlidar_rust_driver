use std::error::Error;
use std::thread;
use std::time::Duration;
use ydlidar_rust_driver::{Lidar, TMiniPlus, ScanPoint};
use log::{info, warn, error, debug, trace};
use colored::*;

fn init_logger() {
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp_secs()
        .init();
}

fn print_header() {
    println!("{}", "=".repeat(60).bright_blue());
    println!("{}", "YDLidar Rust Driver - Enhanced Example".bright_cyan().bold());
    println!("{}", "=".repeat(60).bright_blue());
}

fn print_section(title: &str) {
    println!("\n{} {}", "📋".bright_yellow(), title.bright_white().bold());
    println!("{}", "-".repeat(40).dimmed());
}

fn print_success(msg: &str) {
    println!("{} {}", "✅".green(), msg.green());
}

fn print_error(msg: &str) {
    println!("{} {}", "❌".red(), msg.red());
}

fn print_info(msg: &str) {
    println!("{} {}", "ℹ️".blue(), msg.blue());
}

fn main() -> Result<(), Box<dyn Error>> {
    init_logger();
    print_header();
    
    info!("Starting YDLidar test program");
    
    // Step 1: Connect to device
    print_section("Device Connection");
    info!("Attempting to connect to YDLidar device...");
    
    let mut lidar = match Lidar::<TMiniPlus>::new() {
        Ok(lidar) => {
            print_success("Connected to YDLidar device successfully");
            info!("Device connection established");
            lidar
        }
        Err(e) => {
            print_error(&format!("Failed to connect to device: {}", e));
            error!("Device connection failed: {}", e);
            return Err(e.into());
        }
    };
    
    // Step 2: Get device information
    print_section("Device Information");
    info!("Retrieving device information...");
    
    match lidar.get_device_info() {
        Ok((model, firmware, hardware)) => {
            print_success("Device information retrieved");
            println!("  {} Model: {}", "🔧".bright_green(), model.to_string().bright_white());
            println!("  {} Firmware: {}.{}", "💾".bright_green(), 
                    (firmware >> 8).to_string().bright_white(), 
                    (firmware & 0xFF).to_string().bright_white());
            println!("  {} Hardware: {}", "⚙️".bright_green(), hardware.to_string().bright_white());
            
            info!("Device info - Model: {}, Firmware: {}.{}, Hardware: {}", 
                  model, firmware >> 8, firmware & 0xFF, hardware);
        }
        Err(e) => {
            print_error(&format!("Failed to get device info: {}", e));
            error!("Device info retrieval failed: {}", e);
            return Err(e.into());
        }
    }
    
    // Step 3: Start scanning
    print_section("Scan Operation");
    info!("Starting scan operation...");
    
    match lidar.start_scan() {
        Ok(()) => {
            print_success("Scan started successfully");
            info!("Scan operation initiated");
        }
        Err(e) => {
            print_error(&format!("Failed to start scan: {}", e));
            error!("Scan start failed: {}", e);
            return Err(e.into());
        }
    }
    
    // Allow some time for data to accumulate
    print_info("Waiting for scan data to accumulate...");
    thread::sleep(Duration::from_millis(1000));
    
    // Step 4: Read scan data
    print_section("Scan Data Collection");
    info!("Collecting scan data for 10 seconds...");
    
    println!("{}", "Scan Data Format:".bright_cyan());
    println!("  {} Angle (degrees) | {} Distance (mm) | {} Quality (0-63) | {} Sync", 
             "📐".bright_yellow(), "📏".bright_green(), "⭐".bright_blue(), "🔄".bright_magenta());
    println!("{}", "-".repeat(80).dimmed());
    
    let start_time = std::time::Instant::now();
    let mut total_points = 0;
    let mut sync_count = 0;
    let mut distance_stats = DistanceStats::new();
    
    while start_time.elapsed() < Duration::from_secs(10) {
        match lidar.get_scan_points() {
            Ok(points) => {
                for point in &points {
                    total_points += 1;
                    distance_stats.update(point.distance);
                    
                    if point.sync {
                        sync_count += 1;
                        debug!("Sync marker detected at angle {:.2}°", point.angle);
                    }
                    
                    // Print every 30th point to avoid flooding
                    if total_points % 30 == 0 {
                        print_scan_point(point, total_points);
                    }
                    
                    trace!("Point {}: {:.2}° {}mm Q:{} Sync:{}", 
                           total_points, point.angle, point.distance, point.quality, point.sync);
                }
            }
            Err(e) => {
                warn!("Failed to get scan points: {}", e);
            }
        }
        
        thread::sleep(Duration::from_millis(50));
    }
    
    // Step 5: Display statistics
    print_section("Scan Statistics");
    print_scan_statistics(total_points, sync_count, &distance_stats);
    
    // Step 6: Stop scanning
    print_section("Cleanup");
    info!("Stopping scan operation...");
    
    match lidar.stop_scan() {
        Ok(()) => {
            print_success("Scan stopped successfully");
            info!("Scan operation terminated");
        }
        Err(e) => {
            warn!("Error stopping scan: {}", e);
        }
    }
    
    println!("\n{}", "🎉 Test completed successfully!".bright_green().bold());
    info!("YDLidar test program completed");
    
    Ok(())
}

fn print_scan_point(point: &ScanPoint, index: usize) {
    let angle_str = format!("{:6.2}°", point.angle);
    let distance_str = format!("{:5}mm", point.distance);
    let quality_str = format!("Q:{:2}", point.quality);
    let sync_icon = if point.sync { "🔄" } else { " " };
    
    println!("  {:5} │ {} │ {} │ {} │ {}", 
             index.to_string().dimmed(),
             angle_str.bright_yellow(),
             distance_str.bright_green(), 
             quality_str.bright_blue(),
             sync_icon);
}

fn print_scan_statistics(total_points: usize, sync_count: usize, stats: &DistanceStats) {
    println!("{} Total points collected: {}", 
             "📊".bright_cyan(), total_points.to_string().bright_white().bold());
    println!("{} Sync markers detected: {}", 
             "🔄".bright_magenta(), sync_count.to_string().bright_white().bold());
    println!("{} Full rotations: ~{}", 
             "🌀".bright_green(), (sync_count as f32 / 1.0).round().to_string().bright_white().bold());
    
    if total_points > 0 {
        println!("{} Distance statistics:", "📏".bright_blue());
        println!("    Min: {}mm", stats.min.to_string().bright_green());
        println!("    Max: {}mm", stats.max.to_string().bright_red());
        println!("    Avg: {:.1}mm", stats.average().to_string().bright_yellow());
    }
    
    info!("Statistics - Points: {}, Sync: {}, Rotations: ~{}", 
          total_points, sync_count, sync_count);
}

struct DistanceStats {
    min: u16,
    max: u16,
    sum: u64,
    count: usize,
}

impl DistanceStats {
    fn new() -> Self {
        Self {
            min: u16::MAX,
            max: 0,
            sum: 0,
            count: 0,
        }
    }
    
    fn update(&mut self, distance: u16) {
        if distance > 0 { // Ignore zero distances
            self.min = self.min.min(distance);
            self.max = self.max.max(distance);
            self.sum += distance as u64;
            self.count += 1;
        }
    }
    
    fn average(&self) -> f64 {
        if self.count > 0 {
            self.sum as f64 / self.count as f64
        } else {
            0.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_distance_stats() {
        let mut stats = DistanceStats::new();
        stats.update(1000);
        stats.update(2000);
        stats.update(3000);
        
        assert_eq!(stats.min, 1000);
        assert_eq!(stats.max, 3000);
        assert_eq!(stats.average(), 2000.0);
    }
    
    #[test]
    fn test_device_connection() {
        env_logger::try_init().ok();
        
        match Lidar::<TMiniPlus>::new() {
            Ok(lidar) => {
                info!("Device connection test passed");
                if let Ok((model, firmware, hardware)) = lidar.get_device_info() {
                    info!("Device info - Model: {}, Firmware: {}, Hardware: {}", 
                         model, firmware, hardware);
                }
            }
            Err(e) => {
                warn!("Device connection test failed: {}", e);
                println!("This is expected if no YDLidar device is connected");
            }
        }
    }
}