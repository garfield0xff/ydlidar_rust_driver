use std::error::Error;
use std::thread;
use std::time::Duration;
use ydlidar_rust_driver::{Lidar, TMiniPlus};
use log::{info, warn, error, debug};
use colored::*;

fn init_logger() {
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Debug)
        .format_timestamp_secs()
        .init();
}

fn print_header() {
    println!("{}", "=".repeat(70).bright_blue());
    println!("{}", "YDLidar Enhanced Debug Program".bright_cyan().bold());
    println!("{}", "=".repeat(70).bright_blue());
}

fn print_step(step: usize, title: &str) {
    println!("\n{} {} {}", 
             format!("Step {}", step).bright_magenta().bold(),
             "│".dimmed(),
             title.bright_white().bold());
    println!("{}", "─".repeat(50).dimmed());
}

fn main() -> Result<(), Box<dyn Error>> {
    init_logger();
    print_header();
    
    info!("Starting enhanced debug program");
    
    // Step 1: Check available ports
    print_step(1, "Serial Port Discovery");
    info!("Scanning for available serial ports...");
    
    match serialport::available_ports() {
        Ok(ports) => {
            info!("Found {} serial ports", ports.len());
            println!("{} Available serial ports:", "🔍".bright_yellow());
            
            for (i, port) in ports.iter().enumerate() {
                println!("  {} {}", 
                         format!("{}.", i + 1).dimmed(),
                         port.port_name.bright_cyan());
                
                match &port.port_type {
                    serialport::SerialPortType::UsbPort(usb_info) => {
                        println!("    {} USB Device", "🔌".bright_green());
                        println!("    {} VID: {} PID: {}", 
                                "├─".dimmed(),
                                format!("0x{:04x}", usb_info.vid).bright_yellow(),
                                format!("0x{:04x}", usb_info.pid).bright_yellow());
                        
                        if let Some(manufacturer) = &usb_info.manufacturer {
                            println!("    {} Manufacturer: {}", "├─".dimmed(), manufacturer.bright_white());
                        }
                        if let Some(product) = &usb_info.product {
                            println!("    {} Product: {}", "├─".dimmed(), product.bright_white());
                        }
                        if let Some(serial) = &usb_info.serial_number {
                            println!("    {} Serial: {}", "└─".dimmed(), serial.bright_white());
                        }
                        
                        // Check if this is our target device
                        if usb_info.vid == 0x10c4 && usb_info.pid == 0xea60 {
                            println!("    {} {}", "🎯".bright_green(), "YDLidar Device Detected!".bright_green().bold());
                            info!("YDLidar device found at port: {}", port.port_name);
                        }
                        
                        debug!("USB device details - VID: 0x{:04x}, PID: 0x{:04x}", 
                               usb_info.vid, usb_info.pid);
                    }
                    _ => {
                        println!("    {} Non-USB port", "📡".dimmed());
                    }
                }
                println!();
            }
        }
        Err(e) => {
            error!("Failed to enumerate ports: {}", e);
            println!("{} {}", "❌".red(), format!("Error listing ports: {}", e).red());
            return Err(e.into());
        }
    }
    
    // Step 2: Create LIDAR instance
    print_step(2, "Device Connection");
    info!("Attempting to create LIDAR instance...");
    
    let mut lidar = match Lidar::<TMiniPlus>::new() {
        Ok(lidar) => {
            println!("{} {}", "✅".green(), "LIDAR instance created successfully".green().bold());
            info!("LIDAR connection established");
            lidar
        }
        Err(e) => {
            println!("{} {}", "❌".red(), format!("Failed to create LIDAR instance: {}", e).red());
            error!("LIDAR connection failed: {}", e);
            return Err(e.into());
        }
    };
    
    // Step 3: Get device information
    print_step(3, "Device Information Retrieval");
    info!("Retrieving device information...");
    
    match lidar.get_device_info() {
        Ok((model, firmware, hardware)) => {
            println!("{} {}", "✅".green(), "Device information retrieved successfully".green().bold());
            
            println!("\n{} Device Details:", "📋".bright_blue());
            println!("  {} Model Number: {}", "🔧".bright_green(), model.to_string().bright_white().bold());
            println!("  {} Firmware Version: {}.{}", "💾".bright_green(), 
                    (firmware >> 8).to_string().bright_white().bold(), 
                    (firmware & 0xFF).to_string().bright_white().bold());
            println!("  {} Hardware Version: {}", "⚙️".bright_green(), hardware.to_string().bright_white().bold());
            
            info!("Device info retrieved - Model: {}, Firmware: {}.{}, Hardware: {}", 
                  model, firmware >> 8, firmware & 0xFF, hardware);
            debug!("Raw firmware value: 0x{:04x}", firmware);
        }
        Err(e) => {
            println!("{} {}", "❌".red(), format!("Failed to get device info: {}", e).red());
            error!("Device info retrieval failed: {}", e);
            return Err(e.into());
        }
    }
    
    // Step 4: Test scan operation
    print_step(4, "Scan Operation Test");
    info!("Testing scan functionality...");
    
    match lidar.start_scan() {
        Ok(()) => {
            println!("{} {}", "✅".green(), "Scan started successfully".green().bold());
            info!("Scan operation initiated");
            
            // Wait for data accumulation
            print!("{} Waiting for scan data", "⏳".bright_yellow());
            for i in 0..10 {
                thread::sleep(Duration::from_millis(100));
                print!(".");
                if i % 3 == 2 {
                    print!(" ");
                }
            }
            println!();
            
            // Get sample points
            match lidar.get_scan_points() {
                Ok(points) => {
                    println!("{} {} scan points received", 
                             "📊".bright_cyan(), points.len().to_string().bright_white().bold());
                    
                    if !points.is_empty() {
                        println!("\n{} Sample Points (first 5):", "🎯".bright_green());
                        println!("  {} | {} | {} | {}", 
                                "Angle".bright_yellow().bold(),
                                "Distance".bright_green().bold(),
                                "Quality".bright_blue().bold(),
                                "Sync".bright_magenta().bold());
                        println!("  {}", "─".repeat(40).dimmed());
                        
                        for (i, point) in points.iter().take(5).enumerate() {
                            let sync_indicator = if point.sync { "🔄 Yes" } else { "   No" };
                            println!("  {:>6.2}° | {:>6}mm | {:>7} | {}", 
                                    point.angle.to_string().bright_yellow(),
                                    point.distance.to_string().bright_green(),
                                    point.quality.to_string().bright_blue(),
                                    sync_indicator);
                            
                            debug!("Sample point {}: angle={:.2}°, distance={}mm, quality={}, sync={}", 
                                   i, point.angle, point.distance, point.quality, point.sync);
                        }
                        
                        if points.len() > 5 {
                            println!("  {} ... and {} more points", 
                                    "   ".dimmed(), (points.len() - 5).to_string().dimmed());
                        }
                    } else {
                        warn!("No scan points received");
                        println!("{} {}", "⚠️".bright_yellow(), "No scan points received yet".bright_yellow());
                    }
                    
                    info!("Scan test completed with {} points", points.len());
                }
                Err(e) => {
                    warn!("Failed to get scan points: {}", e);
                    println!("{} {}", "⚠️".bright_yellow(), format!("Failed to get scan points: {}", e).bright_yellow());
                }
            }
            
            // Stop scan
            println!("\n{} Stopping scan...", "🛑".bright_red());
            match lidar.stop_scan() {
                Ok(()) => {
                    println!("{} {}", "✅".green(), "Scan stopped successfully".green().bold());
                    info!("Scan operation terminated");
                }
                Err(e) => {
                    warn!("Error stopping scan: {}", e);
                    println!("{} {}", "⚠️".bright_yellow(), format!("Warning: Error stopping scan: {}", e).bright_yellow());
                }
            }
        }
        Err(e) => {
            println!("{} {}", "❌".red(), format!("Failed to start scan: {}", e).red());
            error!("Scan start failed: {}", e);
            return Err(e.into());
        }
    }
    
    // Step 5: Summary
    print_step(5, "Test Summary");
    println!("{} All tests completed successfully!", "🎉".bright_green());
    println!("{} Device is ready for use", "✨".bright_cyan());
    
    info!("Enhanced debug program completed successfully");
    
    Ok(())
}