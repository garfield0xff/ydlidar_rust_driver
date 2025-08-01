use serialport::{SerialPortType};
use std::time::Duration;
use std::thread;
use log::{info, warn, error, debug, trace};
use colored::*;

fn init_logger() {
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Trace)
        .format_timestamp_secs()
        .init();
}

fn print_header() {
    println!("{}", "=".repeat(70).bright_blue());
    println!("{}", "YDLidar Protocol Analyzer".bright_cyan().bold());
    println!("{}", "=".repeat(70).bright_blue());
}

fn print_command_test(cmd_name: &str, cmd_bytes: &[u8]) {
    println!("\n{} Testing {} Command", "🔬".bright_yellow(), cmd_name.bright_white().bold());
    print!("   Command bytes: ");
    for (i, &byte) in cmd_bytes.iter().enumerate() {
        if i > 0 { print!(" "); }
        print!("{}", format!("0x{:02X}", byte).bright_cyan());
    }
    println!();
    println!("   {}", "─".repeat(50).dimmed());
}

fn print_hex_dump(data: &[u8], title: &str) {
    println!("\n{} {} ({} bytes):", "📦".bright_green(), title.bright_white().bold(), data.len());
    
    if data.is_empty() {
        println!("   {}", "No data received".dimmed());
        return;
    }
    
    print!("   Hex: ");
    for (i, &byte) in data.iter().enumerate() {
        if i > 0 && i % 16 == 0 {
            println!();
            print!("        ");
        } else if i > 0 && i % 8 == 0 {
            print!("  ");
        } else if i > 0 {
            print!(" ");
        }
        
        // Color coding for different byte ranges
        let colored_byte = match byte {
            0xA5 => format!("0x{:02X}", byte).bright_red().bold(),
            0x5A | 0x14 => format!("0x{:02X}", byte).bright_yellow().bold(),
            0x00 => format!("0x{:02X}", byte).dimmed(),
            0x01..=0x0F => format!("0x{:02X}", byte).bright_blue(),
            0x10..=0x7F => format!("0x{:02X}", byte).bright_green(),
            0x80..=0xFF => format!("0x{:02X}", byte).bright_magenta(),
        };
        print!("{}", colored_byte);
    }
    println!();
    
    // ASCII representation
    print!("   ASCII: ");
    for &byte in data.iter() {
        let c = if byte >= 32 && byte <= 126 {
            (byte as char).to_string().bright_white()
        } else {
            ".".dimmed()
        };
        print!("{}", c);
    }
    println!();
    
    // Analysis
    if data.len() >= 2 {
        println!("   Analysis:");
        if data[0] == 0xA5 {
            println!("     {} Header byte detected (0xA5)", "✅".green());
            if data.len() >= 3 {
                println!("     {} Response type: 0x{:02X}", "📋".blue(), data[1]);
                println!("     {} Command code: 0x{:02X}", "🔧".yellow(), data[2]);
            }
        } else if data[0] == 0xAA && data.len() >= 2 && data[1] == 0x55 {
            println!("     {} Scan data header detected (0xAA55)", "🎯".green());
        } else {
            println!("     {} Unexpected header: 0x{:02X}", "⚠️".yellow(), data[0]);
        }
    }
}

fn analyze_response_timing(start_time: std::time::Instant, data_len: usize) {
    let elapsed = start_time.elapsed();
    println!("   Timing: {}ms ({} bytes/sec)", 
             elapsed.as_millis(),
             if elapsed.as_secs_f64() > 0.0 {
                 (data_len as f64 / elapsed.as_secs_f64()) as u32
             } else {
                 0
             });
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    init_logger();
    print_header();
    
    info!("Starting protocol analyzer");
    
    // Find YDLidar port
    let ports = serialport::available_ports()?;
    let lidar_port = ports.iter().find(|port| {
        match &port.port_type {
            SerialPortType::UsbPort(usb_info) => {
                usb_info.vid == 0x10c4 && usb_info.pid == 0xea60
            }
            _ => false,
        }
    });
    
    let port_name = match lidar_port {
        Some(port) => {
            println!("{} Using port: {}", "🔌".bright_green(), port.port_name.bright_cyan().bold());
            info!("Selected port: {}", port.port_name);
            &port.port_name
        }
        None => {
            error!("YDLidar device not found");
            println!("{} {}", "❌".red(), "YDLidar device not found!".red().bold());
            return Ok(());
        }
    };
    
    // Open port
    info!("Opening serial port with 230400 baud rate");
    let mut port = serialport::new(port_name, 230400)
        .timeout(Duration::from_millis(1000))
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .open()?;
    
    port.clear(serialport::ClearBuffer::All)?;
    thread::sleep(Duration::from_millis(100));
    
    // Test commands
    let commands = [
        ("GET_INFO", vec![0xA5, 0x90]),
        ("START_SCAN", vec![0xA5, 0x60]),
        ("STOP_SCAN", vec![0xA5, 0x65]),
        ("GET_HEALTH", vec![0xA5, 0x92]),
    ];
    
    for (cmd_name, cmd_bytes) in &commands {
        print_command_test(cmd_name, cmd_bytes);
        
        // Send command
        let start_time = std::time::Instant::now();
        info!("Sending {} command: {:?}", cmd_name, cmd_bytes);
        
        port.write_all(cmd_bytes)?;
        port.flush()?;
        debug!("Command sent in {}μs", start_time.elapsed().as_micros());
        
        // Wait for response
        thread::sleep(Duration::from_millis(200));
        
        // Read response
        let mut buffer = [0u8; 100];
        match port.read(&mut buffer) {
            Ok(n) => {
                analyze_response_timing(start_time, n);
                print_hex_dump(&buffer[..n], "Response");
                info!("Received {} bytes for {} command", n, cmd_name);
                
                // Log raw bytes for debugging
                let hex_string: String = buffer[..n].iter()
                    .map(|b| format!("{:02X}", b))
                    .collect::<Vec<_>>()
                    .join(" ");
                trace!("{} response bytes: {}", cmd_name, hex_string);
                
                // Special handling for scan data
                if *cmd_name == "START_SCAN" && n > 0 {
                    println!("   {} Scan data stream detected - reading more...", "🌊".bright_blue());
                    
                    // Read additional scan data
                    thread::sleep(Duration::from_millis(500));
                    match port.read(&mut buffer) {
                        Ok(n2) if n2 > 0 => {
                            print_hex_dump(&buffer[..n2], "Scan Data Stream");
                            debug!("Additional scan data: {} bytes", n2);
                        }
                        _ => {
                            debug!("No additional scan data received");
                        }
                    }
                }
            }
            Err(e) => {
                warn!("No response received for {} command: {}", cmd_name, e);
                println!("   {} No response received: {}", "⚠️".bright_yellow(), e.to_string().dimmed());
            }
        }
        
        // Clear buffer between commands
        let _ = port.clear(serialport::ClearBuffer::All);
        thread::sleep(Duration::from_millis(100));
    }
    
    // Protocol summary
    println!("\n{} Protocol Analysis Summary", "📊".bright_blue());
    println!("{}", "─".repeat(50).dimmed());
    println!("• {} Standard command format: 0xA5 + command_byte", "🔹".blue());
    println!("• {} Response format varies by command", "🔹".blue());
    println!("• {} Scan data uses 0xAA55 header (little-endian)", "🔹".blue());
    println!("• {} Baud rate: 230400", "🔹".blue());
    println!("• {} Data bits: 8, Parity: None, Stop bits: 1", "🔹".blue());
    
    info!("Protocol analysis completed");
    
    Ok(())
}