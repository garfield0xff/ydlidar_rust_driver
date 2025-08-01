use serialport::{SerialPort, SerialPortType};
use std::time::Duration;
use std::thread;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Raw YDLidar Communication Debug");
    println!("==============================");
    
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
        Some(port) => &port.port_name,
        None => {
            println!("YDLidar device not found!");
            return Ok(());
        }
    };
    
    println!("Using port: {}", port_name);
    
    // Open port
    let mut port = serialport::new(port_name, 230400)
        .timeout(Duration::from_millis(1000))
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .open()?;
    
    port.clear(serialport::ClearBuffer::All)?;
    thread::sleep(Duration::from_millis(100));
    
    // Try sending GET_INFO command
    println!("Sending GET_INFO command: [0xA5, 0x90]");
    let cmd = [0xA5, 0x90];
    port.write_all(&cmd)?;
    port.flush()?;
    
    println!("Waiting for response...");
    thread::sleep(Duration::from_millis(500));
    
    // Read response
    let mut buffer = [0u8; 50];
    match port.read(&mut buffer) {
        Ok(n) => {
            println!("Received {} bytes:", n);
            print!("Raw bytes: ");
            for i in 0..n {
                print!("0x{:02X} ", buffer[i]);
            }
            println!();
            
            print!("ASCII repr: ");
            for i in 0..n {
                let c = buffer[i];
                if c >= 32 && c <= 126 {
                    print!("{} ", c as char);
                } else {
                    print!(". ");
                }
            }
            println!();
            
            // Check if it starts with expected header
            if n >= 2 {
                println!("First two bytes: 0x{:02X} 0x{:02X}", buffer[0], buffer[1]);
                if buffer[0] == 0xA5 {
                    println!("✓ Response starts with 0xA5 (correct header)");
                    if buffer[1] == 0x14 {
                        println!("✓ Second byte is 0x14 (expected for device info)");
                    } else {
                        println!("✗ Second byte is 0x{:02X}, expected 0x14", buffer[1]);
                    }
                } else {
                    println!("✗ Response doesn't start with 0xA5");
                }
            }
        }
        Err(e) => {
            println!("Error reading response: {}", e);
        }
    }
    
    // Try a different approach - send SOFT_RESTART first
    println!("\nTrying SOFT_RESTART command first...");
    let restart_cmd = [0xA5, 0x40];
    port.write_all(&restart_cmd)?;
    port.flush()?;
    thread::sleep(Duration::from_millis(1000));
    
    // Clear any response
    let mut temp_buf = [0u8; 100];
    let _ = port.read(&mut temp_buf);
    
    // Try GET_INFO again
    println!("Sending GET_INFO command again: [0xA5, 0x90]");
    port.write_all(&cmd)?;
    port.flush()?;
    thread::sleep(Duration::from_millis(500));
    
    match port.read(&mut buffer) {
        Ok(n) => {
            println!("Received {} bytes after restart:", n);
            print!("Raw bytes: ");
            for i in 0..n {
                print!("0x{:02X} ", buffer[i]);
            }
            println!();
        }
        Err(e) => {
            println!("Error reading response after restart: {}", e);
        }
    }
    
    Ok(())
}