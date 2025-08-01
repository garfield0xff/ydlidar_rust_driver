use serialport::SerialPortType;
use std::time::Duration;
use std::thread;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("YDLidar Scan Command Debug");
    println!("==========================");
    
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
    
    // Try sending START_SCAN command
    println!("Sending START_SCAN command: [0xA5, 0x60]");
    let cmd = [0xA5, 0x60];
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
            
            // Check if it starts with expected header
            if n >= 2 {
                println!("First two bytes: 0x{:02X} 0x{:02X}", buffer[0], buffer[1]);
                if buffer[0] == 0xA5 {
                    println!("✓ Response starts with 0xA5 (correct header)");
                    println!("Second byte: 0x{:02X} (this should match command 0x60)", buffer[1]);
                } else {
                    println!("✗ Response doesn't start with 0xA5");
                }
            }
        }
        Err(e) => {
            println!("Error reading response: {}", e);
        }
    }
    
    // Wait a bit more to see if there's additional data (scan data)
    println!("\nWaiting for scan data...");
    thread::sleep(Duration::from_millis(1000));
    
    match port.read(&mut buffer) {
        Ok(n) if n > 0 => {
            println!("Received {} additional bytes (scan data?):", n);
            print!("Raw bytes: ");
            for i in 0..n {
                print!("0x{:02X} ", buffer[i]);
            }
            println!();
        }
        Ok(_) => println!("No additional data received"),
        Err(e) => println!("Error reading additional data: {}", e),
    }
    
    // Send STOP_SCAN command
    println!("\nSending STOP_SCAN command: [0xA5, 0x65]");
    let stop_cmd = [0xA5, 0x65];
    port.write_all(&stop_cmd)?;
    port.flush()?;
    
    Ok(())
}