extern crate serial;

use std::env;
use std::io;
use std::time::Duration;
use serial::SerialPort;

use serial::prelude::*;
//use serial::io::prelude::*;


fn main() -> std::io::Result<()> {
    display_ports();

    println!("Hello, world!");
    let port = serialport::new("/dev/ttyUSB0", 115_200)//TODO: check 115_200--- what baud should i use ??
        .timeout(Duration::from_millis(15))
        .open().expect("Failed to open port");
git 
    println!("Port opened");

    Ok(())

}

//funciton to write to port:
fn write_to_port(port: &mut dyn serialport::SerialPort) {
    let output = "test".as_bytes();
    port.write(output).expect("Write failed");
}

//function to read from port:
fn read_from_port(port: &mut dyn serialport::SerialPort) {
    let mut serial_buf: Vec<u8> = vec![0; 32];
    port.read(serial_buf.as_mut_slice()).expect("Read failed");

    println!("{:?}", serial_buf);
}

//lists available ports:
fn display_ports() {
    let ports = serialport::available_ports().expect("No ports found!");

    for p in ports {
        println!("{:?}", p.port_name);
    }
}