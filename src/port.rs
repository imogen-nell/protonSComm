use serialport::SerialPort;
use std::time::Duration;
use std::thread::sleep;
use std::error::Error;
use std::io::{self, Write};
use serial::prelude::*;
// use simple_logger::SimpleLogger;
use log::{info, warn, error};

const HEADER: [u8;2] = [0xF0, 0xF0];
const C_SCALE: f32 = 0.001;
const V_SCALE: f32 = 1.0;
const P_SCALE: f32 = 0.001; //TODO fix this val
const SCALE: [f32; 3] = [C_SCALE, V_SCALE, P_SCALE];
const MOTORS: [char; 2] = ['A', 'B'];
const ERRORS: [&str; 11] = ["Error Conditon", "Over Current", 
"Loss of Feedback", "Over Speed", "Motor Temp", "IGBTTemp", 
"I2T", "Bridge Fault", "Over Voltage", "Under Voltage", "Precharge V"];

const CRC16_TABLE: [u16; 256]= [0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0];

pub struct Port {
    name: Option<String>,
    baud: Option<u32>,
    port: Box<dyn SerialPort>,
    mode: u8,
    feedback: f32,
    motor_enabled: Option<char>,
    
}

impl Port {
    pub fn new(name: String, baud: u32) -> Self {
        let mut new_port = serialport::new(name, baud)
        .timeout(Duration::from_millis(10))
        .open().expect("Failed to open port");

        let mut p = Port {
            name: None,
            baud: None,
            port: new_port,
            mode: 0,
            feedback: 0.0,
            motor_enabled: None,
        };
        p
    }

    pub fn get_feedback(&self) -> f32 {
        self.feedback
    }

    //will diplay available ports if needed
    pub fn display_ports() {
        let ports = serialport::available_ports().expect("No ports found!");

        for p in ports {
            println!("{:?}", p.port_name);
        }
    }

    //sends a command to the port (motor)
    //Arguments: cmd: &[u8] - the 12 byte command to be sent inc. header & crc
    pub fn write_cmd(&mut self, cmd: &[u8]) {
        self.mode = (cmd[2] & 0b00011000)>>3;
        self.port.write(cmd);
    }

    //calculates the checksum for the command
    //Arguments: cmd: &[u8] - the byte command inc. the header, may includ the crc or not 
    //Returns: u16 - the calculated checksum in the order it should appear in the command send 
    pub fn checksum(cmd: &[u8]) -> u16 {
        let msg = &cmd[2..10];
        let mut crc16: u16 = 0xFFFF; // initial value FFFF always
        for &byte in msg{
            let i = ((crc16>>8)^ byte as u16) as usize;
            crc16 = CRC16_TABLE[i] ^ (crc16 << 8);
        }
        crc16.swap_bytes()
    }

    //verifies the checksum of a command received or to send
    //Arguments: cmd: &[u8] - the byte command inc. the header and the crc 
    //Returns: bool - true if the checksum is correct, false otherwise
    pub fn verify_crc(cmd: &[u8]) -> bool {
        let expected = &cmd[10..12];
        let calculated = Port::checksum(&cmd).to_be_bytes();
        expected == calculated
    }

    //reads the response from the port (motor)
    //used by get_interpret_resp()
    fn read_resp(&mut self)-> Vec<u8> {
        let mut serial_buf: Vec<u8> = vec![0; 12];
        self.port.read(serial_buf.as_mut_slice()).expect("Read failed");
        serial_buf
    }

    //reads the response from the port (motor) and then updates : feedback, motor_enabled, mode
    pub fn get_interpret_resp(&mut self) {
        //get response, exctract status packet
        let mut serial_buf: Vec<u8> = self.read_resp();
        let status_packet = &serial_buf[2..9]; //inclusive of 2, exclusive of 9

        //HEADER
        if HEADER != serial_buf[0..2] {
            log::warn!("Header does not match");

            //wait for header to collect data
            let mut next: Vec<u8> = vec![0; 2];
            self.port.read_exact(&mut next).expect("Read failed");
            while next!= HEADER {
                //waiting
                self.port.read_exact(&mut next).expect("Read failed");
                sleep(Duration::from_millis(10));
            }
            let mut small_buf: Vec<u8> = vec![0; 10];
            self.port.read_exact(&mut small_buf).expect("Read failed");
            next.extend_from_slice(&small_buf);
            
        }
        //error check , pass the whole status pack 
        self.error_check(status_packet);
        
        //CRC
        if !Port::verify_crc(&serial_buf) {
            log::warn!("Checksum does not match");
            return
        }

        //BODY
    
        //find out which motor is enabled, if any, pass byte 0 of statuspack to check
        self.check_motor_enabled(status_packet[0]);

        //find the rmp if in velocity mode, current if in torque mode
        self.feedback = ((((status_packet[2] as i16) << 8) | (status_packet[1] as i16) )as f32 )*  SCALE[self.mode as usize] as f32;

    }

    //checks which motor is enabled, if any
    //Arguments: packet: u8 - byte 0 of the status packet (contains the motor enabled info)
    //updates motor_enabled: Option<char> - true: motor A, false: motor B, None: no motor enabled
    //exoect: True or None only 
    fn check_motor_enabled(&mut self, packet: u8){
        if ((packet & 0b00000011) as usize) == 1 || ((packet & 0b00000011) as usize) == 2  {
            self.motor_enabled = MOTORS.get(((packet & 0b00000011) as usize)-1).cloned();
        }else{
            self.motor_enabled = None;
        }
    }

    //TODO: what to do w the errors????
    //checks the errors in the status packet
    //Arguments: status: &[u8] - the (whole) status packet
    fn error_check(&self, status: &[u8]) {
        //errors in byte 5 bit 0:6 incl., byte 0 bit 3
        let errs = (status[5] & 0b1111110) | ((status[0]& 0b00001000)>>3);
        //some errors in byte 0 bits 4-6 inc.
        let other_errs = status[0] & 0b01110000;
        for i in 0..8 {
            if ((errs & (1<<i))>>i) == 1 {
                log::error!("Error: {}", ERRORS[i]);
            }
            if ((other_errs & (1<<i))>>i) == 1 {
                log::error!("Error: {}", ERRORS[i+4]);
            }
        }

    }

    //creates a command which enables motor a
    //Arguments: mode: u16 - the mode to enable the motor in (0: torque, 1: velocity, 2: position)
    //clear_errs: bool - true if errors should be cleared, false otherwise
    //Returns: [u8;12] - the 12 byte command to enable motor a
    pub fn create_command_long(mode: usize, clear_errs: bool, value: f32)-> [u8;12]{
        let mut cmd = vec![0xF0,0xF0];
        //byte 2
        let mut com_mode = 0b00000001 + (mode <<3);
        if clear_errs {com_mode = com_mode | 0b00000101};
        cmd.push(com_mode as u8);
        //byte 3 -4
        let val = ((value / SCALE[mode] ) as i16).to_le_bytes();
        cmd.push(val[0]);
        cmd.push(val[1]);
        //byte 5-9 for motor b and reserved
        for _ in 0..5{
            cmd.push(0x00);
        }
        //crc
        let crc = Port::checksum(&cmd[..]).to_be_bytes();
        cmd.push(crc[0]);
        cmd.push(crc[1]);

        let mut ret = [0;12];
        ret.copy_from_slice(&cmd);
        if !Self::verify_crc(&ret){log::error!("Checksum is not correct");return [0;12];}

        ret
    }

    //creates a command which enables motor a, doesnt clear errors 
    //Arguments: mode: u16 - the mode to enable the motor in 
    //Arguments: value: f32 - the value of Amps if in current mode, or RPM if in velocity mode
    //Returns: [u8;12] - the 12 byte command corresponding to your mode and value
    pub fn create_command(mode: &str, value : f32)-> [u8;12]{
        let mut cmd = vec![0xF0,0xF0];
        //byte 2
        let mut com_mode = 0b00000001;
        let mut i = 0;
        match mode {
            "c" =>          com_mode = 0b00000001,
            "current" =>    com_mode = 0b00000001,
            "torque" =>     com_mode = 0b00000001,
            "amps" =>       com_mode = 0b00000001,

            "velocity" =>  {com_mode = 0b00001001; i = 1},
            "rpm" =>       {com_mode = 0b00001001; i = 1},
            "v" =>         {com_mode = 0b00001001; i = 1},

            //"position" =>   com_mode = 0b00010001, dont use this 
            _ => log::error!("Invalid mode"),
        }
        cmd.push(com_mode as u8);
        //byte 3 -4
        let val = ((value / SCALE[i] ) as i16).to_le_bytes();
        cmd.push(val[0]);
        cmd.push(val[1]);
        //byte 5-9 for motor b and reserved
        for _ in 0..5{
            cmd.push(0x00);
        }
        //crc
        let crc = Port::checksum(&cmd[..]).to_be_bytes();
        cmd.push(crc[0]);
        cmd.push(crc[1]);

        let mut ret = [0;12];
        ret.copy_from_slice(&cmd);
        if !Self::verify_crc(&ret){log::error!("Checksum is not correct");return [0;12];}

        ret
    }

    //self.feedback = ((((status_packet[2] as i16) << 8) | (status_packet[1] as i16) )as f32 )*  SCALE[self.mode as usize] as f32;

    //a in tor : byte 2 = 00000001 = 0x01 - 1
    //a in vel : byte 2 = 00001001 = 0x09 - 9 
    //a in pos : byte 2 = 00010001 = 0x11 - 17

    //a in tor w clear errs : byte 2 = 00000101 = 0x05 - 5



}