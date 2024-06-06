use serialport::SerialPort;
use std::time::{Duration, Instant};
use std::thread::sleep;
use std::thread;
use std::error::Error;
use std::io::{self, Write};
use cadh::threadsync::DualChannelSync;
use serial::prelude::*;
use crossbeam;
use log::{info, warn, error};
use csv::Writer;

//frequency of commands being excecuted : 1/SLEEP = 200Hz
const RATED_CURRENT     : f32 = 3.45; // [units] = A //change based on motor
const TORQUE_CONSTANT   : f32 = 0.036; // [units]  = Nm/A //change based on motor 
const SLEEP             : f64 = 0.005;//in seconds, 5ms
const HEADER            : [u8;2] = [0xF0, 0xF0];
const HEAD              : [u8;1] = [0xF0];
const C_SCALE           : f32 = 0.001; 
const V_SCALE           : f32 = 1.0;
const P_SCALE           : f32 = 0.0; //never use this 
const SCALE             : [f32; 3] = [C_SCALE, V_SCALE, P_SCALE];
const ERRORS            : [&str; 11] = [ "Over Current", 
"Loss of Feedback", "Over Speed", "Motor Temp", "IGBTTemp", 
"I2T", "Bridge Fault", "Error Conditon", "Over Voltage", "Under Voltage", "Precharge V"];

const DISABLE: &[u8] = &[0xF0,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x31];

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

type Result<T> = std::result::Result<T, Box<dyn Error>>;

pub struct ControllerThread {
    pub inner: DualChannelSync<cmd, data>,
}

pub struct Controller{
    port: Box<dyn SerialPort>,
    feedback: f32,
    state: readstate,
    status: [u8; 10],
    writer : Writer<std::fs::File>,
    //to add when the other packets are defined
    // 042C serialFeedbackEnable
    // 042D serialBaud

    // 042F serialFeedbackRate
    // 0430 serialChecksumErrorCOunt
    // 0431 serialPassCount
    // 0432 serialCharactersReceived
    // 0433 serialNumReportedErrors

}

#[derive(Debug)]
#[derive(PartialEq)]
pub enum readstate{
    Lost,
    Header,
    StatusPacket,
    Crc,
    Read,
}

pub enum cmd{
    SetTorque(f32),
    SetVelocity(f32),
    SetCurrent(f32),
    DisableMotor(),
}

#[derive(Debug)]
pub struct data{
    pub feedback: f32,
}

impl ControllerThread {

    pub fn spawn_feedback_thread(port_name: String, baud: u32) -> Result<Self> {
        let mut wtr = Writer::from_path("commands1.csv")?;
        wtr.write_record(&["Time", "Current Command"]);
        let handle = DualChannelSync::spawn(

            "Feedback Updater",
            
            move |to_main: _, from_main: _| {
            
                log::info!("Feedback thread started!");
                //initialize controller, give ownership to the thread
                let mut controller = Controller::new(port_name, baud).expect("Failed to open port");
                //setup timing
                let interval = Duration::from_secs_f64(SLEEP);
                let mut next_update = Instant::now() + interval;

                loop {
                    //check for commands from main, and excecute
                    if from_main.len() > 10 {log::warn!("{} commands in queue", from_main.len());}
                    for cmd in from_main.try_iter() {                        
                        match cmd {
                            cmd::SetTorque(torque) => { ControllerThread::set_torque(&mut controller, torque);}
                            cmd::SetVelocity(velocity) => { ControllerThread::set_velocity(&mut controller, velocity);}
                            cmd::SetCurrent(current) => { ControllerThread::set_current(&mut wtr, &mut controller, current);}
                            cmd::DisableMotor() => { controller.port.write(DISABLE);}
                        }
                        
                        //wait appropriate time
                        if (std::time::Instant::now())  > next_update {next_update = Instant::now() + interval ;}
                        sleep(next_update-Instant::now());
                        next_update += interval;

                    }

                    controller.change_state();
                    //populate data struct
                    // log::debug!("state: {:?}", controller.state);
                    let data = data { feedback: controller.feedback};
                    //send data to main
                    if let Err(e) = to_main.send(data) {
                        log::warn!("failed to send data: {}", e);
                    }        
                                
                }
            })?;

            Ok(ControllerThread {
                inner: handle,
                // wtr: wtr,
            })
    }

    fn set_torque(controller: &mut Controller, torque: f32) {
        let mut current = torque / TORQUE_CONSTANT;
        if current > RATED_CURRENT {current = RATED_CURRENT;}
        controller.port.write(&Controller::create_command("torque", current));
    }

    fn set_velocity(controller: &mut Controller, velocity: f32) {
        controller.port.write(&Controller::create_command("velocity", velocity));
    }

    fn set_current(wtr: &mut Writer<std::fs::File>, controller: &mut Controller, current: f32) {
        controller.port.write(&Controller::create_command("current", current));
        let time_now = Instant::now();
        // wtr.write_record(&[time_now, current])?;
        wtr.write_record(&[ format!("{:?}", time_now), current.to_string()]);
        wtr.flush();
    }

}

impl Controller {
    //creates and opens serial port, return controller
    pub fn new(port_name: String, baud: u32) -> Result<Self> {
        let mut writer = Writer::from_path("feedback1.csv")?;
        writer.write_record(&["Time", "Feedback"])?;

        let ser_port = serialport::new(port_name, baud)
            .timeout(Duration::from_millis(1000))
            .open()?;
        Ok( Controller {
            port: ser_port,
            feedback: 0.0,
            state: readstate::Lost,
            status: [0; 10],
            writer: writer,
        })
    }
     
    //calculates the checksum for the given packet
    //Arguments: cmd: &[u8] - the byte packet including the header, may include the crc or not 
    //Returns: u16 - the calculated checksum in the order it should appear in the command send 
    fn checksum(cmd: &[u8]) -> u16 {
        let msg = &cmd[2..10];
        let mut crc16: u16 = 0xFFFF; // initial value FFFF always
        for &byte in msg{
            let i = ((crc16>>8)^ byte as u16) as usize;
            crc16 = CRC16_TABLE[i] ^ (crc16 << 8);
        }
        crc16.swap_bytes()
    }

    //verifies the checksum of a command received or to send
    //Arguments: cmd: &[u8] - the 12 byte command inclusing the header and the crc 
    //Returns: bool - true if the checksum is correct, false otherwise
    fn verify_crc(cmd: &[u8]) -> bool {
        let expected = &cmd[10..12];
        let calculated = Controller::checksum(&cmd).to_be_bytes();
        expected == calculated
    }

    //read 1 byte -> decided what to do dep. on curr state
    //read byte: read byte, change state, put byte in right place
    //read msg calls RB untill in final state
    pub fn change_state(&mut self) {
        match self.state{
            readstate::Lost => {
                //find one header byte
                let check: bool = self.find_header();
                if check {self.state = readstate::Header;}
                else {self.state = readstate::Lost;}
                self.change_state();
            }
            readstate::Header => {
                //find second header byte
                let byte = self.next_byte();
                if byte == HEAD  {self.state = readstate::StatusPacket;}
                else {self.state = readstate::Lost;}
                self.change_state();
            }
            readstate::StatusPacket => {
                //get status packet
                let mut status_packet: Vec<u8> = vec![0; 10];
                self.port.read_exact(&mut status_packet).expect("Read failed");
                self.status = status_packet.try_into().unwrap();//as_slice();
                self.state = readstate::Crc;
                self.change_state();
            }
            readstate::Crc => {
                //verify crc
                let mut pack = vec![0xF0, 0xF0];
                pack.extend(&self.status);
                if Controller::verify_crc(&pack[..]) {self.state = readstate::Read;}
                else {log::warn!("Checksum does not match, back to Lost"); self.state = readstate::Lost;}
                self.change_state();
            }
            readstate::Read => {
                //update feedback
                let mode = (self.status[0] & 0b00011000)>>3;
                self.feedback = ((((self.status[2] as i16) << 8) | (self.status[1] as i16) )as f32 )*  SCALE[mode as usize] as f32;
                let time_now = Instant::now();
                if self.feedback > -1.55 && self.feedback < -0.5 || self.feedback > 1.25 && self.feedback < 1.6{
                    self.writer.write_record(&[ format!("{:?}", time_now), self.feedback.to_string()]);
                }
                self.state = readstate::Lost;
                self.writer.flush();

            }
        }
    }

    //looks for one byte of the header, waits until it finds it
    //Returns: bool - true once the header is found
    fn find_header(&mut self) -> bool{
        let mut temp_head: Vec<u8> = vec![0; 1];
        //read next 2 bytes 
        self.port.read_exact(&mut temp_head).expect("Read failed");
        //wait for correct header
        while temp_head != HEAD {
            self.port.read_exact(&mut temp_head).expect("Read failed");
        }
        true
    }

    //reads the next byte from the serial port
    //Returns: Vec<u8> - the byte read
    fn next_byte(&mut self) -> Vec<u8> {
        let mut byte: Vec<u8> = vec![0; 1];
        self.port.read_exact(&mut byte).expect("Read failed");
        byte
    }

    //creates a command which enables motor a, doesnt clear errors 
    //Arguments: mode: u16 - the mode to enable the motor in 
    //Arguments: value: f32 - the value of Amps if in current mode, or RPM if in velocity mode
    //Returns: [u8;12] - the corresponding 12 byte command 
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
        let crc = Controller::checksum(&cmd[..]).to_be_bytes();
        cmd.push(crc[0]);
        cmd.push(crc[1]);

        let mut ret = [0;12];
        ret.copy_from_slice(&cmd);
        if !Self::verify_crc(&ret){log::error!("Checksum is not correct");return [0;12];}

        ret
    }

    //checks if motor a is enabled
    //Arguments: packet: u8 - the status packet byte 0
    //Returns: bool - true if motor a is enabled, false otherwise
    pub fn is_motor_enabled(packet: u8) -> bool{
        if (packet & 0b00000001) ==1 { true }
        else { false }
    }

    //TODO: what to do w the errors????
    //checks the errors in the status packet
    //Arguments: status: &[u8] - the status packet excluding the header
    fn error_check(&self, status: &[u8]) {
        //extract the errors in byte 5 & byte 0
        let errs = status[5] & 0b01111111;
        let other_errs = status[0]& 0b01110100;

        for i in 0..8 {
            if ((errs & (1<<i))>>i) == 1 {
                log::error!("Error: {}", ERRORS[i]);
            }
            if ((other_errs & (1<<i))>>i) == 1 { //2-> 7 , 4-> 8, 5-> 9, 6-> 10
                if i == 2 {
                    log::error!("Error: {}", ERRORS[7]);
                } else {
                    log::error!("Error: {}", ERRORS[i+4]);
                }
            }
        }
    }

 
}