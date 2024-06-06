use std::time::Duration;
use std::thread::sleep;
use protonSComm::port::ControllerThread;
use simple_logger::SimpleLogger;
use std::sync::Mutex;
use protonSComm::port::cmd;

const NAME: &str = "/dev/ttyUSB0";
const VEL_NEGATIVE_1000: &[u8] = &[0xF0,0xF0,0x09,0x18,0xFC,0x00,0x00,0x00,0x00,0x00,0xBE,0x56];
const STOP: &[u8] = &[0xF0,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x31];
const BAUD: u32 = 230400;//460800;// 9600;//
const ENABLE_A: &[u8] = &[0xF0,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xED,0x76];
const DISABLE: &[u8] = &[0xF0,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x31];
// const ENABLE_A_POS: &[u8] = &[0xF0,0xF0,0x11,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x59,0x4B]; ///enable a in velocity mode
const ENABLE_A_TOR: &[u8] = &[0xF0,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xED,0x76]; ///enable a in torque mode
const ENABLE_A_VEL: &[u8] = &[0xF0,0xF0,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x37,0x68]; ///enable a in position

fn main() {
    let mut my_controller = ControllerThread::spawn_feedback_thread(NAME.into(), BAUD).unwrap();
    // my_controller.inner.try_send(cmd::DisableMotor());
    SimpleLogger::new().init().unwrap();
    // sleep(Duration::from_millis(1000));
    my_controller.inner.try_send(cmd::SetCurrent(1.5));
    loop{
        
        my_controller.inner.try_send(cmd::SetCurrent(1.5));
        // my_controller.inner.try_send(cmd::DisableMotor());
        sleep(Duration::from_millis(3000));
        my_controller.inner.try_send(cmd::SetCurrent(-1.5));
        // println!("before : {:?}", my_controller.inner.try_iter().last());
        sleep(Duration::from_millis(3000));
    }
}



//todo : clean code, log, rate of respose