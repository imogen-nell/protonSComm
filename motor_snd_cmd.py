#serial communication with motor controller
import serial
from serial import Serial
import time
# from serial import Serial

ser = serial.Serial(
    port = '/dev/ttyUSB0', 
    baudrate=230400,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    xonxoff=False
)

DISABLE   = [0xF0,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x31]
data = [0x56, 0xBE, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x18, 0x09, 0xF0, 0xF0]
VEL_NEGATIVE_1000 = [0xF0,0xF0,0x09,0x18,0xFC,0x00,0x00,0x00,0x00,0x00,0xBE,0x56]

def interpret_cmd(cmd):
    feedback = ['Current', 'Velocity', 'Position']
    header = cmd[0:2]
    if header != b'\xF0\xF0':
        print("Header not correct.")
        
    print("Motor A Enabled :", bin(cmd[2])[0]==1)
    print("Motor A Error   :", bin(cmd[2])[2] ==1)
    # print("Feedback Mode   :", bin(cmd[4])[-1])
    
    return
    
try:
    if not ser.is_open:
        ser.open()
    ser.write(DISABLE)
    # while True:
    #     ser.write(DISABLE)
    #     # for byte in data:
    #     #     ser.write(byte)
    #     time.sleep(.100)
    #     status = ser.read(12)
    #     interpret_cmd(status)
    #     # print(status)


except Exception as e:
    print("Error:", e)

finally:
    ser.close()

