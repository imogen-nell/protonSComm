#serial communication with motor controller
import serial
from serial import Serial
import time
# from serial import Serial

ser = serial.Serial(
    port = '/dev/ttyUSB0', 
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    xonxoff=False
)

data   = [0xF0,0xF0,0x09,0x18,0xFC,0x00,0x00,0x00,0x00,0x00,0xBE,0x56]
data2 = b'\x0F\x0F\x09\x18\xFC\x00\x00\x00\x00\x00\xBE\x56'
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

    while True:
        ser.write(data)
        # for byte in data:
        #     ser.write(byte)
        ##sleep for 10 milliseconds
        time.sleep(.001)
        status = ser.read(12)
        interpret_cmd(status)
        print(status)


except Exception as e:
    print("Error:", e)

finally:
    ser.close()

