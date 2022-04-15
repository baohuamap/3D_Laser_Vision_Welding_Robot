import serial
import time
from math import log

def bbc(input):
    sum = 0
    while input != 0:
        char = input%0x100
        sum = sum +char
        input = input//0x100
        result = (sum - 0x01)//0x100 + (sum - 0x01)%0x100*0x100
    return result
    
def CombineData(position):
    input = str(position[0]) + ', ' + str(position[1]) + ', ' + str(position[2]) + ', ' + str(position[3]) + ', ' + str(position[4]) + ', ' + str(position[5])
    # out = 0x0130312C303030024D4F564C20302C203530302E302C20312C20
    out = 0x0130312C303030024D4F564C20302C2032302C20312C20
    for i in range(len(input)):
        out = out * 0x100 + ord(input[i])
    for _ in range(8):
        out = out * 0x1000000 + 0x2C2030
    out = out * 0x100 + 0x0D03
    return out

def movL(position):
    command = CombineData(position)
    command = command * 0x10000 + bbc(command)
    return command

def bytes_needed(n): 
    return int(log(n, 256)) + 1
    
command = movL(([466.753,408.688, 223.872, 116.69, 3.76, 167.22]))
print(bytes_needed(256))
print(hex(command))
ser = serial.Serial(
    port='COM4',\
    baudrate=19200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)  

print("connected to: " + ser.portstr)
ser.write(b'\x05')
time.sleep(0.5)
IN = ser.readline()
INdec =int.from_bytes(IN, byteorder='big', signed=False)
print(INdec)
print(IN)
if(INdec == 4144):    # ACK0
    del IN
    del INdec
    print("data link established")
    ser.write(command.to_bytes(bytes_needed(command), 'big'))
    # ser.write(b'\x01\x30\x31\x2c\x30\x30\x30\x02\x4d\x4f\x56\x4c\x20\x30\x2c\x20\x35\x30\x30\x2e\x30\x2c\x20\x31\x2c\x20\x34\x36\x36\x2e\x37\x35\x33\x2c\x20\x34\x30\x38\x2e\x36\x38\x38\x2c\x20\x32\x32\x33\x2e\x38\x37\x32\x2c\x20\x31\x31\x36\x2e\x36\x39\x2c\x20\x33\x2e\x37\x36\x2c\x20\x31\x36\x37\x2e\x32\x32\x2c\x20\x30\x2c\x20\x30\x2c\x20\x30\x2c\x20\x30\x2c\x20\x30\x2c\x20\x30\x2c\x20\x30\x2c\x20\x3d\x03\x97\x11')
    time.sleep(0.5)
    IN = ser.readline()
    print('command',IN)
    INdec =int.from_bytes(IN, byteorder='big', signed=False)
    if (INdec == 4145): #ACK1
        del IN
        del INdec
        ser.write(b'\x04')
        time.sleep(0.5)
        IN = ser.readline() 
        INdec =int.from_bytes(IN, byteorder='big', signed=False)
        print('ack1',IN)
        if (INdec == 5): #EQN
            del IN
            del INdec
            ser.write(b'\x10\x30')  #send ACK0
            time.sleep(0.5)
            IN = ser.readline() 
            INdec =int.from_bytes(IN, byteorder='big', signed=False)
            print('eqn',IN)
            ser.write(b'\x10\x31')
            time.sleep(5)
            INdec =int.from_bytes(IN, byteorder='big', signed=False)
            print('ack1',IN)
