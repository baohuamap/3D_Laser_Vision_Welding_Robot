# https://stackoverflow.com/questions/16077912/python-serial-how-to-use-the-read-or-readline-function-to-read-more-than-1-char
import serial
import time

def SplitData(output):
    string = []
    pos = []
    while output != 0:
        char = output%0x100
        output = output//0x100
        string.insert(0, char - 48)         # convert ascii to number
    end = string.index(-46) + 1
    for _ in range(6):
        string = string[end:-1]
        end = string.index(-4) + 1
        element = string[0: end - 1] 
        if -3 not in element:
            sign = 1
        else:
            sign = -1
            element.remove(-3)
        dot = element.index(-2)
        element.remove(-2)
        number = 0
        for i in range(len(element)):
            number = number + sign * element[i] * 10**(dot - i - 1)
        pos.append(number)
    return pos

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
    ser.write(b'\x01\x30\x31\x2C\x30\x30\x30\x02\x52\x50\x4F\x53\x43\x20\x31\x2C\x20\x30\x0D\x03\x83\x03') # RPOSC command
    time.sleep(0.5)
    IN = ser.readline()
    INdec =int.from_bytes(IN, byteorder='big', signed=False)
    if (INdec == 4145): #ACK1
        del IN
        del INdec
        ser.write(b'\x04')
        time.sleep(0.5)
        IN = ser.readline() 
        INdec =int.from_bytes(IN, byteorder='big', signed=False)
        if (INdec == 5): #EQN
            del IN
            del INdec
            ser.write(b'\x10\x30')  #send ACK0
            time.sleep(0.5)
            IN = ser.readline() 
            INdec =int.from_bytes(IN, byteorder='big', signed=False)
            print(SplitData(INdec))
            # received example:
            # 0x139302c30303102313530322e3332332c3635312e3332312c313537332e3735352c2d33322e33382c2d34392e39382c2d3133392e33312c302c300d03190b
            time.sleep(0.5)
            if (INdec != 0): #EQN
                del IN
                del INdec
                ser.write(b'\x10\x31')  #send ACK0
                time.sleep(0.5)
                IN = ser.readline() 
                INdec =int.from_bytes(IN, byteorder='big', signed=False)
                print(INdec)
