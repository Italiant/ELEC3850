import time
import ctypes
import serial
import sys

ser = serial.Serial(   
    port='/dev/ttyACM0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 2,
    )
counter = 1

if (ser.isOpen() == False):
    ser.open()

ser.flushInput()
ser.flushOutput()

while 1:
    ser.write(counter)
    time.sleep(1)
    #print("Write counter: {}".format(counter))
    x=ser.readline()
    print(x)