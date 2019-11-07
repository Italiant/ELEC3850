import time
import ctypes
import serial
import sys

while(1):

    x = 0
    
    ser = serial.Serial(   
        port='/dev/ttyACM0',
        baudrate = 9600,
        parity = serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
        timeout = 2,
        )

    if (ser.isOpen() == False):
        ser.open()

    ser.flushInput()
    ser.flushOutput()

    x = ser.readline()
    print(x)
        
