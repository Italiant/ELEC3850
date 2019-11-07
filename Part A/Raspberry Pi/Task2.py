import HCSR04
from HCSR04 import C
import RPi.GPIO as GPIO
import time
import lsm303d
import smbus
import math
import RPi.GPIO as GPIO
import struct

rev = GPIO.RPI_REVISION
if rev == 2 or rev == 3:
    bus = smbus.SMBus(1)
else:
    bus = smbus.SMBus(0)

### LSM303 Address ###
LSM303D_ADDR    = 0x1d # Assuming SA0 grounded

### LSM303 Register definitions ###
TEMP_OUT_L      = 0x05
TEMP_OUT_H      = 0x06
STATUS_REG_M    = 0x07
OUT_X_L_M       = 0x08
OUT_X_H_M       = 0x09
OUT_Y_L_M       = 0x0A
OUT_Y_H_M       = 0x0B
OUT_Z_L_M       = 0x0C
OUT_Z_H_M       = 0x0D
WHO_AM_I        = 0x0F
INT_CTRL_M      = 0x12
INT_SRC_M       = 0x13
INT_THS_L_M     = 0x14
INT_THS_H_M     = 0x15
OFFSET_X_L_M    = 0x16
OFFSET_X_H_M    = 0x17
OFFSET_Y_L_M    = 0x18
OFFSET_Y_H_M    = 0x19
OFFSET_Z_L_M    = 0x1A
OFFSET_Z_H_M    = 0x1B
REFERENCE_X     = 0x1C
REFERENCE_Y     = 0x1D
REFERENCE_Z     = 0x1E
CTRL_REG0       = 0x1F
CTRL_REG1       = 0x20
CTRL_REG2       = 0x21
CTRL_REG3       = 0x22
CTRL_REG4       = 0x23
CTRL_REG5       = 0x24
CTRL_REG6       = 0x25
CTRL_REG7       = 0x26
STATUS_REG_A    = 0x27
OUT_X_L_A       = 0x28
OUT_X_H_A       = 0x29
OUT_Y_L_A       = 0x2A
OUT_Y_H_A       = 0x2B
OUT_Z_L_A       = 0x2C
OUT_Z_H_A       = 0x2D
FIFO_CTRL       = 0x2E
FIFO_SRC        = 0x2F
IG_CFG1         = 0x30
IG_SRC1         = 0x31
IG_THS1         = 0x32
IG_DUR1         = 0x33
IG_CFG2         = 0x34
IG_SRC2         = 0x35
IG_THS2         = 0x36
IG_DUR2         = 0x37
CLICK_CFG       = 0x38
CLICK_SRC       = 0x39
CLICK_THS       = 0x3A
TIME_LIMIT      = 0x3B
TIME_LATENCY    = 0x3C
TIME_WINDOW     = 0x3D
ACT_THS         = 0x3E
ACT_DUR         = 0x3F

### Mag scales ###
MAG_SCALE_2     = 0x00 # full-scale is +/- 2 Gauss
MAG_SCALE_4     = 0x20 # +/- 4 Guass
MAG_SCALE_8     = 0x40 # +/- 8 Guass
MAG_SCALE_12    = 0x60 # +/- 12 Guass

ACCEL_SCALE     = 2 # +/- 2g

X = 0
Y = 1
Z = 2

def twos_comp(val, bits):
    # Calculate the 2s complement of int:val #
    if(val&(1<<(bits-1)) != 0):
        val = val - (1<<bits)
    return val
#   return val if val < 32768 else val - 65536

class accelcomp:
    mag = [0,0,0]
    accel = [0,0,0]
    tiltcomp = [0,0,0]
    heading=0
    headingDegrees=0
    tiltHeading=0
    tiltHeadingDegrees=0

    def __init__(self):
        whoami = bus.read_byte_data(LSM303D_ADDR, WHO_AM_I)

        if(whoami == 0x49):
            bus.write_byte_data(LSM303D_ADDR, CTRL_REG1, 0x57) # 0x57 = ODR=50hz, all accel axes on ## maybe 0x27 is Low Res?
            bus.write_byte_data(LSM303D_ADDR, CTRL_REG2, (3<<6)|(0<<3)) # set full scale +/- 2g
            bus.write_byte_data(LSM303D_ADDR, CTRL_REG3, 0x00) # no interrupt
            bus.write_byte_data(LSM303D_ADDR, CTRL_REG4, 0x00) # no interrupt
            bus.write_byte_data(LSM303D_ADDR, CTRL_REG5, (4<<2)) # 0x10 = mag 50Hz output rate
            bus.write_byte_data(LSM303D_ADDR, CTRL_REG6, MAG_SCALE_2) # Magnetic Scale +/1 1.3 Guass
            bus.write_byte_data(LSM303D_ADDR, CTRL_REG7, 0x00) # 0x00 continuous conversion mode

    def getMag(self):
        self.mag[X] = twos_comp(bus.read_byte_data(LSM303D_ADDR, OUT_X_H_M) << 8 | 
                          bus.read_byte_data(LSM303D_ADDR, OUT_X_L_M), 16)
        self.mag[Y] = twos_comp(bus.read_byte_data(LSM303D_ADDR, OUT_Y_H_M) << 8 | 
                          bus.read_byte_data(LSM303D_ADDR, OUT_Y_L_M), 16)
        self.mag[Z] = twos_comp(bus.read_byte_data(LSM303D_ADDR, OUT_Z_H_M) << 8 | 
                          bus.read_byte_data(LSM303D_ADDR, OUT_Z_L_M), 16)

    def getAccel(self):
        accel = [0,0,0]
        accel[X] = twos_comp(bus.read_byte_data(LSM303D_ADDR, OUT_X_H_A) << 8 | 
                           bus.read_byte_data(LSM303D_ADDR, OUT_X_L_A), 16)
        accel[Y] = twos_comp(bus.read_byte_data(LSM303D_ADDR, OUT_Y_H_A) << 8 | 
                           bus.read_byte_data(LSM303D_ADDR, OUT_Y_L_A), 16)
        accel[Z] = twos_comp(bus.read_byte_data(LSM303D_ADDR, OUT_Z_H_A) << 8 | 
                           bus.read_byte_data(LSM303D_ADDR, OUT_Z_L_A), 16)

        for i in range(X, Z+1):
            self.accel[i] = accel[i] / math.pow(2, 15) * ACCEL_SCALE

    def getHeading(self):
        self.heading = math.atan2(self.mag[X], self.mag[Y])

        if self.heading < 0:
            self.heading += 2*math.pi
        if self.heading > 2*math.pi:
            self.heading -= 2*math.pi

        self.headingDegrees = round(math.degrees(self.heading),2)

    def getTiltHeading(self):
        truncate = [0,0,0]
        for i in range(X, Z+1):
            truncate[i] = math.copysign(min(math.fabs(self.accel[i]), 1.0), self.accel[i])
        try:
            pitch = math.asin(-1*truncate[X])
            roll = math.asin(truncate[Y]/math.cos(pitch)) if abs(math.cos(pitch)) >= abs(truncate[Y]) else 0
            # set roll to zero if pitch approaches -1 or 1

            self.tiltcomp[X] = self.mag[X] * math.cos(pitch) + self.mag[Z] * math.sin(pitch)
            self.tiltcomp[Y] = self.mag[X] * math.sin(roll) * math.sin(pitch) + \
                               self.mag[Y] * math.cos(roll) - self.mag[Z] * math.sin(roll) * math.cos(pitch)
            self.tiltcomp[Z] = self.mag[X] * math.cos(roll) * math.sin(pitch) + \
                               self.mag[Y] * math.sin(roll) + \
                               self.mag[Z] * math.cos(roll) * math.cos(pitch)
            self.tiltHeading = math.atan2(self.tiltcomp[Y], self.tiltcomp[X])

            if self.tiltHeading < 0:
                self.tiltHeading += 2*math.pi
            if self.tiltHeading > 2*math.pi:
                self.heading -= 2*math.pi

            self.tiltHeadingDegrees = round(math.degrees(self.tiltHeading),2)

        except Exception:
            print ("AccelX {}, AccelY {}".format(self.accel[X], self.accel[Y]))
            print ("TruncX {}, TruncY {}".format(truncate[X], truncate[Y]))
            print ("Pitch {}, cos(pitch) {}, Bool(cos(pitch)) {}".format(pitch, math.cos(pitch), bool(math.cos(pitch))))
        
        return(self.tiltHeadingDegrees)

    def isMagReady(self):
        temp =  bus.read_byte_data(LSM303D_ADDR, STATUS_REG_M) & 0x03

        return temp

    def update(self):

        self.getAccel()
        self.getMag()
        self.getHeading()
        self.getTiltHeading()

if __name__ == '__main__':

    from time import sleep

    lsm = accelcomp()
    lsm.getMag()
    lsm.getHeading()
    lsm.getTiltHeading()

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

STOP = 19
REVERSE = 21
RIGHT = 6
STRAIGHT = 5
LEFT = 0

GPIO.setup(STOP, GPIO.OUT)
GPIO.setup(REVERSE, GPIO.OUT)
GPIO.setup(RIGHT, GPIO.OUT)
GPIO.setup(STRAIGHT, GPIO.OUT)
GPIO.output(STOP, False)
GPIO.output(REVERSE, False)
GPIO.output(RIGHT, False)
GPIO.output(STRAIGHT, False)

GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

Distancex = 0
N = 0

starttime = time.time()

GPIO.add_event_detect(16, GPIO.RISING)

def callback(self):
    global N, starttime, Distancex, C
    N = N + 1
     
    if(N == 5):
        stoptime = time.time()
        difference = stoptime - starttime
        Speed = 6*pi/5 * N / difference
        if(C == 0):
            Distancex = Distancex + 6*pi/5 * N
        if (C == 1):
            Distancex = Distancex - 6*pi/5 * N
            
        starttime = time.time()
        N = 0
        print("Total Distance: {} cm".format(Distancex))
        print("Speed: {} cm/s".format(Speed))

GPIO.add_event_callback(16, callback)

k = 0
while(True):
    GPIO.output(STRAIGHT, True)
    while(Distancex >= 160):
        GPIO.output(STRAIGHT, False)
        GPIO.output(RIGHT, True)
        lsm.getMag()
        lsm.getHeading()
        lsm.getTiltHeading()
        H2 = lsm.tiltHeadingDegrees
       
        while((H2 >= 130) and (H2 <= 140)):
            GPIO.output(RIGHT, False)
            GPIO.output(STRAIGHT, True)
            if(k == 0):
                Distancex = 0
                k = 1
            while(Distancex >= 160):
                GPIO.output(STRAIGHT, False)
                GPIO.output(RIGHT, True)
                lsm.getMag()
                lsm.getHeading()
                lsm.getTiltHeading()
                H3 = lsm.tiltHeadingDegrees
                
                while((H3 >= 260) and (H3 <= 290)):
                    GPIO.output(RIGHT, False)
                    GPIO.output(STOP, True)

    time.sleep(0.01)
    Heading = 0