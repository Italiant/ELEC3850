import HCSR04
from math import pi
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

STOP = 26
REVERSE = 21

GPIO.setup(STOP, GPIO.OUT)
GPIO.setup(REVERSE, GPIO.OUT)
GPIO.output(STOP, False)
GPIO.output(REVERSE, False)

GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
Distance = 0
N = 0
starttime = time.time()

GPIO.add_event_detect(16, GPIO.RISING)

def callback(self):
    global N, starttime, Distance, C
    N = N + 1
    print("N: {}".format(N))
     
    if(N == 5):
        stoptime = time.time()
        difference = stoptime - starttime
        Speed = 6*pi/5 * N / difference
        if(C == 0):
            Distance = Distance + 6*pi/5 * N
        if (C == 1):
            Distance = Distance - 6*pi/5 * N
            
        starttime = time.time()
        N = 0
        print("Total Distance: {} cm".format(Distance))
        print("Speed: {} cm/s".format(Speed))

GPIO.add_event_callback(16, callback)

A = int (input("Choose stopping distance: "))

while(1):
    distance = HCSR04.ultrasonic()
    while(distance <= A):
        GPIO.output(STOP, True)
        print("stopped")
        C = int (input("Press 1 to reverse: "))
        while(C == 1):
            GPIO.output(REVERSE, True)      
            GPIO.output(STOP, False)
            while(Distance <= 0 or Distance < 5):
                GPIO.output(STOP, True)
    time.sleep(0.05)