#Speed = 6*pi/5 * N / difference
from math import pi
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
N = 0  
starttime = time.time()

GPIO.add_event_detect(16, GPIO.RISING)
def callback(self):
    global N, starttime
    N = N + 1
    print("N: {}".format(N))
     
    if(N == 10):
        stoptime = time.time()
        difference = stoptime - starttime
        Speed = 6*pi/5 * N / difference
        starttime = time.time()
        N = 0
        print("Speed: {} cm/s".format(Speed))
           
GPIO.add_event_callback(16, callback)
