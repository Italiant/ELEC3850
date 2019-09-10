import RPi.GPIO as GPIO
import time

def ultrasonic():

    Trig = 23
    Echo = 24
   
    GPIO.setup(Trig, GPIO.OUT)
    GPIO.setup(Echo, GPIO.IN)

    GPIO.output(Trig, False)
    time.sleep(0.1)

    GPIO.output(Trig, True)
    time.sleep(0.00001)
    GPIO.output(Trig, False)

    while GPIO.input(Echo) == 0:
        pulse_start = time.time()

    while GPIO.input(Echo) == 1:
        pulse_end = time.time()

    duration = ((pulse_end - pulse_start)*17150) - 4

    distance = round(duration, 2)
    print("Distance From Wall: {}".format(distance))
    
	return distance
