# Simple demo of the LSM9DS1 accelerometer, magnetometer, gyroscope.
# Will print the acceleration, magnetometer, and gyroscope values every second.
import time
import board
import busio
import adafruit_lsm9ds1
from math import atan, pi
 
# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

while True:
    # Read acceleration, magnetometer, gyroscope, temperature.
    #accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
#    gyro_x, gyro_y, gyro_z = sensor.gyro
#    temp = sensor.temperature
    Heading = atan(mag_y/mag_x)*180/pi
    print("Heading: {}".format(Heading))

    time.sleep(1.0)