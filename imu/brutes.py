import time
import smbus
from imusensor.MPU9250 import MPU9250

bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, 0x69)

imu.begin()

while True:
    imu.readSensor()  # lecture brute I2C

    print("Accel x: {:.2f} ; Accel y: {:.2f} ; Accel z: {:.2f}".format(
        imu.AccelVals[0],
        imu.AccelVals[1],
        imu.AccelVals[2]
    ))

    print("Gyro  x: {:.2f} ; Gyro  y: {:.2f} ; Gyro  z: {:.2f}".format(
        imu.GyroVals[0],
        imu.GyroVals[1],
        imu.GyroVals[2]
    ))

    print("Mag   x: {:.2f} ; Mag   y: {:.2f} ; Mag   z: {:.2f}".format(
        imu.MagVals[0],
        imu.MagVals[1],
        imu.MagVals[2]
    ))

    print("-" * 40)
    time.sleep(1)
