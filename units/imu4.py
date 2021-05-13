#!/usr/bin/python

from Adafruit_I2C import Adafruit_I2C
from time import sleep
import math

mpu6050 = Adafruit_I2C(0x68, 2)  # ADC 16bit
hmc5883 = Adafruit_I2C(0x1E, 2)  # ADC 12bit
ms5611 = Adafruit_I2C(0x77, 2)  # ADC 24bit

# mpu6050 i2c master enable bit
mpu6050.write8(0x6A, 0)
# mpu6050 i2c bypass enable bit
mpu6050.write8(0x37, 2)
# mpu6050 turn off sleep mode
mpu6050.write8(0x6B, 0)

# hmc5883 continuous mode
hmc5883.write8(0x02, 0x00)


# ms5611 reset
ms5611.writeList(0x1E, [])
sleep(2 / 1000.0)

# ms5611 prom
t = ms5611.readList(0xA2, 2)
c1 = t[0] * 256 + t[1]
t = ms5611.readList(0xA4, 2)
c2 = t[0] * 256 + t[1]
t = ms5611.readList(0xA6, 2)
c3 = t[0] * 256 + t[1]
t = ms5611.readList(0xA8, 2)
c4 = t[0] * 256 + t[1]
t = ms5611.readList(0xAA, 2)
c5 = t[0] * 256 + t[1]
t = ms5611.readList(0xAC, 2)
c6 = t[0] * 256 + t[1]
# c1=40127
# c2=36924
# c3=23317
# c4=23282
# c5=33464
# c6=28312


while True:
    # read mpu6050 accelerations in m/s^2
    ax = (mpu6050.readS8(0x3B) * 256 + mpu6050.readU8(0x3C)) / 16384.0
    ay = (mpu6050.readS8(0x3D) * 256 + mpu6050.readU8(0x3E)) / 16384.0
    az = (mpu6050.readS8(0x3F) * 256 + mpu6050.readU8(0x40)) / 16384.0
    # read mpu6050 temperature in C
    tt = (mpu6050.readS8(0x41) * 256 + mpu6050.readU8(0x42)) / 340.0 + 36.53
    # read mpu6050 gyroscope in degree/s
    gx = (mpu6050.readS8(0x43) * 256 + mpu6050.readU8(0x44)) / 131.0
    gy = (mpu6050.readS8(0x45) * 256 + mpu6050.readU8(0x46)) / 131.0
    gz = (mpu6050.readS8(0x47) * 256 + mpu6050.readU8(0x48)) / 131.0

    # wait hmc5883 to be ready
    while not hmc5883.readU8(0x09) & 0x01 == 1:
        pass
    # read hmc5883 magnetometer in gauss
    magx = (hmc5883.readS8(0x3) * 256 + hmc5883.readU8(0x4)) / 1090.0
    magz = (hmc5883.readS8(0x5) * 256 + hmc5883.readU8(0x6)) / 1090.0
    magy = (hmc5883.readS8(0x7) * 256 + hmc5883.readU8(0x8)) / 1090.0

    z = math.degrees(math.atan2(magy, magx))
    y = math.degrees(-math.atan2(ax, az))
    x = math.degrees(math.atan2(ay, az))

    ms5611.writeList(0x48, [])
    sleep(9 / 1000.0)
    adc = ms5611.readList(0x00, 3)
    d1 = adc[0] * 65536 + adc[1] * 256 + adc[2]
    # d1=9085466
    ms5611.writeList(0x58, [])
    sleep(9 / 1000.0)
    adc = ms5611.readList(0x00, 3)
    d2 = adc[0] * 65536 + adc[1] * 256 + adc[2]
    # d2=8569150
    dt = d2 - c5 * (1 << 8)
    temp = 2000 + dt * c6 / (1 << 23)
    off = c2 * (1 << 16) + c4 * dt / (1 << 7)
    sens = c1 * (1 << 15) + c3 * dt / (1 << 8)
    p = (d1 * sens / (1 << 21) - off) / (1 << 15)

    # print c1,c2,c3,c4,c5,c6,d1,d2,dt,temp,off,sens,p
    tempc = temp / 100.0
    press_now = p / 100.0
    print("tempc=", tempc, "mbar=", press_now)

    # print "angle=",
    # print "{:+03.3f}".format(x),
    # print "{:+03.3f}".format(y),
    # print "{:+03.3f}".format(z),
    # print "mag=",
    # print "{:+03.3f}".format(magx),
    # print "{:+03.3f}".format(magy),
    # print "{:+03.3f}".format(magz),
    # print "accel=",
    # print "{:+03.3f}".format(ax),
    # print "{:+03.3f}".format(ay),
    # print "{:+03.3f}".format(az),
    # print "tempc=",
    # print "{:+03.3f}".format(tt),
    print(gx)
    print(gy)
    print(gz)
