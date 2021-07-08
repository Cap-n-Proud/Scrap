import time
import sys
import board
import busio

print("hello blinka!")

i2c = busio.I2C(board.SCL, board.SDA)

print("I2C devices found: ", [hex(i) for i in i2c.scan()])
