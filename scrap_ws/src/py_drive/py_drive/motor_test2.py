from robot import Robot
import time

r = Robot()
r.set_motors(0.3, 0.3)
time.sleep(3)
r.stop()
