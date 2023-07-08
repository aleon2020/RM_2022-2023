import math
import time
import random
from GUI import GUI
from HAL import HAL

# CONSTANTS
STATE, V, W = 0, 0, 2
OBSTACLE = time.time()

# METHODS
def parse_laser_data(laser_data):
    laser = []
    for i in range(180):
        dist = laser_data.values[i]
        angle = math.radians(i)
        laser += [(dist, angle)]
    return laser

# PROGRAM EXECUTION
while True:
    if STATE == 0:
        HAL.setV(V)
        HAL.setW(W)
        V += 0.0125
        laser_data= HAL.getLaserData()
        data = parse_laser_data(laser_data)
        for i in range(45):
            if data[60+i][0] < 0.5:
                OBSTACLE = time.time()
                STATE = 1
    elif STATE == 1:
        HAL.setV(-1)
        if (time.time() - OBSTACLE) > 1:
            OBSTACLE = time.time()
            TURN = random.randrange(-3, 3)
            STATE = 2
    elif STATE == 2:
        HAL.setV(0)
        HAL.setW(TURN)
        if (time.time() - OBSTACLE) > 2:
            STATE = 3
    elif STATE == 3:
        HAL.setV(2)
        HAL.setW(0)
        laser_data= HAL.getLaserData()
        data = parse_laser_data(laser_data)
        for i in range(45):
            if data[60+i][0] < 0.5:
                OBSTACLE = time.time()
                STATE = 1
