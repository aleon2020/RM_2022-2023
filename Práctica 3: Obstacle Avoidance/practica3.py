import math
import numpy as np
from GUI import GUI
from HAL import HAL

# CONSTANTS
TARGET_REACHED = False
ALPHA = 3.2
BETA = 0.00004

# METHODS
def absolute2relative(x_abs, y_abs, robotx, roboty, robott):
    dx = x_abs - robotx
    dy = y_abs - roboty
    x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
    y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)
    return x_rel, y_rel

def parse_laser_data(laser_data):
    laser = []
    for i in range(len(laser_data.values)):
        dist = laser_data.values[i]/1000.0
        angle = math.radians(i)
        laser += [(dist, angle)]
    return laser

def laser_vector(laser):
    laser_vectorized = []
    for d,a in laser:
        x = d * math.cos(a - math.pi/2) * -1
        y = d * math.sin(a - math.pi/2) * -1
        v = (x,y)
        laser_vectorized += [v]
    return laser_vectorized

def attractive_force(x_rel,y_rel):
    att_phase = math.atan2(y_rel,x_rel)
    att_module = math.sqrt((x_rel**2) + (y_rel**2))
    if att_module > 2:
        att_forcex = 2 * math.cos(att_phase)
        att_forcey = 2 * math.sin(att_phase)
    else:
        att_forcex = att_module * math.cos(att_phase)
        att_forcey = att_module * math.sin(att_phase)
    return att_forcex, att_forcey

def repulsive_force(laser):
    sumX = 0
    sumY = 0
    for d,a in laser:
        x = 1/d * math.cos(a - math.pi/2) * -1
        y = 1/d * math.sin(a - math.pi/2) * -1
        sumX += x
        sumY += y
    rep_forcex = sumX
    rep_forcey = sumY
    return rep_forcex, rep_forcey

currentTarget = GUI.map.getNextTarget()
targetX = currentTarget.getPose().x
targetY = currentTarget.getPose().y

# PROGRAM EXECUTION
while True:
    # SHOW IMAGE
    image = HAL.getImage()
    GUI.showImage(image)
    # CURRENT TARGET
    if TARGET_REACHED:
        currentTarget.setReached(True)
        currentTarget = GUI.map.getNextTarget()
        targetX = currentTarget.getPose().x
        targetY = currentTarget.getPose().y
        TARGET_REACHED = False
    # POSITION OF THE ROBOT
    robotx = HAL.getPose3d().x
    roboty = HAL.getPose3d().y
    robott = HAL.getPose3d().yaw
    # CONVERT ABSOLUTE COORDINATES TO RELATIVE ONES
    targetx_rel, targety_rel = absolute2relative(targetX,targetY,robotx,roboty,robott)
    totalTarget = math.sqrt((targetx_rel**2) + (targety_rel**2))
    # LASER DATA
    laser_data = HAL.getLaserData()
    laser_data_parsed = parse_laser_data(laser_data)
    # OBSTACLE DIRECTION
    laserXY = laser_vector(laser_data_parsed)
    # ATTRACTIVE, REPULSIVE AND TOTAL VECTORS
    att_vector = attractive_force(targetx_rel,targety_rel)
    rep_vector = repulsive_force(laser_data_parsed)
    res_vector = (ALPHA*att_vector[0] + BETA*rep_vector[0],ALPHA*att_vector[1] + BETA*rep_vector[1])
    # VELOCITIES
    HAL.setV(res_vector[0])
    HAL.setW(res_vector[1])
    if totalTarget < 1.5:
        TARGET_REACHED = True
    # FORCES
    GUI.map.targetx = targetX
    GUI.map.targety = targetY
    GUI.map.carx = att_vector[0]
    GUI.map.cary = att_vector[1]
    GUI.map.obsx = rep_vector[0]
    GUI.map.obsy = rep_vector[1]
    GUI.map.avgx = res_vector[0]
    GUI.map.avgy = res_vector[1]
