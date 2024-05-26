from GUI import GUI
from HAL import HAL
import rospy, random, math, time
import numpy as np

# CONSTANTS
PI = math.pi
V = 1
W = V*PI
time_start = time.time()

# METHODS
def bumped(W):
    # INDICA DÓNDE HA DETECTADO EL CHOQUE
    bumper = HAL.getBumperData().bumper
    # DEVUELVE TRUE O FALSE SI HA DETECTADO EL CHOQUE
    crash = bool(HAL.getBumperData().state)
    if crash:
        # SI SE CHOCA POR LA DERECHA, GIRA A LA IZQUIERDA
        if bumper == 0:
            angle = random.uniform(-0.35, -0.25)
            if W < 0:
                W *= -1
        # SI SE CHOCA POR LA IZQUIERDA, GIRA A LA DERECHA
        elif bumper == 2:
            angle = random.uniform(0.25, 0.35)
            if W > 0:
                W *= -1
        # SI SE CHOCA DE FRENTE, RETROCEDE, GIRA UN ÁNGULO RANDOM Y SIGUE HACIA ADELANTE
        else:
            angle = random.uniform(0.45, 0.55)
        HAL.setV(-5)
        HAL.setW(0)
        rospy.sleep(1)
        HAL.setV(0)
        HAL.setW(angle)
        rospy.sleep(1)
    return crash, W

# PROGRAM EXECUTION
while True:
    HAL.setV(V)
    HAL.setW(W)
    bump, W = bumped(W)
    # SI NO SE CHOCA, MARCA EL TIEMPO ACTUAL, Y SI ÉSTE SUPERA AL PERIODO, AUMENTA LA VELOCIDAD
    if not bump:
        print(V, W)
        if (abs(W) >= 0.05):
            time_current = time.time()
            if (time_current - time_start) > (2*PI/abs(W)):
                time_start = time.time()
                V += 0.5
    # SI SE CHOCA, VUELVE A LA VELOCIDAD NORMAL
    else:
        V = 1
