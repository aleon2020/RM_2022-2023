from GUI import GUI
from HAL import HAL
import cv2
import numpy as np

# CONSTANTS
LAST_ERROR = 0
KP = 0.006
KD = 0.006

# PROGRAM EXECUTION
while True:
    IMAGE = HAL.getImage()
    HSV = cv2.cvtColor(IMAGE,cv2.COLOR_BGR2HSV).astype(np.float)
    LOWER_POINT = np.array([0,70,50])
    UPPER_POINT = np.array([10,255,255])
    MASK = cv2.inRange(HSV, LOWER_POINT, UPPER_POINT)
    H, W, D = IMAGE.shape
    SEARCH = 3 * H / 4
    SEARCH = SEARCH + 20
    M = cv2.moments(MASK)
    if M['m00'] > 0:
        X = int(M['m10']/M['m00'])
        Y = int(M['m01']/M['m00'])
        cv2.circle(IMAGE, (X, Y), 20, (0, 0, 255), -1)
        ERROR = X - W/2
        P_CONTROLLER = KP * (float(ERROR))
        D_CONTROLLER = KD * (float(ERROR) - float(LAST_ERROR))
        GUI.showImage(IMAGE)
        HAL.setV(2.25)
        HAL.setW(-P_CONTROLLER - D_CONTROLLER)
        LAST_ERROR = float(ERROR)
