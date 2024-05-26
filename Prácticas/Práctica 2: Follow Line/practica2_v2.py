from GUI import GUI
from HAL import HAL
import math
import cv2
import numpy as np

# VALORES DE LAS CONSTANTES KP, KI Y KD IDEALES PARA CADA CIRCUITO:
# CIRCUITO POR DEFECTO:     KP = 2.25       KI = 1          KD = 5
# CIRCUITO DE MONTMELÓ:     KP = 9          KI = 7          KD = 11
# CIRCUITO DE NURBURGRING:  KP = 9.25       KI = 7.5        KD = 9

# CONSTANTS
V = 1
Vmax = 5
W = 0
KP = 9
KI = 7
KD = 11
prev_error = 0
sum_error = 0
i = 0

# METHODS
def image_filtering(image):
    img_hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    mask = cv2.add(mask0, mask1)
    output = cv2.bitwise_and(image, image, mask = mask)
    contour, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_contour = max(contour, key=cv2.contourArea)
    M = cv2.moments(max_contour)
    if M["m00"] != 0:
        center_x = int(M["m10"] / M["m00"])
        center_y = int(M["m01"] / M["m00"])
        center = (center_x, center_y)
    else:
        center = (329, 399)
    radius = 5
    color = (255, 0, 0)
    thickness = -1
    image = cv2.circle(image, center, radius, color, thickness)
    GUI.showImage(image)
    return center

def speed(V, W, reference, prev_error, sum_error, i):
    center = (329, 399)
    get_x, get_y = reference
    diff_x = center[0] - get_x
    diff_y = center[1] - get_y
    error = diff_x/480
    i += 1
    sum_error -= error
    W = KP * error + KD * (error - prev_error) + KI * sum_error/i
    V = Vmax/(abs(W) + 1)
    if V >= Vmax:
        V = Vmax
    return V , W, error

HAL.setV(0)
HAL.setW(0)

# PROGRAM EXECUTION
while True:
    camera = HAL.getImage()
    center = image_filtering(camera)
    V, W, prev_error = speed(V, W, center, prev_error, sum_error, i)
    HAL.setV(V)
    HAL.setW(W)
