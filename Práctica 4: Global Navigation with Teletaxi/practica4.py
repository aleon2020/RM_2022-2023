import math
import numpy as np
import cv2
from GUI import GUI
from HAL import HAL
from MAP import MAP

# METHODS
class Queue:
    def __init__(self):
        self.list = []
    def isEmpty(self):
        return len(self.list) == 0
    def push(self, item):
        self.list.insert(0,item)
    def pop(self):
        return self.list.pop()

def binarize_map(walls,grid):
    NW_CORNERS = []
    NE_CORNERS = []
    SE_CORNERS = []
    SW_CORNERS = []
    for coords in walls:
        x,y = coords
        grid[x][y] = 255
        if grid[x][y+1] != 0 and grid[x+1][y] != 0 and grid[x-1][y] != 0 and grid[x][y-1] != 0:
            if grid[x-1][y-1] == 0:
                NW_CORNERS.append(coords)
            elif grid[x-1][y+1] == 0:
                NE_CORNERS.append(coords)
            elif grid[x+1][y+1] == 0:
                SE_CORNERS.append(coords)
            else:
                SW_CORNERS.append(coords)
    for it in range(1,2):
        for coords in walls:
            x,y = coords
            if in_bounds(x,y+it) and grid[x][y+it] != 0:
                grid[x][y+it] = 255
            if in_bounds(x,y-it) and grid[x][y-it] != 0:
                grid[x][y-it] = 255
            if in_bounds(x+it,y) and grid[x+it][y] != 0:
                grid[x+it][y] = 255
            if in_bounds(x-it,y) and grid[x-it][y] != 0:
                grid[x-it][y] = 255
        for coords in SE_CORNERS:
            x,y = coords
            if in_bounds(x-1,y-1):
                grid[x-1][y-1] = 255
                for i in range(1,it+1):
                    if in_bounds(x-1,y-1+i):
                        grid[x-1][y-1+i] = 255
                    if in_bounds(x-1+i,y-1):
                        grid[x-1+i][y-1] = 255
        for coords in SW_CORNERS:
            x,y = coords
            if in_bounds(x-1,y+1):
                grid[x-1][y+1] = 255
                for i in range(1,it+1):
                    if in_bounds(x-1,y+1-i):
                        grid[x-1][y+1-i] = 255
                    if in_bounds(x-1+i,y+1):
                        grid[x-1+i][y+1] = 255
        for coords in NW_CORNERS:
            x,y = coords
            if in_bounds(x+1,y+1):
                grid[x+1][y+1] = 255
                for i in range(1,it+1):
                    if in_bounds(x+1,y+1-i):
                        grid[x+1][y+1-i] = 255
                    if in_bounds(x-1+i,y+1):
                        grid[x+1-i][y+1] = 255
        for coords in NE_CORNERS:
            x,y = coords
            if in_bounds(x-1,y-1):
                grid[x-1][y+1] = 255
                for i in range(1,it+1):
                    if in_bounds(x+1,y-1+i):
                        grid[x+1][y-1+i] = 255
                    if in_bounds(x+1-i,y-1):
                        grid[x+1-i][y-1] = 255
    GUI.showNumpy(grid)

def fill_gradient(initial_state,final_state,grid):
    GPP = Queue()
    OBSTACLES = []
    GPP.push((initial_state,0))
    EXTRA_IT = 0
    while not GPP.isEmpty():
        GUI.showNumpy(grid)
        CURRENT_STATE, CURRENT_COST = GPP.pop()
        x,y = CURRENT_STATE
        if final_state == CURRENT_STATE:
            while EXTRA_IT < EXTRA_ITERATIONS:
                if BINARY_MAP[x][y] == 0:
                    OBSTACLES.append(CURRENT_STATE)
                else:
                    for weight in get_unsigned_weights(CURRENT_STATE,CURRENT_COST,grid):
                        set_value_map(initial_state,weight[0],weight[1],grid)
                        GPP.push(weight)
                GUI.showNumpy(grid)
                try:
                    CURRENT_STATE, CURRENT_COST = GPP.pop()
                except IndexError:
                    break
                x,y = CURRENT_STATE
                EXTRA_IT += 1
            break
        if BINARY_MAP[x][y] == 0:
            OBSTACLES.append(CURRENT_STATE)
        else:
            for weight in get_unsigned_weights(CURRENT_STATE,CURRENT_COST,grid):
                set_value_map(initial_state,weight[0],weight[1],grid)
                GPP.push(weight)
    return OBSTACLES

def set_value_map(init_state,state,value,grid):
    x,y = state
    if(state != init_state) and (BINARY_MAP[x][y] != 0):
        if(grid[x][y] == 0) or (value < grid[x][y]):
            grid[x][y] = value

def get_unsigned_weights(state,cost,grid):
    x,y = state
    WEIGHTS = []
    try:
        if(grid[x-1][y] == 0) and ((x-1) >= 0):
            N_NE = ((x-1,y),cost+1)
            WEIGHTS.append(N_NE)
        if(grid[x][y+1] == 0) and ((y+1) < COLUMNS):
            E_NE = ((x,y+1),cost+1)
            WEIGHTS.append(E_NE)
        if(grid[x+1][y] == 0) and ((x+1) < ROWS):
            S_NE = ((x+1,y),cost+1)
            WEIGHTS.append(S_NE)
        if(grid[x][y-1] == 0) and ((y-1) >= 0):
            W_NE = ((x,y-1),cost+1)
            WEIGHTS.append(W_NE)
        if(grid[x-1][y+1] == 0) and ((x-1) >= 0) and((y+1) < COLUMNS):
            NE_NE = ((x-1,y+1),cost + DIAGONAL_DIRECTION)
            WEIGHTS.append(NE_NE)
        if(grid[x+1][y+1] == 0) and ((x+1) < ROWS) and((y+1) < COLUMNS):
            SE_NE = ((x+1,y+1),cost + DIAGONAL_DIRECTION)
            WEIGHTS.append(SE_NE)
        if(grid[x+1][y-1] == 0) and ((x+1) < ROWS) and((y-1) >= 0):
            SW_NE = ((x+1,y-1),cost + DIAGONAL_DIRECTION)
            WEIGHTS.append(SW_NE)
        if(grid[x-1][y-1] == 0) and ((x-1) >= 0) and((y-1) >= 0):
            NW_NE = ((x-1,y-1),cost + DIAGONAL_DIRECTION)
            WEIGHTS.append(NW_NE)
    except IndexError:
        return WEIGHTS
    return WEIGHTS

def assign_weights(state,grid):
    x,y = state
    N = grid[x-1][y]
    E = grid[x][y+1]
    S = grid[x+1][y]
    W = grid[x][y-1]
    NE = grid[x-1][y+1]
    SE = grid[x+1][y+1]
    SW = grid[x+1][y-1]
    NW = grid[x-1][y-1]
    WEIGHTS = [N,E,S,W,NE,SE,SW,NW]
    return WEIGHTS

def path_extraction(target,grid):
    PATH = []
    CURRENT_STATE = list(target)
    while grid[CURRENT_STATE[0]][CURRENT_STATE[1]] != 0:
        PATH.append(CURRENT_STATE)
        x,y = CURRENT_STATE
        WEIGHTS = assign_weights(CURRENT_STATE,grid)
        DIRECTION = WEIGHTS.index(min(WEIGHTS))
        if DIRECTION == N:
            x -= 1
        elif DIRECTION == E:
            y += 1
        elif DIRECTION == S:
            x += 1
        elif DIRECTION == W:
            y -= 1
        elif DIRECTION == NE:
            x -= 1
            y += 1
        elif DIRECTION == SE:
            x += 1
            y += 1
        elif DIRECTION == SW:
            x += 1
            y -= 1
        elif DIRECTION == NW:
            x -= 1
            y -= 1
        CURRENT_STATE = [x,y]
    PATH.append(CURRENT_STATE)
    return PATH

def in_bounds(x,y):
    if x < 0 or x >= ROWS or y < 0 or y >= COLUMNS:
        return False
    else:
        return True

def grid2world(grid_coords):
    x,y = grid_coords
    WORLD_X = (x-200)*1.25
    WORLD_Y = (y-200)*1.25
    return [WORLD_X,WORLD_Y]

# CONSTANTS
N, E, S, W = 0, 1, 2, 3
NE, SE, SW, NW = 4, 5, 6, 7
SUBOBJECTIVE_STEP = 4
SUBOBJECTIVE_REACHED = False
LAST = False
DIAGONAL_DIRECTION = 1.25
GRADIENT_MAP = MAP.getMap()
ORIGINAL_ROWS, ORIGINAL_COLUMNS = GRADIENT_MAP.shape
th, BINARY_MAP = cv2.threshold(GRADIENT_MAP,128,255,cv2.THRESH_BINARY)
EXTRA_ITERATIONS = 500
GRADIENT = np.zeros_like(GRADIENT_MAP)
ROWS, COLUMNS = GRADIENT.shape
Y_POSE, X_POSE = MAP.rowColumn(GUI.getTargetPose())
TARGET = (X_POSE,Y_POSE)
CAR_3D = HAL.getPose3d()
Y_CAR, X_CAR = MAP.rowColumn([CAR_3D.x, CAR_3D.y])
CAR = (X_CAR, Y_CAR)
WALLS_DETECTED = fill_gradient(TARGET,CAR,GRADIENT)
binarize_map(WALLS_DETECTED,GRADIENT)
CURRENT_PATH = path_extraction(CAR,GRADIENT)
NP_PATH = np.empty((len(CURRENT_PATH),2),dtype=np.uint16)
for row in range(len(CURRENT_PATH)):
    NP_PATH[row][0] = CURRENT_PATH[row][1]
    NP_PATH[row][1] = CURRENT_PATH[row][0]
GUI.showPath(NP_PATH)
i = 0
i += SUBOBJECTIVE_STEP
if i >= len(CURRENT_PATH):
    i = len(CURRENT_PATH) - 1
TARGET_X, TARGET_Y = grid2world(CURRENT_PATH[i])
MODULE = math.sqrt(math.pow(TARGET_X,2)+math.pow(TARGET_Y,2))
TARGET_ANGLE = math.asin(TARGET_X/MODULE)
CAR_3D = HAL.getPose3d()
CAR_ANGLE = CAR_3D.yaw
CAR_X = CAR_3D.x
CAR_Y = CAR_3D.y

# PROGRAM EXECUTION
while True:
    if SUBOBJECTIVE_REACHED:
        i += SUBOBJECTIVE_STEP
        if i >= len(CURRENT_PATH):
            i = len(CURRENT_PATH) - 1
            LAST = True
        TARGET_X,TARGET_Y = grid2world(CURRENT_PATH[i])
        MODULE = math.sqrt(math.pow(TARGET_X,2)+math.pow(TARGET_Y ,2))
        TARGET_ANGLE = math.asin(TARGET_X/MODULE)
        SUBOBJECTIVE_REACHED = False
    CAR_3D = HAL.getPose3d()
    CAR_ANGLE = CAR_3D.yaw
    CAR_X = CAR_3D.x
    CAR_Y = CAR_3D.y
    if CAR_ANGLE < 0:
        CAR_ANGLE = 2*math.pi + CAR_ANGLE
    if TARGET_ANGLE < 0:
        TARGET_ANGLE = 2*math.pi + TARGET_ANGLE
    DX = TARGET_X - CAR_X
    DY = TARGET_Y - CAR_Y
    REL_X = DX * math.cos(CAR_ANGLE) - DY * math.sin(-CAR_ANGLE)
    REL_Y = DX * math.sin(-CAR_ANGLE) + DY * math.cos(CAR_ANGLE)
    ATTRACTION_FORCE = (0.1*REL_X+1,0.1*REL_Y)
    HAL.setV(ATTRACTION_FORCE[0])
    HAL.setW(ATTRACTION_FORCE[1])
    if(-1.25 <= REL_X <= 1.25) and (-1.25 <= REL_Y <= 1.25):
        SUBOBJECTIVE_REACHED = True
        if LAST:
            HAL.setV(0)
            HAL.setW(0)
