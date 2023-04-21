import numpy as np

def create_obstacles(num_obstacles=1, obstacle_diameter=0.5, 
    x_min=0, x_max=10, y_min=0, y_max=10, random=True):
    
    """
    Create 2d obstacles in the environment
    """
    obstacles = []

    if random == True:
        for i in range(num_obstacles):
            if i == num_obstacles-1:
                x = GOAL_X
                y = GOAL_Y
                obstacles.append([x, y, obstacle_diameter])
                continue
            x = np.random.uniform(x_min, x_max)
            y = np.random.uniform(y_min, y_max)
            obstacles.append([x, y, obstacle_diameter])

    else:
        
        # # #create a grid of obstacles
        # # for i in range(num_obstacles):
        # #     x = x_min + (x_max - x_min) * (i/(num_obstacles-1))

        # #     for j in range(int(num_obstacles)):
        # #         y = y_min + (y_max - y_min) * (j/(num_obstacles-1))
        # #         obstacles.append([x, y, obstacle_diameter])
        # #         y = y_min + (y_max - y_min) * (i/(num_obstacles-1))

        # #         obstacles.append([x, y, obstacle_diameter])

        # obstacles.append([50, 50, obstacle_diameter])
        # obstacles.append([50, 100, obstacle_diameter])
        # obstacles.append([100, 50, obstacle_diameter])
        # obstacles.append([100, 100, obstacle_diameter])
        # obstacles.append([-50, 50, obstacle_diameter])

        obstacles.append([25, 25, obstacle_diameter])
        obstacles.append([75, 75, obstacle_diameter])
        obstacles.append([-100, -100, obstacle_diameter])
        obstacles.append([-100, 100, obstacle_diameter])

        # obstacles.append([0, 0 , obstacle_diameter])s
        # obstacles.append([100, 100, obstacle_diameter])
        
        # obstacles.append([-150, 150, obstacle_diameter])
        # obstacles.append([-150, 100, obstacle_diameter])
        
        # obstacles.append([-50, -50, obstacle_diameter])
        # obstacles.append([-50, -100, obstacle_diameter])
        # obstacles.append([-100, -50, obstacle_diameter])
        # obstacles.append([-100, -100, obstacle_diameter])

        # obstacles.append([GOAL_X, GOAL_Y, 20])

        # x_array = np.arange(x_min, x_max, 25)
        # y_array = np.arange(y_min, y_max, 25)
        # for x in x_array:
        #     for y in y_array:
        #         obstacles.append([x, y, obstacle_diameter])
                
    return obstacles

## START 
START_X = 0
START_Y = 0
START_PSI = np.deg2rad(0)

## GOAL
GOAL_X = 200
GOAL_Y = 200
GOAL_Z = 45
GOAL_PSI = 0

#### OBSTACLES ####
OBSTACLE_AVOID = False
MOVING_OBSTACLE = False
MULTIPLE_OBSTACLE_AVOID = False
ROBOT_DIAMETER = 5

RADAR_AVOID = False
RADAR_USE_CASADI = False

OBSTACLE_X = 0
OBSTACLE_Y = 0         
OBSTACLE_DIAMETER = 75
OBSTACLE_VX = 0.0
OBSTACLE_VY = 0.0

X_MAX = 150
Y_MAX = 150
X_MIN = -150
Y_MIN = 0

N_OBSTACLES = 5 # +1 for goal
if MULTIPLE_OBSTACLE_AVOID:
    OBSTACLES = create_obstacles(N_OBSTACLES, OBSTACLE_DIAMETER + ROBOT_DIAMETER,
        x_min=X_MIN, x_max=X_MAX, y_min=Y_MIN, y_max=Y_MAX,
        random=False) 
    
    N_OBSTACLES = len(OBSTACLES)

BUFFER_DISTANCE = 10


#NLP solver options
MAX_ITER = 1500
MAX_TIME = 0.1
PRINT_LEVEL = 2
ACCEPT_TOL = 1e-2
ACCEPT_OBJ_TOL = 1e-2   
PRINT_TIME = 0


#Target options
TARGET_DISCHARGE_RATE = 0.1