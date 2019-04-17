# Better keep constants in a single safe place

import numpy as np

# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Constants for occupancy grid.
FREE = 0
UNKNOWN = 1
OCCUPIED = 2

# Robot-related parameters
ROBOT_RADIUS = 0.105 / 2.

SPEED = .2
REVERSE_SPEED = -0.05
MAX_W = 40.0 * np.pi / 180.0
MAX_ACCEL = 0.1  # maximum acceleration [m/ss]
MAX_W_ACCEL = 40.0 * np.pi / 180.0
EPSILON = .1

# Robot Roles
HUMAN_FOLLOWER = "HUMAN_FOLLOWER"
TURTLE_FOLLOWER = "TURTLE_FOLLOWER"

# Robot namespaces 
ROBOT0 = "tb3_0"
ROBOT1 = "tb3_1"
ROBOT2 = "tb3_2"


# Human-related parameters -> measured empirically 
LEG_RADIUS_MIN = 0.03 
LEG_RADIUS_MAX = 0.08 
STRIDE_MAX     = 0.6

# Rule-based follower parameters
# MIN_DISTANCE_TO_TARGET  = 0.25
# MAX_DISTANCE_TO_TARGET  = 1.0

MIN_DISTANCE_TO_TARGET  = 0.5
MAX_DISTANCE_TO_TARGET  = 1.5

MAX_TARGET_DISPLACEMENT = 0.5

# Controller parameters 
# PID

KPu = 0.2
KIu = 0.2
KDu = 0

KPw = 0.1
KIw = 0.1
KDw = 0

# Laser scanner constants 
DEGREES_FIELD_OF_VIEW       = 70
DEGREES_FIELD_RESOLUTION    = 2
DEGREES_CONE_OF_VIEW        = 1


################################
######## LOGS RELATED ##########

LOG_ROOT_DIR            = "../logs"
# PID_CONTROLLER_LOG_PATH = "/PID/pid_log.txt"


SIM_LOOP_TIME = 0.1