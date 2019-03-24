# Better keep constants in a single safe place

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
LEG_RADIUS_MAX = 0.06 
STRIDE_MAX     = 0.6

# Rule-based follower parameters
MIN_DISTANCE_TO_TARGET  = 0.15

# Controller parameters 
# PID

KP = 100
KI = 0.2
KD = 0
