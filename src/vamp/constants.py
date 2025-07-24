DEFAULT_ITERATIONS = 1000000

# TODO: These need to be updated automatically for new robots - or ideally, we read these values
# from the robot module directly
ROBOT_RRT_RANGES = {
    "sphere": 1,
    "ur5": 1.5,
    "panda": 1.0,
    "fetch": 1.0,
    "baxter": 0.5,
    "spot": 1.0,
    }

ROBOT_FIRST_JOINT_LOCATIONS = {
    "fetch": [0.0, 0.0, 0.4],
    "ur5": [0.0, 0.0, 0.91],
    "panda": [0.0, 0.0, 0.0],
    }

ROBOT_MAX_RADII = {
    "ur5": 1.2,
    "fetch": 1.5,
    "panda": 1.19,
    }

POINT_RADIUS = 0.0025
