DEFAULT_ITERATIONS = 1000000

ROBOT_JOINTS = {
    "ur5": [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
        ],
    "panda": [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
        ],
    "fetch": [
        "torso_lift_joint",
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
        ],
    "baxter": [
        "right_s0",
        "right_s1",
        "right_e0",
        "right_e1",
        "right_w0",
        "right_w1",
        "right_w2",
        "left_s0",
        "left_s1",
        "left_e0",
        "left_e1",
        "left_w0",
        "left_w1",
        "left_w2",
        ],
    "stretch" : [
        "joint_lift",
        "joint_arm_l3",
        "joint_arm_l2",
        "joint_arm_l1",
        "joint_arm_l0",
        "joint_wrist_yaw",
        "joint_head_pan",
        "joint_head_tilt",
        "joint_wrist_pitch",
        "joint_wrist_roll",
        "joint_gripper_finger_right",
        "joint_gripper_finger_left",
    ],
    }

ROBOT_RRT_RANGES = {
    "sphere": 1,
    "ur5": 1.5,
    "panda": 1.0,
    "fetch": 1.0,
    "baxter": 0.5,
    "stretch": 1.0,
    }

ROBOT_RADII_RANGES = {
    "baxter": (0.012, 0.08),
    "fetch": (0.012, 0.055),
    "panda": (0.012, 0.06),
    "sphere": (0.2, 0.2),
    "ur5": (0.015, 0.08),
    "stretch": (0.006, 0.105)
    }

ROBOT_FIRST_JOINT_LOCATIONS = {
    "fetch": [0.0, 0.0, 0.4],
    "ur5": [0.0, 0.0, 0.91],
    "panda": [0.0, 0.0, 0.0],
    "stretch": [-0.037385, 0.1666, 0.0],
    }

ROBOT_MAX_RADII = {
    "ur5": 1.2,
    "fetch": 1.5,
    "panda": 1.19,
    "stretch": 0.55,
    }

POINT_RADIUS = 0.0025
