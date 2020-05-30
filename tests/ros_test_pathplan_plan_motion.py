import math
from compas.geometry import Frame, Point, Vector
from compas_fab.backends import RosClient
from compas_fab.backends import RosFileServerLoader
from compas_fab.robots import Configuration
from compas_fab.robots import RobotSemantics
from compas_fab.robots import Robot
from compas.robots import RobotModel


# inputs
scale_factor = 1000
max_step = 10.0
group = "robot12_eaYZ"
target_frame = Frame(
                    [1179.095, 3511.530, 2007.912],
                    [-1.000, 0, 0],
                    [0, 1.000, 0])

start_configuration = Configuration(
    values = (
        0,
        0, -4915,
        0, 0, 0, 0, 0, 0,
        -12237, -4915,
        0, 0, 0, 0, 0, 0,
        38000,
        0, -4915,
        0, 0, 0, 0, 0, 0,
        -12237, -4915,
        0, 0, 0, 0, 0, 0
        ),
    types = (
        2,
        2, 2,
        0, 0, 0, 0, 0, 0,
        2, 2,
        0, 0, 0, 0, 0, 0,
        2,
        2, 2,
        0, 0, 0, 0, 0, 0,
        2, 2,
        0, 0, 0, 0, 0, 0
        ),
    joint_names = (
        'bridge1_joint_EA_X',
        'robot11_joint_EA_Y', 'robot11_joint_EA_Z',
        'robot11_joint_1', 'robot11_joint_2', 'robot11_joint_3', 'robot11_joint_4', 'robot11_joint_5', 'robot11_joint_6',
        'robot12_joint_EA_Y', 'robot12_joint_EA_Z',
        'robot12_joint_1', 'robot12_joint_2', 'robot12_joint_3', 'robot12_joint_4', 'robot12_joint_5', 'robot12_joint_6',
        'bridge2_joint_EA_X',
        'robot21_joint_EA_Y', 'robot21_joint_EA_Z',
        'robot21_joint_1', 'robot21_joint_2', 'robot21_joint_3', 'robot21_joint_4', 'robot21_joint_5', 'robot21_joint_6',
        'robot22_joint_EA_Y', 'robot22_joint_EA_Z',
        'robot22_joint_1', 'robot22_joint_2', 'robot22_joint_3', 'robot22_joint_4', 'robot22_joint_5', 'robot22_joint_6'
        )
    )


# goal_constraints parameters
tolerance_position = 0.001
tolerance_xaxis_deg, tolerance_yaxis_deg, tolerance_zaxis_deg = (1, 1, 1)
tolerances_axes = [math.radians(tolerance_xaxis_deg), math.radians(tolerance_yaxis_deg), math.radians(tolerance_zaxis_deg)]


path_constraints = []


with RosClient('192.168.43.141') as client:

    # Load URDF from ROS with local cache enabled
    loader = RosFileServerLoader(client, local_cache=True)
    loader.robot_name = 'rfl'

    urdf = loader.load_urdf()
    srdf = loader.load_srdf()

    # Create robot model from URDF, SRDF and load geometry
    robot_model = RobotModel.from_urdf_string(urdf)
    # robot_model.load_geometry(loader)
    semantics = RobotSemantics.from_srdf_string(srdf, robot_model)

    robot = Robot(robot_model, None, semantics, client)
    robot.scale(scale_factor)


    # goal_constraints
    goal_constraints = robot.constraints_from_frame(target_frame, tolerance_position, tolerances_axes, group)

    result = robot.plan_motion(goal_constraints,
                               start_configuration,
                               group,
                               planner_id="RRT",
                               path_constraints=path_constraints)

    print(result)
