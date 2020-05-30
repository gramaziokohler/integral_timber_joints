from compas.geometry import Frame
from compas_fab.backends import RosClient
from compas_fab.robots import Configuration
import math

group = "robot11_eaXYZ"

frame_WCF = Frame([19.823254, 6.008556, 0.922020],
                  [-1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0])

frame_WCF_mm = Frame([19823.254, 6008.556, 922.020],
                  [-1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0])


start_configuration = Configuration(
    values = (
        18.700,
        0, -4.900,
        0, 0, 0, 0, 0, 0,
        -11.000, -4.900,
        0, 0, 0, 0, 0, 0,
        35.000,
        0, -4.900,
        0, 0, 0, 0, 0, 0,
        -11.000, -4.900,
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

start_configuration_mm = Configuration(
    values = (
        18700,
        0, -4900,
        0, 0, 0, 0, 0, 0,
        -11000, -4900,
        0, 0, 0, 0, 0, 0,
        35000,
        0, -4900,
        0, 0, 0, 0, 0, 0,
        -11000, -4900,
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

# Plan with Meters

with RosClient('192.168.183.129') as client:

    robot = client.load_robot(load_geometry=False,
                              urdf_param_name='/robot_description',
                              srdf_param_name='/robot_description_semantic',
                              precision=None,
                              local_cache_directory=None)

    # goal_constraints
    goal_constraints = robot.constraints_from_frame(frame_WCF, tolerance_position, tolerances_axes, group)

    try:
        trajectory = robot.plan_motion(goal_constraints,
                                    start_configuration,
                                    group,
                                    allowed_planning_time=2.0,
                                    planner_id="RRT",
                                    path_constraints=path_constraints)
        print("Computed kinematic path with %d configurations." % len(trajectory.points))
        print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)

    except Exception as e:
        print("robot.plan_motion error:" , e)


# Plan with Millimeters

with RosClient('192.168.183.129') as client:

    robot = client.load_robot(load_geometry=False,
                              urdf_param_name='/robot_description',
                              srdf_param_name='/robot_description_semantic',
                              precision=None,
                              local_cache_directory=None)

    robot.scale(1000)

    # goal_constraints
    goal_constraints = robot.constraints_from_frame(frame_WCF_mm, tolerance_position, tolerances_axes, group)

    try:
        trajectory = robot.plan_motion(goal_constraints,
                                    start_configuration_mm,
                                    group,
                                    allowed_planning_time=2.0,
                                    planner_id="RRT",
                                    path_constraints=path_constraints)
        print("Computed kinematic path with %d configurations." % len(trajectory.points))
        print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)

    except Exception as e:
        print("robot.plan_motion error:" , e)

