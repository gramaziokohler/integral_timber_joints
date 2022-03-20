from termcolor import cprint
import numpy as np

from compas.robots import Joint
from compas_fab.robots import Configuration
from compas_fab_pychoreo.client import PyChoreoClient
from pybullet_planning import draw_pose, set_camera_pose, unit_pose, LockRenderer, set_camera

from integral_timber_joints.planning.utils import convert_rfl_robot_conf_unit
from integral_timber_joints.planning.visualization import rfl_camera
from integral_timber_joints.planning.parsing import rfl_setup

############################################

MAIN_ROBOT_ID = 'robot11'
BARE_ARM_GROUP = 'robot11'
GANTRY_ARM_GROUP = 'robot11_eaXYZ'
GANTRY_GROUP = 'robot11_gantry'

############################################

import ikfast_abb_irb4600_40_255
TRAC_IK_TIMEOUT = 1.0 # 0.1
TRAC_IK_TOL = 1e-6
USE_TRACK_IK = True
try:
    import trac_ik_python
except ImportError:
    USE_TRACK_IK = False
    try:
        import ikfast_abb_irb4600_40_255
    except ImportError as e:
        # TODO: script to compile automatically
        raise ImportError('Please install TRAC-IK or compile IKFast.')
# cprint('Use Track IK: {}'.format(USE_TRACK_IK), 'yellow')

############################################

def rfl_robot_joint_names(robot_id='robot12', include_gantry=False):
    template = ['robot_joint_EA_Y', 'robot_joint_EA_Z', 'robot_joint_1', 'robot_joint_2',
        'robot_joint_3', 'robot_joint_4', 'robot_joint_5', 'robot_joint_6']
    joint_names = [robot_id + jn.split('robot')[1] for jn in template]
    if include_gantry:
        bridge_id = 1 if ('11' in robot_id or '12' in robot_id) else 2
        bridge_joint_name = 'bridge{}_joint_EA_X'.format(bridge_id)
        return [bridge_joint_name] + joint_names
    else:
        return joint_names

############################################
# TODO use arm group from SRDF
def get_gantry_control_joint_names(robot_id='robot11'):
    return rfl_robot_joint_names(robot_id, True)[:3]

# meter
GANTRY_X_LIMIT = (10, 28) # (0, 37)
GANTRY_Y_LIMIT = (-9.5, 0) # (-9.65, 0)
GANTRY_Z_LIMIT = (-5, -1.7) # (-4.915, -1)

# rm_limits = [(-175, 175), (-85, 145), (-175, 70), (-181, 181), (-120, 120), (-181, 181)]
def get_gantry_robot_custom_limits(robot_id='robot11'):
    joint_names = rfl_robot_joint_names(robot_id, True)
    return {
        joint_names[0] : GANTRY_X_LIMIT,
        joint_names[1] : GANTRY_Y_LIMIT,
        joint_names[2] : GANTRY_Z_LIMIT,
        joint_names[3] : (-3.054326, 3.05432),
        joint_names[4] : (-1.483529, 2.53072),
        joint_names[5] : (-3.054326, 1.22173),
        joint_names[6] : (-3.159045, 3.15904),
        joint_names[7] : (-2.094395, 2.09439),
        joint_names[8] : (-3.159045, 3.15904),
    }

def get_tolerances(robot, low_res=False):
    # for discussions on tolerance, see: https://github.com/gramaziokohler/integral_timber_joints/issues/145
    joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    joint_types = robot.get_joint_types_by_names(joint_names)
    res_ratio = 10 if low_res else 1.0
    # TODO joint resolution and weight from joint name
    # * threshold to check joint flipping
    joint_jump_tolerances = {}
    joint_compare_tolerances = {}
    joint_resolutions = {}
    for jt_name, jt_type in zip(joint_names, joint_types):
        # 0.1 rad = 5.7 deg
        if jt_type == Joint.REVOLUTE:
            joint_jump_tolerances[jt_name] = 10.0 * np.pi / 180.0 # 0.174 rad
            joint_resolutions[jt_name] = 10.0 * np.pi / 180.0 * res_ratio# 0.174 rad
            joint_compare_tolerances[jt_name] = 0.0017 # rad, try tightened to 0.001 if possible
        elif jt_type == Joint.PRISMATIC:
            joint_jump_tolerances[jt_name] = 0.05 # meter
            joint_resolutions[jt_name] = 0.05 * res_ratio # meter
            joint_compare_tolerances[jt_name] = 1e-5
        else:
            raise ValueError("Strange joint type {} | {}".format(jt_type, jt_name))
    tolerances = {
        'joint_jump_tolerances' : joint_jump_tolerances,
        'joint_compare_tolerances' : joint_compare_tolerances,
        'frame_compare_distance_tolerance' : 0.0011, # meter
        'frame_compare_axis_angle_tolerance' : 0.0025, # rad
        'joint_resolutions' : joint_resolutions,
        'joint_weights' : {jn : weight for jn, weight in zip(joint_names, R11_JOINT_WEIGHTS)},
        'joint_custom_limits' : get_gantry_robot_custom_limits(MAIN_ROBOT_ID),
        # the collision is counted when penetration distance is bigger than this value
        'collision_distance_threshold' : 0.0012, # in meter,
    }
    return tolerances

# TODO joint weight as np.reciprocal(joint velocity bound) from URDF
R11_JOINT_WEIGHTS = np.reciprocal([0.1, 0.1, 0.1,
        2.618, 2.618, 2.618, 6.2832, 6.2832, 7.854])

########################################

def load_RFL_world(viewer=True, verbose=False):
    urdf_filename, semantics = rfl_setup()
    client = PyChoreoClient(viewer=viewer, verbose=verbose)
    client.connect()

    with LockRenderer():
        robot = client.load_robot(urdf_filename)
    robot.semantics = semantics
    # robot's unique body index in pybullet
    robot_uid = client.get_robot_pybullet_uid(robot)

    # * draw base frame and locate camera in pybullet
    draw_pose(unit_pose(), length=1.)
    # cam_info = pp.get_camera()
    cam_info = rfl_camera()
    set_camera(cam_info.yaw, cam_info.pitch, cam_info.dist, cam_info.target)

    return client, robot, robot_uid
