from compas_fab.robots import Configuration
from termcolor import cprint
import numpy as np

from compas_fab_pychoreo.client import PyChoreoClient
from pybullet_planning import draw_pose, set_camera_pose, unit_pose, LockRenderer

from integral_timber_joints.planning.utils import convert_rfl_robot_conf_unit
from integral_timber_joints.planning.visualization import rfl_camera
from integral_timber_joints.planning.parsing import rfl_setup

############################################

MAIN_ROBOT_ID = 'robot11'
BARE_ARM_GROUP = 'robot11'
GANTRY_ARM_GROUP = 'robot11_eaXYZ'
GANTRY_GROUP = 'robot11_gantry'

# a preset configuration, only used in initializing the state
# this set the other unused robot to the right configurations
R11_INTER_CONF_VALS = convert_rfl_robot_conf_unit([21000.0, 0.0, -4900.0,
    0.0, -22.834741999999999, -30.711554, 0.0, 57.335655000000003, 0.0])
R12_INTER_CONF_VALS = convert_rfl_robot_conf_unit([-12237, -4000.8486330000001,
    0.0, -80.0, 65.0, 65.0, 20.0, -20.0])
R21_IDLE_CONF_VALS = convert_rfl_robot_conf_unit([38000, 0, -4915,
    0, 0, 0, 0, 0, 0])
R22_IDLE_CONF_VALS = convert_rfl_robot_conf_unit([-12237, -4915,
    0, 0, 0, 0, 0, 0])

############################################

# 2 for prismatic, 0 for revoluted
RFL_SINGLE_ARM_JOINT_TYPES = [2, 2, 0, 0, 0, 0, 0, 0,]

try:
    import ikfast_abb_irb4600_40_255
    IK_MODULE = ikfast_abb_irb4600_40_255
except ImportError as e:
    IK_MODULE = None
    cprint('{}, Using pybullet ik fn instead'.format(e), 'red')

# https://github.com/yijiangh/coop_assembly/blob/dev/src/coop_assembly/planning/robot_setup.py#L147
R11_JOINT_WEIGHTS = np.reciprocal([0.1, 0.1, 0.1,
        2.618, 2.618, 2.618, 6.2832, 6.2832, 7.854])
R11_JOINT_RESOLUTIONS = 10*np.array([0.01, 0.01, 0.01,
        0.01079, 0.00725, 0.012249, 0.009173, 0.037541, 0.01313])

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

def to_rlf_robot_full_conf(robot11_confval, robot12_confval):
    return Configuration(
        joint_values = (*robot11_confval, *robot12_confval,
                  *R21_IDLE_CONF_VALS, *R22_IDLE_CONF_VALS),
        joint_types = (
            *([2] + RFL_SINGLE_ARM_JOINT_TYPES), *RFL_SINGLE_ARM_JOINT_TYPES,
            *([2] + RFL_SINGLE_ARM_JOINT_TYPES), *RFL_SINGLE_ARM_JOINT_TYPES,),
        joint_names = (
            *rfl_robot_joint_names('robot11', include_gantry=True),
            *rfl_robot_joint_names('robot12', include_gantry=False),
            *rfl_robot_joint_names('robot21', include_gantry=True),
            *rfl_robot_joint_names('robot22', include_gantry=False),
            )
        )

############################################
# TODO use arm group from SRDF
def get_gantry_control_joint_names(robot_id='robot11'):
    return rfl_robot_joint_names(robot_id, True)[:3]

# meter
GANTRY_X_LIMIT = (14.5, 28) # (0, 37)
GANTRY_Y_LIMIT = (-9.5, 0) # (-9, 0)
GANTRY_Z_LIMIT = (-5, -1.7) # (-5, -1)

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
    cam = rfl_camera()
    set_camera_pose(cam['target'], cam['location'])

    return client, robot, robot_uid
