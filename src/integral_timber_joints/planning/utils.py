import os, sys
import logging
from numpy import deg2rad, rad2deg
from termcolor import colored
from plyer import notification

from compas_fab.robots import Configuration, JointTrajectory, JointTrajectoryPoint, Duration
from integral_timber_joints.process import RoboticMovement

# fallback tolerance in meter, used for frame comparison
FRAME_TOL = 0.001

###########################################
# borrowed from: https://github.com/compas-dev/compas_fab/blob/3efe608c07dc5b08653ee4132a780a3be9fb93af/src/compas_fab/backends/pybullet/utils.py#L83
def get_logger(name):
    logger = logging.getLogger(name)

    try:
        from colorlog import ColoredFormatter
        formatter = ColoredFormatter("%(log_color)s%(levelname)-8s%(reset)s %(white)s%(message)s",
                                     datefmt=None,
                                     reset=True,
                                     log_colors={'DEBUG': 'cyan', 'INFO': 'green',
                                                 'WARNING': 'yellow',
                                                 'ERROR': 'red', 'CRITICAL': 'red',
                                                 }
                                     )
    except ImportError:
        formatter = logging.Formatter('%(asctime)s | %(name)s | %(levelname)s | %(message)s')

    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

    return logger

LOGGER = get_logger('itj_planning')

##########################################

def convert_robot_conf_unit(conf_vals, length_scale=1e-3, angle_unit='rad', prismatic_ids=range(0,2), revoluted_ids=range(2,8)):
    """angle_unit is the target angle unit to be converted into
    """
    if angle_unit == 'rad':
        angle_fn = deg2rad
    elif angle_unit == 'deg':
        angle_fn = rad2deg
    else:
        raise ValueError
    return [conf_vals[i]*length_scale for i in prismatic_ids] + [angle_fn(conf_vals[i]) for i in revoluted_ids]

def convert_rfl_robot_conf_unit(conf_vals, length_scale=1e-3, angle_unit='rad'):
    assert len(conf_vals) == 2+6 or len(conf_vals) == 3+6
    prismatic_id_until = 1 if len(conf_vals) == 8 else 2
    return convert_robot_conf_unit(conf_vals, length_scale, angle_unit,
        prismatic_ids=range(0, prismatic_id_until+1), revoluted_ids=range(prismatic_id_until+1, len(conf_vals)))

##########################################

def reverse_trajectory(traj):
    if traj is None:
        return traj
    jt_traj_pts = []
    joint_names = traj.points[0].joint_names
    for i, conf in enumerate(traj.points[::-1]):
        jt_traj_pt = JointTrajectoryPoint(conf.joint_values, conf.joint_types, time_from_start=Duration(i*1,0))
        jt_traj_pt.joint_names = conf.joint_names
        jt_traj_pts.append(jt_traj_pt)
    return JointTrajectory(trajectory_points=jt_traj_pts, joint_names=joint_names,
        start_configuration=jt_traj_pts[0], fraction=1.0)

def merge_trajectories(trajs):
    # assert len(trajs) > 1
    jt_traj_pts = []
    joint_names = trajs[0].points[0].joint_names
    cnt = 0
    for traj in trajs:
        if not traj:
            continue
        for conf in traj.points:
            jt_traj_pt = JointTrajectoryPoint(conf.joint_values, conf.joint_types, time_from_start=Duration(cnt,0))
            jt_traj_pt.joint_names = conf.joint_names
            jt_traj_pts.append(jt_traj_pt)
            cnt += 1
    return JointTrajectory(trajectory_points=jt_traj_pts, joint_names=joint_names,
        start_configuration=jt_traj_pts[0], fraction=1.0)

##########################################

def notify(msg=''):
    """Send a desktop notification for a given msg.
    See : https://pypi.org/project/plyer/

    Parameters
    ----------
    msg : str, optional
        message content, by default ''
    """
    if 'ipykernel' not in sys.modules:
        try:
            notification.notify(
                title='itj_planning',
                message=msg,
                app_icon=None,  # e.g. 'C:\\icon_32x32.ico'
                timeout=2,  # seconds
            )
        except ImportError:
            pass

def print_title(x, log_level='debug'):
    msg = colored(x, 'blue', 'on_white', attrs=['bold'])
    if log_level == 'debug':
        LOGGER.debug(msg)
    elif log_level == 'info':
        LOGGER.info(msg)
    elif log_level == 'error':
        LOGGER.error(msg)
    elif log_level == 'warn':
        LOGGER.warn(msg)

def color_from_success(success : bool):
    return 'green' if success else 'red'

##########################################

def robotic_movement_ids_from_beam_ids(process, beam_ids, movement_id=None):
    robotic_movement_ids = []
    if movement_id is not None:
        if not movement_id.startswith('A'):
            _movement_id = process.movements[int(movement_id)].movement_id
        else:
            _movement_id = movement_id
        robotic_movement_ids.append(_movement_id)
    else:
        for beam_id in beam_ids:
            for m in process.get_movements_by_beam_id(beam_id):
                if isinstance(m, RoboticMovement):
                    robotic_movement_ids.append(m.movement_id)
    return robotic_movement_ids

def beam_ids_from_argparse_seq_n(process, seq_n, movement_id=None, msg_prefix='Solving'):
    full_seq_len = len(process.assembly.sequence)
    if movement_id is not None:
        if movement_id.startswith('A'):
            movement = process.get_movement_by_movement_id(movement_id)
            beam_ids = [process.get_beam_id_from_movement_id(movement_id)]
        else:
            movement = process.movements[int(movement_id)]
            beam_ids = [process.get_beam_id_from_movement_id(movement.movement_id)]
        global_movement_id = process.movements.index(movement)
        LOGGER.info(colored('{} for movement #({}) {}'.format(msg_prefix, global_movement_id, movement.movement_id), 'cyan'))
    else:
        seq_n = seq_n or list(range(full_seq_len))
        for seq_i in seq_n:
            assert seq_i < full_seq_len and seq_i >= 0, 'Invalid seq_n input {}'.format(seq_i)
        if len(seq_n) == 0:
            # defaults to solve all beams
            beam_ids = [process.assembly.sequence[si] for si in range(seq_n, full_seq_len)]
        elif len(seq_n) == 1:
            beam_ids = [process.assembly.sequence[seq_n[0]]]
        elif len(seq_n) == 2:
            beam_ids = [process.assembly.sequence[seq_i] for seq_i in range(seq_n[0], seq_n[1]+1)]
        else:
            beam_ids = [process.assembly.sequence[seq_i] for seq_i in seq_n]
        LOGGER.info(colored('{} for beam {}'.format(msg_prefix, beam_ids), 'cyan'))
    return beam_ids
