import sys
from numpy import deg2rad, rad2deg
from termcolor import cprint
from plyer import notification

from compas_fab.robots import Configuration, JointTrajectory, JointTrajectoryPoint, Duration

# in meter, used for frame comparison
FRAME_TOL = 0.001

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
    cprint(msg, 'yellow')

def print_title(x):
    print('\n\n')
    cprint(x, 'blue', 'on_white', attrs=['bold'])

def color_from_success(success : bool):
    return 'green' if success else 'red'
