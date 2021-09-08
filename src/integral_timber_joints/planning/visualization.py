import numpy as np
from termcolor import cprint

import pybullet_planning as pp
from pybullet_planning import GREY, BLUE, YELLOW, GREEN, draw_pose, has_gui, wait_if_gui, wait_for_duration, LockRenderer
from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement

################################################

BEAM_COLOR = GREY
GRIPPER_COLOR = BLUE
CLAMP_COLOR = YELLOW
TOOL_CHANGER_COLOR = GREEN

def color_from_object_id(object_id):
    if object_id.startswith('c'):
        return CLAMP_COLOR
    elif object_id.startswith('g'):
        return GRIPPER_COLOR
    elif object_id.startswith('t'):
        return TOOL_CHANGER_COLOR
    elif object_id.startswith('b'):
        return BEAM_COLOR
    else:
        return None

def sample_colors(num, lower=0.0, upper=1.0):
    # sample a color map based on a given integer number
    from matplotlib import cm
    # lower=0.0, upper=0.75
    # return [colorsys.hsv_to_rgb(h, s=1, v=1) for h in reversed(np.linspace(lower, upper, num, endpoint=True))]
    viridis = cm.get_cmap('viridis', 12)
    return viridis(np.linspace(lower, upper, num, endpoint=True))

################################################

def rfl_camera(scale=1e-3):
    """Set the camera of the pybullet simulator to a particular pose.
    """
    camera = {
        'location': np.array([14830.746366, 17616.580504, 9461.594828])*scale,
        'target' : np.array([24470.185559, 7976.896428, 2694.413294])*scale,
        'lens' : 50.0*scale,
        'up_direction':np.array([0.314401,-0.314409,0.895712])*scale,
    }
    return camera

################################################

def visualize_movement_trajectory(client, robot, process, m, step_sim=True, step_duration=0.1, draw_polylines=False, line_color=GREEN):
    """[summary]

    Parameters
    ----------
    client : [type]
    robot : compas_fab Robot
    process : [type]
    m : Movement
        the target movement to visualize
    step_sim : bool, optional
        Set to True if wish to step through the simulation conf-by-conf.
        False will simulate in a smooth fashion, by default True
    """
    from integral_timber_joints.planning.stream import set_state
    from integral_timber_joints.planning.robot_setup import GANTRY_ARM_GROUP
    if not has_gui():
        return
    start_scene = process.get_movement_start_scene(m)
    with LockRenderer():
        set_state(client, robot, process, start_scene)
    robot_uid = client.get_robot_pybullet_uid(robot)
    flange_link_name = robot.get_end_effector_link_name(group=GANTRY_ARM_GROUP)
    tool_link = pp.link_from_name(robot_uid, flange_link_name)
    last_point = None
    if isinstance(m, RoboticMovement):
        print('===')
        cprint('Viz:')
        if m.trajectory is not None:
            cprint(m.short_summary, 'green')
            for jt_traj_pt in m.trajectory.points:
                client.set_robot_configuration(robot, jt_traj_pt)
                if draw_polylines:
                    tool_pose = pp.get_link_pose(robot_uid, tool_link)
                    # pp.draw_pose(tool_pose)
                    if last_point:
                        pp.add_line(last_point, tool_pose[0], color=line_color)
                    last_point = tool_pose[0]
                if step_sim:
                    wait_if_gui('Step conf.')
                else:
                    wait_for_duration(step_duration)
        else:
            has_start_conf = process.movement_has_start_robot_config(m)
            has_end_conf = process.movement_has_end_robot_config(m)
            cprint('No traj found for {}\n -- has_start_conf {}, has_end_conf {}'.format(m.short_summary,
                has_start_conf, has_end_conf), 'yellow')
            wait_if_gui()
    end_scene = process.get_movement_end_scene(m)
    with LockRenderer():
        set_state(client, robot, process, end_scene)
    if step_sim:
        wait_if_gui('End scene.')
