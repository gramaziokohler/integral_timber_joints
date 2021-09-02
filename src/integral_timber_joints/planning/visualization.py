import numpy as np
from termcolor import cprint

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

def visualize_movement_trajectory(client, robot, process, m, step_sim=True, step_duration=0.1):
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
    if isinstance(m, RoboticMovement):
        print('===')
        cprint('Viz:')
        if m.trajectory is not None:
            cprint(m.short_summary, 'green')
            for jt_traj_pt in m.trajectory.points:
                client.set_robot_configuration(robot, jt_traj_pt)
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
