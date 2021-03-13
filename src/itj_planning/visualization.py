import numpy as np
from termcolor import cprint

from pybullet_planning import GREY, BLUE, YELLOW, GREEN, draw_pose, has_gui, wait_if_gui, wait_for_duration
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
    camera = {
        'location': np.array([14830.746366, 17616.580504, 9461.594828])*scale,
        'target' : np.array([24470.185559, 7976.896428, 2694.413294])*scale,
        'lens' : 50.0*scale,
        'up_direction':np.array([0.314401,-0.314409,0.895712])*scale,
    }
    return camera

################################################

def visualize_movement_trajectory(client, robot, process, m, step_sim=True):
    from .stream import set_state
    if not has_gui() or not isinstance(m, RoboticMovement):
        return
    print('===')
    cprint('Viz:')
    start_state = process.get_movement_start_state(m)
    set_state(client, robot, process, start_state)
    if m.trajectory is not None:
        cprint(m.short_summary, 'green')
        for jt_traj_pt in m.trajectory.points:
            client.set_robot_configuration(robot, jt_traj_pt)

            # for name, attachments in client.pychoreo_attachments.items():
            #     print('attached name: ', name)
            #     for attachment in attachments:
            #         print('Grasp pose: {}'.format(attachment.grasp_pose))

            if step_sim:
                wait_if_gui('Step conf.')
            else:
                wait_for_duration(0.1)
    else:
        has_start_conf = process.movement_has_start_robot_config(m)
        has_end_conf = process.movement_has_end_robot_config(m)
        cprint('No traj found for {}\n -- has_start_conf {}, has_end_conf {}'.format(m.short_summary,
            has_start_conf, has_end_conf), 'yellow')
    end_state = process.get_movement_end_state(m)
    set_state(client, robot, process, end_state)
    if step_sim:
        wait_if_gui('End state.')

