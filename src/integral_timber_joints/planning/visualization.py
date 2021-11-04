import numpy as np
from termcolor import cprint

import pybullet_planning as pp
from pybullet_planning import GREY, BLUE, YELLOW, GREEN, draw_pose, has_gui, wait_if_gui, wait_for_duration, LockRenderer, CameraInfo
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

def rfl_camera():
    """Set the camera of the pybullet simulator to a particular pose.
    """
    # scale=1e-3
    # camera = {
    #     'location': np.array([14830.746366, 17616.580504, 9461.594828])*scale,
    #     'target' : np.array([24470.185559, 7976.896428, 2694.413294])*scale,
    #     'lens' : 50.0*scale,
    #     'up_direction':np.array([0.314401,-0.314409,0.895712])*scale,
    # }
    cam_info = CameraInfo(width=875, height=802, viewMatrix=(0.7501025199890137, 0.09660778194665909, -0.6542271375656128, 0.0, -0.6613215804100037, 0.10957715660333633, -0.7420556545257568, 0.0, 0.0, 0.9892723560333252, 0.14608290791511536, 0.0, -11.155038833618164, -4.927146911621094, 13.52570629119873, 1.0), projectionMatrix=(0.9165713787078857, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0, -0.02000020071864128, 0.0), cameraUp=(0.0, 0.0, 1.0), cameraForward=(0.6542271375656128, 0.7420556545257568, -0.14608290791511536), horizontal=(16367.5732421875, -14430.3349609375, 0.0), vertical=(1932.15576171875, 2191.543212890625, 19785.447265625), yaw=-41.40074920654297, pitch=-8.399992942810059, dist=7.179621696472168, target=(22.38941192626953, 8.52734088897705, 1.8495957851409912))
    return cam_info

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
