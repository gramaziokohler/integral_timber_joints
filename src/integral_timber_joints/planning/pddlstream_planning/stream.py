from collections import namedtuple
import time
from itertools import product, chain
from compas.geometry.primitives import frame
from pybullet_planning.interfaces.env_manager.user_io import wait_if_gui
from termcolor import cprint
import pybullet_planning as pp

from compas_fab.robots import Configuration
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose, pose_from_transformation

from integral_timber_joints.planning.robot_setup import GANTRY_ARM_GROUP
from integral_timber_joints.process.state import SceneState
from integral_timber_joints.planning.state import set_state, gantry_base_generator
from integral_timber_joints.planning.stream import _get_sample_bare_arm_ik_fn

from compas.robots import Configuration
from compas_fab.robots import Trajectory

######################################

def assign_fluent_state(client, robot, process, fluents):
    # * manually create a state according to the fluent facts and set_state
    state = SceneState(process)
    # by default all the attachment status is None (False)
    for oid in chain(process.assembly.sequence, process.tool_ids):
        state[(oid, 'a')] = False
    for fluent in fluents:
        name, args = fluent[0], fluent[1:]
        # print(fluent)
        if name == 'atpose':
            # (AtPose ?object ?pose)
            object_name, frame = args
            state[(object_name, 'f')] = frame
        elif name == 'attached':
            # (Attached ?object ?grasp)
            object_name, grasp_transform = args
            state[(object_name, 'a')] = True
            state[(object_name, 'g')] = grasp_transform
        else:
            raise ValueError(name)
    set_state(client, robot, process, state)
    return state

###########################################

from termcolor import colored
from integral_timber_joints.planning.solve import compute_movement
from integral_timber_joints.process.action import PickBeamWithGripperAction
from integral_timber_joints.process.movement import RoboticMovement, RoboticFreeMovement
from integral_timber_joints.planning.visualization import visualize_movement_trajectory

def apply_movement_state_diff(scene, movements, debug=False):
    for key in scene.object_keys:
        for m in movements[::-1]:
            if key in m.state_diff:
                scene[key] = m.state_diff[key]
                if debug:
                    print(colored(key, 'blue' if key[0]=='robot' else 'grey'), m.state_diff[key])
                break # ! break movement loop
    return scene

def get_action_ik_fn(client, process, robot, action_name, options=None):
    debug = options.get('debug', False)
    verbose = options.get('verbose', False)
    def sample_fn(obj_name, tool_name, fluents=[]):
        # ! :inputs (?object ?tool)
        # ! :outputs (?conf1 ?conf2 ?traj)
        # * assign `AtPose` and `Attached` predicates to state
        # start_state of the action
        # pp.wait_if_gui('PickBeam: Before assign fluent state')
        _action_scene = assign_fluent_state(client, robot, process, fluents)
        # print(client._print_object_summary())
        # pp.wait_if_gui('PickBeam: After assign fluent state')

        # if pddl_action.name == 'pick_beam_with_gripper':
        itj_action = PickBeamWithGripperAction(seq_n=0, act_n=0, beam_id=obj_name, gripper_id=tool_name)

        itj_action.create_movements(process)
        end_scene = _action_scene.copy()
        for i, movement in enumerate(itj_action.movements):
            if debug:
                print('='*5)
                print('{} - {}'.format(i, movement.short_summary))
            movement.create_state_diff(process)
            start_scene = end_scene.copy()
            start_scene = apply_movement_state_diff(start_scene, itj_action.movements[:i], debug=debug)
            # if debug:
                # print('~~~')
            end_scene = start_scene.copy()
            end_scene = apply_movement_state_diff(end_scene, [itj_action.movements[i]], debug=debug)
            # * Robot state is not the same as previous.
            end_scene[process.robot_config_key] = None
            # itj_action.movements[i].state_diff[process.robot_config_key]

            # * skip free and non-robotic movements
            if not isinstance(movement, RoboticMovement) or \
               isinstance(movement, RoboticFreeMovement):
               continue

            if compute_movement(client, robot, process, movement,
                options=options, diagnosis=options['diagnosis'], given_state_pair=(start_scene, end_scene)):
                # * conf backward-propagation
                m_id = itj_action.movements.index(movement)
                back_id = m_id-1
                while back_id >= 0:
                    back_m = itj_action.movements[back_id]
                    if not isinstance(back_m, RoboticMovement) or isinstance(back_m, RoboticFreeMovement):
                        back_m.state_diff[process.robot_config_key] = movement.trajectory.points[0]
                    else:
                        break
                    back_id -= 1
                if debug:
                    wait_if_gui('Start sim')
                    with pp.WorldSaver():
                        visualize_movement_trajectory(client, robot, process, movement, step_sim=False,
                            given_state_pair=(start_scene, end_scene), draw_polylines=True)
            else:
                cprint('sample pick element fails for {}.'.format(movement.short_summary), 'red')
                return None
        cprint('sample pick element succeeds for {}|{}.'.format(obj_name, tool_name), 'green')
        return (itj_action,)
    return sample_fn
