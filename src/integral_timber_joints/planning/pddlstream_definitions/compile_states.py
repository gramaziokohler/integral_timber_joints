from copy import copy
from compas.geometry import Transformation, Frame

from integral_timber_joints.process.state import ObjectState, SceneState, copy_state_dict
from integral_timber_joints.planning.visualization import color_from_object_id

def find_last_diff(plan_state_diff, action_index, scene, start_state):
    """Function to search backwards for the last diff related to a key"""
    for key in scene.keys_with_unknown_state(skip_robot_config=True):
        for i in range(action_index, -1, -1):
            if key in plan_state_diff[i]:
                scene[key] = plan_state_diff[i][key]
        if key in start_state:
            scene[key] = start_state[key]
        print("Warning: Cannot find diff for state: %s" % str(key))
    return scene

def states_from_pddlstream_plan(process, plan):
    plan_state_diff = []
    for action in plan:
        state_diff = {}
        if action.name == 'pick_element_from_rack':
            # TODO screwdriver needs special care here
            beam_id = action.args[0]
            tool_id = action.args[-1]
        elif action.name == 'place_element_on_structure':
            beam_id = action.args[0]
            tool_id = action.args[-1]
        elif action.name == 'pick_gripper_from_rack':
            tool_id = action.args[0]
            tool = process.tool(process.tool_id)
            toolchanger = process.robot_toolchanger
            tool_storage_frame_wcf = tool.tool_storage_frame
            t_world_from_toolbase = Transformation.from_frame(tool_storage_frame_wcf)
            tool_storage_frame_t0cf = Frame.from_transformation(t_world_from_toolbase * toolchanger.t_tcf_from_t0cf)
            state_diff[(tool_id, 'f')] = tool_storage_frame_wcf
            state_diff[('tool_changer', 'f')] = tool_storage_frame_t0cf
        elif action.name == 'pick_clamp_from_rack':
            tool_id = action.args[0]
        elif action.name == 'place_tool_at_rack':
            tool_id = action.args[0]
        elif action.name == 'pick_clamp_from_joint':
            tool_id = action.args[0]
            joint_key = (action.args[1], action.args[2])
        elif action.name == 'place_clamp_at_joint':
            tool_id = action.args[0]
            joint_key = (action.args[1], action.args[2])
        elif action.name == 'move':
            pass
        plan_state_diff.append(state_diff)

    # states = [process.initial_state]
    # scene = SceneState(process)
