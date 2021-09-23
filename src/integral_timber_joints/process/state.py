try:
    from typing import Dict, List, Optional, Tuple

    from integral_timber_joints.process import RobotClampAssemblyProcess
except:
    pass

import itertools

from compas.data import Data
from compas.geometry import Frame
from compas.geometry.transformations.transformation import Transformation
from compas.robots import Configuration


class SceneState(Data):
    """
    This is a dictionary like object that holds all the states of all objects in a given moment.
    This static moment is refered to as a scene.

    A movement has a start scene and end scene.
    """

    def __init__(self, process=None):
        # type: (RobotClampAssemblyProcess) -> None

        self.object_state_dict = {}

        self.object_keys = []
        if process is not None:
            assembly = process.assembly
            # self.beam_states = {} # type: dict[str, list]

            # compile a list of keys that should contain state values
            for beam_id in assembly.sequence:
                self.object_keys.append((beam_id, 'f'))
                self.object_keys.append((beam_id, 'a'))
            for tool_id in process.tool_ids:
                self.object_keys.append((tool_id, 'f'))
                self.object_keys.append((tool_id, 'a'))
                self.object_keys.append((tool_id, 'c'))

        # Singleton items
        self.object_keys.append(('tool_changer', 'f'))  # TC Base Frame
        self.object_keys.append(('tool_changer', 'a'))  # Always True
        self.object_keys.append(('robot', 'f'))  # Robot Target Frame
        self.object_keys.append(('robot', 'c'))  # Full Configuration

    @classmethod
    def from_data(cls, data):
        """Construct a RobotClampAssemblyProcess from structured data.
        Overridden the from_data method because we have to assign self to dependency.process
        """
        scene = cls()
        scene.data = data
        return scene

    def to_data(self):
        return self.data

    def __getitem__(self, key):
        if key not in self.object_state_dict:
            return None
        else:
            return self.object_state_dict[key]

    def __setitem__(self, key, value):
        self.object_state_dict[key] = value

    def __len__(self):
        return len(self.object_state_dict)

    def __contains__(self, key):
        return key in self.object_state_dict

    @property
    def data(self):
        # Flatten Tuple dict keys to strings
        flattened_dict = {}
        for key, value in self.object_state_dict.items():
            flattened_dict[str(key)] = value
        data = {
            'object_keys': self.object_keys,
            'flattened_dict': flattened_dict,
        }
        return data

    @data.setter
    def data(self, data):
        self.object_keys = data.get('object_keys', [])
        # Unflatten dict keys to Tuples
        import ast
        flattened_dict = data.get('flattened_dict', {})
        for key, value in flattened_dict.items():
            self[ast.literal_eval(key)] = value

    def has_unknown_state(self, skip_robot_config=True):
        # type: (RobotClampAssemblyProcess) -> bool
        return len(list(self.keys_with_unknown_state, skip_robot_config)) > 0

    def keys_with_unknown_state(self, skip_robot_config=True):
        for key in self.object_keys:
            if skip_robot_config and key == ('robot', 'c'):
                continue
            if key not in self:
                yield key


class ObjectState(object):
    """ Base class of a state object.
    Note that the State object should exist in a state dictionary where the key refers to an object id.

    `ObjectState.current_frame`
    - A frame describing the loaction of an object. `None` can be used to represent an un-transformed state.

    `ObjectState.kinematic_config`
    - A dictionary to describe the kinematic configuration of a RobotModel or ToolModel (such as Clamp and Gripper)
    - The dictionary uses `joint names` as keys and `values` in degrees or millimeters.
    - `None` can be used for Beams and Env Objects that does not have kinematic state

    `ObjectState.attached_to_robot`
    - Bool value describing if an object is attached to the robot.
    - Note that the ToolChanger is always attached to the robot's flange. (Level 1)
    - Gripper or Clamp can be attached to a robot's ToolChanger. (Level 2)
    - Beam can only be attached to a Gripper or Clamp (Level 3)
    """

    def __init__(self, current_frame=None, attached_to_robot=False, kinematic_config=None):
        self.current_frame = current_frame  # type: Frame
        self.attached_to_robot = attached_to_robot  # type: bool
        self.kinematic_config = kinematic_config  # type: ignore

    def to_data(self):
        """Simpliest way to get this class serialized.
        """
        return self.data

    @classmethod
    def from_data(cls, data):
        """Construct a Movement from structured data. Subclass must add their properity to
        the data properity.
        """
        state = cls()
        state.data = data
        return state

    @property
    def data(self):
        data = {
            'current_frame': self.current_frame,
            'kinematic_config': self.kinematic_config,
            'attached_to_robot': self.attached_to_robot,
        }
        return data

    @data.setter
    def data(self, data):
        self.current_frame = data.get('current_frame', None)
        self.kinematic_config = data.get('kinematic_config', None)
        self.attached_to_robot = data.get('attached_to_robot', False)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "State: current frame: {} | config: {} | attached to robot: {}".format(
            self.current_frame, self.kinematic_config, self.attached_to_robot)

    def __eq__(self, other):
        if not hasattr(other, 'current_frame') or not hasattr(other, 'attached_to_robot') or not hasattr(other, 'kinematic_config'):
            return False
        if self.current_frame != other.current_frame:
            return False
        if self.attached_to_robot != other.attached_to_robot:
            return False
        if self.kinematic_config != other.kinematic_config:
            return False
        return True


def get_object_from_flange(object_states, object_id):
    flange_frame = object_states['robot'].current_frame
    object_frame = object_states[object_id].current_frame
    world_from_flange = Transformation.from_frame(flange_frame)
    world_from_object = Transformation.from_frame(object_frame)

    return world_from_object.inverse() * world_from_flange


def copy_state_dict(target_state_dict, source_state_dict, clear=False, deep_copy=False):
    # type: (dict[str, ObjectState], dict[str, ObjectState], bool, bool) -> None
    """Copy one state dictionary to another.
    If `clear = True`, keys not present in the source_state_dict are removed.

    If `deep_copy = True`, deep copy is made for each object State """
    if clear:
        target_state_dict.clear()
    for object_id, state in source_state_dict.items():
        if deep_copy:
            target_state_dict[object_id] = ObjectState.from_data(state.to_data())
        else:
            target_state_dict[object_id] = state
