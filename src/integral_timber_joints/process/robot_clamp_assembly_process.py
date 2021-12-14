from copy import deepcopy

import compas
from compas.data import Data
from compas.datastructures import Mesh
from compas.geometry import Frame, Transformation, Translation, Vector, Shape, Cylinder, dot_vectors
from compas.robots import RobotModel
from compas.rpc import Proxy
from compas_fab.robots import Configuration
from itertools import chain

from geometric_blocking import blocked
from integral_timber_joints.assembly import Assembly, BeamAssemblyMethod
from integral_timber_joints.geometry import Beam, EnvironmentModel, Joint
from integral_timber_joints.process.action import Action
from integral_timber_joints.process.dependency import ComputationalDependency, ComputationalResult
from integral_timber_joints.process.movement import Movement, RoboticFreeMovement, RoboticLinearMovement, RoboticMovement, \
    RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement
from integral_timber_joints.tools.beam_storage import BeamStorage
from integral_timber_joints.tools.clamp import Clamp
from integral_timber_joints.tools.gripper import Gripper
from integral_timber_joints.tools.screwdriver import Screwdriver
from integral_timber_joints.tools.pickup_station import GripperAlignedPickupStation, PickupStation, StackedPickupStation
from integral_timber_joints.tools.robot_wrist import RobotWrist
from integral_timber_joints.tools.tool import Tool
from integral_timber_joints.tools.tool_changer import ToolChanger


if compas.IPY:
    pass
else:
    # Type Checking imports
    from typing import TYPE_CHECKING
    if TYPE_CHECKING:
        from typing import List, Dict, Tuple, Optional
        from termcolor import colored, cprint
        from integral_timber_joints.process.state import ObjectState, SceneState


class RobotClampAssemblyProcess(Data):

    # Importing functions from neighbouring files to reduce this file size.
    from .compute_process_assembly_tools import (
        assign_tool_type_to_joints,
    )

    from .compute_process_action_movement import (
        recompute_initial_state,
        assign_unique_action_numbers,
        create_actions_from_sequence,
        create_movements_from_action,
        optimize_actions_place_pick_clamp,
        optimize_actions_place_pick_gripper,
        debug_print_process_actions_movements,
    )

    from .compute_process_screwdriver import (
        compute_screwdriver_positions,
    )

    from .compute_process_gripper import(
        assign_gripper_to_beam,
        compute_gripper_grasp_pose,
        set_grasp_face_following_assembly_direction,
        set_grasp_face_following_guide_vector,
        adjust_gripper_pos,
        override_grasp_face,
    )

    from .compute_process_pickup import(
        compute_pickup_frame,
        compute_pickup_location_by_aligning_corner,
        compute_beam_pickupapproach,
        compute_beam_finalretract,
        compute_beam_pickupretract,
        compute_storeage_frame,
    )

    # Constants for clamp jaw positions at different key positions.
    clamp_appraoch_position = 220
    clamp_inclamp_position = 210
    clamp_final_position = 100

    robot_config_key = ('robot', 'c')
    ROBOT_GANTRY_ARM_GROUP = 'robot11_eaXYZ'
    ROBOT_END_LINK = 'robot11_tool0'

    def __init__(self, assembly=None):
        # type: (Assembly) -> None

        super(RobotClampAssemblyProcess, self).__init__()

        if assembly is not None:
            assembly = assembly.copy()     # type: Assembly
        self.attributes = {}
        self.attributes['assembly'] = assembly
        self.attributes['clamps'] = {}
        self.attributes['grippers'] = {}
        self.attributes['screwdrivers'] = {}

        self._robot_model = None                                # RobotModel do not get saved with process
        self._robot_model_load_success = None                         # RobotModel load attempt result
        self.attributes['robot_toolchanger'] = None             # ToolChanger
        self.attributes['robot_wrist'] = None                   # RobotWrist
        self.attributes['robot_initial_config'] = None          # tuple(list(float), flaot)
        self.attributes['pickup_station'] = None                # PickupStation
        self.attributes['beam_storage'] = None                  # BeamStorage
        self.attributes['environment_models'] = {}              # dict[str, Mesh]

        self.attributes['initial_state'] = {}                   # dict(str, ObjectState)
        # self.attributes['intermediate_states'] = []             # list(dict(str, ObjectState))

        self.attributes['dependency'] = ComputationalDependency(self)   # ComputationalDependency

    @classmethod
    def from_data(cls, data):
        """Construct a RobotClampAssemblyProcess from structured data.
        Overridden the from_data method because we have to assign self to dependency.process
        """
        process = cls()
        process.data = data
        process.dependency.process = process
        # print ("Assigned self to dependency")
        return process

    def to_data(self):
        return self.data

    @property
    def data(self):
        """Return a data dict of this data structure for serialization.
        """
        data = {
            'attributes': self.attributes,
        }
        return data

    @data.setter
    def data(self, data):
        if 'data' in data:
            data = data['data']
        self.attributes.update(data.get('attributes') or {})

    @property
    def robot_model(self):
        # type: () -> RobotModel
        if self._robot_model_load_success is None:
            self.load_robot_model()
        return self._robot_model

    @property
    def robot_toolchanger(self):
        # type: () -> ToolChanger
        return self.attributes['robot_toolchanger']

    @robot_toolchanger.setter
    def robot_toolchanger(self, value):
        # type: (RobotWrist) -> None
        self.attributes['robot_toolchanger'] = value

    @property
    def robot_wrist(self):
        # type: () -> RobotWrist
        return self.attributes['robot_wrist']

    @robot_wrist.setter
    def robot_wrist(self, value):
        # type: (RobotWrist) -> None
        self.attributes['robot_wrist'] = value

    @property
    def robot_initial_config(self):
        # type: () -> Configuration
        """Returns the `robot_initial_config` that is set by the user.
        - If user have not set this attribute, and if the `process.robot_model` is present, the `zero_configuration()` is returned.
        - Else, return None.
        """

        if self.attributes['robot_initial_config'] is None:
            if self.robot_model is not None:
                return self.robot_model.zero_configuration()

        return self.attributes['robot_initial_config']

    @robot_initial_config.setter
    def robot_initial_config(self, value):
        # type: (Configuration) -> None
        self.attributes['robot_initial_config'] = value

    @property
    def actions(self):
        # type: () -> list[Action]
        actions = []
        for beam_id in self.assembly.sequence:
            actions.extend(self.assembly.get_beam_attribute(beam_id, 'actions'))
        return actions

    @property
    def movements(self):
        # type: () -> list[Movement]
        all_movements = []
        for action in self.actions:
            for movement in action.movements:
                all_movements.append(movement)
        return all_movements

    @property
    def pickup_station(self):
        # type: () -> PickupStation
        return self.attributes['pickup_station']

    @pickup_station.setter
    def pickup_station(self, value):
        # type: (PickupStation) -> None
        self.attributes['pickup_station'] = value

    @property
    def beam_storage(self):
        # type: () -> BeamStorage
        return self.attributes['beam_storage']

    @beam_storage.setter
    def beam_storage(self, value):
        # type: (BeamStorage) -> None
        self.attributes['beam_storage'] = value

    @property
    def environment_models(self):
        # type: () -> dict[str, Mesh]
        return self.attributes['environment_models']

    @environment_models.setter
    def environment_models(self, value):
        # type: (dict[str, Mesh]) -> None
        self.attributes['environment_models'] = value

    @property
    def dependency(self):
        # type: () -> ComputationalDependency
        return self.attributes['dependency']

    @property
    def initial_state(self):
        # type: () -> SceneState
        return self.attributes['initial_state']

    @initial_state.setter
    def initial_state(self, value):
        # type: (SceneState) -> None
        self.attributes['initial_state'] = value

    # ----------------
    # Properity access
    # ----------------

    @property
    def assembly(self):
        # type: () -> Assembly
        return self.attributes['assembly']

    def add_clamp(self, clamp):
        self.attributes['clamps'][clamp.name] = clamp.copy()

    def add_screwdriver(self, screwdriver):
        self.attributes['screwdrivers'][screwdriver.name] = screwdriver.copy()

    def add_gripper(self, gripper):
        self.attributes['grippers'][gripper.name] = gripper.copy()

    def add_tool(self, tool):
        # type: (Tool) -> None
        """Add new or overwrite existing Clamp, Screwdriver or Gripper.
        Add or Replace behaviour depends on the tool.name """
        if tool.__class__ is Clamp:
            self.attributes['clamps'][tool.name] = tool.copy()
        elif tool.__class__ is Screwdriver:
            self.attributes['screwdrivers'][tool.name] = tool.copy()
        elif tool.__class__ is Gripper:
            self.attributes['grippers'][tool.name] = tool.copy()
        else:
            raise TypeError("%s Class is weird." % tool.__class__.__name__)

    def delete_clamp(self, clamp_id):
        if clamp_id in self.attributes['clamps']:
            del self.attributes['clamps'][clamp_id]

    def delete_screwdriver(self, screwdriver_id):
        if screwdriver_id in self.attributes['screwdrivers']:
            del self.attributes['screwdrivers'][screwdriver_id]

    def delete_gripper(self, gripper_id):
        if gripper_id in self.attributes['grippers']:
            del self.attributes['grippers'][gripper_id]

    def delete_tool(self, tool_id):
        # type: (str) -> None
        """Delete existing Clamp, Screwdriver or Gripper
        by the tool_id given."""
        if tool_id in self.attributes['clamps']:
            del self.attributes['clamps'][tool_id]
        elif tool_id in self.attributes['screwdrivers']:
            del self.attributes['screwdrivers'][tool_id]
        elif tool_id in self.attributes['grippers']:
            del self.attributes['grippers'][tool_id]
        else:
            raise ValueError("Tool id %s cannot be found to be deleted." % tool_id)

    def clamp(self, clamp_id):
        # type: (str) -> Clamp
        return self.attributes['clamps'][clamp_id]

    def screwdriver(self, screwdriver_id):
        # type: (str) -> Screwdriver
        return self.attributes['screwdrivers'][screwdriver_id]

    def gripper(self, gripper_id):
        # type: (str) -> Clamp
        return self.attributes['grippers'][gripper_id]

    @property
    def clamps(self):
        # type: () -> list[Clamp]
        return self.attributes['clamps'].values()

    @property
    def screwdrivers(self):
        # type: () -> list[Screwdriver]
        return self.attributes['screwdrivers'].values()

    @property
    def grippers(self):
        # type: () -> list[Gripper]
        return self.attributes['grippers'].values()

    def tool(self, tool_id):
        # type: (str) -> Tool
        if tool_id in self.attributes['grippers']:
            return self.gripper(tool_id)
        elif tool_id in self.attributes['clamps']:
            return self.clamp(tool_id)
        elif tool_id in self.attributes['screwdrivers']:
            return self.screwdriver(tool_id)
        else:
            raise KeyError("tool_id ({}) cannot be found in process.grippers/screwdrivers/clamps".format(tool_id))

    @property
    def tools(self):
        # type: () -> list[Tool]
        """Return a list of all grippers, screwdrivers and clamps."""
        return list(self.clamps) + list(self.screwdrivers) + list(self.grippers)

    @property
    def tool_ids(self):
        # type: () -> list[str]
        """Return all Clamp, Screwdriver and Gripper ids."""
        return list(self.attributes['clamps'].keys()) + list(self.attributes['screwdrivers'].keys()) + list(self.attributes['grippers'].keys())

    def environment_model(self, env_id):
        # type: (str) -> EnvironmentModel
        return self.environment_models[env_id]

    @property
    def available_clamp_types(self):
        # type: () -> set[str]
        return set([clamp.type_name for clamp in self.clamps])

    @property
    def available_assembly_tool_types(self):
        # type: () -> set[str]
        return set([clamp.type_name for clamp in chain(self.clamps, self.screwdrivers)])

    @property
    def available_screwdriver_types(self):
        # type: () -> set[str]
        return set([screwdriver.type_name for screwdriver in self.screwdrivers])

    @property
    def available_gripper_types(self):
        # type: () -> set[str]
        return set([gripper.type_name for gripper in self.grippers])

    def get_one_tool_by_type(self, type_name):
        # type: (str) -> Tool
        """Get one of the Clamp, Screwdriver or Gripper that belongs to the given type"""
        for tool in self.tools:
            if (tool.type_name == type_name):
                return tool
        return None

    def get_one_gripper_by_type(self, type_name):
        # type: (str) -> Gripper
        """Get one of the clamp that belongs to the given type"""
        for gripper in self.grippers:
            if (gripper.type_name == type_name):
                return gripper
        return None

    def get_tool_type_of_joint(self, joint_id):
        # type: (tuple[str, str]) -> str
        """Returns the clamp_type used at the joint.
        `joint_id` expects (earlier_beam_id, later_beam_id)

        None, if the joint do not need a clamp (determined by assembly_method)
        """
        beam_id = joint_id[1]  # This id is the beam to be assembled
        assembly_method = self.assembly.get_assembly_method(beam_id)
        if assembly_method in [BeamAssemblyMethod.GROUND_CONTACT, BeamAssemblyMethod.UNDEFINED, BeamAssemblyMethod.MANUAL_ASSEMBLY]:
            return None
        else:
            return self.assembly.get_joint_attribute(joint_id, 'tool_type')

    def get_tool_of_joint(self, joint_id, position_name=''):
        # type: (tuple[str, str], Optional[str]) -> Tool
        """Returns the clamp/screwdriver object being set at the position
        specified by 'position_name', default '' will be translated to
        'clamp_wcf_final' or 'screwdriver_assembled_attached',
        refering to the attached position on the beam.

        If joint_attribute 'tool_id' is set, the exact tool object is returned
        otherwise a random tool of the correct type (from 'tool_type') is returned.

        If 'position_name' = None, clamp frame will not be modified.

        If get_tool_type_of_joint(joint_id) is None (meaning joint require no tool),
        None is returned.

        Warning: This clamp object is not deep-copied.
        Modifying it will change its definition in the process.
        """
        # Fetching the correct tool object
        beam_id = joint_id[0]
        tool_id = self.assembly.get_joint_attribute(joint_id, 'tool_id')
        tool_type = self.get_tool_type_of_joint(joint_id)
        if tool_type is None:
            return None
        if tool_id is not None and tool_id in self.tool_ids:
            tool = self.tool(tool_id)
            # print("Getting Joint(%s) Tool %s (%s)" % (joint_id, tool_type, tool_id))
        else:
            tool = self.get_one_tool_by_type(tool_type)
            # print("Getting Joint(%s) get_one_tool_by_type Tool %s (%s)" % (joint_id, tool_type, tool_id))

        # Setting position
        if position_name is not None:
            if position_name == "":
                if isinstance(tool, Screwdriver):
                    position_name = 'screwdriver_assembled_attached'
                elif isinstance(tool, Clamp):
                    position_name = 'clamp_wcf_final'
                else:
                    raise KeyError("get_tool_of_joint: %s('%s') is not a Clamp or Screwdriver." % (tool_type, tool_id))
            tool_wcf = self.assembly.get_joint_attribute(joint_id, position_name)
            # print("Setting it to position %s = %s"  % (position_name, tool_wcf))
            if tool_wcf is None:
                raise KeyError("get_tool_of_joint(joint_id, position_name): cannot find `%s` in joint attribute `%s`" % (joint_id, position_name))
            tool.current_frame = tool_wcf
        return tool

    def get_tool_t0cf_at(self, joint_id, position_name):
        # type: (tuple[str, str], str) -> Frame
        """ Returns the t0cf (flange frame) of the robot (in WCF)
        when the Clamp/Screwdriver is at the specified key position.
        Taking into account of the tool changer.

        Actually it should be called T0CF (Tool0 coordinate frame). Tool0 means no tool.

        process.robot_toolchanger, gripper must be set before calling this.

        Note
        ----
        - This is the robot flange.
        - The result can be used directly for path planning.
        """
        world_from_toolbase = self.assembly.get_joint_attribute(joint_id, position_name)
        if world_from_toolbase is None:
            print("Warning ! Attempt to process.get_tool_t0cf_at(%s, %s). Cannot find position name in joint attribute." % (str(joint_id), position_name))
        tool_changer = self.robot_toolchanger
        tool_changer.set_current_frame_from_tcp(world_from_toolbase)

        return tool_changer.current_frame.copy()

    def get_gripper_of_beam(self, beam_id, position_name=''):
        # type: (str, Optional[str]) -> Gripper
        """Returns the gripper used for a beam specified by beam_attribute
        'gripper_id' and 'gripper_type'. It can be a Clamp/Screwdriver/Gripper
        object.

        If 'gripper_id' is set, the exact tool object is returned
        otherwise a random tool of the correct type (from 'gripper_type') is returned.

        - If 'position_name' is given, the gripper will be set at the position where
        the beam is at `position_name`.
        - If left at default value '', it will be translated to
        'assembly_wcf_final', refering to the assembled position of the beam.
        - If 'position_name' = None, gripper frame will not be modified.

        Warning: This gripper object is not deep-copied.
        Modifying it will change its definition in the process.
        """
        # * Fetching the correct tool object
        gripper_id = self.assembly.get_beam_attribute(beam_id, 'gripper_id')
        gripper_type = self.assembly.get_beam_attribute(beam_id, 'gripper_type')
        if gripper_id is not None and gripper_id in self.tool_ids:
            gripper = self.tool(gripper_id)
            # print("Getting Joint(%s) Tool %s (%s)" % (joint_id, tool_type, tool_id))
        else:
            gripper = self.get_one_tool_by_type(gripper_type)
            # print("Getting Joint(%s) get_one_tool_by_type Tool %s (%s)" % (joint_id, tool_type, tool_id))

        # * Resolve position_name
        if position_name is None:
            return gripper
        elif position_name == "":
            position_name = 'assembly_wcf_final'

        # * Set Beam position and gripper position
        # Get the Gripper Tip Frame (TCP) at the beam's final-frame (gripper_tcp_in_wcf)
        gripper_tcp_in_ocf = self.assembly.get_beam_attribute(beam_id, "gripper_tcp_in_ocf")
        if gripper_tcp_in_ocf is None:
            raise KeyError("gripper_tcp_in_ocf is None for Beam %s" % (beam_id))
        t_beam_from_grippertip = Transformation.from_frame(gripper_tcp_in_ocf)

        f_world_from_beam = self.assembly.get_beam_attribute(beam_id, position_name)
        if f_world_from_beam is None:
            raise KeyError("f_world_from_beam is None for Beam %s" % (beam_id))
        t_world_from_beam = Transformation.from_frame(f_world_from_beam)

        # Main formular from world to gripper tip
        t_world_from_grippertip = t_world_from_beam * t_beam_from_grippertip
        gripper.set_current_frame_from_tcp(Frame.from_transformation(t_world_from_grippertip))

        return gripper

    def get_available_tool_type_dict(self):
        # type: () -> Dict[str, str]
        """Returns a dictionary of available tools.
        Dictonary `key` is `tool_type`
        Dictionary Item is a list of `tool_id` belonging to that type, sorted.
        """
        available_tools = {}
        for tool in self.tools:
            if not tool.type_name in available_tools:
                available_tools[tool.type_name] = [tool.name]
            else:
                available_tools[tool.type_name].append(tool.name)
        # Sort them according to id names
        for tool_type in available_tools:
            available_tools[tool_type].sort()

        return available_tools

    # -----------------------
    # Jaw Approach Algorithms
    # -----------------------

    def compute_jawapproach_vector_length(self, beam_id, vector_dir, min_dist=20, max_dist=150):
        # type: (str, Vector, float, float) -> Vector
        """Return a `assembly_vector_jawapproach` with correct length that will clear the clamp jaws
        Current implementation do not tkae into account of the angle of the clamp.
        Need to be improved when angled lap joints exist.
        """
        # Compute the length of the movement to clear the clamp jaw
        clearance_length = min_dist
        for joint_id in self.assembly.get_joint_ids_with_tools_for_beam(beam_id):
            clamp = self.get_tool_of_joint(joint_id, None)
            clearance_length = max(clearance_length, clamp.jaw_clearance_vectors_in_wcf.length)
        clearance_length = min(max_dist, clearance_length)

        return vector_dir.unitized().scaled(clearance_length)

    def compute_jawapproach_vector_from_guide_vector_dir(self, beam_id):
        # type: (str) -> Vector
        """Return a `assembly_vector_jawapproach` that follows the direction of `design_guide_vector_jawapproach`
        """
        # Get the guide Vector from beam_attribute
        design_guide_vector_jawapproach = self.assembly.get_beam_attribute(beam_id, 'design_guide_vector_jawapproach')
        assert design_guide_vector_jawapproach is not None

        # Check to see if the guide is blocked by any of the clamp jaws
        for joint_id in self.assembly.get_joint_ids_with_tools_for_beam(beam_id):
            clamp = self.get_tool_of_joint(joint_id, 'clamp_wcf_final')
            if blocked(clamp.jaw_blocking_vectors_in_wcf, design_guide_vector_jawapproach.scaled(-1.0)):
                return None

        # Compute the vector length to clear jaw, and return the `assembly_vector_jawapproach`
        return self.compute_jawapproach_vector_length(beam_id, design_guide_vector_jawapproach)

    def compute_jawapproach_vector_from_clamp_jaw_non_blocking_region(self, beam_id, verbose=False):
        # type: (str, bool) -> Vector
        """Return a `assembly_vector_jawapproach` that follows the direction of `design_guide_vector_jawapproach`
        Returns None if the clamps do not present a feasible region.
        """

        # Package to compute Feasible Region
        geometric_blocking = Proxy(package='geometric_blocking')

        # Collect blocking vectors from attached clamps
        blocking_vectors = []
        for joint_id in self.assembly.get_joint_ids_with_tools_for_beam(beam_id):
            clamp = self.get_tool_of_joint(joint_id, 'clamp_wcf_final')
            blocking_vectors += clamp.jaw_blocking_vectors_in_wcf
        # Fix rounding error for blocking vectors (Some -0.0 causes problems)
        ROUNDING_DIGITS = 10
        blocking_vectors = [Vector(round(x, ROUNDING_DIGITS), round(y, ROUNDING_DIGITS), round(z, ROUNDING_DIGITS)) for x, y, z in blocking_vectors]

        # Compute the analytical result of the feasible region and take a average from the resulting region
        feasible_disassem_rays, lin_set = geometric_blocking.compute_feasible_region_from_block_dir(blocking_vectors)
        if len(feasible_disassem_rays) == 0:
            print('- Warning : no feasible_disassem_rays from compute_feasible_region_from_block_dir')
            return None

        if verbose:
            print('feasible_disassem_rays from compute_feasible_region_from_block_dir: %s' % feasible_disassem_rays)

        # # Remove the feasible_rays that are linear (bi-directional)
        if len(feasible_disassem_rays) > 1:
            feasible_disassem_rays = [Vector(*ray) for (index, ray) in enumerate(feasible_disassem_rays) if (index not in lin_set)]
        feasible_rays_averaged = Vector.sum_vectors(feasible_disassem_rays)

        # Compute the vector length to clear jaw, and return the `assembly_vector_jawapproach`
        return self.compute_jawapproach_vector_length(beam_id, feasible_rays_averaged.scaled(-1))

    def compute_jawapproach_vector_from_joints_y(self, beam_id):
        # type: (str) -> Vector
        """Search for the `assembly_vector_jawapproach` vector from the engaging joint's Y direction.
        This is effectively the direction pointing to the end of the beam.
        """
        # Take the face_id from the first joint with neighbour
        joint_id = list(self.assembly.get_joints_of_beam_connected_to_already_built(beam_id))[0]
        face_id = self.assembly.joint(joint_id).face_id

        face_y_axis = self.assembly.beam(beam_id).reference_side_wcf(face_id).yaxis

        # Gather blocking vectors from all clamps
        blocking_vectors = []
        for joint_id in self.assembly.get_joint_ids_with_tools_for_beam(beam_id):
            clamp = self.get_tool_of_joint(joint_id, 'clamp_wcf_final')
            blocking_vectors += clamp.jaw_blocking_vectors_in_wcf

        # Check if either direction of the vector is free
        if not blocked(blocking_vectors, face_y_axis):
            return self.compute_jawapproach_vector_length(beam_id, face_y_axis.scaled(-1.0))
        if not blocked(blocking_vectors, face_y_axis.scaled(-1.0)):
            return self.compute_jawapproach_vector_length(beam_id, face_y_axis.copy())
        return None

    def search_valid_jawapproach_vector_prioritizing_guide_vector(self, beam_id, verbose=False):
        # type: (str, bool) -> Vector
        """Search for the `assembly_vector_jawapproach` and `assembly_wcf_inclampapproach`.
        1st priority is to follow direction of `design_guide_vector_jawapproach`.
        2nd priority is to use a vector in the middle of the feisible non-blocking region from the clamp jaw.
        Returns None if no valid vector can be found or no clamps are attached.

        Side Effect
        -----------
        beam_attribute 'assembly_vector_jawapproach' will be set.
        beam_attribute 'assembly_wcf_inclampapproach' will be set.
        """
        # Exit if no clamp is needed to assemble this beam.
        if len(list(self.assembly.get_joint_ids_with_tools_for_beam(beam_id))) == 0:
            return None

        # Compute vector with different strategies.
        # Each strategy function must return None if no valid solution is found.
        assembly_vector_jawapproach = self.compute_jawapproach_vector_from_guide_vector_dir(beam_id)
        if assembly_vector_jawapproach is None:
            assembly_vector_jawapproach = self.compute_jawapproach_vector_from_clamp_jaw_non_blocking_region(beam_id, verbose=verbose)
        if assembly_vector_jawapproach is None:
            return None

        # Save result in beam_attribute
        self.assembly.set_beam_attribute(beam_id, 'assembly_vector_jawapproach', assembly_vector_jawapproach)

        # Compute 'ssembly_wcf_inclampapproach' from reversing 'assembly_vector_jawapproach'
        assembly_wcf_inclamp = self.assembly.get_beam_attribute(beam_id, 'assembly_wcf_inclamp')
        T = Translation.from_vector(assembly_vector_jawapproach.scaled(-1))
        assembly_wcf_inclampapproach = assembly_wcf_inclamp.transformed(T)
        self.assembly.set_beam_attribute(beam_id, 'assembly_wcf_inclampapproach', assembly_wcf_inclampapproach)

        return assembly_vector_jawapproach

    def search_valid_jawapproach_vector_prioritizing_beam_side(self, beam_id, verbose=False):
        # type: (str, bool) -> Vector
        """Search for the `assembly_vector_jawapproach` and `assembly_wcf_inclampapproach`.
        1st priority is to follow direction of engaging-joints reference side Y direction.
        2nd priority is to use a vector in the middle of the feisible non-blocking region from the clamp jaw.
        Returns None if no valid vector can be found or no clamps are attached.

        State Change
        ------------
        This functions sets the following beam_attribute
        - 'assembly_vector_jawapproach'
        - 'assembly_wcf_inclampapproach'

        Return
        ------
        `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied or if valid vector cannot be found.
        `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)
        """
        if self.assembly.get_assembly_method(beam_id) != BeamAssemblyMethod.CLAMPED:
            return ComputationalResult.ValidNoChange

        # Exit if no clamp is needed to assemble this beam.
        joint_ids = list(self.assembly.get_joint_ids_with_tools_for_beam(beam_id))
        if len(joint_ids) == 0:
            return ComputationalResult.ValidCanContinue

        # Check to ensure prerequisite
        for joint_id in joint_ids:
            if self.assembly.get_joint_attribute(joint_id, 'clamp_wcf_final') is None:
                return ComputationalResult.ValidCannotContinue

        # Compute vector with different strategies.
        # Each strategy function must return None if no valid solution is found.
        assembly_vector_jawapproach = self.compute_jawapproach_vector_from_joints_y(beam_id)
        if assembly_vector_jawapproach is None:
            assembly_vector_jawapproach = self.compute_jawapproach_vector_from_clamp_jaw_non_blocking_region(beam_id, verbose=verbose)
        if assembly_vector_jawapproach is None:
            return ComputationalResult.ValidCannotContinue

        # Save result in beam_attribute
        self.assembly.set_beam_attribute(beam_id, 'assembly_vector_jawapproach', assembly_vector_jawapproach)

        # Compute 'ssembly_wcf_inclampapproach' from reversing 'assembly_vector_jawapproach'
        assembly_wcf_inclamp = self.assembly.get_beam_attribute(beam_id, 'assembly_wcf_inclamp')
        T = Translation.from_vector(assembly_vector_jawapproach.scaled(-1))
        assembly_wcf_inclampapproach = assembly_wcf_inclamp.transformed(T)
        self.assembly.set_beam_attribute(beam_id, 'assembly_wcf_inclampapproach', assembly_wcf_inclampapproach)

        return ComputationalResult.ValidCanContinue

    # ---------------------------------------------------------------
    # Getting Tools (or their frames) positioned at specific location
    # ---------------------------------------------------------------

    def get_gripper_baseframe_for_beam_at(self, beam_id, attribute_name):
        """ Returns the base frame (base frame) of the gripper (in WCF)
        when the beam is at different key position.

        beam_attribute (`gripper_id` or `gripper_type` )
        and the specified `assembly_wcf_*` must be set before calling this function.

        Note
        ----
        - This is not the robot flange, we still have the toolchanger

        - The result can be used directly for gripper.current_frame = get_gripper_baseframe_for_beam_at()
        """
        # Get Gripper Object
        gripper = self.get_gripper_of_beam(beam_id, attribute_name)

        return gripper.current_frame

    def get_gripper_t0cp_for_beam_at(self, beam_id, attribute_name):
        """ Returns the t0cp (flange frame) of the robot (in WCF)
        when the beam is attached to a gripper at the specified key position.
        Taking into account of the tool changer.

        Actually it should be called T0CF (Tool0 coordinate frame). Tool0 means no tool.

        process.robot_toolchanger, gripper must be set before calling this.

        Note
        ----
        - This is the robot flange.
        - The result can be used directly for path planning.
        """
        gripper_frame_in_wcf = self.get_gripper_baseframe_for_beam_at(beam_id, attribute_name)
        tool_changer = self.robot_toolchanger
        tool_changer.set_current_frame_from_tcp(gripper_frame_in_wcf)

        return tool_changer.current_frame.copy()

    def get_beam_frame_from_gripper(self, beam_id, gripper):
        # type: (str, Gripper) -> Frame
        """ Returns the beam frame (in WCF) from the position of a gripper.
        This is effectively the final step of the forward kinematics

        The grasp is retrived from beam attribute `gripper_tcp_in_ocf`
        The gripper.current_frame should be set to the intended location

        """
        # Gripper TCP expressed in world's coordinate
        world_from_gripper_tcp = Transformation.from_frame(gripper.current_tcf)

        # Beam frame expressed in the world's coordinate
        world_from_beam = world_from_gripper_tcp * self.assembly.get_t_gripper_tcf_from_beam(beam_id)
        return Frame.from_transformation(world_from_beam)

    def get_tool_features_on_beam(self, beam_id):
        # type: (str) -> List[Shape]
        assembly = self.assembly
        # * Retrieve Clamps and Screwdrivers attached to this beam
        other_feature_shapes = []

        assembly_method = self.assembly.get_assembly_method(beam_id)

        def draw_drill_cylinders_of_tool_at_wcf(tool):
            cylinders = []
            t_world_tool_at_final = Transformation.from_frame(tool.current_frame)
            for line in tool.gripper_drill_lines:
                cylinder = Cylinder.from_line_radius(line, tool.gripper_drill_diameter/2.0)
                cylinders.append(cylinder.transformed(t_world_tool_at_final))
            return cylinders

        # * Gripper (except if it is manually assembled)
        if assembly_method != BeamAssemblyMethod.MANUAL_ASSEMBLY:
            if self.assembly.get_beam_attribute(beam_id, "gripper_tcp_in_ocf") is None:
                print("Warning: gripper_tcp_in_ocf is None while calling process.get_tool_features_on_beam(%s)" % (beam_id))
            else:
                gripper = self.get_gripper_of_beam(beam_id)
                other_feature_shapes += draw_drill_cylinders_of_tool_at_wcf(gripper)

        # * Clamps Attached to this beam - Need to check neighbour's assembly method
        for neighbour_id in assembly.get_unbuilt_neighbors(beam_id):
            if assembly.get_assembly_method(neighbour_id) == BeamAssemblyMethod.CLAMPED:
                clamp = self.get_tool_of_joint((beam_id, neighbour_id))  # type: Clamp
                if clamp is not None:
                    other_feature_shapes += draw_drill_cylinders_of_tool_at_wcf(clamp)

        # * Screwdrivers Attached to this beam
        if assembly_method in BeamAssemblyMethod.screw_methods:
            for neighbour_id in assembly.get_already_built_neighbors(beam_id):
                screwdriver = self.get_tool_of_joint((neighbour_id, beam_id))
                if screwdriver is not None:
                    other_feature_shapes += draw_drill_cylinders_of_tool_at_wcf(screwdriver)
        return other_feature_shapes

    # -----------------
    # Clamps Algorithms
    # -----------------

    def get_clamp_orientation_options(self, beam_id):
        """Each beam assembly may have multiple clamps involved
        Each clamp may have multiple orientation for attaching to the structure.
        This function returns the orientation possibility for each joint

        returns {'joint_id' : [attachment_frame])}]

        """
        joints_with_clamps_id = self.assembly.get_joint_ids_with_tools_for_beam(beam_id)

        results = {}
        for joint_id in joints_with_clamps_id:
            joint = self.assembly.joint(joint_id)                   # type: Joint
            neighbour_beam = self.assembly.beam(joint_id[0])
            clamp_attachment_frames = joint.get_clamp_frames(neighbour_beam)  # type: Frame
            results[joint_id] = clamp_attachment_frames
        return results

    def flip_clamp_guide_vector(self, beam_id):
        """ Flip the direction of design_guide_vector_jawapproach.

        The resulting design_guide_vector_jawapproach is either
        the +ve or -ve of beam(beam_id).frame.xaxis

        Using assembly.beam(beam_id).frame.xaxis is not perfect, future
        TODO should change this to align with Beam Face Y Axis.
        """

        guide_vector = self.assembly.get_beam_attribute(beam_id, 'design_guide_vector_jawapproach')  # type: Vector
        print("Previous guide_vector: ", guide_vector)
        #  Check if the current guide vector is pointing to the same way as the beam X
        beam_xaxis = self.assembly.beam(beam_id).frame.xaxis
        if dot_vectors(guide_vector, beam_xaxis) >= 0:
            self.assembly.set_beam_attribute(beam_id, 'design_guide_vector_jawapproach', beam_xaxis.scaled(-1))
        else:
            self.assembly.set_beam_attribute(beam_id, 'design_guide_vector_jawapproach', beam_xaxis.scaled(1))
        print("New guide_vector: ", self.assembly.get_beam_attribute(beam_id, 'design_guide_vector_jawapproach'))

    def search_valid_clamp_orientation_with_guiding_vector(self, beam_id, verbose=False):
        # type: (str, bool) -> ComputationalResult
        """ Search and choose clamp attachment frame based on guide vector alignment

        This is applied to all the joints on beam(beam_id) that needs to be clamped.
        where joint(neighbour_id, beam_id)

        Joint attribute 'design_guide_vector_jawapproach' should be set before hand.
        The -Y Axis Vector of the clamp will attempt to align with this vector.
        The vector that is used to place a beam into the jaw.

        State Change
        ------------
        This functions sets the joint attribute `clamp_wcf_final`

        Return
        ------
        `ComputationalResult.ValidCanContinue`
        """
        if self.assembly.get_assembly_method(beam_id) != BeamAssemblyMethod.CLAMPED:
            return ComputationalResult.ValidNoChange

        guiding_vector = self.assembly.get_beam_attribute(beam_id, 'design_guide_vector_jawapproach')

        # A Helper function to select best frame.
        def choose_frame_by_guide_vector(frames, guide_vector):
            """ Finds the frame that has the best aligned X axis to the given guide_vector """
            results = []
            for frame in frames:
                results.append((dot_vectors(guiding_vector, frame.yaxis.scaled(-1.0)), frame))
            guide_score, best_frame = sorted(results, key=lambda x: x[0])[-1]  # Lst item of the sorted list has the best
            return best_frame

        # Loop through all the clamp_orientation_options
        # chosen_frames = []
        for joint_id, attachment_frames in self.get_clamp_orientation_options(beam_id).items():
            selected_frame = choose_frame_by_guide_vector(attachment_frames, guiding_vector)

            # Set clamp tcp to selected_frame using set_current_frame_from_tcp()
            clamp = self.get_tool_of_joint(joint_id, None)
            if clamp is None:
                continue
            clamp.set_current_frame_from_tcp(selected_frame)

            # Save clamp.current_frame as 'clamp_wcf_final'
            self.assembly.set_joint_attribute(joint_id, 'clamp_wcf_final', clamp.current_frame)
            # chosen_frames.append(selected_frame.copy())
            #print ("Beam (%s) Joint (%s), we need clamp type (%s) at %s" % (beam_id, joint_id, self.get_tool_type_of_joint(joint_id), selected_frame))
        return ComputationalResult.ValidCanContinue

    def compute_clamp_attachapproach_attachretract_detachapproach(self, beam_id, verbose=False):
        """ Compute and set 'clamp_wcf_attachapproach1', 'clamp_wcf_attachapproach2',
        'clamp_wcf_attachretract' and `clamp_wcf_detachapproach'
        from 'clamp_wcf_final' and clamp intrinsic properties.

        Side Effect
        -----------
        One of the process.clamps instance clamp.current_frame is modified.

        State Change
        ------------
        This functions sets the following joint_attribute
        - 'clamp_wcf_attachapproach1'
        - 'clamp_wcf_attachapproach2'
        - 'clamp_wcf_attachretract'
        - 'clamp_wcf_detachapproach'

        Return
        ------
        `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied
        `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)
        """
        if self.assembly.get_assembly_method(beam_id) != BeamAssemblyMethod.CLAMPED:
            return ComputationalResult.ValidNoChange

        joint_ids = self.assembly.get_joint_ids_with_tools_for_beam(beam_id)
        if verbose:
            print("Beam (%s)" % beam_id)
        # Check to ensure prerequisite
        for joint_id in joint_ids:
            if self.assembly.get_joint_attribute(joint_id, 'clamp_wcf_final') is None:
                return ComputationalResult.ValidCannotContinue
        for joint_id in joint_ids:
            if verbose:
                print("|- Clamp at Joint (%s-%s)" % joint_id)
            clamp = self.get_tool_of_joint(joint_id, 'clamp_wcf_final')

            # clamp_wcf_attachapproach is based on moveing clamp_wcf_final backwards along clamp.approach_vector
            # ------------------------------------------------------------
            # Compute the approach vector in wcf
            # approach_vector_wcf = clamp.current_frame.to_world_coordinates(clamp.approach_vector)
            approach1_vector_wcf = clamp.current_frame.to_world_coordinates(clamp.approach1_vector)
            approach2_vector_wcf = clamp.current_frame.to_world_coordinates(clamp.approach2_vector)

            # compute approach frame
            # clamp_wcf_attachapproach = clamp.current_frame.transformed(Translation.from_vector(approach_vector_wcf.scaled(-1)))
            clamp_wcf_attachapproach2 = clamp.current_frame.transformed(Translation.from_vector(approach2_vector_wcf.scaled(-1)))
            clamp_wcf_attachapproach1 = clamp_wcf_attachapproach2.transformed(Translation.from_vector(approach1_vector_wcf.scaled(-1)))

            self.assembly.set_joint_attribute(joint_id, 'clamp_wcf_attachapproach1', clamp_wcf_attachapproach1)
            self.assembly.set_joint_attribute(joint_id, 'clamp_wcf_attachapproach2', clamp_wcf_attachapproach2)

            # clamp_wcf_attachretract is based on tool.tool_pick_up_frame transformed to wcf
            # ------------------------------------------------------------
            clamp_wcf_attachretract = clamp.tool_pick_up_frame_in_wcf(clamp.current_frame)
            self.assembly.set_joint_attribute(joint_id, 'clamp_wcf_attachretract', clamp_wcf_attachretract)

            # clamp_wcf_detachapproach is based on tool.tool_pick_up_frame transformed to wcf
            # ------------------------------------------------------------
            clamp_wcf_detachapproach = clamp_wcf_attachretract.copy()
            self.assembly.set_joint_attribute(joint_id, 'clamp_wcf_detachapproach', clamp_wcf_detachapproach)

            if verbose:
                print("|  |- clamp_wcf_attachapproach1 = %s" % clamp_wcf_attachapproach1)
                print("|  |- clamp_wcf_attachapproach2 = %s" % clamp_wcf_attachapproach2)
            if verbose:
                print("|  |- clamp_wcf_final = %s" % clamp.current_frame)
            if verbose:
                print("|  |- clamp_wcf_attachretract = %s" % clamp_wcf_attachretract)

        return ComputationalResult.ValidCanContinue

    def compute_clamp_detachretract(self, beam_id, verbose=False):
        """ Compute and set `clamp_wcf_detachretract1` and `clamp_wcf_detachretract2`
        from 'clamp_wcf_final' and clamp intrinsic properties.

        Side Effect
        -----------
        One of the process.clamps instance clamp.current_frame is modified.

        State Change
        ------------
        This functions sets the joint attribute `clamp_wcf_detachretract1`,
        `clamp_wcf_detachretract2`

        Return
        ------
        `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied
        `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

        """
        if self.assembly.get_assembly_method(beam_id) != BeamAssemblyMethod.CLAMPED:
            return ComputationalResult.ValidNoChange

        joint_ids = self.assembly.get_joint_ids_with_tools_for_beam(beam_id)
        if verbose:
            print("Beam (%s)" % beam_id)
        # Check to ensure prerequisite
        for joint_id in joint_ids:
            if self.assembly.get_joint_attribute(joint_id, 'clamp_wcf_final') is None:
                return ComputationalResult.ValidCannotContinue
        for joint_id in joint_ids:
            if verbose:
                print("|- Clamp at Joint (%s-%s)" % joint_id)
            clamp = self.get_tool_of_joint(joint_id, 'clamp_wcf_final')

            # Moving clamp_wcf_final backwards along clamp.detachretract1_vector
            # ------------------------------------------------------------
            # Compute the approach vector in wcf
            detachretract1_vector_wcf = clamp.current_frame.to_world_coordinates(clamp.detachretract1_vector)
            detachretract2_vector_wcf = clamp.current_frame.to_world_coordinates(clamp.detachretract2_vector)

            # compute detachretract frame by translating along retract vectors
            clamp_wcf_detachretract1 = clamp.current_frame.transformed(Translation.from_vector(detachretract1_vector_wcf))
            clamp_wcf_detachretract2 = clamp_wcf_detachretract1.transformed(Translation.from_vector(detachretract2_vector_wcf))

            self.assembly.set_joint_attribute(joint_id, 'clamp_wcf_detachretract1', clamp_wcf_detachretract1)
            self.assembly.set_joint_attribute(joint_id, 'clamp_wcf_detachretract2', clamp_wcf_detachretract2)

            if verbose:
                print("|  |- clamp_wcf_detachretract1 = %s" % clamp_wcf_detachretract1)
            if verbose:
                print("|  |- clamp_wcf_detachretract2 = %s" % clamp_wcf_detachretract2)
        return ComputationalResult.ValidCanContinue

    # -----------------
    # Environment Model
    # -----------------

    def get_new_environment_model_id(self):
        # type: () -> str
        """ Returns the next available env_id in the format of 'e%i'.
        Starting from 'e0' to maximum 'e999', it returns the next unused env_id
        If there are holes between the used_id, the hole will be returned.
        """
        for i in range(1000):
            env_id = 'e%i' % i
            if env_id not in self.environment_models:
                return env_id

    def add_environment_model(self, env_model, env_id=None):
        # type: (EnvironmentModel, str) -> str
        """ Adds an environment model to the process
        env_id is automatically assigned if left None.

        env_id is returned.
        """
        if env_id is None:
            env_id = self.get_new_environment_model_id()
        # Assign id as name. Save it to dictionary.
        env_model.name = env_id
        self.environment_models[env_id] = env_model
        return env_id

    # -------------------------
    # Action Movement and Scene
    # -------------------------

    def get_actions_by_beam_id(self, beam_id):
        # type: (str) -> list[Action]
        """ Get an ordered list of Action related to a beam"""

        return self.assembly.get_beam_attribute(beam_id, 'actions')

    def get_movements_by_beam_id(self, beam_id):
        # type: (str) -> list[Movement]
        """ Get an ordered list of Movements related to a beam"""
        return [movement for action in self.get_actions_by_beam_id(beam_id) for movement in action.movements]

    def get_movement_summary_by_beam_id(self, beam_id):
        movements = self.get_movements_by_beam_id(beam_id)
        print('=====')
        print('Summary:')
        for i, m in enumerate(movements):
            start_state = self.get_movement_start_scene(m)
            end_state = self.get_movement_end_scene(m)
            has_start_frame = start_state[('robot', 'f')] is not None
            has_end_frame = end_state[('robot', 'f')] is not None
            print('---')
            print('({}) {} \npriority {} | has start conf {}, TCP {} | has end conf {}, TCP {}{}'.format(
                i, _colored_movement_short_summary(m), _colored_planning_priority(m.planning_priority),
                _colored_is_none(self.movement_has_start_robot_config(m)), _colored_is_none(has_start_frame),
                _colored_is_none(self.movement_has_end_robot_config(m)), _colored_is_none(has_end_frame),
                ' | has traj ' + _colored_is_none(m.trajectory) if isinstance(m, RoboticMovement) else ''
            ))

    def get_movement_by_movement_id(self, movement_id):
        # type: (str) -> Movement
        for movement in self.movements:
            if movement.movement_id == movement_id:
                return movement
        else:
            raise ValueError('No movement with id {} found!'.format(movement_id))
        # return None

    def get_movement_start_scene(self, movement):
        # type: (Movement) -> SceneState
        """ return the start state before the movment."""
        movements = self.movements
        index = movements.index(movement)
        if index == 0:
            return self.initial_state
        else:
            return self.get_movement_end_scene(movements[index-1])

    def get_movement_end_scene(self, movement):
        # type: (Movement) -> SceneState
        """

        """
        from integral_timber_joints.process.state import  SceneState
        movements = self.movements
        start_state = self.initial_state
        index = movements.index(movement)
        scene = SceneState(self)

        def find_last_diff(key):
            """Function to search backwards for the last diff related to a key"""
            for i in range(index, -1, -1):
                if key in movements[i].state_diff:
                    scene[key] = movements[i].state_diff[key]
                    return
            if key in start_state:
                scene[key] = start_state[key]
                return
            print("Warning: Cannot find diff for state: %s" % str(key))

        # Robot state is not the same as previous.
        if self.robot_config_key in movements[index].state_diff:
            scene[self.robot_config_key] = movements[index].state_diff[self.robot_config_key]
        else:
            scene[self.robot_config_key] = None

        for key in scene.keys_with_unknown_state(skip_robot_config=True):
            # print("Finding Key: %s" % str(key))
            find_last_diff(key)
        return scene

    def get_movements_by_planning_priority(self, beam_id, priority):
        # type: (str, int) -> list[Movement]
        return [m for m in self.get_movements_by_beam_id(beam_id) if m.planning_priority == priority]

    def get_action_of_movement(self, movement):
        # type: (Movement) -> Action
        """Returns the Action object in which the movement belongs to.

        The Movement object should be the original instance retrived from the process.
        a.k.a not deep copied.

        returns None if the Movement cannot be found (likely you have made a deep copy somewhere.)
        """
        for action in self.actions:
            for _movement in action.movements:
                # @YJ: Does this really work? what if I change some attributes in the movement?
                # @VL: Thiw works because you should not be making deep copies of the movements.
                if movement is _movement:
                    return action
        return None

    def get_beam_id_of_movement(self, movement):
        # (Movement) -> str
        action = self.get_action_of_movement(movement)
        return self.assembly.sequence[action.seq_n]

    def get_beam_id_from_movement_id(self, movement_id):
        # (str) -> str
        movement = self.get_movement_by_movement_id(movement_id)
        return self.get_beam_id_of_movement(movement)

    # --------------------
    # Robot Configurations
    # --------------------

    def set_initial_state_robot_config(self, robot_configuration):
        # type: (Configuration) -> None
        """Changes the default value of the initial state robot config."""
        self.initial_state[self.robot_config_key] = robot_configuration

    def get_movement_start_robot_config(self, movement):
        # type: (Movement) -> Optional[Configuration]
        """Get the robot configuration at the begining of a Movement.
        None if the configuration is undefined.
        """
        movements = self.movements
        index = movements.index(movement)
        if index == 0:
            return self.initial_state[self.robot_config_key]
        else:
            return self.get_movement_end_robot_config(movements[index - 1])

    def get_movement_end_robot_config(self, movement):
        # type: (Movement) -> Optional[Configuration]
        """Get the robot configuration at the begining of a Movement.
        None if the configuration is undefined.
        """
        if self.robot_config_key in movement.state_diff:
            return movement.state_diff[self.robot_config_key]
        else:
            return None

    def set_movement_start_robot_config(self, movement, robot_configuration):
        # type: (Movement, Configuration) -> None
        """Sets or changes the robot configuration at the begining of a Movement.
        If attempt to set the start of the first movement, the initial state will be changed.

        Setting the configuration to None will remove the configuration.

        Note that the adjoining start and end configuration of two Movements are pointed to the same object.
        Setting one of them will automatically change the other.
        """
        movements = self.movements
        index = movements.index(movement)
        if index == 0:
            self.initial_state[self.robot_config_key] = robot_configuration
        else:
            self.set_movement_end_robot_config(movements[index - 1], robot_configuration)

    def set_movement_end_robot_config(self, movement, robot_configuration):
        # type: (Movement, Configuration) -> None
        """Sets or changes the robot configuration at the end of a Movement.

        Setting the configuration to None will remove the configuration.

        Note that the adjoining start and end configuration of two Movements are pointed to the same object.
        Setting one of them will automatically change the other.
        """
        if robot_configuration is not None:
            movement.state_diff[self.robot_config_key] = robot_configuration
        else:
            if self.robot_config_key in movement.state_diff:
                del movement.state_diff[self.robot_config_key]

    def movement_has_start_robot_config(self, movement):
        # type: (Movement) -> bool
        """Returns True if the movement's start scene has a defined robot configuration."""
        movements = self.movements
        index = movements.index(movement)
        if index == 0:
            return self.initial_state[self.robot_config_key] is not None
        else:
            return self.movement_has_end_robot_config(movements[index-1])
        return

    def movement_has_end_robot_config(self, movement):
        # type: (Movement) -> bool
        """Returns True if the movement's end scene has a defined robot configuration."""
        if self.robot_config_key in movement.state_diff:
            if movement.state_diff[self.robot_config_key] is not None:
                return True
        return False

    def get_object_from_id(self, object_id):
        if object_id.startswith('c') or object_id.startswith('g'):
            return self.tool(object_id)
        elif object_id.startswith('t'):
            return self.robot_toolchanger
        elif object_id.startswith('b'):
            return self.assembly.beam(object_id)
        elif object_id.startswith('robot'):
            raise ValueError('robot object is not stored within the Process class.')

    # ------------------------------
    # Loading Externally Saved Files
    # ------------------------------

    # TODO load specific movement_id and neighbors

    def load_external_movement(self, process_folder_path, movement, subdir='movements', verbose=False):
        # type: (str, Movement, str, bool) -> list[Movement]
        """Load one external Movement from nearby folder if they exist,
        replace the movement in the process with the new movement.

        Returns the new movement if loaded successfully, otherwise None if file does not exist.
        """
        import json
        import os
        from compas.utilities import DataDecoder

        movement_path = os.path.join(process_folder_path, movement.get_filepath(subdir=subdir))
        if os.path.exists(movement_path):
            if verbose:
                print("Loading External Movement File: movement_path%s" % movement_path)
            with open(movement_path, 'r') as f:
                try:
                    movement.data = json.load(f, cls=DataDecoder).data
                except Exception as e:
                    print('Error loading Movement from: %s' % movement_path)
                    raise
                    # import traceback
                    # tb_str = ''.join(traceback.format_exception(None, e, e.__traceback__))
                    # print(tb_str)
            return movement
        else:
            return None

    def load_external_movements(self, process_folder_path, movement_id=None, verbose=False):
        # type: (str, str, bool) -> list[Movement]
        """Load External Movements from nearby folder if they exist, replace the movements
        with new movements, returns the list of movements modified.
        If movement_id is None, all movements will be parsed. Otherwise only the given movement
        and its neighbors will be parsed."""

        if movement_id:
            target_movement = self.get_movement_by_movement_id(movement_id)
            target_movements = [target_movement]
            m_id = self.movements.index(target_movement)
            if m_id-1 >= 0:
                target_movements.append(self.movements[m_id-1])
            if m_id+1 < len(self.movements):
                target_movements.append(self.movements[m_id+1])
            # print(target_movements)
        else:
            target_movements = self.movements

        movements_modified = []
        for movement in target_movements:
            new_movement = self.load_external_movement(process_folder_path, movement, verbose=verbose)
            if new_movement is not None:
                movements_modified.append(new_movement)
        return movements_modified

    def load_robot_model(self):
        # type: () -> None
        """Load Robot model from URDF at default location
        under the submodule in integral_timber_joint

        The RobotModel is stored in process.robot_model()
        This RobotModel is scaled by 1000 to be used with mm scale.

        For example
        -----------
        ```
        robot_frame = process.robot_model.forward_kinematics(process.robot_initial_config.scaled(1000), process.ROBOT_END_LINK)
        ```

        """

        import os

        from compas.rpc import Proxy

        # Workaround for fact that itj module is installed with a windows shortcut inside Rhino Plugin folder.
        # Direct retrival of itj module.__file__ will only result in the Rhino Plugin folder
        # not the installation folder.

        # Side effect of this workwround is that a Proxy server is started.

        if compas.is_rhino():
            info = Proxy('integral_timber_joints.module_info')
            submodule_path = info.itj_submodule_folder()
        else:
            import integral_timber_joints.module_info as info
            submodule_path = info.itj_submodule_folder()

        # Load files
        if os.path.exists(os.path.join(submodule_path, 'rfl_description')):
            print("Loading RobotModel from URDF at: %s" % submodule_path)

            from compas.robots.model import RobotModel
            from compas.robots.resources.basic import LocalPackageMeshLoader

            # Example code from https://compas.dev/compas/latest/api/generated/compas.robots.RobotModel.load_geometry.html
            loader = LocalPackageMeshLoader(os.path.join(submodule_path, 'rfl_description'), 'rfl_description')
            urdf = loader.load_urdf('rfl_pybullet.urdf')
            robot_model = RobotModel.from_urdf_file(urdf)
            robot_model.load_geometry(loader)
            robot_model.scale(1000)
            self._robot_model = robot_model  # type: RobotModel
            self._robot_model_load_success = True
            print("- %s Loaded: %i Links, %i Joints" % (robot_model.name, len(robot_model.links), len(robot_model.joints)))
        else:
            # You should have this repo https://github.com/yijiangh/rfl_description/tree/victor/cables as submodule
            print("RobotModel submodule does not exist in: %s" % submodule_path)
            self._robot_model_load_success = False

    def to_symbolic_problem_data(self):
        """Return a dict that contains only the data used for symbolic planning.
        Used for easier data communication without installing ITJ.
        """
        data = {'assembly' : {}}
        data['assembly']['sequence'] = []
        for b in self.assembly.sequence:
            beam_gripper_type = self.assembly.get_beam_attribute(b, "gripper_type")
            b_data = {'beam_id' : b, 'beam_gripper_type' : beam_gripper_type}
            b_data['grounded'] = False
            if self.assembly.get_assembly_method(b) == BeamAssemblyMethod.GROUND_CONTACT:
                b_data['grounded'] = True
            data['assembly']['sequence'].append(b_data)
        data['assembly']['joints'] = []
        for j in self.assembly.joint_ids():
            data['assembly']['joints'].append({'joint_id' : j, 'tool_type' : self.assembly.get_joint_attribute(j, 'tool_type')})
        data['clamps'] = {}
        for c in self.clamps:
            data['clamps'][c.name] = {'type_name' : c.type_name}
        data['screwdrivers'] = {}
        for sd in self.screwdrivers:
            data['screwdrivers'][sd.name] = {'type_name' : sd.type_name}
        data['grippers'] = {}
        for g in self.grippers:
            data['grippers'][g.name] = {'type_name' : g.type_name}
        return data


def _colored_is_none(value):
    try:
        if value is None:
            return colored('None', 'red')
        else:
            return colored(value, 'green' if value else 'red')
    except:
        return value


def _colored_planning_priority(p):
    color_from_p = {1: 'blue', 0: 'magenta', -1: 'white'}
    try:
        return colored(p, color_from_p[p])
    except:
        return p


def _colored_movement_short_summary(m):
    try:
        if isinstance(m, RoboticClampSyncLinearMovement) or isinstance(m, RobotScrewdriverSyncLinearMovement):
            return colored(m.short_summary, 'red', 'on_yellow')
        elif isinstance(m, RoboticLinearMovement):
            return colored(m.short_summary, 'white', 'on_blue')
        elif isinstance(m, RoboticFreeMovement):
            return colored(m.short_summary, 'yellow', 'on_cyan')
        else:
            return m.short_summary
    except:
        print('exception')
        return m.short_summary
