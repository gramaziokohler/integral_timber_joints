from copy import deepcopy

from compas.datastructures import Mesh, Network
from compas.geometry import Transformation, Translation
from compas.geometry._core._algebra import dot_vectors
from compas.geometry.primitives.frame import Frame
from compas.geometry.primitives.vector import Vector
from compas.robots.model.robot import RobotModel
from compas.rpc import Proxy
from compas_fab.robots.configuration import Configuration

from geometric_blocking import blocked
from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry import Beam, EnvironmentModel, Joint
from integral_timber_joints.process.action import Action
from integral_timber_joints.process.dependency import ComputationalDependency, ComputationalResult
from integral_timber_joints.process.movement import Movement, RoboticMovement
from integral_timber_joints.process.state import ObjectState, copy_state_dict
from integral_timber_joints.tools.beam_storage import BeamStorage
from integral_timber_joints.tools.clamp import Clamp
from integral_timber_joints.tools.gripper import Gripper
from integral_timber_joints.tools.pickup_station import GripperAlignedPickupStation, PickupStation, StackedPickupStation
from integral_timber_joints.tools.robot_wrist import RobotWrist
from integral_timber_joints.tools.tool import Tool
from integral_timber_joints.tools.tool_changer import ToolChanger

try:
    from termcolor import colored, cprint
except:
    pass


class RobotClampAssemblyProcess(Network):

    from .algorithms import (assign_tools_to_actions, create_actions_from_sequence, create_movements_from_actions, debug_print_process_actions_movements,
                             optimize_actions_place_pick_clamp, optimize_actions_place_pick_gripper)

    # Constants for clamp jaw positions at different key positions.
    clamp_appraoch_position = 220
    clamp_inclamp_position = 210
    clamp_final_position = 100

    def __init__(self, assembly=None):
        # type: (Assembly) -> None

        super(RobotClampAssemblyProcess, self).__init__()

        if assembly is not None:
            assembly = assembly.copy()     # type: Assembly
        self.attributes['assembly'] = assembly
        self.attributes['clamps'] = {}
        self.attributes['grippers'] = {}

        self.attributes['robot_model'] = None                         # RobotModel
        self.attributes['robot_toolchanger'] = None             # ToolChanger
        self.attributes['robot_wrist'] = None                   # RobotWrist
        self.attributes['robot_initial_config'] = None          # tuple(list(float), flaot)
        self.attributes['actions'] = []                         # list[Action]
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

    @property
    def robot_model(self):
        # type: () -> RobotModel
        return self.attributes['robot_model']

    @robot_model.setter
    def robot_model(self, value):
        # type: (RobotModel) -> None
        self.attributes['robot_model'] = value

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
        return self.attributes['robot_initial_config']

    @robot_initial_config.setter
    def robot_initial_config(self, value):
        # type: (Configuration) -> None
        self.attributes['robot_initial_config'] = value

    @property
    def actions(self):
        # type: () -> list[Action]
        return self.attributes['actions']

    @actions.setter
    def actions(self, value):
        # type: (list[Action]) -> None
        self.attributes['actions'] = value

    @property
    def movements(self):
        # type: () -> list[Movement]
        all_movements = []
        for action in self.actions:
            for movement in action.movements:
                all_movements.append(movement)
        return all_movements

    @property
    def states(self):
        # type: () -> list[dict[str, ObjectState]]
        all_states = [self.initial_state]
        for action in self.actions:
            for movement in action.movements:
                all_states.append(movement.end_state)
        return all_states

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
        # type: () -> dict[str, ObjectState]
        return self.attributes['initial_state']

    @initial_state.setter
    def initial_state(self, value):
        # type: (dict(str, ObjectState)) -> None
        self.attributes['initial_state'] = value

    @property
    def intermediate_states(self):
        # type: () -> list[dict[str, ObjectState]]
        return [movement.end_state for movement in self.movements]

    # @intermediate_states.setter
    # def intermediate_states(self, value):
    #     # type: (list(dict(str, ObjectState))) -> None
    #     self.attributes['intermediate_states'] = value
    # -----------------------
    # Properity access
    # -----------------------

    @property
    def assembly(self):
        # type: () -> Assembly
        return self.attributes['assembly']

    def add_clamp(self, clamp):
        self.attributes['clamps'][clamp.name] = clamp.copy()

    def add_gripper(self, gripper):
        self.attributes['grippers'][gripper.name] = gripper.copy()

    def delete_clamp(self, clamp_id):
        if clamp_id in self.attributes['clamps']:
            del self.attributes['clamps'][clamp_id]

    def delete_gripper(self, gripper_id):
        if gripper_id in self.attributes['grippers']:
            del self.attributes['grippers'][gripper_id]

    def clamp(self, clamp_id):
        # type: (str) -> Clamp
        return self.attributes['clamps'][clamp_id]

    @property
    def clamps(self):
        # type: () -> list[Clamp]
        return self.attributes['clamps'].values()

    def gripper(self, gripper_id):
        # type: (str) -> Clamp
        return self.attributes['grippers'][gripper_id]

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
        else:
            raise KeyError("tool_id ({}) cannot be found in process.grippers or process.clamps".format(tool_id))

    @property
    def tools(self):
        # type: () -> list(Tool)
        """Return a list of all grippers and clamps."""
        return list(self.clamps) + list(self.grippers)

    @property
    def tool_ids(self):
        # type: () -> list(str)
        """Return all gripper and clamp ids."""
        return list(self.attributes['grippers'].keys()) + list(self.attributes['clamps'].keys())

    def environment_model(self, env_id):
        # type: (str) -> EnvironmentModel
        return self.environment_models[env_id]

    @property
    def available_clamp_types(self):
        return set([clamp.type_name for clamp in self.clamps])

    @property
    def available_gripper_types(self):
        return set([gripper.type_name for gripper in self.grippers])

    # -----------------------
    # Clamp
    # -----------------------

    def get_one_clamp_by_type(self, type_name):
        # type: (str) -> Clamp
        """Get one of the clamp that belongs to the given type"""
        for clamp in self.clamps:
            if (clamp.type_name == type_name):
                return clamp
        return None

    def get_clamp_type_of_joint(self, joint_id):
        # type: (tuple[str, str]) -> str
        """Returns the clamp_type used at the joint. None, if the joint do not need a clamp"""
        if self.assembly.get_joint_attribute(joint_id, 'clamp_used'):
            return self.assembly.get_joint_attribute(joint_id, 'clamp_type')
        else:
            return None

    def get_clamp_of_joint(self, joint_id, position_name='clamp_wcf_final'):
        # type: (tuple[str, str], bool) -> Clamp
        """Returns one of the clamp object being set at the position
        specified by 'position_name', default is 'clamp_wcf_final'.
        Refering to the attached position on the beam.

        If 'position_name' = '', clamp frame will not be modified.

        Warning: This clamp object is not deep-copied.
        Modifying it will change its definition in the process.
        """
        clamp_type = self.get_clamp_type_of_joint(joint_id)
        clamp = self.get_one_clamp_by_type(clamp_type)

        if position_name != "":
            clamp_wcf = self.assembly.get_joint_attribute(joint_id, position_name)
            assert clamp_wcf is not None
            clamp.current_frame = clamp_wcf
        return clamp

    def get_clamp_t0cf_at(self, joint_id, position_name):
        # type: (tuple[str, str], bool) -> Clamp
        """ Returns the t0cf (flange frame) of the robot (in WCF)
        when the clamp is at the specified key position.
        Taking into account of the tool changer.

        Actually it should be called T0CF (Tool0 coordinate frame). Tool0 means no tool.

        process.robot_toolchanger, gripper must be set before calling this.

        Note
        ----
        - This is the robot flange.
        - The result can be used directly for path planning.
        """
        world_from_clampbase = self.assembly.get_joint_attribute(joint_id, position_name)
        tool_changer = self.robot_toolchanger
        tool_changer.set_current_frame_from_tcp(world_from_clampbase)

        return tool_changer.current_frame.copy()

    def get_clamp_ids_for_beam(self, beam_id):
        # type: (str) -> tuple[str, str]
        for neighbour_id in self.assembly.get_already_built_neighbors(beam_id):
            joint_id = (neighbour_id, beam_id)
            if self.assembly.get_joint_attribute(joint_id, 'clamp_used'):
                yield joint_id

    def compute_jawapproach_vector_length(self, beam_id, vector_dir, min_dist=20, max_dist=150):
        # type: (str, Vector, float, float) -> Vector
        """Return a `assembly_vector_jawapproach` with correct length that will clear the clamp jaws
        Current implementation do not tkae into account of the angle of the clamp.
        Need to be improved when angled lap joints exist.
        """
        # Compute the length of the movement to clear the clamp jaw
        clearance_length = min_dist
        for joint_id in self.get_clamp_ids_for_beam(beam_id):
            clamp = self.get_clamp_of_joint(joint_id, '')
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
        for joint_id in self.get_clamp_ids_for_beam(beam_id):
            clamp = self.get_clamp_of_joint(joint_id, 'clamp_wcf_final')
            if blocked(clamp.jaw_blocking_vectors_in_wcf, design_guide_vector_jawapproach.scaled(-1.0)):
                return None

        # Compute the vector length to clear jaw, and return the `assembly_vector_jawapproach`
        return self.compute_jawapproach_vector_length(beam_id, design_guide_vector_jawapproach)

    def compute_jawapproach_vector_from_clamp_jaw_non_blocking_region(self, beam_id):
        # type: (str) -> Vector
        """Return a `assembly_vector_jawapproach` that follows the direction of `design_guide_vector_jawapproach`
        Returns None if the clamps do not present a feasible region.
        """

        # Package to compute Feasible Region
        geometric_blocking = Proxy(package='geometric_blocking')

        # Collect blocking vectors from attached clamps
        blocking_vectors = []
        for joint_id in self.get_clamp_ids_for_beam(beam_id):
            clamp = self.get_clamp_of_joint(joint_id, 'clamp_wcf_final')
            blocking_vectors += clamp.jaw_blocking_vectors_in_wcf
        # Fix rounding error for blocking vectors (Some -0.0 causes problems)
        ROUNDING_DIGITS = 10
        blocking_vectors = [Vector(round(x, ROUNDING_DIGITS), round(y, ROUNDING_DIGITS), round(z, ROUNDING_DIGITS)) for x, y, z in blocking_vectors]

        # Compute the analytical result of the feasible region and take a average from the resulting region
        feasible_disassem_rays, lin_set = geometric_blocking.compute_feasible_region_from_block_dir(blocking_vectors)
        if len(feasible_disassem_rays) == 0:
            return None
        # # Remove the feasible_rays that are linear (bi-directional)
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
        for joint_id in self.get_clamp_ids_for_beam(beam_id):
            clamp = self.get_clamp_of_joint(joint_id, 'clamp_wcf_final')
            blocking_vectors += clamp.jaw_blocking_vectors_in_wcf

        # Check if either direction of the vector is free
        if not blocked(blocking_vectors, face_y_axis):
            return self.compute_jawapproach_vector_length(beam_id, face_y_axis.scaled(-1.0))
        if not blocked(blocking_vectors, face_y_axis.scaled(-1.0)):
            return self.compute_jawapproach_vector_length(beam_id, face_y_axis.copy())
        return None

    def search_valid_jawapproach_vector_prioritizing_guide_vector(self, beam_id):
        # type: (str) -> Vector
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
        if len(list(self.get_clamp_ids_for_beam(beam_id))) == 0:
            return None

        # Compute vector with different strategies.
        # Each strategy function must return None if no valid solution is found.
        assembly_vector_jawapproach = self.compute_jawapproach_vector_from_guide_vector_dir(beam_id)
        if assembly_vector_jawapproach is None:
            assembly_vector_jawapproach = self.compute_jawapproach_vector_from_clamp_jaw_non_blocking_region(beam_id)
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

    def search_valid_jawapproach_vector_prioritizing_beam_side(self, beam_id):
        # type: (str) -> Vector
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
        # Exit if no clamp is needed to assemble this beam.
        joint_ids = list(self.get_clamp_ids_for_beam(beam_id))
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
            assembly_vector_jawapproach = self.compute_jawapproach_vector_from_clamp_jaw_non_blocking_region(beam_id)
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

    # ----------------------------------
    # Gripper / Grip Pose Algorithms
    # ----------------------------------

    def get_one_gripper_by_type(self, type_name):
        # type: (str) -> Gripper
        """Get one of the clamp that belongs to the given type"""
        for gripper in self.grippers:
            if (gripper.type_name == type_name):
                return gripper
        return None

    def override_grasp_face(self, beam_id, grasp_face):
        """Manually override `gripper_grasp_face` for a specified beam
        `grasp_face` can only be within 1 - 4, overrange value will be wrapped

        State Change
        ------------
        This functions sets the following beam_attribute
        - 'gripper_grasp_face'

        Dependency Trigger
        ------------------
        Invalidate: 'compute_gripper_grasp_pose' and downstream

        """
        grasp_face = (grasp_face - 1) % 4 + 1
        self.assembly.set_beam_attribute(beam_id, 'gripper_grasp_face', grasp_face)
        # Dependency Trigger
        self.dependency.invalidate(beam_id, self.compute_gripper_grasp_pose)
        return True

    def search_grasp_face_from_guide_vector_dir(self, beam_id):
        # type: (str) -> Vector
        """Return the best face number (1-4) for creating `gripper_tcp_in_ocf`
        where the Z-Axis of the tcp_in_WCF, when beam is at 'assembly_wcf_final',
        follows the direction of guide vector `design_guide_vector_grasp`

        Side Effect
        -----------
        beam_attribute 'gripper_grasp_face' will be set.
        """
        # Get the guide Vector from beam_attribute
        design_guide_vector_grasp = self.assembly.get_beam_attribute(beam_id, 'design_guide_vector_grasp').unitized()
        assert design_guide_vector_grasp is not None

        # Try different grasp face and choose the one that aligns best.
        beam = self.assembly.beam(beam_id)
        best_face = 0
        best_score = -1
        for gripper_grasp_face in range(1, 5):
            gripper_tcp_in_ocf = beam.grasp_frame_ocf(gripper_grasp_face, 0)
            gripper_tcp_in_wcf = gripper_tcp_in_ocf.transformed(Transformation.from_frame(beam.frame))
            # Compute the alignment score using dot product
            alignment_score = gripper_tcp_in_wcf.zaxis.dot(design_guide_vector_grasp)
            if alignment_score > best_score:
                best_score = alignment_score
                best_face = gripper_grasp_face

        self.assembly.set_beam_attribute(beam_id, 'gripper_grasp_face', best_face)
        return best_face

    def search_grasp_face_from_joint_assembly_direction(self, beam_id):
        # type: (str) -> Vector
        """Return the best face number (1-4) for creating `gripper_tcp_in_ocf`
        where grasp face normal is the opposite direction of the beam's assembly direction.

        State Change
        ------------
        This functions sets the following beam_attribute
        - 'gripper_grasp_face'

        Dependency Trigger
        ------------------
        Invalidate: 'compute_gripper_grasp_pose' and downstream
        """
        beam = self.assembly.beam(beam_id)
        for joint_id in self.assembly.get_joints_of_beam_connected_to_already_built(beam_id):
            joint = self.assembly.joint(joint_id)
            selected_face = (joint.face_id + 1) % 4 + 1
            self.assembly.set_beam_attribute(beam_id, 'gripper_grasp_face', selected_face)
            # Dependency Trigger
            self.dependency.invalidate(beam_id, self.compute_gripper_grasp_pose)
            # Only the first joint is considered
            return True
        return False
        raise NotImplementedError

    def assign_gripper_to_beam(self, beam_id, verbose=False):
        """Assign a gripper type using available grippers based on the beam's length.
        Beam must fit within gripper `beam_length_limits`, if multiple options allow,
        the gripper with the closest `target_beam_length` will be chosen.

        If the attribute `gripper_type` is already assigned, this function will not chage it.

        State Change
        ------------
        This functions sets the following beam_attribute
        - 'gripper_type'

        Return
        ------
        `ComputationalResult.ValidCannotContinue` if no suitable gripper can be found
        `ComputationalResult.ValidCanContinue` if a suitable gripper can be found
        """
        beam_length = self.assembly.beam(beam_id).length
        chosen_gripper_type = None
        chosen_gripper_ideal = None

        # Do not change anything if gripper_type is already set
        if self.assembly.get_beam_attribute(beam_id, "gripper_type") is not None:
            if verbose:
                print("Beam (%s) gripper_type (%s) has already been set. No change made by assign_gripper_to_beam()." %
                      (beam_id, self.assembly.get_joint_attribute(beam_id, "gripper_type")))
            return ComputationalResult.ValidNoChange

        for gripper_type in self.available_gripper_types:
            gripper = self.get_one_gripper_by_type(gripper_type)
            # Check if beam length is within limits
            if beam_length >= gripper.beam_length_limits[0] and beam_length <= gripper.beam_length_limits[1]:
                # Compute beam length vs ideal length and make decision
                length_to_ideal = abs(beam_length - gripper.target_beam_length)
                if chosen_gripper_type is None or length_to_ideal < chosen_gripper_ideal:
                    chosen_gripper_type = gripper_type
                    chosen_gripper_ideal = length_to_ideal

        # In cases no suitable gripper is available
        if chosen_gripper_type is None:
            if verbose:
                print("No suitable gripper assigned to %s" % (beam_id))
            return ComputationalResult.ValidCannotContinue
        # Set state and return
        self.assembly.set_beam_attribute(beam_id, "gripper_type", chosen_gripper_type)
        if verbose:
            print("Gripper Type: %s assigned to %s" % (chosen_gripper_type, beam_id))

        return ComputationalResult.ValidCanContinue

    def compute_gripper_grasp_pose(self, beam_id):
        """ Compute grasp pose for the beam and gripper.
        Default values will be applied if 'gripper_grasp_dist_from_start' and 'gripper_grasp_face'
        are not set. Otherwise previous values will be preserved to calculate 'gripper_tcp_in_ocf'.

        Gripper should be assigned before.

        State Change
        ------------
        This functions sets the following beam_attribute
        - 'gripper_grasp_dist_from_start' (if default)
        - 'gripper_grasp_face' (if default)
        - 'gripper_tcp_in_ocf'

        Return
        ------
        `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied
        `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

        """
        # Check to ensure prerequisite
        if self.assembly.get_beam_attribute(beam_id, 'gripper_type') is None:
            return ComputationalResult.ValidCannotContinue

        beam = self.assembly.beam(beam_id)

        # Use previous values if exist
        def grasp_face(beam_id):
            return self.assembly.get_beam_attribute(beam_id, "gripper_grasp_face")

        gripper_grasp_dist_from_start = self.assembly.get_beam_attribute(beam_id, "gripper_grasp_dist_from_start")

        # Apply default values if None
        if grasp_face(beam_id) not in [1, 2, 3, 4]:  # Default method
            gripper_grasp_face = self.search_grasp_face_from_joint_assembly_direction(beam_id)

        if grasp_face(beam_id) not in [1, 2, 3, 4]:  # Backup plan
            gripper_grasp_face = self.search_grasp_face_from_guide_vector_dir(beam_id)

        if grasp_face(beam_id) not in [1, 2, 3, 4]:  # Default plan
            self.assembly.set_beam_attribute(beam_id, "gripper_grasp_face", 1)
            print("Someting wrong, gripper_grasp_face is not in [1,2,3,4] after search. Grasp face defaulted to ", 1)

        if gripper_grasp_dist_from_start is None:
            gripper_grasp_dist_from_start = beam.length / 2.0
            self.assembly.set_beam_attribute(beam_id, "gripper_grasp_dist_from_start", gripper_grasp_dist_from_start)

        # Compute gripper_tcp_in_ocf
        gripper_tcp_in_ocf = beam.grasp_frame_ocf(grasp_face(beam_id), gripper_grasp_dist_from_start)
        self.assembly.set_beam_attribute(beam_id, "gripper_tcp_in_ocf", gripper_tcp_in_ocf)
        return ComputationalResult.ValidCanContinue

    def get_gripper_baseframe_for_beam_at(self, beam_id, attribute_name):
        """ Returns the base frame (base frame) of the gripper (in WCF)
        when the beam is at different key position.

        beam_attribute `gripper_type` and the various `assembly_wcf_*` must be set before calling this function.

        Note
        ----
        - This is not the robot flange, we still have the toolchanger
        - The result can be used directly for gripper.current_frame = get_gripper_baseframe_for_beam_at()
        """
        # Get Gripper Object
        gripper_type = self.assembly.get_beam_attribute(beam_id, 'gripper_type')
        gripper = self.get_one_gripper_by_type(gripper_type)

        # Get the Gripper Tip Frame (TCP) at the beam's final-frame (gripper_tcp_in_wcf)
        gripper_tcp_in_ocf = self.assembly.get_beam_attribute(beam_id, "gripper_tcp_in_ocf")
        beam_frame_wcf = self.assembly.beam(beam_id).frame
        gripper_tcp_in_wcf = gripper_tcp_in_ocf.transformed(Transformation.from_frame(beam_frame_wcf))

        # Move that TCP Frame (gripper_tcp_in_wcf) to the given beam's location (attribute_name)
        T = self.assembly.get_beam_transformaion_to(beam_id, attribute_name)
        if T is None:
            return None
        gripper_tcp_in_wcf = gripper_tcp_in_wcf.transformed(T)

        # Find Gripper Base Frame from Gripper Tip Frame (TCP)
        T = Transformation.from_frame_to_frame(gripper.tool_coordinate_frame, gripper_tcp_in_wcf)
        return Frame.worldXY().transformed(T)

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
        """ Returns the beam frame (in WCF) from the position of a gripper.
        This is effectively the final step of the forward kinematics

        The grasp is retrived from beam attribute `gripper_tcp_in_ocf`
        The gripper.current_frame should be set to the intended location

        ### Credits:
        YiJiang contributed this rather clear way of expressing everytihing in Transformation.
        If one day we have time, we should all switch to this notation to be consistent with
        robotics code community.

        x_from_y, means Y expressed in the X's coordinate system
        """
        # Gripper TCP expressed in world's coordinate
        world_from_gripper_tcp = Transformation.from_frame(gripper.current_tcf)

        # Grasp
        beam_ocf_from_gripper_tcp = Transformation.from_frame(self.assembly.get_beam_attribute(beam_id, "gripper_tcp_in_ocf"))

        # gripper_inverse
        gripper_tcp_from_beam_ocf = beam_ocf_from_gripper_tcp.inverse()

        # Beam frame expressed in the world's coordinate
        world_from_beam = world_from_gripper_tcp * gripper_tcp_from_beam_ocf
        return Frame.from_transformation(world_from_beam)

    def get_gripper_of_beam(self, beam_id, beam_position_name='assembly_wcf_final'):
        # type: (str, bool) -> Gripper
        """Returns one of the gripper object being set at the specified position.
        The beam_position_name must be present in the beam attributes.

        If the beam_position_name is not specified, 'assembly_wcf_final' will be used.
        Refering to the assembled position of the beam.

        Warning: The returned gripper object is not deep-copied.
        Modifying it will change its definition in the process.
        """
        gripper_type = self.assembly.get_beam_attribute(beam_id, 'gripper_type')
        gripper = self.get_one_gripper_by_type(gripper_type)

        # Set the
        gripper_wcf_final = self.get_gripper_baseframe_for_beam_at(beam_id, beam_position_name)
        assert gripper_wcf_final is not None
        gripper.current_frame = gripper_wcf_final

        return gripper

    def adjust_gripper_pos(self, beam_id, amount):
        """ Modify the grasp pose 'gripper_grasp_dist_from_start'

        'gripper_tcp_in_ocf'

        Gripper should be assigned before.

        State Change
        ------------
        This functions updates the following beam_attribute
        - 'gripper_grasp_dist_from_start'
        - 'gripper_tcp_in_ocf'

        Return
        ------
        `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied
        `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

        Dependency Trigger
        ------------------
        Invalidate: 'compute_gripper_grasp_pose' and downstream
        """
        # Check to ensure prerequisite
        if self.assembly.get_beam_attribute(beam_id, 'gripper_type') is None:
            return ComputationalResult.ValidCannotContinue

        beam = self.assembly.beam(beam_id)

        gripper_grasp_face = self.assembly.get_beam_attribute(beam_id, "gripper_grasp_face")
        gripper_grasp_dist_from_start = self.assembly.get_beam_attribute(beam_id, "gripper_grasp_dist_from_start")
        gripper_grasp_dist_from_start += amount
        self.assembly.set_beam_attribute(beam_id, "gripper_grasp_dist_from_start", gripper_grasp_dist_from_start)

        # Recompute beam grasp_frame
        gripper_tcp_in_ocf = beam.grasp_frame_ocf(gripper_grasp_face, gripper_grasp_dist_from_start)
        self.assembly.set_beam_attribute(beam_id, "gripper_tcp_in_ocf", gripper_tcp_in_ocf)
        # Dependency Trigger
        self.dependency.invalidate(beam_id, self.compute_gripper_grasp_pose)
        return ComputationalResult.ValidCanContinue

    # -----------------------------------------------
    # Beam Storage / Pick Up / Retract Algorithms
    # -----------------------------------------------

    def compute_storeage_frame(self, beam_id):
        # type(int) -> None
        """Compute the storage frame.
        This can be performed right after assigning sequence.
        Or TODO optionally after changing grasp face.


        State Change
        ------------
        This functions sets the following beam_attribute
        - 'assembly_wcf_storage'
        """
        # Use the origin as storage frame if there is no beam_storage object
        if self.beam_storage is None:
            self.assembly.set_beam_attribute(beam_id, 'assembly_wcf_storage', Frame.worldXY())

        beam = self.assembly.beam(beam_id)
        storage_frame = self.beam_storage.get_storage_frame(self.assembly.sequence.index(beam_id), len(self.assembly.sequence))
        self.assembly.set_beam_attribute(beam_id, 'assembly_wcf_storage', storage_frame)

    def compute_pickup_frame(self, beam_id):
        """ Compute the pickup frame of a beam
        Beam assembly direcion must be valid. Grasp face and PickupStation and must be assigned before.

        State Change
        ------------
        This functions sets the following beam_attribute
        - 'assembly_wcf_pickup'

        Return
        ------
        `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied
        `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

        """
        # Check to ensure prerequisite
        if self.pickup_station is None:
            return ComputationalResult.ValidCannotContinue

        # Switching computation function depdns on the Pickup station type
        if isinstance(self.pickup_station, StackedPickupStation):
            print("StackedPickupStation not implemented at compute_pickup_frame")
            return ComputationalResult.ValidCannotContinue
        elif isinstance(self.pickup_station, GripperAlignedPickupStation):
            if self.pickup_station.compute_pickup_frame(self, beam_id):
                return ComputationalResult.ValidCanContinue
            else:
                return ComputationalResult.ValidCannotContinue
        elif isinstance(self.pickup_station, PickupStation):
            return self.compute_pickup_location_at_corner_aligning_pickup_location(beam_id)
        else:
            print("Unknown Pickupstation type for compute_pickup_frame")
            return ComputationalResult.ValidCannotContinue

    def compute_alignment_corner_from_grasp_face(self, beam_id, align_face_X0=True, align_face_Y0=True, align_face_Z0=True):
        # type: (str, bool, bool, bool) -> int
        """Returns one corner (int 1 - 8) of the chosen beam
        in relation to the picking face set in 'gripper_grasp_face'.
        There are 8 possible alignment relationship described by the three bool values.

        The retrived int can be used in beam.corner_ocf(alignment_corner)
        to retrive the corner coordinates in ocf of beam.

        Example
        -------
        Beam-start, Y-Neg, bottom-slignment:  align_face_X0 = True, align_face_Y0 = True, align_face_Z0 = True (typical)
        Beam-end, Y-Neg, bottom-slignment:  align_face_X0 = False, align_face_Y0 = False, align_face_Z0 = True
        Beam-start, Y-Pos, bottom-slignment:  align_face_X0 = True, align_face_Y0 = False, align_face_Z0 = True
        """
        # Compute storage alignment corner based on 'gripper_grasp_face'
        gripper_grasp_face = self.assembly.get_beam_attribute(beam_id, 'gripper_grasp_face')  # type: int
        assert gripper_grasp_face is not None
        print('gripper_grasp_face = %s' % gripper_grasp_face)

        corner = 0
        if (not align_face_Z0) and align_face_Y0:
            corner = 1
        elif align_face_Z0 and align_face_Y0:
            corner = 2
        elif align_face_Z0 and (not align_face_Y0):
            corner = 3
        elif (not align_face_Z0) and (not align_face_Y0):
            corner = 4
        else:
            raise Exception("Something is really wrong is aligning corners: compute_alignment_corner_from_grasp_face")

        # For corners 1 - 4, adding the corner number will suffice because corner 1 to 4 are corresponding to face 1 - 4
        corner = (gripper_grasp_face + corner - 2) % 4 + 1

        # Corner 5-8 is only related to whether align_face_X0
        if not align_face_X0:
            corner = corner + 4

        return corner

    def compute_pickup_location_at_corner_aligning_pickup_location(self, beam_id):
        # type: (str, PickupStation) -> None
        """ Compute 'assembly_wcf_pickup' alignment frame
        by aligning a choosen corner relative to the 'gripper_grasp_face'
        to the given pickup_station_frame.

        Note
        ----
        This function cannot be used for beam center alignment.

        Side Effect
        -----------
        beam_attribute 'assembly_wcf_pickup' will be set.

        Example
        -------
        For a beam-start alignment: align_face_X0 = True, align_face_Y0 = True, align_face_Z0 = True
        For a beam-end alignment: align_face_X0 = False, align_face_Y0 = False, align_face_Z0 = True
        e.g
        """
        if self.pickup_station is None:
            return ComputationalResult.ValidCannotContinue

        pickup_station_frame = self.pickup_station.alignment_frame
        align_face_X0 = self.pickup_station.align_face_X0
        align_face_Y0 = self.pickup_station.align_face_Y0
        align_face_Z0 = self.pickup_station.align_face_Z0

        # Compute alignment frame origin - self.compute_alignment_corner_from_grasp_face()
        beam = self.assembly.beam(beam_id)  # type: Beam
        alignment_corner = self.compute_alignment_corner_from_grasp_face(
            beam_id,
            self.pickup_station.align_face_X0,
            self.pickup_station.align_face_Y0,
            self.pickup_station.align_face_Z0)
        #print ('alignment_corner = %s' % alignment_corner)
        alignment_corner_ocf = beam.corner_ocf(alignment_corner)

        # Compute alignment frame X and Y axis - derived from 'gripper_grasp_face' frame
        gripper_grasp_face = self.assembly.get_beam_attribute(beam_id, 'gripper_grasp_face')
        gripper_grasp_face_frame_ocf = beam.reference_side_ocf(gripper_grasp_face)
        alignment_vector_X = gripper_grasp_face_frame_ocf.xaxis
        alignment_vector_Y = gripper_grasp_face_frame_ocf.yaxis
        if not align_face_X0:
            alignment_vector_X.scale(-1.0)
        if not align_face_Y0:
            alignment_vector_Y.scale(-1.0)

        # Alignment frame
        alignment_frame_ocf = Frame(alignment_corner_ocf, alignment_vector_X, alignment_vector_Y)

        # Compute the Transformation needed to bring beam OCF to meet with storage
        T = Transformation.from_frame(beam.frame)
        alignment_frame_wcf = alignment_frame_ocf.transformed(T)
        T = Transformation.from_frame_to_frame(alignment_frame_wcf, pickup_station_frame)

        # Compute the beam ocf in the storage position, save result 'assembly_wcf_pickup'
        assembly_wcf_pickup = beam.frame.transformed(T)
        self.assembly.set_beam_attribute(beam_id, 'assembly_wcf_pickup', assembly_wcf_pickup)

        print("compute_pickup_location_at_corner_aligning_pickup_location computed")
        return ComputationalResult.ValidCanContinue

    def compute_gripper_approach_vector_wcf_final(self, beam_id, verbose=False):
        # type: (str, bool) -> Vector
        """Compute gripper approach_vector (wcf)
        when beam is at final location (beam.frame)

        Return
        ------
        'approach_vector_wcf_final'
        """
        beam = self.assembly.beam(beam_id)
        if verbose:
            print("beam_id = %s " % beam_id)

        # Get approach vector from gripper
        gripper_type = self.assembly.get_beam_attribute(beam_id, 'gripper_type')
        assert gripper_type is not None
        gripper = self.get_one_gripper_by_type(gripper_type)
        approach_vector_tcf = gripper.approach_vector.transformed(gripper.transformation_from_t0cf_to_tcf)
        if verbose:
            print("approach_vector_tcf = %s " % approach_vector_tcf)

        # Express the approach_vector in ocf of beam (beam.frame coordinate frame)
        gripper_tcp_in_ocf = self.assembly.get_beam_attribute(beam_id, 'gripper_tcp_in_ocf')
        T = Transformation.from_frame_to_frame(Frame.worldXY(), gripper_tcp_in_ocf)
        approach_vector_ocf = approach_vector_tcf.transformed(T)
        if verbose:
            print("approach_vector_ocf = %s " % approach_vector_ocf)

        # Express approach vector in World (wcf) for beam in 'assembly_wcf_final'
        T = Transformation.from_frame_to_frame(Frame.worldXY(), beam.frame)
        approach_vector_wcf_final = approach_vector_ocf.transformed(T)
        if verbose:
            print("approach_vector_wcf_final = %s " % approach_vector_wcf_final)

        return approach_vector_wcf_final

    def compute_beam_pickupapproach(self, beam_id):
        """ Compute gripper retract positions from 'assembly_wcf_pickup'.
        Approach vector is taken from gripper's approach vector (tcf) -> (beam ocf)

        Gripper, Pickupstation and beam_attribute'assembly_wcf_pickup' should be set before hand

        State Change
        ------------
        This functions sets the following beam_attribute
        - 'assembly_wcf_pickupapproach'

        Return
        ------
        `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied
        `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

        """
        # Check to ensure prerequisite
        if self.pickup_station is None:
            print("pickup_station is not set")
            return ComputationalResult.ValidCannotContinue
        if self.assembly.get_beam_attribute(beam_id, 'assembly_wcf_pickup') is None:
            print("assembly_wcf_pickup is not set")
            return ComputationalResult.ValidCannotContinue
        if self.assembly.get_beam_attribute(beam_id, 'gripper_type') is None:
            print("gripper_type is not set")
            return ComputationalResult.ValidCannotContinue

        approach_vector_wcf_final = self.compute_gripper_approach_vector_wcf_final(beam_id)

        # Express approach vector in World (wcf) when beam is in 'assembly_wcf_pickup'
        T = self.assembly.get_beam_transformaion_to(beam_id, 'assembly_wcf_pickup')
        approach_vector_wcf_storage = approach_vector_wcf_final.transformed(T)

        # Compute assembly_wcf_pickupapproach (wcf)
        T = Translation.from_vector(approach_vector_wcf_storage.scaled(-1))
        assembly_wcf_pickup = self.assembly.get_beam_attribute(beam_id, 'assembly_wcf_pickup')
        assert assembly_wcf_pickup is not None
        assembly_wcf_pickupapproach = assembly_wcf_pickup.transformed(T)
        self.assembly.set_beam_attribute(beam_id, 'assembly_wcf_pickupapproach', assembly_wcf_pickupapproach)

        return ComputationalResult.ValidCanContinue

    def compute_beam_finalretract(self, beam_id):
        """ Compute gripper retract positions from 'assembly_wcf_final'.
        Retraction direction and amount wcf) is taken from gripper attribute 'approach_vector', reversed.

        Gripper should be assigned before hand.

        State Change
        ------------
        This functions sets the following beam_attribute
        - 'assembly_wcf_finalretract'

        Return
        ------
        `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied
        `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

        """
        # Check to ensure prerequisite
        if self.assembly.get_beam_attribute(beam_id, 'gripper_type') is None:
            print("gripper_type is not set")
            return ComputationalResult.ValidCannotContinue

        approach_vector_wcf_final = self.compute_gripper_approach_vector_wcf_final(beam_id)

        # Compute assembly_wcf_finalretract (wcf)
        T = Translation.from_vector(approach_vector_wcf_final.scaled(-1))
        assembly_wcf_final = self.assembly.get_beam_attribute(beam_id, 'assembly_wcf_final')
        assembly_wcf_finalretract = assembly_wcf_final.transformed(T)
        self.assembly.set_beam_attribute(beam_id, 'assembly_wcf_finalretract', assembly_wcf_finalretract)

        return ComputationalResult.ValidCanContinue

    def compute_beam_pickupretract(self, beam_id):
        # type: (str) -> None
        """Compute 'assembly_wcf_pickupretract' from beam attributes:
        by transforming 'assembly_wcf_pickup' along 'PickupStation.pickup_retract_vector'.

        State Change
        ------------
        This functions sets the following beam_attribute
        - 'assembly_wcf_pickupretract'

        Return
        ------
        `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied
        `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

        """
        # Check to ensure prerequisite
        if self.pickup_station is None:
            print("pickup_station is not set")
            return ComputationalResult.ValidCannotContinue

        retract_vector = self.pickup_station.pickup_retract_vector

        # Compute assembly_wcf_pickupretract
        assembly_wcf_pickup = self.assembly.get_beam_attribute(beam_id, 'assembly_wcf_pickup')
        T = Translation.from_vector(retract_vector)
        assembly_wcf_pickupretract = assembly_wcf_pickup.transformed(T)
        self.assembly.set_beam_attribute(beam_id, 'assembly_wcf_pickupretract', assembly_wcf_pickupretract)

        return ComputationalResult.ValidCanContinue

    # -----------------------
    # Clamps Algorithms
    # -----------------------

    def assign_clamp_type_to_joints(self, beam_id, verbose=True):
        """Assign clamp_types to joints based on the joint's preference and clamp availability.

        If the attribute `clamp_type` is already assigned, this function will not chage it.

        State Change
        ------------
        This functions sets the joint attribute `clamp_type`

        Return
        ------
        `ComputationalResult.ValidCannotContinue` if no suitable clamp is found not satisfied
        `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)
        """
        # Loop through all the beams and look at their previous_built neighbour.
        something_failed = False
        something_changed = False
        for joint_id in self.get_clamp_ids_for_beam(beam_id):
            # Do not change anything if clamp_type is already set
            if self.assembly.get_joint_attribute(joint_id, "clamp_type") is not None:
                if verbose:
                    print("Joint (%s) clamp_type (%s) has already been set. No change made by assign_clamp_type_to_joints()." %
                          (joint_id, self.assembly.get_joint_attribute(joint_id, "clamp_type")))
                continue

            # Loop through the list of clamp types requested by the joint.
            for clamp_type in self.assembly.joint(joint_id).clamp_types:
                # Check if the preferred clamp exist.
                if clamp_type in self.available_clamp_types:
                    self.assembly.set_joint_attribute(joint_id, "clamp_type", clamp_type)
                    something_changed = True

            if self.get_clamp_type_of_joint(joint_id) is None:
                print("WARNING: Cannot assign clamp types. Joint (%s) demand clamp Type: %s" % (joint_id, self.assembly.joint(joint_id).clamp_types))
                something_failed = True
            else:
                if verbose:
                    print("Joint (%s) assigned clamp_type: %s" % (joint_id, self.assembly.get_joint_attribute(joint_id, "clamp_type")))

        # Return results
        if something_failed:
            return ComputationalResult.ValidCannotContinue
        else:
            if something_changed:
                return ComputationalResult.ValidCanContinue
            else:
                return ComputationalResult.ValidNoChange

    def get_clamp_orientation_options(self, beam_id):
        """Each beam assembly may have multiple clamps involved
        Each clamp may have multiple orientation for attaching to the structure.
        This function returns the orientation possibility for each joint

        returns {'joint_id' : [attachment_frame])}]

        """
        joints_with_clamps_id = self.get_clamp_ids_for_beam(beam_id)

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

    def search_valid_clamp_orientation_with_guiding_vector(self, beam_id):
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
            clamp = self.get_clamp_of_joint(joint_id, '')
            if clamp is None:
                continue
            clamp.set_current_frame_from_tcp(selected_frame)

            # Save clamp.current_frame as 'clamp_wcf_final'
            self.assembly.set_joint_attribute(joint_id, 'clamp_wcf_final', clamp.current_frame)
            # chosen_frames.append(selected_frame.copy())
            #print ("Beam (%s) Joint (%s), we need clamp type (%s) at %s" % (beam_id, joint_id, self.get_clamp_type_of_joint(joint_id), selected_frame))
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
        joint_ids = self.assembly.get_joint_ids_of_beam_clamps(beam_id)
        if verbose:
            print("Beam (%s)" % beam_id)
        # Check to ensure prerequisite
        for joint_id in joint_ids:
            if self.assembly.get_joint_attribute(joint_id, 'clamp_wcf_final') is None:
                return ComputationalResult.ValidCannotContinue
        for joint_id in joint_ids:
            if verbose:
                print("|- Clamp at Joint (%s-%s)" % joint_id)
            clamp = self.get_clamp_of_joint(joint_id, 'clamp_wcf_final')

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
        joint_ids = self.assembly.get_joint_ids_of_beam_clamps(beam_id)
        if verbose:
            print("Beam (%s)" % beam_id)
        # Check to ensure prerequisite
        for joint_id in joint_ids:
            if self.assembly.get_joint_attribute(joint_id, 'clamp_wcf_final') is None:
                return ComputationalResult.ValidCannotContinue
        for joint_id in joint_ids:
            if verbose:
                print("|- Clamp at Joint (%s-%s)" % joint_id)
            clamp = self.get_clamp_of_joint(joint_id, 'clamp_wcf_final')

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

    # -----------------------
    # Dependency Computation
    # -----------------------

    def compute_all(self, beam_id):
        return ComputationalResult.ValidCanContinue

    def copy(self):
        return deepcopy(self)

    # -----------------------
    # Environment Model
    # -----------------------

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

    # -----------------------
    # Action and Movements
    # -----------------------

    from .algorithms import compute_initial_state, compute_intermediate_states

    def get_action_by_beam_id(self, beam_id):
        # type: (str) -> list[Action]
        """ Get an ordered list of Action related to a beam"""
        return [action for action in self.actions if self.assembly.sequence[action.seq_n] == beam_id]

    def get_movements_by_beam_id(self, beam_id):
        # type: (str) -> list[Movement]
        """ Get an ordered list of Movements related to a beam"""
        return [movement for action in self.get_action_by_beam_id(beam_id) for movement in action.movements]

    def get_movement_summary_by_beam_id(self, beam_id):
        movements = self.get_movements_by_beam_id(beam_id)
        print('=====')
        print('Summary:')
        for i, m in enumerate(movements):
            print('---')
            print('({}) {} \npriority {} | has start conf {} | has end conf {} | has traj {}'.format(
                i, m.short_summary, _colored_planning_priority(m.planning_priority),
                _colored_is_none(self.movement_has_start_robot_config(m)), _colored_is_none(self.movement_has_end_robot_config(m)),
                _colored_is_none(m.trajectory is not None if isinstance(m, RoboticMovement) else None)
            ))

    def get_movement_start_state(self, movement):
        # type: (Movement) -> dict[str, ObjectState]
        """ return the start state before the movment """
        start_state = self.initial_state
        for _movement in self.movements:
            if _movement is movement:
                return start_state
            start_state = _movement.end_state
        raise Exception("Given Movement object does not exist in self.movements.")

    def get_movement_end_state(self, movement):
        # type: (Movement) -> dict[str, ObjectState]
        """ return the end state after the movment """
        return movement.end_state

    def get_movements_by_planning_priority(self, beam_id, priority):
        # type: (str, int) -> list[Movement]
        return [m for m in self.get_movements_by_beam_id(beam_id) if m.planning_priority == priority]

    def set_movement_start_state(self, movement, state_dict, deep_copy=False):
        # type: (Movement, dict[str, ObjectState], bool) -> None
        """Set the start state of a movement, effectively changing the end state of the previous movement.
        Attempt to set the start state of the first movement will modify process.initial_state.

        An optional deep copy is made for each object State """

        # In order to reuse the function get_movement_start_state(),
        # we keep the pointer to the dictionary, clear all the contents and fill in the new stuff.
        # this is handeled by the copy_state_dict()
        start_state = self.get_movement_start_state(movement)
        copy_state_dict(start_state, state_dict, clear=False, deep_copy=deep_copy)

    def set_movement_end_state(self, movement, state_dict, deep_copy=False):
        # type: (Movement, dict[str, ObjectState], bool) -> None
        """Set the end state of a movement, effectively changing the end state of the previous movement.
        Attempt to set the start state of the first movement will modify process.initial_state.

        An optional deep copy is made for each object State """

        # In order to reuse the function get_movement_start_state(),
        # we keep the pointer to the dictionary, clear all the contents and fill in the new stuff.
        # this is handeled by the copy_state_dict()
        start_state = self.get_movement_end_state(movement)
        copy_state_dict(start_state, state_dict, clear=False, deep_copy=deep_copy)

    def get_action_of_movement(self, movement):
        # type: (Movement) -> Action
        """Returns the Action object in which the movement belongs to"""
        for action in self.actions:
            for _movement in action.movements:
                # ? Does this really work? what if I change some attributes in the movement?
                if movement is _movement:
                    return action

    def movement_has_start_robot_config(self, movement):
        # type: (Movement) -> bool
        """Returns True if the movement's start_state.['robot'].kinematic_config is not None """
        state = self.get_movement_start_state(movement)
        return state['robot'].kinematic_config is not None

    def movement_has_end_robot_config(self, movement):
        # type: (Movement) -> bool
        """Returns True if the movement's end_state.['robot'].kinematic_config is not None """
        state = self.get_movement_end_state(movement)
        return state['robot'].kinematic_config is not None


def _colored_is_none(value):
    if value is None:
        return colored('None', 'yellow')
    else:
        return colored(value, 'green' if value else 'red')


def _colored_planning_priority(p):
    color_from_p = {1: 'blue', 0: 'magenta', -1: 'white'}
    return colored(p, color_from_p[p])
