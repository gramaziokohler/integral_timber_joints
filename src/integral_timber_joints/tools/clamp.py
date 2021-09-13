#from compas_fab.robots import Robot

from compas.geometry import Frame, Point, Transformation, Vector
from compas.geometry.transformations.translation import Translation

from compas.robots import Axis, Joint, Link
# from compas.datastructures import Mesh
from compas_fab.robots import Configuration

from integral_timber_joints.tools.gripper import Gripper


class Clamp (Gripper):
    """ Clamp object represents a robotic clamp that has both
    gripper jaw and clamp jaw. This is a subclass of Gripper.

    The object keep track of the RobotModel and the kinematic configuration.
    """

    def __init__(self, name,
                 type_name="Clamp",
                 tool_coordinate_frame=None,
                 tool_pick_up_frame=None,
                 tool_storage_frame=None,
                 gripper_jaw_position_min=0,
                 gripper_jaw_position_max=100,
                 clamp_jaw_position_min=0,
                 clamp_jaw_position_max=100,
                 approach_vector=None):

        # Call Tool init
        if tool_coordinate_frame is None:
            tool_coordinate_frame = Frame.worldXY()

        super(Clamp, self).__init__(
            name,
            type_name,
            tool_coordinate_frame,
            tool_pick_up_frame,
            tool_storage_frame,
            gripper_jaw_position_min,
            gripper_jaw_position_max,
            approach_vector
        )

        # --------------------------------------------------------
        # Extrinsic properities / state (Gripper Specific)
        # --------------------------------------------------------

        self.clamp_jaw_position = clamp_jaw_position_min  # type: float

        # --------------------------------------------------------
        # Intrinsic properities (Gripper Specific)
        # --------------------------------------------------------

        self.clamp_jaw_limits = (clamp_jaw_position_min, clamp_jaw_position_max)  # type: tuple[float, float]

    # --------------------------------------------------------------
    # Functions to get and set attributes from attributes dictionary.
    # --------------------------------------------------------------

    @property
    def clamp_jaw_position(self):
        return self.attributes.get('clamp_jaw_position', 0)

    @clamp_jaw_position.setter
    def clamp_jaw_position(self, v):
        self.attributes['clamp_jaw_position'] = v

    @property
    def clamp_jaw_limits(self):
        return self.attributes.get('clamp_jaw_limits', (0, 100))

    @clamp_jaw_limits.setter
    def clamp_jaw_limits(self, v):
        self.attributes['clamp_jaw_limits'] = v

    @property
    def jaw_block_vectors(self):
        """ List of Vectors for computing jaw_approach directions.
        Vector direction is the direction where the beam cannot move towards. """
        return self.attributes.get('jaw_block_vectors', [])

    @jaw_block_vectors.setter
    def jaw_block_vectors(self, v):
        self.attributes['jaw_block_vectors'] = v

    @property
    def jaw_clearance_vector(self):
        """ A directional vector describing how much distance and direction
        the beam-in-jaw has to move to clear the jaw if moved out.
        """
        return self.attributes.get('jaw_clearance_vector', Vector(0, 0, 0))

    @jaw_clearance_vector.setter
    def jaw_clearance_vector(self, v):
        self.attributes['jaw_clearance_vector'] = v

    @property
    def detachretract1_vector(self):
        """ A directional vector describing how much distance and direction
        the beam-in-jaw has to move to clear the jaw if moved out.
        """
        return self.attributes.get('detachretract1_vector', Vector(0, 0, 0))

    @detachretract1_vector.setter
    def detachretract1_vector(self, v):
        self.attributes['detachretract1_vector'] = v

    @property
    def detachretract2_vector(self):
        """ A directional vector describing how much distance and direction
        the beam-in-jaw has to move to clear the jaw if moved out.
        """
        return self.attributes.get('detachretract2_vector', Vector(0, 0, 0))

    @detachretract2_vector.setter
    def detachretract2_vector(self, v):
        self.attributes['detachretract2_vector'] = v

    @property
    def approach1_vector(self):
        """ A directional vector describing how much distance and direction
        the beam-in-jaw has to move to clear the jaw if moved out.
        """
        return self.attributes.get('approach1_vector', Vector(0, 0, 0))

    @approach1_vector.setter
    def approach1_vector(self, v):
        self.attributes['approach1_vector'] = v

    @property
    def approach2_vector(self):
        """ A directional vector describing how much distance and direction
        the beam-in-jaw has to move to clear the jaw if moved out.
        """
        return self.attributes.get('approach2_vector', Vector(0, 0, 0))

    @approach2_vector.setter
    def approach2_vector(self, v):
        self.attributes['approach2_vector'] = v  # robot_model.jaw_clearance_vector = Vector(110, 0, 0)

    # ----------------------------------
    # Functions for computing the complexed approach and retract
    # ----------------------------------

    @ property
    def tool_storage_approach_frame2(self):
        # type: () -> Frame
        """Compute the approach frame in wcf.
        Part of PlaceClampToStorageAction
        """
        approach2_vector_wcf = self.tool_storage_frame.to_world_coordinates(self.approach2_vector)
        return self.tool_storage_frame.transformed(Translation.from_vector(approach2_vector_wcf.scaled(-1)))

    @ property
    def tool_storage_approach_frame1(self):
        # type: () -> Frame
        """Compute the approach frame in wcf.
        Part of PlaceClampToStorageAction
        """
        approach1_vector_wcf = self.tool_storage_frame.to_world_coordinates(self.approach1_vector)
        return self.tool_storage_approach_frame2.transformed(Translation.from_vector(approach1_vector_wcf.scaled(-1)))

    @ property
    def tool_storage_retract_frame1(self):
        # type: () -> Frame
        """ Compute the retract frame in wcf
        Part of PickClampFromStorageAction
        """
        detachretract1_vector_wcf = self.tool_storage_frame.to_world_coordinates(self.detachretract1_vector)
        return self.tool_storage_frame.transformed(Translation.from_vector(detachretract1_vector_wcf))

    @ property
    def tool_storage_retract_frame2(self):
        # type: () -> Frame
        """ Compute the retract frame in wcf
        Part of PickClampFromStorageAction
        """
        detachretract2_vector_wcf = self.tool_storage_frame.to_world_coordinates(self.detachretract2_vector)
        return self.tool_storage_retract_frame1.transformed(Translation.from_vector(detachretract2_vector_wcf))

    # ----------------------------------
    # Functions for kinematic state
    # ----------------------------------

    # --------------------------------------------------------
    # State Setting Functions
    # --------------------------------------------------------

    def _set_kinematic_state(self, state_dict):
        self.clamp_jaw_position = state_dict['clamp_jaw_position']
        super(Clamp, self)._set_kinematic_state(state_dict)

    def _get_kinematic_state(self):
        state_dict = super(Clamp, self)._get_kinematic_state()
        state_dict.update({'clamp_jaw_position': self.clamp_jaw_position})
        return state_dict

    @property
    def current_configuration(self):
        """Gets the current Configuration of the joints in the underlying RobotModel
        This can be used to update the artist or update the robot.
        """
        values, types, joint_names = [], [], []
        for joint in self.get_configurable_joints():
            if joint.name.startswith('joint_gripper_'):
                values.append(self.gripper_jaw_position)
                types.append(Joint.PRISMATIC)
                joint_names.append(joint.name)
            if joint.name.startswith('joint_clamp_'):
                values.append(self.clamp_jaw_position)
                types.append(Joint.PRISMATIC)
                joint_names.append(joint.name)
        return Configuration(values, types, joint_names)

    # --------------------------------------------------------
    # Convinence Functions
    # --------------------------------------------------------

    @property
    def jaw_blocking_vectors_in_wcf(self):
        T = Transformation.from_frame(self.current_frame)
        return [vec.transformed(T) for vec in self.jaw_block_vectors]

    @property
    def jaw_clearance_vectors_in_wcf(self):
        T = Transformation.from_frame(self.current_frame)
        return self.jaw_clearance_vector.transformed(T)

    def open_gripper(self):
        self.gripper_jaw_position = self.gripper_jaw_limits[1]

    def close_gripper(self):
        self.gripper_jaw_position = self.gripper_jaw_limits[0]

    def open_clamp(self):
        self.clamp_jaw_position = self.clamp_jaw_limits[1]

    def close_clamp(self):
        self.clamp_jaw_position = self.clamp_jaw_limits[0]

# --------------------------------------------------------
# Factory to construct Clamp
# --------------------------------------------------------


def Lap90ClampFactory(
    name,
    type_name,
    gripper_jaw_position_min,
    gripper_jaw_position_max,
    clamp_jaw_position_min,
    clamp_jaw_position_max,
    tool_coordinate_frame,  # Ref to T0CF (TCF ~= TCP)
    tool_pick_up_frame,     # Ref to T0CF
    tool_storage_frame,     # Ref to WCF
    mesh_gripper_base,
    mesh_gripper_jaw_l,
    mesh_gripper_jaw_r,
    mesh_clamp_jaw_l,
    mesh_clamp_jaw_r,
    approach_vector,
    detachretract1_vector,
    detachretract2_vector,
):
    """ A Parallel gripper will have a base and two gripper jaw.
    Modelling guide
    ---------------
    The left jaws opens towards -Y direction.
    The right jaw opens towards +Y direction.
    The clamp jaw opens towards +Z direction.
    The clamp jaw closes (clamping) towards -Z direction.
    The clamp jaw opening faces +X direction.
    """
    robot_model = Clamp(name, type_name)
    robot_model.gripper_jaw_limits = (gripper_jaw_position_min, gripper_jaw_position_max)
    robot_model.clamp_jaw_limits = (clamp_jaw_position_min, clamp_jaw_position_max)

    robot_model.tool_coordinate_frame = tool_coordinate_frame
    robot_model.tool_pick_up_frame = tool_pick_up_frame
    robot_model.tool_storage_frame = tool_storage_frame
    robot_model.approach_vector = approach_vector               # This vector is ref to t0cf
    robot_model.detachretract1_vector = detachretract1_vector   # This vector is ref to t0cf
    robot_model.detachretract2_vector = detachretract2_vector   # This vector is ref to t0cf

    #world_link = robot_model.add_link('world')
    gripper_base = robot_model.add_link('gripper_base', visual_meshes=mesh_gripper_base, collision_meshes=mesh_gripper_base)
    gripper_jaw_l = robot_model.add_link('gripper_jaw_l', visual_meshes=mesh_gripper_jaw_l, collision_meshes=mesh_gripper_jaw_l)
    gripper_jaw_r = robot_model.add_link('gripper_jaw_r', visual_meshes=mesh_gripper_jaw_r, collision_meshes=mesh_gripper_jaw_r)
    clamp_jaw_l = robot_model.add_link('clamp_jaw_l', visual_meshes=mesh_clamp_jaw_l, collision_meshes=mesh_clamp_jaw_l)
    clamp_jaw_r = robot_model.add_link('clamp_jaw_r', visual_meshes=mesh_clamp_jaw_r, collision_meshes=mesh_clamp_jaw_r)

    #robot_model.add_joint('world_base_fixed_joint', Joint.FIXED, world_link, base_link)
    robot_model.add_joint('joint_gripper_jaw_l', Joint.PRISMATIC, gripper_base, gripper_jaw_l, axis=[0, -1, 0], limit=robot_model.gripper_jaw_limits)
    robot_model.add_joint('joint_gripper_jaw_r', Joint.PRISMATIC, gripper_base, gripper_jaw_r, axis=[0, 1, 0], limit=robot_model.gripper_jaw_limits)
    robot_model.add_joint('joint_clamp_jaw_l', Joint.PRISMATIC, gripper_jaw_l, clamp_jaw_l, axis=[0, 0, 1], limit=robot_model.clamp_jaw_limits)
    robot_model.add_joint('joint_clamp_jaw_r', Joint.PRISMATIC, gripper_jaw_r, clamp_jaw_r, axis=[0, 0, 1], limit=robot_model.clamp_jaw_limits)

    # A constant list of vectors (ref t0cf) where the beam-in-jaw is blocked by the jaw
    robot_model.jaw_block_vectors = [Vector(0, 0, 1.0), Vector(0, 0, -1.0), Vector(-1.0, 0, 0)]
    # A directional vector describing how much distance the beam-in-jaw has to move to clear the jaw if moved out.
    robot_model.jaw_clearance_vector = Vector(110, 0, 0)
    return robot_model


def CL3Factory(
    name,
    type_name,
    gripper_jaw_position_min,
    gripper_jaw_position_max,
    clamp_jaw_position_min,
    clamp_jaw_position_max,
    tool_coordinate_frame,  # Ref to T0CF (TCF ~= TCP)
    tool_pick_up_frame,     # Ref to T0CF
    tool_storage_frame,     # Ref to WCF
    base_mesh,
    gripper_jaw_mesh,
    clamp_jaw_mesh,
    approach1_vector,
    approach2_vector,
    detachretract1_vector,
    detachretract2_vector,
    gripper_drill_lines,
    gripper_drill_diameter
):
    """ A Parallel gripper will have a base and two gripper jaw.
    Modelling guide
    ---------------
    The gripper jaws opens towards +Z direction of tool_coordinate_frame.
    The clamp jaws opens towards +Z direction of tool_coordinate_frame.

    The joints name should start with 'joint_gripper_' or 'joint_clamp_'
    """
    robot_model = Clamp(name, type_name)
    robot_model.gripper_jaw_limits = (gripper_jaw_position_min, gripper_jaw_position_max)
    robot_model.clamp_jaw_limits = (clamp_jaw_position_min, clamp_jaw_position_max)

    robot_model.tool_coordinate_frame = tool_coordinate_frame
    robot_model.tool_pick_up_frame = tool_pick_up_frame
    robot_model.tool_storage_frame = tool_storage_frame
    robot_model.approach1_vector = approach1_vector               # This vector is ref to t0cf
    robot_model.approach2_vector = approach2_vector               # This vector is ref to t0cf
    robot_model.detachretract1_vector = detachretract1_vector   # This vector is ref to t0cf
    robot_model.detachretract2_vector = detachretract2_vector   # This vector is ref to t0cf
    robot_model.gripper_drill_lines = gripper_drill_lines
    robot_model.gripper_drill_diameter = gripper_drill_diameter

    #world_link = robot_model.add_link('world')
    gripper_base = robot_model.add_link('gripper_base', visual_meshes=base_mesh, collision_meshes=base_mesh)
    gripper_jaw = robot_model.add_link('gripper_jaw', visual_meshes=gripper_jaw_mesh, collision_meshes=gripper_jaw_mesh)
    clamp_jaw = robot_model.add_link('clamp_jaw', visual_meshes=clamp_jaw_mesh, collision_meshes=clamp_jaw_mesh)

    #robot_model.add_joint('world_base_fixed_joint', Joint.FIXED, world_link, base_link)
    robot_model.add_joint('joint_gripper_jaw', Joint.PRISMATIC, gripper_base, gripper_jaw, axis=tool_coordinate_frame.zaxis, limit=robot_model.gripper_jaw_limits)
    robot_model.add_joint('joint_clamp_jaw', Joint.PRISMATIC, gripper_base, clamp_jaw, axis=tool_coordinate_frame.zaxis, limit=robot_model.clamp_jaw_limits)

    # A constant list of vectors (ref t0cf) where the beam-in-jaw is blocked by the jaw.
    # Vector direction is the contact surface normal from the in-jaw beam side
    # (alternatively) Vector direction is the direction where the beam cannot move towards.
    robot_model.jaw_block_vectors = [tool_coordinate_frame.zaxis.copy(), tool_coordinate_frame.zaxis.scaled(-1), Vector(0, 0, -1)]
    # A directional vector describing how much distance the beam-in-jaw has to move to clear the jaw if moved out.
    robot_model.jaw_clearance_vector = tool_coordinate_frame.yaxis.scaled(100)
    return robot_model


if __name__ == "__main__":
    c = Clamp('c1')
    print(c)
    pass
