#from compas_fab.robots import Robot

from compas.datastructures import Mesh
from compas.geometry import Frame, Point, Transformation, Vector, Translation
from compas.robots import Axis, Joint, Link
from compas_fab.robots import Configuration

from integral_timber_joints.tools import Tool


class Gripper (Tool):
    """ Gripper object represents a robotic gripper that has both
    gripper jaw. This is a subclass of Tool and Robot Model.

    The object keep track of the RobotModel and the kinematic configuration.
    """

    def __init__(self, name,
                 type_name="Gripper",
                 tool_coordinate_frame=None,
                 tool_pick_up_frame=None,
                 tool_storage_frame=None,
                 gripper_jaw_position_min=0,
                 gripper_jaw_position_max=100,
                 target_beam_length=1000,
                 beam_length_limit_min=500,
                 beam_length_limit_max=5000,
                 approach_vector=None,
                 storage_approach_vector=None
                 ):

        # Call Tool init
        if tool_coordinate_frame is None:
            tool_coordinate_frame = Frame.worldXY()
        super(Gripper, self).__init__(name, type_name, tool_coordinate_frame)

        # --------------------------------------------------------
        # Extrinsic properities (Gripper Specific)
        # --------------------------------------------------------
        self.gripper_jaw_position = gripper_jaw_position_min

        # --------------------------------------------------------
        # Intrinsic properities (Gripper Specific)
        # --------------------------------------------------------

        self.approach_vector = approach_vector
        self.storage_approach_vector = storage_approach_vector
        self.tool_pick_up_frame = tool_pick_up_frame
        self.tool_storage_frame = tool_storage_frame
        self.gripper_jaw_limits = (gripper_jaw_position_min, gripper_jaw_position_max)  # type: tuple[float, float]
        self.target_beam_length = target_beam_length
        self.beam_length_limits = (beam_length_limit_min, beam_length_limit_max)
    # --------------------------------------------------------------
    # Functions to get and set attributes from attributes dictionary.
    # --------------------------------------------------------------

    @property
    def gripper_jaw_position(self):
        return self.attributes.get('gripper_jaw_position', 0)

    @gripper_jaw_position.setter
    def gripper_jaw_position(self, v):
        self.attributes['gripper_jaw_position'] = v

    @property
    def gripper_jaw_limits(self):
        return self.attributes.get('gripper_jaw_limits', (0, 100))

    @gripper_jaw_limits.setter
    def gripper_jaw_limits(self, v):
        self.attributes['gripper_jaw_limits'] = v

    @property
    def target_beam_length(self):
        return self.attributes.get('target_beam_length', 1000)

    @target_beam_length.setter
    def target_beam_length(self, v):
        self.attributes['target_beam_length'] = v

    @property
    def beam_length_limits(self):
        # type: () -> tuple(float, float)
        """Min max limits of beam lengths this gripper can hold:
        (beam_length_limit_min, beam_length_limit_max) """
        return self.attributes.get('beam_length_limits', (500, 5000))

    @beam_length_limits.setter
    def beam_length_limits(self, v):
        self.attributes['beam_length_limits'] = v

    @property
    def approach_vector(self):
        # type: () -> Vector
        """Vector for gripper to approach beam
        """
        return self.attributes.get('approach_vector', None)

    @approach_vector.setter
    def approach_vector(self, v):
        self.attributes['approach_vector'] = v

    @property
    def storage_approach_vector(self):
        # type: () -> Vector
        """Vector for gripper to approach storage
        """
        return self.attributes.get('storage_approach_vector', self.approach_vector)

    @storage_approach_vector.setter
    def storage_approach_vector(self, v):
        self.attributes['storage_approach_vector'] = v

    @property
    def tool_pick_up_frame(self):
        return self.attributes.get('tool_pick_up_frame', None)

    @tool_pick_up_frame.setter
    def tool_pick_up_frame(self, v):
        self.attributes['tool_pick_up_frame'] = v

    @property
    def tool_storage_frame(self):
        return self.attributes.get('tool_storage_frame', None)

    @tool_storage_frame.setter
    def tool_storage_frame(self, v):
        self.attributes['tool_storage_frame'] = v

    @property
    def gripper_drill_lines(self):
        """ List of Lines where the interface between gripper with the
        beams require drilling guide holes """
        return self.attributes.get('gripper_drill_lines', [])

    @gripper_drill_lines.setter
    def gripper_drill_lines(self, v):
        self.attributes['gripper_drill_lines'] = v

    @property
    def gripper_drill_diameter(self):
        """ List of Lines where the interface between gripper with the
        beams require drilling guide holes """
        return self.attributes.get('gripper_drill_diameter', 10)

    @gripper_drill_diameter.setter
    def gripper_drill_diameter(self, v):
        self.attributes['gripper_drill_diameter'] = v

    # ----------------------------------
    # Functions for kinematic state
    # ----------------------------------

    def _set_kinematic_state(self, state_dict):
        # type: (dict[str, float]) -> None
        self.gripper_jaw_position = state_dict['gripper_jaw_position']

    def _get_kinematic_state(self):
        # type: () -> dict[str, float]
        return {'gripper_jaw_position': self.gripper_jaw_position}

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
        return Configuration(values, types, joint_names)

    # --------------------------------------------------------
    # Convinence Functions
    # --------------------------------------------------------

    def open_gripper(self):
        self.gripper_jaw_position = self.gripper_jaw_limits[1]

    def close_gripper(self):
        self.gripper_jaw_position = self.gripper_jaw_limits[0]


    # --------------------------------------------------------
    # Transformation
    # --------------------------------------------------------

    @property
    def t_approach(self):
        return Translation.from_vector(self.approach_vector)

# --------------------------------------------------------
# Factory to construct Gripper
# --------------------------------------------------------


def ParallelGripperFactory(
    name,
    type_name,
    gripper_jaw_position_min,
    gripper_jaw_position_max,
    tool_coordinate_frame,
    tool_pick_up_frame,
    tool_storage_frame,
    mesh_gripper_base,
    mesh_gripper_jaw_l,
    mesh_gripper_jaw_r,
    target_beam_length,
    beam_length_limit_min,
    beam_length_limit_max,
    approach_vector,
    jaw_vector_l=None,           # In Ref to T0CF
    jaw_vector_r=None,           # In Ref to T0CF
    gripper_drill_lines=[],
    gripper_drill_diameter=10,

):
    """ A Parallel gripper will have a base and two gripper jaw.
    Modelling guide
    ---------------
    The left jaws opens towards -Y direction.
    The right jaw opens towards +Y direction.
    The gripper opening should point towards +Z direction
    """
    robot_model = Gripper(name,
                          type_name=type_name,
                          tool_coordinate_frame=tool_coordinate_frame,
                          tool_pick_up_frame=tool_pick_up_frame,
                          tool_storage_frame=tool_storage_frame,
                          gripper_jaw_position_min=gripper_jaw_position_min,
                          gripper_jaw_position_max=gripper_jaw_position_max,
                          target_beam_length=target_beam_length,
                          beam_length_limit_min=beam_length_limit_min,
                          beam_length_limit_max=beam_length_limit_max,
                          approach_vector=approach_vector
                          )

    robot_model.gripper_drill_lines = gripper_drill_lines
    robot_model.gripper_drill_diameter = gripper_drill_diameter

    gripper_base = robot_model.add_link('gripper_base', visual_meshes=mesh_gripper_base, collision_meshes=mesh_gripper_base)
    gripper_jaw_l = robot_model.add_link('gripper_jaw_l', visual_meshes=mesh_gripper_jaw_l, collision_meshes=mesh_gripper_jaw_l)
    gripper_jaw_r = robot_model.add_link('gripper_jaw_r', visual_meshes=mesh_gripper_jaw_r, collision_meshes=mesh_gripper_jaw_r)

    if jaw_vector_l is None:
        jaw_vector_l = [0, -1, 0]
    if jaw_vector_r is None:
        jaw_vector_r = [0, 1, 0]

    #robot_model.add_joint('world_base_fixed_joint', Joint.FIXED, world_link, base_link)
    robot_model.add_joint('joint_gripper_jaw_l', Joint.PRISMATIC, gripper_base, gripper_jaw_l, axis=jaw_vector_l, limit=robot_model.gripper_jaw_limits)
    robot_model.add_joint('joint_gripper_jaw_r', Joint.PRISMATIC, gripper_base, gripper_jaw_r, axis=jaw_vector_r, limit=robot_model.gripper_jaw_limits)

    return robot_model


def MultiFingerParallelGripperFactory(
    name,
    type_name,
    gripper_jaw_position_min,
    gripper_jaw_position_max,
    tool_coordinate_frame,      # TCF / TCP (In Ref to T0CF)
    tool_pick_up_frame,
    tool_storage_frame,
    mesh_gripper_base,
    meshes_gripper_jaw_l,    # type: list[Mesh] # Accepts multiple meshes
    meshes_gripper_jaw_r,    # type: list[Mesh] # Accepts multiple meshes
    jaw_vector_l,           # In Ref to T0CF
    jaw_vector_r,           # In Ref to T0CF
    target_beam_length,
    beam_length_limit_min,
    beam_length_limit_max,
    approach_vector,
    storageapproach_vector,
):
    """ A Parallel gripper will have a base and two gripper jaw.
    Modelling guide
    ---------------
    The left jaws opens towards -Y direction.
    The right jaw opens towards +Y direction.
    The gripper opening should point towards +Z direction
    """

    robot_model = Gripper(name,
                          type_name=type_name,
                          tool_coordinate_frame=tool_coordinate_frame,
                          tool_pick_up_frame=tool_pick_up_frame,
                          tool_storage_frame=tool_storage_frame,
                          gripper_jaw_position_min=gripper_jaw_position_min,
                          gripper_jaw_position_max=gripper_jaw_position_max,
                          target_beam_length=target_beam_length,
                          beam_length_limit_min=beam_length_limit_min,
                          beam_length_limit_max=beam_length_limit_max,
                          approach_vector=approach_vector,
                          storage_approach_vector=storageapproach_vector,
                          )
    # Gripper Base and Jaw Links
    #world_link = robot_model.add_link('world')
    gripper_base = robot_model.add_link('gripper_base', mesh_gripper_base)
    gripper_jawlinks_l = []
    for i, mesh in enumerate(meshes_gripper_jaw_l):
        gripper_jawlinks_l.append(robot_model.add_link('gripper_jaw_l%i' % i, visual_meshes=[mesh], collision_meshes=[mesh]))
    gripper_jawlinks_r = []
    for i, mesh in enumerate(meshes_gripper_jaw_r):
        gripper_jawlinks_r.append(robot_model.add_link('gripper_jaw_r%i' % i, visual_meshes=[mesh], collision_meshes=[mesh]))

    # Jaw Joints
    #robot_model.add_joint('world_base_fixed_joint', Joint.FIXED, world_link, base_link)
    for link in gripper_jawlinks_l:
        robot_model.add_joint('joint_gripper_%s' % link.name, Joint.PRISMATIC, gripper_base, link, axis=jaw_vector_l, limit=robot_model.gripper_jaw_limits)
    for link in gripper_jawlinks_r:
        robot_model.add_joint('joint_gripper_%s' % link.name, Joint.PRISMATIC, gripper_base, link, axis=jaw_vector_r, limit=robot_model.gripper_jaw_limits)

    return robot_model


if __name__ == "__main__":
    g = Gripper('g1')
    print(g)
    pass
