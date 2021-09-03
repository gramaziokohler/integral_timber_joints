from compas.geometry import Frame, Point, Transformation, Vector, Translation
from compas.robots import Axis, Joint, Link

# from compas.datastructures import Mesh
from compas_fab.robots import Configuration

from integral_timber_joints.tools.gripper import Gripper


class Screwdriver (Gripper):
    """ Screwdriver object represents a robotic screwdriver that has has integrated gripper.
    This is a subclass of Gripper as it can attach itself to a Beam or can be used to pickup Beam.

    The object is a subclass or ToolModel and RobotModel and contains the kinematic relationship of the tool.
    The screwdriver consist of a main body, a main screwing shaft that rotates
    and two pin gripper assembly that moves together diagonally.
    """

    def __init__(self, name,
                 type_name="Screwdriver",
                 tool_coordinate_frame=None,
                 tool_pick_up_frame=None,
                 tool_storage_frame=None,
                 gripper_jaw_position_min=0,
                 gripper_jaw_position_max=47,
                 approach_vector=None,
                 storageapproach1_vector=None,
                 storageapproach2_vector=None,
                 detachretract_vector=None):

        # Call Tool init
        if tool_coordinate_frame is None:
            tool_coordinate_frame = Frame.worldXY()
        if tool_pick_up_frame is None:
            tool_pick_up_frame = Frame.worldXY()
        if tool_storage_frame is None:
            tool_storage_frame = Frame.worldXY()
        if approach_vector is None:
            approach_vector = Vector(0, 0, 1)
        if detachretract_vector is None:
            detachretract_vector = Vector(0, 0, -1)

        super(Screwdriver, self).__init__(
            name,
            type_name,
            tool_coordinate_frame,
            tool_pick_up_frame,
            tool_storage_frame,
            gripper_jaw_position_min=gripper_jaw_position_min,
            gripper_jaw_position_max=gripper_jaw_position_max,
            approach_vector=approach_vector,
            storage_approach_vector=None,
        )

        # --------------------------------------------------------
        # Extrinsic properities / state (Gripper Specific)
        # --------------------------------------------------------

        self.screw_position = 0  # type: float

        # --------------------------------------------------------
        # Intrinsic properities (Gripper Specific)
        # --------------------------------------------------------

        self.detachretract_vector = detachretract_vector
        self.screw_pitch_mm = 5
        self.screw_length_mm = 5
        if storageapproach1_vector is not None:
            self.storageapproach1_vector = storageapproach1_vector
        if storageapproach2_vector is not None:
            self.storageapproach2_vector = storageapproach2_vector
        # self.clamp_jaw_limits = (clamp_jaw_position_min, clamp_jaw_position_max)  # type: tuple[float, float]

    # --------------------------------------------------------------
    # Functions to get and set attributes from attributes dictionary.
    # --------------------------------------------------------------

    @property
    def screw_position(self):
        return self.attributes.get('screw_position', 0)

    @screw_position.setter
    def screw_position(self, v):
        self.attributes['screw_position'] = v

    @property
    def detachretract_vector(self):
        """ A directional vector (in T0CF) describing how much distance and direction
        the tool need to pull out when retracting from beam or from storage.
        """
        return self.attributes.get('detachretract_vector', Vector(0, 0, 0))

    @detachretract_vector.setter
    def detachretract_vector(self, v):
        self.attributes['detachretract_vector'] = v

    @property
    def storageapproach1_vector(self):
        """ A directional vector (in T0CF) describing how to place screwdriver onto storage rack
        Step 1 (Faster)
        """
        return self.attributes.get('storageapproach1_vector', Vector(0, 0, 0))

    @storageapproach1_vector.setter
    def storageapproach1_vector(self, v):
        self.attributes['storageapproach1_vector'] = v

    @property
    def storageapproach2_vector(self):
        """ A directional vector (in T0CF) describing how to place screwdriver onto storage rack
        Step 2 (Slower)
        """
        return self.attributes.get('storageapproach2_vector', Vector(0, 0, 0))

    @storageapproach2_vector.setter
    def storageapproach2_vector(self, v):
        self.attributes['storageapproach2_vector'] = v

    # ----------------------------------
    # Functions for computing the complexed approach and retract
    # ----------------------------------

    @ property
    def tool_storage_approach_frame2(self):
        # type: () -> Frame
        """Compute the approach frame in wcf.
        Part of PlaceScrewdriverToStorageAction
        Step 2 (Slower)
        """
        vector_wcf = self.tool_storage_frame.to_world_coordinates(self.storageapproach2_vector)
        return self.tool_storage_frame.transformed(Translation.from_vector(vector_wcf.scaled(-1)))

    @ property
    def tool_storage_approach_frame1(self):
        # type: () -> Frame
        """Compute the approach frame in wcf.
        Part of PlaceScrewdriverToStorageAction
        Step 1 (Faster)
        """
        vector_wcf = self.tool_storage_frame.to_world_coordinates(self.storageapproach1_vector)
        return self.tool_storage_approach_frame2.transformed(Translation.from_vector(vector_wcf.scaled(-1)))

    @ property
    def tool_storage_retract_frame1(self):
        # type: () -> Frame
        """ Compute the retract frame in wcf
        Part of PickScrewdriverFromStorageAction
        Step 1 (Slower)
        """
        vector_wcf = self.tool_storage_frame.to_world_coordinates(self.storageapproach1_vector)
        return self.tool_storage_frame.transformed(Translation.from_vector(vector_wcf))

    @ property
    def tool_storage_retract_frame2(self):
        # type: () -> Frame
        """ Compute the retract frame in wcf
        Part of PickScrewdriverFromStorageAction
        Step 2 (Faster)
        """
        vector_wcf = self.tool_storage_frame.to_world_coordinates(self.storageapproach2_vector)
        return self.tool_storage_retract_frame1.transformed(Translation.from_vector(vector_wcf))

    # ----------------------------------
    # Functions for kinematic state
    # ----------------------------------

    # --------------------------------------------------------
    # State Setting Functions
    # --------------------------------------------------------

    def _set_kinematic_state(self, state_dict):
        self.screw_position = state_dict['screw_position']
        super(Screwdriver, self)._set_kinematic_state(state_dict)

    def _get_kinematic_state(self):
        state_dict = super(Screwdriver, self)._get_kinematic_state()
        state_dict.update({'screw_position': self.screw_position})
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

    def zero_screw(self):
        self.screw_position = 0

# --------------------------------------------------------
# Factory to construct Clamp
# --------------------------------------------------------


def SL1ScrewdriverFactory(
    name,
    type_name,
    gripper_jaw_position_min,
    gripper_jaw_position_max,
    tool_coordinate_frame,  # Ref to T0CF (TCF ~= TCP)
    tool_pick_up_frame,     # Ref to T0CF
    tool_storage_frame,     # Ref to WCF
    mesh_gripper_base,
    mesh_gripper_jaw_l,
    mesh_gripper_jaw_r,
    mesh_screw,
    gripper_jaw_l_axis,
    gripper_jaw_r_axis,
    approach_vector,
    detachretract_vector,
    storageapproach1_vector,
    storageapproach2_vector
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
    robot_model = Screwdriver(name,
                              type_name=type_name,
                              tool_coordinate_frame=tool_coordinate_frame,
                              tool_pick_up_frame=tool_pick_up_frame,
                              tool_storage_frame=tool_storage_frame,
                              gripper_jaw_position_min=gripper_jaw_position_min,
                              gripper_jaw_position_max=gripper_jaw_position_max,
                              approach_vector=approach_vector,
                              detachretract_vector=detachretract_vector,
                              storageapproach1_vector=storageapproach1_vector,
                              storageapproach2_vector=storageapproach2_vector,
                              )
    robot_model.gripper_jaw_limits = (gripper_jaw_position_min, gripper_jaw_position_max)

    robot_model.storageapproach1_vector = storageapproach1_vector   # This vector is ref to t0cf
    robot_model.storageapproach2_vector = storageapproach2_vector   # This vector is ref to t0cf

    #world_link = robot_model.add_link('world')
    gripper_base = robot_model.add_link('gripper_base', visual_meshes=mesh_gripper_base, collision_meshes=mesh_gripper_base)
    gripper_jaw_l = robot_model.add_link('gripper_jaw_l', visual_meshes=mesh_gripper_jaw_l, collision_meshes=mesh_gripper_jaw_l)
    gripper_jaw_r = robot_model.add_link('gripper_jaw_r', visual_meshes=mesh_gripper_jaw_r, collision_meshes=mesh_gripper_jaw_r)
    screw = robot_model.add_link('screwdriver_screw', visual_meshes=mesh_screw, collision_meshes=mesh_screw)

    #robot_model.add_joint('world_base_fixed_joint', Joint.FIXED, world_link, base_link)
    robot_model.add_joint('joint_gripper_jaw_l', Joint.PRISMATIC, gripper_base, gripper_jaw_l, axis=gripper_jaw_l_axis.unitized(), limit=robot_model.gripper_jaw_limits)
    robot_model.add_joint('joint_gripper_jaw_r', Joint.PRISMATIC, gripper_base, gripper_jaw_r, axis=gripper_jaw_r_axis.unitized(), limit=robot_model.gripper_jaw_limits)
    robot_model.add_joint('screwdriver_screw', Joint.REVOLUTE, gripper_base, screw, axis=tool_coordinate_frame.zaxis, limit=robot_model.gripper_jaw_limits)

    return robot_model


if __name__ == "__main__":
    c = Screwdriver('c1')
    print(c)
    print(c.data)
    print(c.current_configuration)
    pass
