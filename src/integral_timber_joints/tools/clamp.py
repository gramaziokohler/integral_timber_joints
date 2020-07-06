#from compas_fab.robots import Robot

from compas.robots import Link, Axis, Joint

from compas.geometry import Frame, Point, Vector
from compas.geometry import Transformation
# from compas.datastructures import Mesh

from integral_timber_joints.tools import Tool

class Clamp (Tool):
    def __init__(self, name,
        type_name = "Clamp",
        tool_coordinate_frame = None,
        tool_pick_up_frame  = None,
        tool_storage_frame = None,
        gripper_jaw_position_min = 0,
        gripper_jaw_position_max = 100,
        clamp_jaw_position_min = 0,
        clamp_jaw_position_max = 100):

        # Call Tool init
        if tool_coordinate_frame is None: tool_coordinate_frame = Frame.worldXY()
        Tool.__init__(self, name, type_name, tool_coordinate_frame)

        # --------------------------------------------------------
        # Extrinsic properities / state (Gripper Specific)
        # --------------------------------------------------------

        self._gripper_jaw_position = gripper_jaw_position_min # type: float
        self._clamp_jaw_position = clamp_jaw_position_min # type: float

        # --------------------------------------------------------
        # Intrinsic properities (Gripper Specific)
        # --------------------------------------------------------

        self.gripper_jaw_limits = (gripper_jaw_position_min, gripper_jaw_position_max) # type: Tuple[float, float]
        self.clamp_jaw_limits = (clamp_jaw_position_min, clamp_jaw_position_max) # type: Tuple[float, float]

    # --------------------------------------------------------
    # State Setting Functions
    # --------------------------------------------------------

    @property
    def gripper_jaw_position(self):
        return self._gripper_jaw_position

    @gripper_jaw_position.setter
    def gripper_jaw_position(self, position):
        self._gripper_jaw_position = position
        for joint in self.get_configurable_joints():
            if joint.name.startswith('joint_gripper_'):
                joint.position = position
        # self.get_joint_by_name('joint_gripper_jaw_l').position = position
        # self.get_joint_by_name('joint_gripper_jaw_r').position = position

    @property
    def clamp_jaw_position(self):
        return self._clamp_jaw_position

    @clamp_jaw_position.setter
    def clamp_jaw_position(self, position):
        self._clamp_jaw_position = position
        for joint in self.get_configurable_joints():
            if joint.name.startswith('joint_clamp_'):
                joint.position = position
        # self.get_joint_by_name('joint_clamp_jaw_l').position = position
        # self.get_joint_by_name('joint_clamp_jaw_r').position = position

    def _set_kinematic_state(self, state_dict):
        self.gripper_jaw_position = state_dict['gripper_jaw_position']
        self.clamp_jaw_position = state_dict['clamp_jaw_position']

    def _get_kinematic_state(self):
        return {'gripper_jaw_position' : self.gripper_jaw_position, 'clamp_jaw_position' : self.clamp_jaw_position}

    @property
    def jaw_blocking_vectors_in_wcf(self):
        T = Transformation.from_frame(self.current_frame)
        return [vec.transformed(T) for vec in self.jaw_block_vectors]

    @property
    def jaw_clearance_vectors_in_wcf(self):
        T = Transformation.from_frame(self.current_frame)
        return self.jaw_clearance_vector.transformed(T)

    # --------------------------------------------------------
    # Convinence Functions
    # --------------------------------------------------------

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
    robot_model = Clamp(name,type_name)
    robot_model.gripper_jaw_limits = (gripper_jaw_position_min, gripper_jaw_position_max)
    robot_model.clamp_jaw_limits = (clamp_jaw_position_min, clamp_jaw_position_max)

    robot_model.tool_coordinate_frame = tool_coordinate_frame
    robot_model.tool_pick_up_frame = tool_pick_up_frame
    robot_model.tool_storage_frame = tool_storage_frame
    robot_model.approach_vector = approach_vector               # This vector is ref to t0cf
    robot_model.detachretract1_vector = detachretract1_vector   # This vector is ref to t0cf
    robot_model.detachretract2_vector = detachretract2_vector   # This vector is ref to t0cf

    #world_link = robot_model.add_link('world')
    gripper_base = robot_model.add_link('gripper_base', mesh_gripper_base)
    gripper_jaw_l = robot_model.add_link('gripper_jaw_l',mesh_gripper_jaw_l)
    gripper_jaw_r = robot_model.add_link('gripper_jaw_r', mesh_gripper_jaw_r)
    clamp_jaw_l = robot_model.add_link('clamp_jaw_l',mesh_clamp_jaw_l)
    clamp_jaw_r = robot_model.add_link('clamp_jaw_r', mesh_clamp_jaw_r)

    #robot_model.add_joint('world_base_fixed_joint', Joint.FIXED, world_link, base_link)
    robot_model.add_joint('joint_gripper_jaw_l', Joint.PRISMATIC, gripper_base, gripper_jaw_l, axis = [0,-1,0], limit = robot_model.gripper_jaw_limits)
    robot_model.add_joint('joint_gripper_jaw_r', Joint.PRISMATIC, gripper_base, gripper_jaw_r, axis = [0,1,0], limit = robot_model.gripper_jaw_limits)
    robot_model.add_joint('joint_clamp_jaw_l', Joint.PRISMATIC, gripper_jaw_l, clamp_jaw_l, axis = [0,0,1], limit = robot_model.clamp_jaw_limits)
    robot_model.add_joint('joint_clamp_jaw_r', Joint.PRISMATIC, gripper_jaw_r, clamp_jaw_r, axis = [0,0,1], limit = robot_model.clamp_jaw_limits)

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
    ):
    """ A Parallel gripper will have a base and two gripper jaw.
    Modelling guide
    ---------------
    The gripper jaws opens towards +Z direction of tool_coordinate_frame.
    The clamp jaws opens towards +Z direction of tool_coordinate_frame.

    The joints name should start with 'joint_gripper_' or 'joint_clamp_'
    """
    robot_model = Clamp(name,type_name)
    robot_model.gripper_jaw_limits = (gripper_jaw_position_min, gripper_jaw_position_max)
    robot_model.clamp_jaw_limits = (clamp_jaw_position_min, clamp_jaw_position_max)

    robot_model.tool_coordinate_frame = tool_coordinate_frame
    robot_model.tool_pick_up_frame = tool_pick_up_frame
    robot_model.tool_storage_frame = tool_storage_frame
    robot_model.approach1_vector = approach1_vector               # This vector is ref to t0cf
    robot_model.approach2_vector = approach2_vector               # This vector is ref to t0cf
    robot_model.detachretract1_vector = detachretract1_vector   # This vector is ref to t0cf
    robot_model.detachretract2_vector = detachretract2_vector   # This vector is ref to t0cf

    #world_link = robot_model.add_link('world')
    gripper_base = robot_model.add_link('gripper_base', base_mesh)
    gripper_jaw = robot_model.add_link('gripper_jaw',gripper_jaw_mesh)
    clamp_jaw = robot_model.add_link('clamp_jaw',clamp_jaw_mesh)

    #robot_model.add_joint('world_base_fixed_joint', Joint.FIXED, world_link, base_link)
    robot_model.add_joint('joint_gripper_jaw', Joint.PRISMATIC, gripper_base, gripper_jaw, axis = tool_coordinate_frame.zaxis, limit = robot_model.gripper_jaw_limits)
    robot_model.add_joint('joint_clamp_jaw', Joint.PRISMATIC, gripper_base, clamp_jaw, axis = tool_coordinate_frame.zaxis, limit = robot_model.clamp_jaw_limits)

    # A constant list of vectors (ref t0cf) where the beam-in-jaw is blocked by the jaw.
    # Vector direction is the contact surface normal from the in-jaw beam side
    # (alternatively) Vector direction is the direction where the beam cannot move towards.
    robot_model.jaw_block_vectors = [tool_coordinate_frame.zaxis.copy(), tool_coordinate_frame.zaxis.scaled(-1), Vector(0,0,-1)]
    # A directional vector describing how much distance the beam-in-jaw has to move to clear the jaw if moved out.
    robot_model.jaw_clearance_vector = tool_coordinate_frame.yaxis.scaled(100)
    return robot_model


if __name__ == "__main__":
    c = Clamp('c1')
    print (c)
    pass
