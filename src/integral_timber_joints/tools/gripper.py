#from compas_fab.robots import Robot

from compas.robots import Link, Axis, Joint

from compas.geometry import Frame, Point, Vector
from compas.geometry import Transformation
# from compas.datastructures import Mesh

from integral_timber_joints.tools import Tool

class Gripper (Tool):
    def __init__(self, name,
        type_name = "Gripper",
        tool_coordinate_frame = None,
        tool_pick_up_frame  = None,
        tool_storage_frame = None,
        gripper_jaw_position_min = 0,
        gripper_jaw_position_max = 100):

        # Call Tool init
        if tool_coordinate_frame is None: tool_coordinate_frame = Frame.worldXY()
        Tool.__init__(self, name, type_name, tool_coordinate_frame)

        # --------------------------------------------------------
        # Extrinsic properities (Gripper Specific)
        # --------------------------------------------------------

        self._gripper_jaw_position = gripper_jaw_position_min # type: float

        # --------------------------------------------------------
        # Intrinsic properities (Gripper Specific)
        # --------------------------------------------------------

        self.gripper_jaw_limits = (gripper_jaw_position_min, gripper_jaw_position_max) # type: Tuple[float, float]

    # --------------------------------------------------------
    # Factory to construct Gripper
    # --------------------------------------------------------

    @ classmethod
    def ParallelGripperFactory(cls,
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
        approach_vector
        ):
        """ A Parallel gripper will have a base and two gripper jaw.
        Modelling guide
        ---------------
        The left jaws opens towards -Y direction.
        The right jaw opens towards +Y direction.
        The gripper opening should point towards +Z direction
        """
        robot_model = cls(name,type_name)
        robot_model.gripper_jaw_limits = (gripper_jaw_position_min, gripper_jaw_position_max)

        robot_model.tool_coordinate_frame = tool_coordinate_frame
        robot_model.tool_pick_up_frame = tool_pick_up_frame
        robot_model.tool_storage_frame = tool_storage_frame
        robot_model.approach_vector = approach_vector       # This vector is ref to t0cf
        # TODO: This and other paramters might need a clean up.

        #world_link = robot_model.add_link('world')
        gripper_base = robot_model.add_link('gripper_base', mesh_gripper_base)
        gripper_jaw_l = robot_model.add_link('gripper_jaw_l',mesh_gripper_jaw_l)
        gripper_jaw_r = robot_model.add_link('gripper_jaw_r', mesh_gripper_jaw_r)

        #robot_model.add_joint('world_base_fixed_joint', Joint.FIXED, world_link, base_link)
        robot_model.add_joint('joint_gripper_jaw_l', Joint.PRISMATIC, gripper_base, gripper_jaw_l, axis = [0,-1,0], limit = robot_model.gripper_jaw_limits)
        robot_model.add_joint('joint_gripper_jaw_r', Joint.PRISMATIC, gripper_base, gripper_jaw_r, axis = [0,1,0], limit = robot_model.gripper_jaw_limits)

        return robot_model

    # --------------------------------------------------------
    # State Setting Functions
    # --------------------------------------------------------

    @property
    def gripper_jaw_position(self):
        return self._gripper_jaw_position

    @gripper_jaw_position.setter
    def gripper_jaw_position(self, position):
        self._gripper_jaw_position = position
        self.get_joint_by_name('joint_gripper_jaw_l').position = position
        self.get_joint_by_name('joint_gripper_jaw_r').position = position

    def _set_kinematic_state(self, state_dict):
        self.gripper_jaw_position = state_dict['gripper_jaw_position']

    def _get_kinematic_state(self):
        return {'gripper_jaw_position' : self.gripper_jaw_position}

    # --------------------------------------------------------
    # Convinence Functions
    # --------------------------------------------------------

    def open_gripper(self):
        self.gripper_jaw_position = self.gripper_jaw_position_max

    def close_gripper(self):
        self.gripper_jaw_position = self.gripper_jaw_position_min

if __name__ == "__main__":
    g = Gripper('g1')
    print (g)
    pass
