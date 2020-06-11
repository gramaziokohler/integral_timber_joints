from abc import abstractmethod
from copy import deepcopy

from compas.geometry import Frame, Transformation
from compas.robots import RobotModel
from compas_fab.artists import BaseRobotArtist
from compas_fab.robots import Configuration


class CompasRobotArtist(BaseRobotArtist):
    def transform(self, compas_mesh, transformation):
        compas_mesh.transform(transformation)

    def draw_geometry(self, geometry, name=None, color=None):
        return deepcopy(geometry)  # I think a copy would be required, to avoid screwing up stuff from the source model


class Tool (RobotModel):
    """ A tool is a RobotModel describing a robotic tool.
    Typically being attached to a robot or placed on a tool changer.
    A tool is capable of kinematic components and can be modelled and simulated.

    We inherete from the RobotModel class for its ability to perform forward-kinematics.

    The tool (it's meshes and other reference frame) should be modeled at the origin.
    """

    def __init__(self,
                 name,
                 type_name,
                 tool_coordinate_frame,      # Ref to T0CF (TCF ~= TCP)
                 tool_pick_up_frame=None,  # Ref to T0CF
                 tool_storage_frame=None,  # Ref to WCF
                 ):

        # RobotModel.__init__(self, name)
        super(Tool, self).__init__(name)

        self.name = name
        self.type_name = type_name

        # --------------------------------------------------------
        # Extrinsic properities of the tool
        # --------------------------------------------------------

        self.is_visible = True  # type: bool                # If the tool is in scene
        self.is_attached = False  # type: bool               # If the tool is currently attached to the robot
        self._current_frame = Frame.worldXY()  # type: Frame # Current location of the tool in the WCF

        # --------------------------------------------------------
        # Intrinsic properities of the tool
        # --------------------------------------------------------

        """ Tool tip in reference to T0CP.
        This is the offset from the T0CP to the Gripping Pose
        I used to call it placement_frame but now deprecated"""
        self.tool_coordinate_frame = tool_coordinate_frame  # type: Frame

        """ Tool changer approach frame in reference to T0CP.
        This is the position of the robot before picking up the tool via toolchanger.
        I used to use the concept of approach_vector but now deprecated"""
        self.tool_pick_up_frame = tool_pick_up_frame  # type: Frame

        """ Storage location in wcf. If no toolchange is intended, can be set to None """
        self.tool_storage_frame = tool_storage_frame  # type: Frame

    # --------------------------------------------------------
    # Frame to Frame Transformation function
    # --------------------------------------------------------

    @property
    def current_frame(self):
        """ Getting the current frame T0CP directly"""
        return self._current_frame.copy()

    @current_frame.setter
    def current_frame(self, frame):
        """ Setting the current frame T0CP directly"""
        self._current_frame = frame.copy()

    def set_current_frame_from_tcp(self, tcp_frame):
        # type: (Frame) -> Frame
        """ Computing the current_frame by supplying where the tool tip should go.

        Side Effect
        -----------
        self.current_frame is updated
        """
        T = Transformation.from_frame_to_frame(self.tool_coordinate_frame, tcp_frame)
        base_frame = Frame.worldXY().transformed(T)
        self.current_frame = base_frame
        return base_frame.copy()

    @property
    def transformation_from_tcf_to_t0cf(self):
        return self.transformation_from_t0cf_to_tcf.inverse()

    @property
    def transformation_from_t0cf_to_tcf(self):
        return Transformation.from_frame(self.tool_coordinate_frame)

    def tool_pick_up_frame_in_wcf(self, current_frame):
        return self.tool_pick_up_frame.transformed(Transformation.from_frame_to_frame(Frame.worldXY(), current_frame))

    def tool_coordinate_frame_in_wcf(self, current_frame):
        return self.tool_coordinate_frame.transformed(Transformation.from_frame_to_frame(Frame.worldXY(), current_frame))
    # --------------------------------------------------------
    # State Setting Functions
    # --------------------------------------------------------

    @abstractmethod
    def _get_kinematic_state(self):
        """ Get the kinematic state of all the joints as a dictionary."""
        pass

    @abstractmethod
    def _set_kinematic_state(self, state_dict):
        """ Set the kinematic state from a dictionary."""
        pass

    def get_state(self):
        state_dict = self._get_kinematic_state()  # type: Dict[str, float]
        state_dict["current_frame"] = self._current_frame.copy()
        state_dict['is_visible'] = self.is_visible
        state_dict['is_attached'] = self.is_attached
        return state_dict

    def set_state(self, state_dict):
        self._set_kinematic_state(state_dict)
        self._current_frame = state_dict["current_frame"]
        self.is_visible = state_dict['is_visible']
        self.is_attached = state_dict['is_attached']

    # --------------------------------------------------------
    # Visualization Function
    # --------------------------------------------------------

    def draw_visuals(self):
        # type: () -> List[Mesh]
        """ Returns the COMPAS Mesh of the underlying RobotModel
        in the location determined by its internal state.
        """
        # Return empty list if is_visible is False
        if not self.is_visible:
            return []

        # Create a new robot artist is necessary
        artist = CompasRobotArtist(self)

        # Grab all joint values and create a Configuration (Artist want it)
        joint_names = [j.name for j in self.get_configurable_joints()]
        joint_positions = [j.position for j in self.get_configurable_joints()]
        robot_configuration = Configuration.from_prismatic_and_revolute_values(joint_positions, [])

        # Artist performs update of the transformations
        artist.update(robot_configuration, joint_names)
        # print (joint_positions)

        # Post-transforming the visualization mesh according to world joint.
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.current_frame)
        visual_meshes = [mesh.transformed(T) for mesh in artist.draw_visual()]
        return visual_meshes

    # --------------------------------------------------------
    # String Representations
    # --------------------------------------------------------

    def ToString(self):
        """Function for Grasshopper Tool Tip"""
        return "%s (%s)" % (self.type_name, self.name)

    def __str__(self):
        """ For mirroring the ToString function for print() """
        return self.ToString()

    def copy(self):
        return deepcopy(self)


if __name__ == "__main__":
    # t = Tool('T1', 'tool')
    # print (t)
    pass
