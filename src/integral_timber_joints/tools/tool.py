from copy import deepcopy

from compas.geometry import Frame, Transformation
from compas.datastructures import Mesh

try:
    from compas.robots.model.robot import RobotModel
    from compas.robots.model.tool import ToolModel
except:
    from compas.robots import RobotModel, ToolModel

from compas.robots.base_artist import BaseRobotModelArtist
from compas_fab.robots import Configuration

try:
    from typing import Dict, List, Optional, Tuple

    from integral_timber_joints.process.state import ObjectState
except:
    pass


class CompasRobotArtist(BaseRobotModelArtist):
    def transform(self, compas_mesh, transformation):
        compas_mesh.transform(transformation)

    def draw_geometry(self, geometry, name=None, color=None):
        return deepcopy(geometry)  # I think a copy would be required, to avoid screwing up stuff from the source model


class Tool (ToolModel):
    """ A tool is a RobotModel describing a robotic tool.
    Typically being attached to a robot or placed on a tool changer.
    A tool in this repo's context is capable of kinematic movements
    with movable components that can be modelled and simulated.

    We inherete from the RobotModel class for its ability to perform forward-kinematics.

    The tool (it's meshes and other reference frame) should be modeled where the root
    of the tool (T0CF) is at the origin. In the presence of a tool changer in between,
    assume the tool changer's tip is the root.

    tool_storage_frame is the location (in WCF) where the tool is stored. Assuming the
    tool is moved from the tool_coordinate_frame to the tool_storage_frame
    """

    def __init__(self,
                 name,
                 type_name,
                 tool_coordinate_frame,      # Ref to T0CF (TCF ~= TCP)
                 tool_pick_up_frame=None,  # Ref to T0CF
                 tool_storage_frame=None,  # Ref to WCF
                 ):

        # RobotModel.__init__(self, name)
        super(Tool, self).__init__(None, tool_coordinate_frame, name=name)

        """ attribute dictionary allowing easy storage of custom data.
        primitive data types and compas data will survive serialization. 
        """
        self.attributes = {}

        """ remove the hardcoded attached_tool_link link in the ToolModel
        """
        hardcoded_link = self.get_link_by_name('attached_tool_link')
        if hardcoded_link is not None:
            self.links.remove(hardcoded_link)

        self.type_name = type_name

        # --------------------------------------------------------
        # Extrinsic properities of the tool
        # --------------------------------------------------------

        self.is_visible = True  # type: bool            # If the tool is in scene
        self.is_attached = False  # type: bool          # If the tool is currently attached to the robot
        self.current_frame = Frame.worldXY()            # type: Frame # Current location of the tool in the WCF

        # --------------------------------------------------------
        # Intrinsic properities of the tool
        # --------------------------------------------------------

        """ Tool changer approach frame in reference to T0CP.
        This is the position of the robot before picking up the tool via toolchanger.
        I used to use the concept of approach_vector but now deprecated"""
        self.tool_pick_up_frame = tool_pick_up_frame  # type: Frame

        """ Storage location in wcf. If no toolchange is intended, can be set to None """
        self.tool_storage_frame = tool_storage_frame  # type: Frame

    @property
    def data(self):
        """Returns the data dictionary that represents the tool.
        """
        data = super(Tool, self)._get_data()
        # Serialization of the attributes dictionary
        data['attributes'] = self.attributes
        return data

    @data.setter
    def data(self, data):
        # def _set_data(self, data):
        super(Tool, self)._set_data(data)
        # Deserialization of the attributes dictionary
        self.attributes.update(data.get('attributes') or {})

    # --------------------------------------------------------------
    # Routing attributes from ToolModel
    # --------------------------------------------------------------

    @property
    def tool_coordinate_frame(self):
        """ Getting the current frame T0CP from ToolModel.frame attribute"""
        return self.frame.copy()

    @tool_coordinate_frame.setter
    def tool_coordinate_frame(self, frame):
        """ Setting the current frame T0CP directly"""
        self.frame = frame.copy()

    # --------------------------------------------------------------
    # Functions to get and set attributes from attributes dictionary.
    # --------------------------------------------------------------

    @property
    def current_frame(self):
        """ Getting the current frame T0CP directly"""
        return self.attributes.get('current_frame', Frame.worldXY())

    @current_frame.setter
    def current_frame(self, frame):
        """ Setting the current frame T0CP directly"""
        self.attributes['current_frame'] = frame.copy()

    @property
    def type_name(self):
        """ Getting the type_name.
        Different instance of the same clamp should have the same type_name but different name."""
        return self.attributes.get('type_name', 'UnknownToolType')

    @type_name.setter
    def type_name(self, value):
        """ Setting the type_name
        Different instance of the same clamp should have the same type_name but different name."""
        self.attributes['type_name'] = value

    @property
    def is_visible(self):
        """ Returns if the object should be visible. """
        return self.attributes.get('is_visible', True)

    @is_visible.setter
    def is_visible(self, value):
        """ Set if the object should be visible."""
        self.attributes['is_visible'] = value

    @property
    def is_attached(self):
        """ Returns if the object is attached to the robot """
        return self.attributes.get('is_attached', True)

    @is_attached.setter
    def is_attached(self, value):
        """ Set if the object is attached to the robot """
        self.attributes['is_attached'] = value

    @property
    def tool_pick_up_frame(self):
        """ Get the tool_pick_up_frame frame. In reference to the T0CP
        This is the position of the robot before picking up the tool via toolchanger."""
        return self.attributes.get('tool_pick_up_frame', None)

    @tool_pick_up_frame.setter
    def tool_pick_up_frame(self, frame):
        """ Set the tool_pick_up_frame frame. In reference to the T0CP
        This is the position of the robot before picking up the tool via toolchanger."""
        if frame is None:
            self.attributes['tool_pick_up_frame'] = None
        else:
            self.attributes['tool_pick_up_frame'] = frame.copy()

    @property
    def tool_storage_frame(self):
        """ Get the tool_storage_frame frame. In reference to WCF.
        This is the position where the tool is stored."""
        return self.attributes.get('tool_storage_frame', None)

    @tool_storage_frame.setter
    def tool_storage_frame(self, frame):
        """ Set the tool_storage_frame frame. In reference to WCF.
        This is the position where the tool is stored."""
        if frame is None:
            self.attributes['tool_storage_frame'] = None
        else:
            self.attributes['tool_storage_frame'] = frame.copy()

    @property
    def current_configuration(self):
        """Gets the current Configuration of the joints in the underlying RobotModel
        This can be used to update the artist or update the robot.
        """
        # This should be implemented by child class.
        raise NotImplementedError

    @property
    def current_tcf(self):
        """ Getting the current tool coordinate frame (tool tip) in WCF"""
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.current_frame)
        return self.tool_coordinate_frame.transformed(T)

    # --------------------------------------------------------
    # Frame to Frame Transformation function
    # --------------------------------------------------------

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

    def _get_kinematic_state(self):
        # type: () -> dict[str, float]
        """ Get the kinematic state of all the joints as a dictionary."""
        pass

    def _set_kinematic_state(self, state_dict):
        # type: (dict[str, float]) -> None
        """ Set the kinematic state from a dictionary."""
        pass

    def get_state(self):
        state_dict = self._get_kinematic_state()  # type: Dict[str, float]
        state_dict["current_frame"] = self.current_frame.copy()
        state_dict['is_visible'] = self.is_visible
        state_dict['is_attached'] = self.is_attached
        return state_dict

    def set_state(self, state_dict):
        self._set_kinematic_state(state_dict)
        self.current_frame = state_dict["current_frame"]
        self.is_visible = state_dict['is_visible']
        self.is_attached = state_dict['is_attached']

    # --------------------------------------------------------
    # Visualization Function
    # --------------------------------------------------------

    def draw_visual(self, artist=None):
        # type: (BaseRobotModelArtist) -> list[any[Mesh, RhinoMesh]]
        """ Returns the COMPAS Mesh of the underlying RobotModel
        in the location determined by its internal state.

        If provided with a type of BaseRobotModelArtist, this result will be painted by that artist.
        Otherwise compas_meshes will be returned.

        If the specific Artist draw_visual() returns geometry,
        they will be returned.
        """
        # Return empty list if is_visible is False
        if not self.is_visible:
            return []

        # Create a new robot artist is necessary
        if artist is None:
            artist = CompasRobotArtist(self)

        # Grab all joint values and create a Configuration (Artist want it)
        root_transformation = Transformation.from_frame(self.current_frame)
        joint_state = self.current_configuration.joint_dict

        # Artist performs update of the transformations
        artist._transform_link_geometry(self.root, root_transformation, True)
        artist._update(self, joint_state, parent_transformation=root_transformation)

        result = artist.draw_visual()

        return result

    def draw_state(self, state, robot_world_transform=None):
        # type: (ObjectState, Optional[Transformation]) -> List[Mesh]
        """Returns the collision meshe(s) of the Tool for a given state.

        Two transformation is performed:
        - `state.current_frame` if not `None` (typically it is None for EnvironmentMesh)
        - `robot_world_transform` if not `None`
        """
        # Save previous frame for reverting changes
        previous_frame = self.current_frame
        previous_kinematic_state = self._get_kinematic_state()

        # Apply changes to current_frame and kinematic_state
        assert state.current_frame is not None
        self.current_frame = state.current_frame
        
        if state.kinematic_config is not None:
            self._set_kinematic_state(state.kinematic_config) 

        # Draw meshes at the frame and revert the change to self.current_frame
        transformed_meshes = self.draw_visual() # type: List[Mesh]

        # Return the tool to previous state
        self.current_frame = previous_frame
        self._set_kinematic_state(previous_kinematic_state)

        # Finalize transformation to robot world
        if robot_world_transform is not None:
            assert isinstance(robot_world_transform, Transformation)
            [mesh.transform(robot_world_transform) for mesh in transformed_meshes]

        return transformed_meshes
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
