from compas.geometry import Frame, Transformation

from integral_timber_joints.tools import Tool
from compas_fab.robots import Configuration

class ToolChanger (Tool):
    def __init__(self,
                 name,
                 tool_coordinate_frame,     #type: Frame # The frame that connects to the tool
                 collision_mesh = None,
                 type_name = "ToolChanger"
                 ):

        # Call Tool init
        super(ToolChanger, self).__init__(name, type_name, tool_coordinate_frame)

        self.collision_mesh = collision_mesh

    # --------------------------------------------------------------
    # The collision mesh is stored in a Link object
    # --------------------------------------------------------------

    # @property
    # def root_link_name(self):
    #     """Name of the Link object in Robot Model"""
    #     return 'toolchanger_base'

    @property
    def collision_mesh(self):
        toolchanger_base_link = self.get_link_by_name('toolchanger_base')
        return toolchanger_base_link.collision.geometry

    @collision_mesh.setter
    def collision_mesh(self, collision_mesh):
        """This can be either one or a list of meshes"""
        from compas.robots import Axis, Joint, Link
        if collision_mesh is not None:
            # Collision mesh stored in both visual and collision objects
            gripper_base = self.add_link('toolchanger_base', visual_meshes = collision_mesh, collision_meshes = collision_mesh)
            self.root = gripper_base
            self._create(self.root, Transformation())

    # --------------------------------------------------------------
    # This ToolChanger has no Kinematics
    # --------------------------------------------------------------

    @property
    def current_configuration(self):
        return Configuration([], [], [])
