from compas.geometry import Frame, Transformation

from integral_timber_joints.tools import Tool


class ToolChanger (Tool):
    def __init__(self,
                 name,
                 tool_coordinate_frame,     #type: Frame # The frame that connects to the tool
                 collision_mesh,
                 type_name="ToolChanger"
                 ):

        # Call Tool init
        super(ToolChanger, self).__init__(name, type_name, tool_coordinate_frame)

        self.collision_mesh = collision_mesh

    def draw_visuals(self):
        # type: () -> List[Mesh]
        """ Returns the COMPAS Mesh of the ToolChanger
        in the location determined by its internal state.
        """
        # Return empty list if is_visible is False
        if not self.is_visible:
            return []

        # Transform the visualization mesh to current_frame.
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.current_frame)
        visual_meshes = [self.collision_mesh.transformed(T)]
        return visual_meshes
