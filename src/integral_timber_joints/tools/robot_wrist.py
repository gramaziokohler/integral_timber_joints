from compas.geometry import Frame, Transformation
from compas.datastructures import Mesh
from integral_timber_joints.tools import Tool


class RobotWrist (object):
    """ This robot wrist is a list of convex mesh models for sollision detection.
    The meshes should be modeled in the reference frame of the robot flange.
    """
    def __init__(self,
                 collision_meshs    #type: list[Mesh]
                 ):

        self.collision_meshs = collision_meshs
        self.current_frame = Frame.worldXY()
        self.is_visible = True

    def draw_visuals(self):
        # type: () -> list[Mesh]
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
