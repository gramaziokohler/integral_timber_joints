from compas.datastructures import Mesh
from compas.geometry import Frame, Transformation

from integral_timber_joints.tools.tool_changer import ToolChanger


class RobotWrist (ToolChanger):
    """ This robot wrist is a list of convex mesh models for sollision detection.
    The meshes should be modeled in the reference frame of the robot flange.
    """

    def __init__(self,
                 name,
                 collision_mesh=None,
                 type_name="RobotWrist"
                 ):

        # Call Tool init
        super(RobotWrist, self).__init__(name, Frame.worldXY(), None, collision_mesh, type_name)

    @property
    def root_link_name(self):
        """Name of the Link object in Robot Model"""
        return 'robot_wrist'
