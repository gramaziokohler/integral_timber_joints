from copy import deepcopy

from compas.datastructures import Mesh
from compas.geometry import distance_point_point, dot_vectors, intersection_line_line, intersection_line_plane, intersection_segment_segment, subtract_vectors
from compas.geometry import Cylinder, Line, Plane, Point, Vector, Box, Circle
from compas.geometry import Projection, Translation

from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.geometry.joint import Joint
from integral_timber_joints.geometry.utils import *


class Screw_SL(object):
    """
    Screw Class containing a geometry constructor to create cylindrical holes for screws.

    This joint object contains all the geometrical information including a frame in WCF
    to recreate all the joint feature geometries. This Screw is not WCF independent and must
    be updated when the hosting beam / joint are transformed.

    This specific screw is used for SL1 screwdrivers. The screw has three diameters at different locations.
    Including the diameter used by the SL1 screwdriver, this translates to four diameters
    used to create the drill hole. The center of the frame is located on the cheek face of the lap joint.

    The origin of the `center_frame` is at the center of the screw hole.
    The Z axis vector is idential to the assemlby direction, which points to the smaller diameters.

    `joint_move_thickness` is the thickness of the joint on the moving side (screw head).
    This should be set equal to the joint's thickness setting.
    `joint_stay_thickness` is a rough number that should be larger than the thickness of the stationary side (screw thread)
    `screw_total_length` is the length of the screw head, determins the length of the largest diameter hole.
    `screw_total_length` is the total length of the screw. Starting from the screw head, the hole diameter will transition
    to the fourth (smallest) hole size after this distance.

    The same Screw data can be used on both Moving and Stationary side of the joint. The flag
    `is_joint_on_beam_move` should be set accordingly to return the correct features.

    """

    def __init__(self,
                 center_frame=None,  # type: Frame
                 joint_move_thickness=70,  # type: float
                 joint_stay_thickness=100,  # type: float
                 is_screw_on_beam_move=False,  # type: bool
                 screw_head_length=8,  # type: float
                 screw_total_length=110,  # type: float
                 screw_diameters=(32, 19, 16, 10)  # type: tuple[float, float, float, float]
                 ):

        self.center_frame = deepcopy(center_frame)
        self.joint_move_thickness = joint_move_thickness
        self.joint_stay_thickness = joint_stay_thickness
        self.is_screw_on_beam_move = is_screw_on_beam_move
        self.screw_head_length = screw_head_length
        self.screw_total_length = screw_total_length
        self.screw_diameters = screw_diameters

    def get_feature_meshes(self, BeamRef):
        # type: (Beam) -> list[Mesh]
        """Compute the negative mesh volume of the joint.
        Parameters
        ----------
        BeamRef -> integral_timber_joint.geometry.Beam
            The Beam object this joint is attached to

        Returns
        -------
        object
            A compas.Mesh

        Note
        ----
        The self.mesh is updated with the new mesh

        """
        OVERSIZE = 10.0

        def cylinder_mesh (plane, diameter, height, offset):
            circle = Circle(plane, diameter / 2)
            cylinder = Cylinder(circle, height)
            offset_vector = plane.normal.unitized().scaled(height / 2 + offset)
            cylinder.transform(Translation.from_vector(offset_vector))
            return Mesh.from_vertices_and_faces(* cylinder.to_vertices_and_faces(u=16))

        # Two larger diameter cylinders on beam side
        plane = Plane.from_frame(self.center_frame)
        if self.is_screw_on_beam_move:
            # Larger diameter cylinder / Screw Head
            height = self.screw_head_length + OVERSIZE
            offset = - (self.joint_move_thickness + OVERSIZE)
            cyl1 = cylinder_mesh (plane, self.screw_diameters[0], height, offset)

            # Smaller diameter cylinder / Through hole
            height = self.joint_move_thickness + OVERSIZE
            offset = - self.joint_move_thickness
            cyl2 = cylinder_mesh (plane, self.screw_diameters[1], height, offset)

            return [cyl1, cyl2]

        else:
            # Larger diameter cylinder / Left-in-Screw Thread
            height = self.screw_total_length - self.joint_move_thickness + OVERSIZE
            offset = - OVERSIZE
            cyl1 = cylinder_mesh (plane, self.screw_diameters[2], height, offset)

            # Smaller diameter cylinder / Pull Screw Thread
            height = self.joint_stay_thickness + OVERSIZE
            offset = 0
            cyl2 = cylinder_mesh (plane, self.screw_diameters[3], height, offset)

            return [cyl1, cyl2]

