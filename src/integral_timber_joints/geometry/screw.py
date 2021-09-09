from copy import deepcopy

from compas.data import Data
from compas.datastructures import Mesh
from compas.geometry import distance_point_point, dot_vectors, intersection_line_line, intersection_line_plane, intersection_segment_segment, subtract_vectors
from compas.geometry import Cylinder, Line, Plane, Point, Vector, Box, Circle, Shape
from compas.geometry import Projection, Translation

from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.geometry.joint import Joint
from integral_timber_joints.geometry.utils import *

OVERSIZE = 10.0


def cylinder_with_offset(plane, diameter, height, offset):
    circle = Circle(plane, diameter / 2)
    cylinder = Cylinder(circle, height)
    offset_vector = plane.normal.unitized().scaled(height / 2 + offset)
    cylinder.transform(Translation.from_vector(offset_vector))
    return cylinder


def cylinder_mesh(plane, diameter, height, offset):
    cylinder = cylinder_with_offset(plane, diameter, height, offset)
    return Mesh.from_vertices_and_faces(* cylinder.to_vertices_and_faces(u=16))


class Screw_SL(Data):
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

    `head_side_thickness` is the thickness of the joint on the moving side (screw head).
    This should be set equal to the joint's thickness setting.
    `thread_side_thickness` is a rough number that should be larger than the thickness of the stationary side (screw thread)
    `screw_total_length` is the length of the screw head, determins the length of the largest diameter hole.
    `screw_total_length` is the total length of the screw. Starting from the screw head, the hole diameter will transition
    to the fourth (smallest) hole size after this distance.

    The same Screw data can be used on both Moving and Stationary side of the joint. The flag
    `is_joint_on_beam_move` should be set accordingly to return the correct features.

    """

    def __init__(
        self,
        name="SL40_40",
        center_line=None,  # type: Line
        head_side_thickness=50,  # type: float
        screw_lengths=(8, 40, 80, 120),  # type: tuple[float, float, float, float]
        screw_diameters=(32, 19, 16, 10),  # type: tuple[float, float, float, float]
        valid = True,
    ):
        """
        The extrinsic parameter of the screw are dependent on both beams across the joint.
        Refered to as `head_side` and `thread_side`.
        The default intrinsic values are set equal to the S40_40 screw.

        The screw have 4 features:
        - Screw Head
        - Screw Shank
        - Screw Thread
        - Screw Pull Screw (This is not actual part of the screw but a dist/diameter where the pull screw engage)

        Creating one screw object is sufficient to obtain both head side and thread side boolean features.

        `center_line` : from head to thread side along the full thickness of joint
        `head_side_thickness` : thickness of the joint on head side
        `screw_lengths` : accumulated distance of where the features end, measured from the start of screw head.
        `screw_diameters` : diameter of the the cylindrical features.

        """
        self.name = name
        self.center_line = deepcopy(center_line)
        self.head_side_thickness = head_side_thickness

        self.screw_lengths = screw_lengths
        self.screw_diameters = screw_diameters
        self.valid = valid

    @property
    def data(self):
        data = {
            'name': self.name,
            'center_line': self.center_line,
            'head_side_thickness': self.head_side_thickness,
            'screw_lengths': self.screw_lengths,
            'screw_diameters': self.screw_diameters,
            'valid': self.valid,
        }
        return data

    @data.setter
    def data(self, data):
        self.name = data.get('name', None)
        self.center_line = data.get('center_line', None)
        self.head_side_thickness = data.get('head_side_thickness', 50)
        self.screw_lengths = data.get('screw_lengths', (8, 40, 80, 120))
        self.screw_diameters = data.get('screw_diameters', (32, 19, 16, 10))
        self.valid = data.get('valid', False)

    @property
    def total_length(self):
        return self.center_line.length

    @property
    def thread_side_thickness(self):
        return self.total_length - self.head_side_thickness

    def get_head_side_feature_shapes(self):
        # type: () -> list[Shape]
        """Returns the negative features of the screw.
        (Screw Head Side)
        """
        features = []

        # Larger diameter cylinder / Screw Head
        start_pt = self.center_line.point_from_start(-OVERSIZE)
        end_pt = self.center_line.point_from_start(self.screw_lengths[0])
        features.append(Cylinder.from_line_radius(Line(start_pt, end_pt), self.screw_diameters[0]/2))

        # Smaller diameter cylinder / Through hole
        start_pt = self.center_line.point_from_start(0)
        end_pt = self.center_line.point_from_start(self.screw_lengths[2])
        features.append(Cylinder.from_line_radius(Line(start_pt, end_pt), self.screw_diameters[1]/2))

        return features

    def get_thread_side_feature_shapes(self):
        # type: () -> list[Shape]
        """Returns the negative features of the screw.
        (Screw Thread Side)
        """
        features = []

        # Shank extending to thread side
        if self.head_side_thickness < self.screw_lengths[1]:
            start_pt = self.center_line.point_from_start(0)
            end_pt = self.center_line.point_from_start(self.screw_lengths[1])
            features.append(Cylinder.from_line_radius(Line(start_pt, end_pt), self.screw_diameters[1]/2))

        # Larger diameter cylinder / Screw Thread
        start_pt = self.center_line.point_from_start(self.screw_lengths[1]-OVERSIZE)
        end_pt = self.center_line.point_from_start(self.screw_lengths[2])
        features.append(Cylinder.from_line_radius(Line(start_pt, end_pt), self.screw_diameters[2]/2))

        # Smaller diameter cylinder / Pull Screw Thread
        start_pt = self.center_line.point_from_start(self.screw_lengths[2]-OVERSIZE)
        end_pt = self.center_line.point_from_start(self.screw_lengths[3]+OVERSIZE)
        features.append(Cylinder.from_line_radius(Line(start_pt, end_pt), self.screw_diameters[3]/2))

        # Final pull screw relief for extra long joint.
        if self.total_length > self.screw_lengths[3]:
            start_pt = self.center_line.point_from_start(self.screw_lengths[3])
            end_pt = self.center_line.point_from_start(self.total_length+OVERSIZE)
            features.append(Cylinder.from_line_radius(Line(start_pt, end_pt), self.screw_diameters[2]/2))

        return features

    def get_head_side_feature_meshes(self):
        # type: (Beam) -> list[Mesh]
        """Compute the negative mesh volume of the joint. (Screw Head Side)

        Returns
        -------
        list[Mesh]
            A list of compas.Mesh
        """

        shapes = self.get_head_side_feature_shapes()
        return [Mesh.from_vertices_and_faces(* shape.to_vertices_and_faces(u=16)) for shape in shapes]

    def get_thread_side_feature_meshes(self):
        # type: (Beam) -> list[Mesh]
        """Compute the negative mesh volume of the joint. (Screw Thread Side)

        Returns
        -------
        list[Mesh]
            A list of compas.Mesh
        """

        shapes = self.get_thread_side_feature_shapes()
        return [Mesh.from_vertices_and_faces(* shape.to_vertices_and_faces(u=16)) for shape in shapes]

    def flip(self):
        #type: () -> None
        """Flips the screw center line.
        The head_side_thickness is changed to original_total_length - original_head_side_thickness

        This flip is only meaningful to planar joints.
        Flipping Non planar joints require a totally different screw line vector.
        Consider recreating a new Screw entirely.
        """
        new_thickness = self.total_length - self.head_side_thickness
        self.head_side_thickness = new_thickness

        new_line = Line(self.center_line.end, self.center_line.start)
        self.center_line = new_line

    @classmethod
    def AutoLength_Factory(cls, center_line=None, head_side_thickness=50.0):
        # type: (Line, float) -> Screw_SL
        """Creates one of the two available screws based on head_side_thickness and total length.
        AL40_40 and SL60_50

        If neither screws are possible due to over dimension.
        """
        # SL40_40
        if head_side_thickness >= 30.0 and head_side_thickness <= 60.0 and center_line.length >= 99.0:
            return Screw_SL(
                name="SL40_40",
                center_line=center_line,  # type: Line
                head_side_thickness=head_side_thickness,  # type: float
                screw_lengths=(8, 40, 80, 120),  # type: tuple[float, float, float, float]
                screw_diameters=(32, 19, 16, 10),  # type: tuple[float, float, float, float]
            )
        # SL60_50
        elif head_side_thickness >= 30.0 and head_side_thickness <= 90.0 and center_line.length >= 130.0:
            return Screw_SL(
                name="SL60_50",
                center_line=center_line,  # type: Line
                head_side_thickness=head_side_thickness,  # type: float
                screw_lengths=(8, 60, 110, 150),  # type: tuple[float, float, float, float]
                screw_diameters=(32, 19, 16, 10),  # type: tuple[float, float, float, float]
            )
        # * Invalid placeholder is created such there is a geometry for creating boolean and for UI selection
        # Create in invalid paramter place holder
        elif head_side_thickness <= 60.0:
            return Screw_SL(
                name="SL40_40_Invalid",
                center_line=center_line,  # type: Line
                head_side_thickness=head_side_thickness,  # type: float
                screw_lengths=(8, 40, 80, 120),  # type: tuple[float, float, float, float]
                screw_diameters=(32, 19, 16, 10),  # type: tuple[float, float, float, float]
                valid = False,
            )
        # Create in invalid paramter place holder
        else:
            return Screw_SL(
                name="SL60_50_Invalid",
                center_line=center_line,  # type: Line
                head_side_thickness=head_side_thickness,  # type: float
                screw_lengths=(8, 60, 110, 150),  # type: tuple[float, float, float, float]
                screw_diameters=(32, 19, 16, 10),  # type: tuple[float, float, float, float]
                valid = False,
            )
        return None
