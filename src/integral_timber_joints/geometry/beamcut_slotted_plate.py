# Beamcut_plane Object (inheret from Beamcut class)
#   Beamcut_plane object are used to model a beam termination resulting from a single saw cut.
#
#   The cut can either trim off the entire beam end (plane intersect with 4 refernce edges)
#   or it can trim only a portion of it (plane intersect with 2 or 3 reference edges)
#
#   Intersection of the

import math

import compas
from compas.datastructures import Mesh
from compas.geometry import Box, Polyhedron, Transformation, Shape, Cylinder
from compas.geometry.intersections import intersection_line_plane
from compas.geometry import Line, Point, Vector, Frame, Plane, transform_points


from integral_timber_joints.geometry.beamcut import Beamcut
from integral_timber_joints.geometry.utils import polyhedron_box_from_vertices, move_vertex_from_neighbor

try:
    from integral_timber_joints.geometry.beam import Beam
except:
    pass


class BeamcutPlateSlot(Beamcut):

    def __init__(self, on_beam_start=True, face_id=1, length=180, width=8, dowel_diameter=14, dowel_dist_from_top=60):
        # type: (bool, float, float, float, float, float) -> None
        self.on_beam_start = on_beam_start
        self.face_id = face_id
        self.length = length
        self.width = width
        self.dowel_diameter = dowel_diameter
        self.dowel_dist_from_top = dowel_dist_from_top

    @property
    def data(self):
        data = {
            'on_beam_start': self.on_beam_start,
            'face_id': self.face_id,
            'length': self.length,
            'width': self.width,
            'dowel_diameter': self.dowel_diameter,
            'dowel_dist_from_top': self.dowel_dist_from_top,
        }
        return data

    @data.setter
    def data(self, data):
        self.on_beam_start = data['on_beam_start']
        self.face_id = data.get('face_id', 1)
        self.length = data.get('length', 180)
        self.width = data.get('width', 8)
        self.dowel_diameter = data.get('dowel_diameter', 14)
        self.dowel_dist_from_top = data.get('dowel_dist_from_top', 60)

    def is_start(self):
        return self.on_beam_start

    def _construct_slot_and_one_dowel(self, beam):
        # type: (Beam) -> list[Shape]
        """Helper to construct one tetrahedral polyhedron at a corner.
        Returns the polyhedron on the reference_side_frame coordinates in WCF
        """

        OVERSIZE = 10.0
        vertices = []
        face_width = beam.get_face_width(self.face_id)
        face_height = beam.get_face_height(self.face_id)
        # Bottom Points
        vertices.append([- OVERSIZE, face_width/2 + self.width/2,  -face_height - OVERSIZE])
        vertices.append([- OVERSIZE, face_width/2 - self.width/2,  -face_height - OVERSIZE])
        vertices.append([self.length, face_width/2 - self.width/2,  -face_height - OVERSIZE])
        vertices.append([self.length, face_width/2 + self.width/2,  -face_height - OVERSIZE])
        # Top Points
        vertices.append([- OVERSIZE, face_width/2 + self.width/2,  OVERSIZE])
        vertices.append([- OVERSIZE, face_width/2 - self.width/2,  OVERSIZE])
        vertices.append([self.length, face_width/2 - self.width/2,  OVERSIZE])
        vertices.append([self.length, face_width/2 + self.width/2,  OVERSIZE])

        if self.on_beam_start:
            face_frame = beam.reference_side_wcf(self.face_id)
        else:
            face_frame = beam.ending_reference_side_wcf(self.face_id)

        T = Transformation.from_frame(face_frame)
        vertices = transform_points(vertices, T)
        slot_box = polyhedron_box_from_vertices(vertices)

        # Cylinder
        start_pt = Point(self.length - self.dowel_dist_from_top, OVERSIZE, -face_height / 2,)
        end_pt = Point(self.length - self.dowel_dist_from_top, face_width + OVERSIZE, -face_height / 2,)

        line = Line(start_pt, end_pt)
        cylinder = Cylinder.from_line_radius(line, self.dowel_diameter/2.0)
        cylinder.transform(T)

        return [slot_box, cylinder]

    # ###########################
    # Transformation of Extrinsic
    # ###########################

    def transform(self, transformation):
        # type: (Transformation) -> None
        """Transforming the beamcut object in WCF.
        Typically called by assembly.transform when initiated by user."""
        pass

    # #############
    # Boolean Shape
    # #############

    def get_feature_shapes(self, beam):
        # type: (Beam) -> list[Shape]
        """Compute the negative mesh volume of the joint.
        """
        # Construct boolean geometry
        shapes = self._construct_slot_and_one_dowel(beam)
        return shapes


if __name__ == "__main__":

    # Create beam in origin pointing X
    beam = Beam(Frame.worldXY(), 1000, 100, 150)

    beamcut = BeamcutPlateSlot()
    print(beamcut.get_feature_meshes(beam))
