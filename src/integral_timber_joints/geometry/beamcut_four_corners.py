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
from compas.geometry import Box, Polyhedron, Transformation, Shape
from compas.geometry.intersections import intersection_line_plane
from compas.geometry import Line, Point, Vector, Frame, Plane


from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.geometry.beamcut import Beamcut
from integral_timber_joints.geometry.utils import polyhedron_box_from_vertices, move_vertex_from_neighbor


class BeamcutFourCorners(Beamcut):

    def __init__(self, on_beam_start=True, dist_x=200, dist_y=21.1, dist_z=36.6):
        # type: (bool, float, float, float) -> None
        self.on_beam_start = on_beam_start
        self.dist_x = dist_x
        self.dist_y = dist_y
        self.dist_z = dist_z

    @property
    def data(self):
        data = {
            'on_beam_start': self.on_beam_start,
            'dist_x': self.dist_x,
            'dist_y': self.dist_y,
            'dist_z': self.dist_z,
        }
        return data

    @data.setter
    def data(self, data):
        self.on_beam_start = data['on_beam_start']
        self.dist_x = data['dist_x']
        self.dist_y = data['dist_y']
        self.dist_z = data['dist_z']

    def is_start(self):
        return self.on_beam_start

    def _construct_tetra_polyhedron(self, reference_side_frame):
        # type: (Frame) -> Polyhedron
        """Helper to construct one tetrahedral polyhedron at a corner.
        Returns the polyhedron on the reference_side_frame coordinates in WCF
        """
        OVERSIZE = 10.0
        point_o = Point(-OVERSIZE, -OVERSIZE, OVERSIZE)
        point_x = Point(self.dist_x, 0, 0)
        point_y = Point(0, self.dist_y, 0)
        point_z = Point(0, 0, - self.dist_z)
        vertices = [point_o, point_x, point_y, point_z]
        move_vertex_from_neighbor(vertices, [1, 1, 2, 2, 3, 3], [2, 3, 1, 3, 1, 2], OVERSIZE)  # Expand triangle
        faces = [[0, 1, 2], [0, 3, 1], [0, 2, 3], [3, 2, 1]]
        polyhedron = Polyhedron(vertices, faces)
        polyhedron.transform(Transformation.from_change_of_basis(reference_side_frame, Frame.worldXY()))
        return polyhedron

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

        Note
        ----
        The self.mesh is updated with the new mesh

        """

        # Get reference sides from Beam
        if self.on_beam_start:
            tet_frames = [beam.reference_side_wcf(i) for i in range(1, 5)]
        else:
            tet_frames = [beam.ending_reference_side_wcf(i) for i in range(1, 5)]

        # Construct tets and return
        shapes = []
        for frame in tet_frames:
            shapes.append(self._construct_tetra_polyhedron(frame))
        return shapes


if __name__ == "__main__":

    # Create beam in origin pointing X
    beam = Beam(Frame.worldXY(), 1000, 100, 150)

    beamcut = BeamcutFourCorners(True)
    print(beamcut.get_feature_meshes(beam))
