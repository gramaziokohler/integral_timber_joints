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
from compas.geometry import Box, Frame, Plane
from compas.geometry.primitives.vector import Vector
from compas.geometry.primitives.point import Point
from compas.geometry.primitives.line import Line

from compas.geometry import distance_point_point, intersection_line_line
from compas.geometry.intersections import intersection_line_plane

from integral_timber_joints.geometry.beamcut import Beamcut
from integral_timber_joints.geometry.beam import Beam

class Beamcut_plane(Beamcut):

    def __init__(self, plane_ocf):
        # type: (bool, Plane) -> None
        self.plane_ocf = plane_ocf          # type: (Plane) # Plane with normal pointing to the off cut. Ref to beam local coordinate frame.
        self.mesh = None

    def is_start (self):
        normal = self.plane_ocf.normal
        return normal.x < 0.0

    def get_feature_mesh(self, BeamRef):
        # type: (Beam) -> Mesh
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

        # Get reference edges from Beam
        # We cannot use reference_edge_ocf() because that does not incluse the boolean safe offset.

        ref_edges_ocf = []
        ref_edges_ocf.append(Line(Point(0, -OVERSIZE, -OVERSIZE), Point(BeamRef.length, -OVERSIZE, -OVERSIZE)))
        ref_edges_ocf.append(Line(Point(0,  BeamRef.height + OVERSIZE, -OVERSIZE), Point(BeamRef.length,  BeamRef.height + OVERSIZE, -OVERSIZE)))
        ref_edges_ocf.append(Line(Point(0,  BeamRef.height + OVERSIZE, BeamRef.width + OVERSIZE), Point(BeamRef.length,  BeamRef.height + OVERSIZE, BeamRef.width + OVERSIZE)))
        ref_edges_ocf.append(Line(Point(0, -OVERSIZE, BeamRef.width + OVERSIZE), Point(BeamRef.length, -OVERSIZE, BeamRef.width + OVERSIZE)))

        # Compute reference edge and plane intersection
        edge_intersect_points_ocf = [intersection_line_plane(line, self.plane_ocf) for line in ref_edges_ocf]

        
        # To construct the 8 corners of a box:
        # With respect to the local Z axis, the vertices of the bottom
        # face are listed first in clockwise direction, starting at the bottom left corner.
        # The vertices of the top face are listed in counterclockwise direction.
        vertices = []

        # Construct corners of the box - flat face
        if self.is_start():
            flat_face_X = min([-OVERSIZE] + [(point[0] - OVERSIZE) for point in edge_intersect_points_ocf])
            flat_face_points = [Point(flat_face_X, line.start.y, line.start.z) for line in ref_edges_ocf]
            vertices = edge_intersect_points_ocf + [flat_face_points[i] for i in [0,3,2,1]]
        else:
            flat_face_X = max([BeamRef.length + OVERSIZE] + [(point[0] + OVERSIZE) for point in edge_intersect_points_ocf])
            flat_face_points = [Point(flat_face_X, line.start.y, line.start.z) for line in ref_edges_ocf]
            vertices = flat_face_points + [edge_intersect_points_ocf[i] for i in [0,3,2,1]]  # Note order is different because box top vs bottom direction is different
 
        box = Box(Frame.worldXY(), 1, 1, 1)
        boolean_box_mesh = Mesh.from_vertices_and_faces(vertices, box.faces)
        boolean_box_mesh = BeamRef.frame.to_world_coordinates(boolean_box_mesh)

        # Draw boolean box and assign to self.mesh
        self.mesh = boolean_box_mesh
        return self.mesh

def Beamcut_plane_from_beam_plane_wcf_intersection(beam, plane, auto_extension):
    pass

if __name__ == "__main__":

    # Create beam in origin pointing X
    beam = Beam(Frame.worldXY(), 1000, 100, 150)

    beamcut = Beamcut_plane(Plane(Point(0,0,0), Vector(-1, -0.1, -0.1)))
    print (beamcut.get_feature_mesh(beam))

    beamcut = Beamcut_plane(Plane(Point(800,0,0), Vector(1, -0.1, -0.1)))
    print (beamcut.get_feature_mesh(beam))

    

