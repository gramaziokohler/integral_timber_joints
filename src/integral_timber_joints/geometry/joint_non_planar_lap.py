# Joint_lap Object (inheret from Joint class)
#   Joint_lap object are used to model variable angle lap joint.

import math
from copy import deepcopy

import compas
from compas.datastructures import Mesh
from compas.geometry import intersection_line_line, intersection_line_plane, intersection_segment_segment, subtract_vectors, norm_vector, project_point_plane
from compas.geometry import is_point_infront_plane, distance_point_point, dot_vectors, distance_point_plane
from compas.geometry import Frame, Line, Plane, Point, Vector
from compas.geometry import Box, Polyhedron
from compas.geometry import Projection, Translation

from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.geometry.joint import Joint
from integral_timber_joints.geometry.utils import *
from integral_timber_joints.assembly.beam_assembly_method import BeamAssemblyMethod


try:
    from typing import Dict, List, Optional, Tuple, cast, Any

except:
    pass


class JointNonPlanarLap(Joint):
    """
    Joint class containing non planar lap joints of variable angles

    This joint object contains all the geometrical information including the beam_frame in WCF
    to recreate all the joint feature geometries. This joint is not WCF independent and must
    be updated when the hosting beam(s) are transformed.

    The accompany function JointNonPlanarLap.from_beam_beam_intersection()
    precomputes 8 joint-corners points (pt_jc[]) and determine which beam face are intersected.
    Refer to notes for the order of the points.
    During feature generation, pt_jc[4 to 7] are projected to the center_frame XY plane
    resulting in pt_proj[], used for generated the feature geometries.

    The `center_frame` is also pre-computed and located on the parting plane of the two joints,
    the origin of the center-frame is at the center of the screw hole.
    The Z axis vector is idential to the assemlby direction.

    At the moment only joints with 2 intact edge AND 2 broken edge on BOTH beams are supported.

    The joint assums the Z axis (of `beam_move_face_id`) of Moving Beam (`beam_move`) to be the assembly
    direction and automatically removes the hook feature on the Stationary Beam
    (`beam_stay`) along the assembly path when `axial_dot_product`` <> 0.

    The same joint data can be used on both Moving and Stationary side of the joint. The flag
    `is_joint_on_beam_move` should be set accordingly to return the correct features.

    """

    def __init__(self,
                 center_frame=None,  # type: Frame
                 thickness=None,  # type: float
                 beam_move_face_id=1,  # type: int
                 beam_stay_face_id=1,  # type: int
                 axial_dot_product=0.0,  # type: float
                 pt_jc=[],  # type: list[Point]
                 is_joint_on_beam_move=False,  # type: bool
                 name=None):
        """
        Parameters
        ----------
        `center_frame` -

        :param face_id:   int
        """
        self.center_frame = deepcopy(center_frame) # type: (Frame)
        self._thickness = thickness
        self.beam_move_face_id = beam_move_face_id
        self.beam_stay_face_id = beam_stay_face_id
        self.axial_dot_product = axial_dot_product
        self.pt_jc = deepcopy(pt_jc)
        self.is_joint_on_beam_move = is_joint_on_beam_move
        self.name = name

    @property
    def data(self):
        data = {
            'center_frame': self.center_frame,
            'thickness': self.thickness,
            'beam_move_face_id': self.beam_move_face_id,
            'beam_stay_face_id': self.beam_stay_face_id,
            'axial_dot_product': self.axial_dot_product,
            'pt_jc': self.pt_jc,
            'is_joint_on_beam_move': self.is_joint_on_beam_move,
            'name': self.name,
        }
        return data

    @classmethod
    def from_data(cls, data):
        """Construct a Joint object from structured data.
        This class method must be overridden by an inherited class.
        """
        joint = cls()
        joint.center_frame = data.get('center_frame', None)
        joint.thickness = data.get('thickness', 70)
        joint.beam_move_face_id = data.get('beam_move_face_id', 1)
        joint.beam_stay_face_id = data.get('beam_stay_face_id', 1)
        joint.axial_dot_product = data.get('axial_dot_product', 0.0)
        joint.pt_jc = data.get('pt_jc', [])
        joint.is_joint_on_beam_move = data.get('is_joint_on_beam_move', True)
        joint.name = data.get('name', "")
        return joint

    @property
    def face_id(self):
        if self.is_joint_on_beam_move:
            return self.beam_move_face_id
        else:
            return self.beam_stay_face_id

    @property
    def height(self):
        # First compute which edge is the longest (0-4 or 1-5 or 2-6 or 3-7)
        longest_edge = 0
        longest_length = 0
        for i in range(4):
            j = i + 4
            length = distance_point_point(self.pt_jc[i], self.pt_jc[j])
            if length > longest_length:
                longest_length = length
                longest_edge = i

        # Return the projection distance of longest edge
        if self.is_joint_on_beam_move:
            return distance_point_plane(self.pt_jc[longest_edge+4], Plane.from_frame(self.center_frame))
        else:
            return distance_point_plane(self.pt_jc[longest_edge], Plane.from_frame(self.center_frame))

    @property
    def thickness(self):
        return self._thickness

    @thickness.setter
    def thickness(self, value):
        self._thickness = value

    # #####################
    # Modifyable Parameters
    # #####################

    @property
    def parameter_keys(self):
        # type: () -> list[str]
        return ['thickness']

    def get_parameter(self, key):
        # type: (str) -> Any
        if key == 'thickness':
            return self.thickness
        raise KeyError("%s is invalid for JointHalfLap" % key)

    def set_parameter(self, key, value):
        # type: (str, Any) -> None
        if key == "thickness":
            # Offset `center_frame`
            offset_amount = value - self.thickness
            self.center_frame.point = self.center_frame.point + self.center_frame.normal.scaled(offset_amount)
            # Change thickness parameter
            self.thickness = value

            return
        raise KeyError("%s is invalid for JointHalfLap" % key)

    # #####################
    # Joint Shape
    # #####################

    def get_feature_shapes(self, BeamRef):
        # type: (Beam) -> list[Mesh]
        """Compute the negative shapes of the joint.

        Parameters
        ----------
        BeamRef -> integral_timber_joint.geometry.Beam
            The Beam object this joint is attached to

        Returns
        -------
        object
            A compas.Mesh


        """
        OVERSIZE = max(50.0, self.thickness)

        # Project Point [4 to 7] on cheek plane (pt_p)
        center_plane = Plane.from_frame(self.center_frame)
        P = Projection.from_plane(center_plane)
        pt_p = [Point(* self.pt_jc[i+4]).transformed(P) for i in range(4)]

        # Intersect Points (pt_x)
        corner_lines = [Line(self.pt_jc[i], self.pt_jc[i+4]) for i in range(4)]
        pt_x = [intersection_line_plane(line, center_plane) for line in corner_lines]

        # Joint corners alias
        pt_c = self.pt_jc

        # Create solid negative geometry
        if self.is_joint_on_beam_move:
            # Only one feature mesh
            vertices = []
            if self.axial_dot_product > 0:
                vertices.append(pt_x[0])
                vertices.append(pt_x[1])
                vertices.append(pt_p[2])
                vertices.append(pt_p[3])
            else:
                vertices.append(pt_p[0])
                vertices.append(pt_p[1])
                vertices.append(pt_x[2])
                vertices.append(pt_x[3])
            vertices.extend(pt_c[4:8])
            # Add offset
            move_vertex_from_neighbor(vertices, [0, 3, 7, 4], [1, 2, 6, 5], OVERSIZE)
            move_vertex_from_neighbor(vertices, [1, 2, 6, 5], [0, 3, 7, 4], OVERSIZE)
            move_vertex_from_neighbor(vertices, [4, 5, 6, 7], [0, 1, 2, 3], OVERSIZE)
            shape = polyhedron_box_from_vertices(vertices)
            return [shape]

        else:
            vertices = []
            # Degenerate case where angle is very close to zero.
            # Boolean geometry is just a simple box
            if abs(self.axial_dot_product) < 1e-4:
                vertices.append(pt_c[0])
                vertices.append(pt_c[1])
                vertices.append(pt_c[2])
                vertices.append(pt_c[3])
                vertices.append(pt_x[0])
                vertices.append(pt_x[1])
                vertices.append(pt_x[2])
                vertices.append(pt_x[3])
                # Add offset
                move_vertex_from_neighbor(vertices, [0, 1, 2, 3], [4, 5, 6, 7], OVERSIZE)
                move_vertex_from_neighbor(vertices, [0, 1, 4, 5], [3, 2, 7, 6], OVERSIZE)
                move_vertex_from_neighbor(vertices, [3, 2, 7, 6], [0, 1, 4, 5], OVERSIZE)
                shape = polyhedron_box_from_vertices(vertices)
                return [shape]

            # In most case a 5 point polygon for a single boolean.
            # In the past this was done as two simplier geometry
            # but cgal just cannot boolean when meshes are coplanar

            if self.axial_dot_product > 0:
                vertices.append(pt_p[2])
                vertices.append(pt_c[6])
                vertices.append(pt_c[2])
                vertices.append(pt_c[1])
                vertices.append(pt_x[1])
                vertices.append(pt_p[3])
                vertices.append(pt_c[7])
                vertices.append(pt_c[3])
                vertices.append(pt_c[0])
                vertices.append(pt_x[0])
            else:
                vertices.append(pt_p[0])
                vertices.append(pt_c[4])
                vertices.append(pt_c[0])
                vertices.append(pt_c[3])
                vertices.append(pt_x[3])
                vertices.append(pt_p[1])
                vertices.append(pt_c[5])
                vertices.append(pt_c[1])
                vertices.append(pt_c[2])
                vertices.append(pt_x[2])

            # Patching a problem where in a degenerate case on stationary beam
            # when center plane is above reference edge. The corner points pt_c[0-3]
            # can be above the intersected point pt_x[0-3]

            def patch_corner_points_beyond_center_frame(i, j, offset=1):
                """ Vertice[i] is the point to be moved back to behind center frame"""
                if Vector.from_start_end(vertices[i], vertices[j]).dot(self.center_frame.normal) < 0:
                    vertices[i] = [a+b for a, b in zip(vertices[j], self.center_frame.normal.scaled(- offset))]

            patch_corner_points_beyond_center_frame(2, 0)
            patch_corner_points_beyond_center_frame(3, 4)

            patch_corner_points_beyond_center_frame(7, 5)
            patch_corner_points_beyond_center_frame(8, 9)

            # Add OVERSIZE offset
            move_vertex_from_neighbor(vertices, [1, 6], [0, 5], OVERSIZE)
            move_vertex_from_neighbor(vertices, [2, 7], [3, 8], OVERSIZE)
            move_vertex_from_neighbor(vertices, [2, 7, 3, 8], [1, 6, 4, 9], OVERSIZE)
            move_vertex_from_neighbor(vertices, [3, 8, 4, 9], [2, 7, 0, 5], OVERSIZE)
            shape = polyhedron_box_from_vertices(vertices)
            return [shape]

    def get_clamp_frames(self, beam):
        # type: (Beam) -> list[Frame]
        """Compute the possible frames where the clamp can be attached.
        The clamp frame is located at the opposite side of the face_id.

        Orgin of the frame is located on the surface of that face, at the center point of the joint.
        X Axis is along the length of the beam. (2 possible orientations)
        Z axis is pointing into the beam.

        Parameters
        ----------
        beam : Beam
            Beam Object

        Returns
        -------
        list(Frame)
            Frames in WCF
        """
        # The clamp frame locate at the opposite

        reference_side_wcf = beam.reference_side_wcf((self.face_id - 1 + 2) % 4 + 1)
        origin = project_point_plane(self.center_frame.point, Plane.from_frame(reference_side_wcf))

        forward_clamp = Frame(origin, reference_side_wcf.xaxis, reference_side_wcf.yaxis.scaled(-1))
        backward_clamp = Frame(origin, reference_side_wcf.xaxis.scaled(-1), reference_side_wcf.yaxis.scaled(1))
        return [forward_clamp, backward_clamp]

    def get_assembly_direction(self, beam):
        '''
        Returns the only possible assembly direction.
        '''
        # print "Dist%s" % self.distance
        face_frame = beam.get_face_plane(self.face_id)
        return face_frame.normal.scaled(-1 * beam.get_face_height(self.face_id))

    def assembly_tool_types(self, beam_assembly_method):
        # type: (BeamAssemblyMethod) -> list[str]
        """Returns a list of clamps types that can assemble this joint
        """

        if beam_assembly_method == BeamAssemblyMethod.SCREWED_WITH_GRIPPER:
            return ['SL1', 'SL1_G200']
        elif beam_assembly_method == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
            return ['SL1_G200', 'SL1']        # Preferentially requesting SL1_G200 (this is likely to be assigned to the gripping joint)
        else:
            print("Warning: Joint Non planar cannot be assembled with Assemble Method %s" % BeamAssemblyMethod.value_to_names_dict[beam_assembly_method])
            return ['SL1']

    def get_joint_center_at_solid_side(self, beam):
        # type: (Beam) -> Point

        # Center line for intersection
        center_line = Line(self.center_frame.point, self.center_frame.point + self.center_frame.zaxis)

        if self.is_joint_on_beam_move:
            beam_move = beam
            far_ref_side_m = beam_move.reference_side_wcf((self.beam_move_face_id + 1) % 4 + 1)
            screw_line_start = intersection_line_plane(center_line, Plane.from_frame(far_ref_side_m))
            return screw_line_start
        else:
            beam_stay = beam
            far_ref_side_s = beam_stay.reference_side_wcf((self.beam_stay_face_id + 1) % 4 + 1)
            screw_line_end = intersection_line_plane(center_line, Plane.from_frame(far_ref_side_s))
            return screw_line_end

    @classmethod
    def from_beam_beam_intersection(cls, beam_stay, beam_move, thickness=None, joint_face_id_stay=None, joint_face_id_move=None):
        # type: (Beam, Beam, float, int, int) -> tuple[JointNonPlanarLap, JointNonPlanarLap, Line]
        ''' Compute the intersection between two beams.

        `beam_stay` must be the earlier beam in assembly sequence
        `beam_move` must be the later beam in assembly sequence

        Returns a tuple of [JointNonPlanarLap, JointNonPlanarLap] when a valid joint pair can be found.

        Note. At the moment the center lines of two beams must not directly intersect.
        There need to be some offset

        `thickness` is the thickness of the lap joint on the moving_beam.

        If `joint_face_id_stay` or `joint_face_id_move` is provided, those faces will be used.

        If no intersection can be found or the two beam are not coplanar,
        Returns a tuple of (None, None, None)
        '''
        # * Compute intersections and check for no-joint scenarios.
        # Find center line intersection and distances
        ctl_m = beam_move.get_center_line()
        ctl_s = beam_stay.get_center_line()
        cp_m, cp_s = intersection_line_line(ctl_m, ctl_s)

        # If center lines are parallel, intersection result will be None:
        if cp_m is None or cp_s is None:
            return (None, None, None)

        # Find which beam face to be facing each other
        # We use the intersection points of the two lines as a guide and compare with
        v_ms = subtract_vectors(cp_s, cp_m)
        v_sm = subtract_vectors(cp_m, cp_s)
        if norm_vector(v_ms) < 1e-5:
            # If the intersection is too close, we do not process it. TODO: Make it auto select faces.
            return (None, None, None)

        normal_dot_product_m = [dot_vectors(beam_move.reference_side_wcf(i + 1).zaxis, v_ms) for i in range(4)]
        normal_dot_product_s = [dot_vectors(beam_stay.reference_side_wcf(i + 1).zaxis, v_sm) for i in range(4)]
        if joint_face_id_stay is None:
            joint_face_id_stay = normal_dot_product_s.index(max(normal_dot_product_s)) + 1
        if joint_face_id_move is None:
            joint_face_id_move = normal_dot_product_m.index(max(normal_dot_product_m)) + 1

        # Find the reference side frame and reference edges
        ref_side_m = beam_move.reference_side_wcf(joint_face_id_move)
        ref_side_s = beam_stay.reference_side_wcf(joint_face_id_stay)
        ref_edge_m0 = beam_move.reference_edge_wcf(joint_face_id_move)
        ref_edge_m1 = beam_move.reference_edge_wcf(joint_face_id_move-1)
        # s0 and s1 are defined as which ever is more negative to ref_side_m.xaxis
        ref_edge_s0 = beam_stay.reference_edge_wcf(joint_face_id_stay)
        ref_edge_s1 = beam_stay.reference_edge_wcf(joint_face_id_stay-1)
        if dot_vectors(ref_side_s.yaxis, ref_side_m.xaxis) < 0.0:
            ref_edge_s0, ref_edge_s1 = ref_edge_s1, ref_edge_s0

        # Compute 8 intersection points
        ref_plane_m0 = Plane(ref_edge_m0.start, ref_side_m.yaxis)
        ref_plane_m1 = Plane(ref_edge_m1.start, ref_side_m.yaxis)
        ref_plane_s0 = Plane(ref_edge_s0.start, ref_side_s.yaxis)
        ref_plane_s1 = Plane(ref_edge_s1.start, ref_side_s.yaxis)

        lpx_pts = []
        lpx_pts.append(intersection_line_plane(ref_edge_s0, ref_plane_m0))
        lpx_pts.append(intersection_line_plane(ref_edge_s0, ref_plane_m1))
        lpx_pts.append(intersection_line_plane(ref_edge_s1, ref_plane_m1))
        lpx_pts.append(intersection_line_plane(ref_edge_s1, ref_plane_m0))
        lpx_pts.append(intersection_line_plane(ref_edge_m0, ref_plane_s0))
        lpx_pts.append(intersection_line_plane(ref_edge_m1, ref_plane_s0))
        lpx_pts.append(intersection_line_plane(ref_edge_m1, ref_plane_s1))
        lpx_pts.append(intersection_line_plane(ref_edge_m0, ref_plane_s1))

        # Check if any of the first 8 points lie on the move_beam body. If not, there is no true intersection.
        ref_plane_m_top = Plane(ref_edge_m0.start, ref_side_m.zaxis.inverted())
        some_point_has_contact = any([is_point_infront_plane(point, ref_plane_m_top) for point in lpx_pts])
        if not some_point_has_contact:
            return (None, None, None)

        # Check if all of the first 8 points lie outside the stay_beam start or end.
        # If all of them are outside, there is no true intersection.
        ref_plane_s_start = Plane.from_frame(beam_stay.reference_side_wcf(5))
        all_points_outside_start = all([is_point_infront_plane(point, ref_plane_s_start) for point in lpx_pts])
        ref_plane_s_end = Plane.from_frame(beam_stay.reference_side_wcf(6))
        all_points_outside_end = all([is_point_infront_plane(point, ref_plane_s_end) for point in lpx_pts])
        if all_points_outside_start or all_points_outside_end:
            return (None, None, None)

        # Check if all of the first 8 points lie outside the move_beam start or end.
        # If all of them are outside, there is no true intersection.
        ref_plane_s_start = Plane.from_frame(beam_move.reference_side_wcf(5))
        all_points_outside_start = all([is_point_infront_plane(point, ref_plane_s_start) for point in lpx_pts])
        ref_plane_s_end = Plane.from_frame(beam_move.reference_side_wcf(6))
        all_points_outside_end = all([is_point_infront_plane(point, ref_plane_s_end) for point in lpx_pts])
        if all_points_outside_start or all_points_outside_end:
            return (None, None, None)

        # * Joint Thickness hard coded default:
        if thickness is None:
            thickness = 70

        # * Compute joint center (center on lap cheek plane, on centerline intersection)
        # Screw hole axis passes through this hole.
        # Projection of centerline and find intersection
        ref_side_m_projection_plane = Plane.from_frame(ref_side_m)
        P = Projection.from_plane(ref_side_m_projection_plane)
        ctl_m_projected = ctl_m.transformed(P)
        ctl_s_projected = ctl_s.transformed(P)
        cp_m_projected, cp_s_projected = intersection_line_line(ctl_m_projected, ctl_s_projected)
        # Translating the frame to the cheek plane
        offset_amount = thickness - beam_move.get_face_height(joint_face_id_move)
        T = Translation.from_vector(ref_side_m.zaxis.unitized().scaled(offset_amount))
        joint_center_frame = Frame(Point(* cp_m_projected).transformed(T), ref_side_m.xaxis, ref_side_m.yaxis)

        # Precompute this dot product for deciding the hook triangle direction
        axial_dot_product = dot_vectors(ref_side_s.zaxis, ref_side_m.xaxis)

        # Construct joint objects
        joint_m = JointNonPlanarLap(joint_center_frame, thickness, joint_face_id_move, joint_face_id_stay, axial_dot_product, lpx_pts,
                                    is_joint_on_beam_move=True, name='%s-%s' % (beam_move.name, beam_stay.name))
        joint_s = JointNonPlanarLap(joint_center_frame, thickness, joint_face_id_move, joint_face_id_stay, axial_dot_product, lpx_pts,
                                    is_joint_on_beam_move=False, name='%s-%s' % (beam_move.name, beam_stay.name))

        # * Compute Screw line
        center_line = Line(joint_center_frame.point, joint_center_frame.point + joint_center_frame.zaxis)

        far_ref_side_m = beam_move.reference_side_wcf((joint_face_id_move + 1) % 4 + 1)
        far_ref_side_s = beam_stay.reference_side_wcf((joint_face_id_stay + 1) % 4 + 1)

        screw_line_start = intersection_line_plane(center_line, Plane.from_frame(far_ref_side_m))
        screw_line_end = intersection_line_plane(center_line, Plane.from_frame(far_ref_side_s))

        screw_line = Line(screw_line_start, screw_line_end)
        return (joint_s, joint_m, screw_line)


if __name__ == "__main__":
    import os
    import tempfile

    import compas

    # #Test to create Joint_90lap object. Serialize and deserialize.
    # #j.data and q.data should have the same value
    # #Create Joint object
    # from compas.geometry import Frame
    # joint = Joint_90lap(180,1,50,100,100)
    # print (joint.data)
    # #Save Joint to Json
    # joint.to_json(os.path.join(tempfile.gettempdir(), "joint.json"),pretty=True)
    # #Load saved Joint Object
    # loaded_joint = Joint_90lap.from_json(os.path.join(tempfile.gettempdir(), "joint.json"))
    # #Assert that the two Joint objects are different objects
    # assert (joint is not loaded_joint)
    # print("Test 1: Comparing two beam data dictionary:")
    # assert (joint.data == loaded_joint.data)
    # if (joint.data == loaded_joint.data):
    #     print("Correct")
    # else:
    #     print("Incorrect")
    # print (joint.data)
