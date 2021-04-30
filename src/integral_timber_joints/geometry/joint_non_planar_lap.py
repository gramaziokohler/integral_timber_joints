# Joint_lap Object (inheret from Joint class)
#   Joint_lap object are used to model variable angle lap joint.

import math
from copy import deepcopy

import compas
from compas.datastructures import Mesh
from compas.geometry import intersection_line_line, intersection_line_plane, intersection_segment_segment, subtract_vectors, norm_vector
from compas.geometry import is_point_in_halfspace, distance_point_point, dot_vectors
from compas.geometry import Frame, Line, Plane, Point, Vector
from compas.geometry import Box
from compas.geometry import Projection, Translation

from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.geometry.screw import Screw_SL
from integral_timber_joints.geometry.joint import Joint
from integral_timber_joints.geometry.utils import *


class JointNonPlanarLap(Joint):
    """
    Joint class containing non planar lap joints of variable angles

    This joint object contains all the geometrical information including the beam_frame in WCF
    to recreate all the joint feature geometries. This joint is not WCF independent and must
    be updated when the hosting beam(s) are transformed.

    The accompany function non_planar_lap_joint_from_beam_beam_intersection()
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
                 has_screw=False,  # type: bool
                 name=None):
        """
        Parameters
        ----------
        `center_frame` -

        :param face_id:   int
        """
        self.center_frame = deepcopy(center_frame)
        self.thickness = thickness
        self.beam_move_face_id = beam_move_face_id
        self.beam_stay_face_id = beam_stay_face_id
        self.axial_dot_product = axial_dot_product
        self.pt_jc = deepcopy(pt_jc)
        self.is_joint_on_beam_move = is_joint_on_beam_move
        self.has_screw = has_screw
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
            'has_screw': self.has_screw,
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
        joint.thickness = data.get('center_frame', 70)
        joint.beam_move_face_id = data.get('beam_move_face_id', 1)
        joint.beam_stay_face_id = data.get('beam_stay_face_id', 1)
        joint.axial_dot_product = data.get('axial_dot_product', 0.0)
        joint.pt_jc = data.get('pt_jc', [])
        joint.is_joint_on_beam_move = data.get('is_joint_on_beam_move', True)
        joint.has_screw = data.get('has_screw', False)
        joint.name = data.get('name', "")
        return joint

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

        # Project Point [4 to 7] on cheek plane (pt_p)
        center_plane = Plane.from_frame(self.center_frame)
        P = Projection.from_plane(center_plane)
        pt_p = [Point(* self.pt_jc[i+4]).transformed(P) for i in range(4)]

        # Intersect Points (pt_x)
        corner_lines = [Line(self.pt_jc[i], self.pt_jc[i+4]) for i in range(4)]
        pt_x = [intersection_line_plane(line, center_plane) for line in corner_lines]

        # Joint corners alias
        pt_c = self.pt_jc


        # Create the screw hole
        screw_features = []
        if self.has_screw:
            screw_features = Screw_SL(self.center_frame, self.thickness, 150, self.is_joint_on_beam_move).get_feature_meshes(BeamRef)

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
            mesh = mesh_box_from_vertices(vertices)
            # Add offset
            mesh_move_vertex_from_neighbor(mesh, [0, 3, 7, 4], [1, 2, 6, 5], OVERSIZE)
            mesh_move_vertex_from_neighbor(mesh, [1, 2, 6, 5], [0, 3, 7, 4], OVERSIZE)
            mesh_move_vertex_from_neighbor(mesh, [4, 5, 6, 7], [0, 1, 2, 3], OVERSIZE)
            return [mesh] + screw_features

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
                mesh = polygon_box_from_vertices(vertices)
                mesh_move_vertex_from_neighbor(mesh, [0, 1, 2, 3], [4, 5, 6, 7], OVERSIZE)
                mesh_move_vertex_from_neighbor(mesh, [0, 1, 4, 5], [3, 2, 7, 6], OVERSIZE)
                mesh_move_vertex_from_neighbor(mesh, [3, 2, 7, 6], [0, 1, 4, 5], OVERSIZE)
                return [mesh] + screw_features

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

            mesh = polygon_box_from_vertices(vertices)
            mesh_move_vertex_from_neighbor(mesh, [1, 6], [0, 5], OVERSIZE)
            mesh_move_vertex_from_neighbor(mesh, [2, 7], [3, 8], OVERSIZE)
            mesh_move_vertex_from_neighbor(mesh, [2, 7, 3, 8], [1, 6, 4, 9], OVERSIZE)
            mesh_move_vertex_from_neighbor(mesh, [3, 8, 4, 9], [2, 7, 0, 5], OVERSIZE)
            return [mesh] + screw_features

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
        origin = reference_side_wcf.to_world_coordinates(Point(
            self.distance + self.angled_lead / 2 + self.angled_length / 2,
            beam.get_face_height(self.face_id) / 2,
            0))

        forward_clamp = Frame(origin, reference_side_wcf.xaxis, reference_side_wcf.yaxis.scaled(-1))
        backward_clamp = Frame(origin, reference_side_wcf.xaxis.scaled(-1), reference_side_wcf.yaxis)
        return [forward_clamp, backward_clamp]

    # This method is not generalizable and should be removed in future
    def get_assembly_direction(self, beam):
        '''
        Returns the only possible assembly direction.
        '''
        # print "Dist%s" % self.distance
        face_frame = beam.get_face_plane(self.face_id)
        return face_frame.normal.scaled(-1 * beam.get_face_height(self.face_id))

    def swap_faceid_to_opposite_face(self):
        # Face id flip
        old_face_id = self.face_id
        new_id = (old_face_id + 1) % 4 + 1
        self.face_id = new_id

        # Distance point change by angled_lead distance
        self.distance = self.distance + self.angled_lead
        # Angle flip
        self.angle = 180 - self.angle

    @property
    def clamp_types(self):
        # Returns a list of clamps types that can assemble this joint
        clamps = []
        clamps.append('SL1')
        return clamps


def non_planar_lap_joint_from_beam_beam_intersection(beam_move, beam_stay, thickness=None):
    # type: (Beam, Beam, float) -> tuple[JointNonPlanarLap, JointNonPlanarLap]
    ''' Compute the intersection between two beams.
    Returns a tuple of [JointNonPlanarLap, JointNonPlanarLap] when a valid joint pair can be found.

    Note. At the moment the center lines of two beams must not directly intersect.
    There need to be some offset

    `thickness` is the thickness of the lap joint on the moving_beam.

    If no intersection can be found or the two beam are not coplanar,
    Returns a tuple of [None, None]
    '''

    # Find center line intersection and distances
    ctl_m = beam_move.get_center_line()
    ctl_s = beam_stay.get_center_line()
    cp_m, cp_s = intersection_line_line(ctl_m, ctl_s)

    # Find which beam face to be facing each other
    # We use the intersection points of the two lines as a guide and compare with
    v_ms = subtract_vectors(cp_s, cp_m)
    v_sm = subtract_vectors(cp_m, cp_s)
    if norm_vector(v_ms) < 1e-5:
        # If the intersection is too close, we do not process it. TODO: Make it auto select faces.
        return [None, None]

    normal_dot_product_m = [dot_vectors(beam_move.reference_side_wcf(i + 1).zaxis, v_ms) for i in range(4)]
    normal_dot_product_s = [dot_vectors(beam_stay.reference_side_wcf(i + 1).zaxis, v_sm) for i in range(4)]
    joint_face_id_m = normal_dot_product_m.index(max(normal_dot_product_m)) + 1
    joint_face_id_s = normal_dot_product_s.index(max(normal_dot_product_s)) + 1

    # Find the reference side frame and reference edges
    ref_side_m = beam_move.reference_side_wcf(joint_face_id_m)
    ref_side_s = beam_stay.reference_side_wcf(joint_face_id_s)
    ref_edge_m0 = beam_move.reference_edge_wcf(joint_face_id_m)
    ref_edge_m1 = beam_move.reference_edge_wcf(joint_face_id_m-1)
    # s0 and s1 are defined as which ever is more negative to ref_side_m.xaxis
    ref_edge_s0 = beam_stay.reference_edge_wcf(joint_face_id_s)
    ref_edge_s1 = beam_stay.reference_edge_wcf(joint_face_id_s-1)
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
    some_point_has_contact = any([is_point_in_halfspace(point, ref_plane_m_top) for point in lpx_pts])
    if not some_point_has_contact:
        return [None, None]

    # Check if all of the first 8 points lie outside the stay_beam start or end.
    # If all of them are outside, there is no true intersection.
    ref_plane_s_start = Plane.from_frame(beam_stay.reference_side_wcf(5))
    all_points_outside_start = all([is_point_in_halfspace(point, ref_plane_s_start) for point in lpx_pts])
    ref_plane_s_end = Plane.from_frame(beam_stay.reference_side_wcf(6))
    all_points_outside_end = all([is_point_in_halfspace(point, ref_plane_s_end) for point in lpx_pts])
    if all_points_outside_start or all_points_outside_end:
        return [None, None]

    # Check if all of the first 8 points lie outside the move_beam start or end.
    # If all of them are outside, there is no true intersection.
    ref_plane_s_start = Plane.from_frame(beam_move.reference_side_wcf(5))
    all_points_outside_start = all([is_point_in_halfspace(point, ref_plane_s_start) for point in lpx_pts])
    ref_plane_s_end = Plane.from_frame(beam_move.reference_side_wcf(6))
    all_points_outside_end = all([is_point_in_halfspace(point, ref_plane_s_end) for point in lpx_pts])
    if all_points_outside_start or all_points_outside_end:
        return [None, None]


    # Joint Thickness hard coded default:
    if thickness is None:
        thickness = 70

    # Compute joint center (center on lap cheek plane, on centerline intersection)
    # Screw hole axis passes through this hole.
    # Projection of centerline and find intersection
    ref_side_m_projection_plane = Plane.from_frame(ref_side_m)
    P = Projection.from_plane(ref_side_m_projection_plane)
    ctl_m_projected = ctl_m.transformed(P)
    ctl_s_projected = ctl_s.transformed(P)
    cp_m_projected, cp_s_projected = intersection_line_line(ctl_m_projected, ctl_s_projected)
    # Translating the frame to the cheek plane
    offset_amount = thickness - beam_move.get_face_height(joint_face_id_m)
    T = Translation.from_vector(ref_side_m.zaxis.unitized().scaled(offset_amount))
    joint_center_frame = Frame(Point(* cp_m_projected).transformed(T), ref_side_m.xaxis, ref_side_m.yaxis)

    # Precompute this dot product for deciding the hook triangle direction
    axial_dot_product = dot_vectors(ref_side_s.zaxis, ref_side_m.xaxis)

    # Construct joint objects
    joint_m = JointNonPlanarLap(joint_center_frame, thickness, joint_face_id_m, joint_face_id_s, axial_dot_product, lpx_pts,
                                is_joint_on_beam_move=True, name='%s-%s' % (beam_move.name, beam_stay.name))
    joint_s = JointNonPlanarLap(joint_center_frame, thickness, joint_face_id_m, joint_face_id_s, axial_dot_product, lpx_pts,
                                is_joint_on_beam_move=False, name='%s-%s' % (beam_move.name, beam_stay.name))

    return (joint_m, joint_s)


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