# Joint_lap Object (inheret from Joint class)
#   Joint_lap object are used to model variable angle lap joint.

import math
from ast import literal_eval

import compas
from compas.datastructures import Mesh
from compas.geometry import Box, Frame, Point, Line, Transformation, Vector
from compas.geometry import Projection, Translation, transformations
from compas.geometry import distance_point_point, intersection_line_line, intersection_segment_segment, dot_vectors, transform_points

from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.geometry.joint import Joint
from integral_timber_joints.geometry.screw import Screw_SL
from integral_timber_joints.geometry.utils import *


try:
    from typing import Dict, List, Optional, Tuple, cast, Any

    from integral_timber_joints.process import RobotClampAssemblyProcess
    from integral_timber_joints.assembly.beam_assembly_method import BeamAssemblyMethod
except:
    pass


class JointPolylineLap(Joint):
    """
    joint class containing varied joints
    """

    def __init__(
            self,
            face_id=1,  # type: int
            center_distance=100,  # type: float
            top_side_thickness=50,  # type: float
            corner_pts=[],   # type: list[Point]
            polylines=None,  # type: list[list[Tuple[float, float]]]
            is_joint_on_beam_move=False,  # type: bool
            is_joint_on_top=False,  # type: bool
            name=None  # type: str
    ):
        """
        :param distance:  double
        :param face_id:   int
        """
        if polylines is None:
            polylines = [[[0, 0], [1, 0]]] * 4

        self.face_id = face_id  # The face id of the face to be cut
        self.center_distance = center_distance  # Distance measured from the start of the beam to the first corner (0 or 1) of the joint
        self.top_side_thickness = top_side_thickness        # Height is the depth of the material removed from the ref face
        self.corner_pts = corner_pts        # Eight corner points - refer to drawing
        self.polylines = polylines        # Height is the depth of the material removed from the ref face
        self.is_joint_on_beam_move = is_joint_on_beam_move  # If true, the line 01 23 45 67 are cutting along the Y axis of the ref face.
        self.is_joint_on_top = is_joint_on_top  # If true, the joint opening is on the reference face / point 4567.
        self.name = name

    @property
    def data(self):
        data = {
            'face_id': self.face_id,
            'distance': self.distance,
            'angle': self.angle,
            'beam_width': self.beam_width,
            'nbr_beam_width': self.nbr_beam_width,
            'height': self.height,
            'polylines': self.polylines,
            'thickness': self.thickness,
            'name': self.name,
        }
        return data

    @classmethod
    def from_data(cls, data):
        """Construct a Joint object from structured data.
        This class method must be overridden by an inherited class.
        """
        joint = cls()
        joint.face_id = data.get('face_id', 1)
        joint.distance = data.get('distance', 100)
        joint.angle = data.get('angle', 90)
        joint.beam_width = data.get('beam_width', 100)
        joint.nbr_beam_width = data.get('nbr_beam_width', 100)
        joint.height = data.get('height', 50)
        joint.polylines = data.get('polylines', [])
        joint.thickness = data.get('thickness', 50)
        joint.name = data.get('name', None)
        return joint

    @property
    def _total_thickness(self):
        return max([distance_point_point(self.corner_pts[i], self.corner_pts[i+4]) for i in range(4)])

    @property
    def distance_at_center(self):
        return self.center_distance

    # ###########################
    # Transformation of Extrinsic
    # ###########################

    def transform(self, transformation):
        # type: (Transformation) -> None
        """Transforming the joint object in WCF.
        Typically called by assembly.transform when initiated by user."""
        self.center_frame.transform(transformation)
        self.corner_pts = transform_points(self.corner_pts, transformation)

    # #####################
    # Modifyable Parameters
    # #####################
    @property
    def thickness(self):
        # type: () -> float
        if self.is_joint_on_top:
            return self._total_thickness - self.top_side_thickness
        else:
            return self.top_side_thickness

    @thickness.setter
    def thickness(self, value):
        # type: (float) -> None
        if self.is_joint_on_top:
            self.top_side_thickness = self._total_thickness - value
        else:
            self.top_side_thickness = value

    @property
    def param_string(self):
        # type: () -> str
        return str([self.top_side_thickness, self.polylines]).replace(" ", "")

    @param_string.setter
    def param_string(self, value):
        # type: (str) -> None
        values = literal_eval(value)
        assert len(values) == 2
        self.top_side_thickness = float(values[0])
        assert len(values[1]) == 4
        self.polylines = values[1]

    @property
    def parameter_keys(self):
        # type: () -> list[str]
        return ['param_string']

    def get_parameter(self, key):
        # type: (str) -> Any
        if key == 'param_string':
            return self.param_string
        raise KeyError("%s is invalid for JointPolylineLap" % key)

    def set_parameter(self, key, value):
        # type: (str, Any) -> None
        if key == "param_string":
            self.param_string = value
            return
        raise KeyError("%s is invalid for JointPolylineLap" % key)

    # #####################
    # Joint Shape
    # #####################

    def _quad_at_height(self, height_fraction):
        # type: (float) -> list[Point]
        """Return four points refering to the quad at specified height.
        - height is 0.0, point 0-3 are returned.
        - height is 1.0, point 4-7 are returned.
        - in between heights are interpolated.
        """
        points = []
        return [Line(self.corner_pts[i], self.corner_pts[i+4]).point(height_fraction) for i in range(4)]

    def _polyline_at_height(self, line_index, height_fraction):
        # type: (int, float) -> list[Point]
        """Returns a list of points (polyline-ish) as evaluated
        from the uvw coordinate of the box formed by the corner points."""
        line_index = line_index % 4
        quad = self._quad_at_height(height_fraction)
        # rotate the quad based on index
        for _ in range(line_index):
            quad.append(quad.pop(0))

        # evaluate point on the quad
        return [eval_quad(quad, u, v)for u, v in self.polylines[line_index]]

    def _polyline_at_top(self, line_index, oversize=True):
        # type: (int, bool) -> list[Point]
        if oversize:
            return self._polyline_at_height(line_index, 1.2)
        else:
            return self._polyline_at_height(line_index, 1.0)

    def _polyline_at_btm(self, line_index, oversize=True):
        # type: (int, bool) -> list[Point]
        if oversize:
            return self._polyline_at_height(line_index, -0.2)
        else:
            return self._polyline_at_height(line_index, 0.0)

    def _polyline_at_mid(self, line_index, oversize = 0):
        # type: (int, float) -> list[Point]
        height_fraction = 1.0 - (self.top_side_thickness / self._total_thickness) + oversize
        return self._polyline_at_height(line_index, height_fraction)

    def _extline_at_height(self, line_index, height_fraction):
        # type: (int, float) -> list[Point]
        """Returns two points that can be added to evaluated polylines
        for creating a complete cycle with oversize.
        """
        line_index = line_index % 4
        quad = self._quad_at_height(height_fraction)
        # rotate the quad based on index
        for _ in range(line_index):
            quad.append(quad.pop(0))
        return [eval_quad(quad, 0, -0.2), eval_quad(quad, 1, -0.2)]

    def _extline_at_top(self, line_index, oversize=True):
        # type: (int, bool) -> list[Point]
        if oversize:
            return self._extline_at_height(line_index, 1.2)
        else:
            return self._extline_at_height(line_index, 1.0)

    def _extline_at_btm(self, line_index, oversize=True):
        # type: (int, bool) -> list[Point]
        if oversize:
            return self._extline_at_height(line_index, -0.2)
        else:
            return self._extline_at_height(line_index, 0.0)

    def _extline_at_mid(self, line_index, oversize = 0):
        # type: (int, float) -> list[Point]
        height_fraction = 1.0 - (self.top_side_thickness / self._total_thickness) + oversize
        return self._extline_at_height(line_index, height_fraction)

    def get_feature_shapes(self, BeamRef):
        # type: (Beam) -> list[Mesh]
        """Compute the negative shape of the joint.
        There are three feature shapes.
        - First is the bowtie shape on the open side (refside where the joint is cut across)
        - Second and third are two side cut from on the solid side.

        The side cuts could be absent if there are no side cuts.

        Parameters
        ----------
        BeamRef -> integral_timber_joint.geometry.Beam
            The Beam object this joint is attached to

        Returns
        -------
        object
            A compas.Mesh

        """
        shapes = []
        # i is an index that help rotate the quad index by 1
        i = 0 if self.is_joint_on_beam_move else 1
        if self.is_joint_on_top:
            poly_line_mid = self._polyline_at_mid(i) + self._extline_at_mid(i+1) + self._polyline_at_mid(i+2) + self._extline_at_mid(i+3)
            poly_line_top = self._polyline_at_top(i) + self._extline_at_top(i+1) + self._polyline_at_top(i+2) + self._extline_at_top(i+3)
            # vertices = simplify_polygon(poly_line_mid) + simplify_polygon(poly_line_top)
            # shapes.append(polyhedron_box_from_vertices(vertices))
            print("vertices a ", poly_line_mid + poly_line_top)
            shapes.append(polyhedron_box_from_vertices(poly_line_mid + poly_line_top))
        else:
            poly_line_btm = self._polyline_at_btm(i) + self._extline_at_btm(i+1) + self._polyline_at_btm(i+2) + self._extline_at_btm(i+3)
            poly_line_mid = self._polyline_at_mid(i) + self._extline_at_mid(i+1) + self._polyline_at_mid(i+2) + self._extline_at_mid(i+3)
            # vertices = simplify_polygon(poly_line_btm) + simplify_polygon(poly_line_mid)
            # shapes.append(polyhedron_box_from_vertices(vertices))
            print("vertices b ",  poly_line_btm + poly_line_mid)

            shapes.append(polyhedron_box_from_vertices(poly_line_btm + poly_line_mid))

        # Adding the two side cuts (if they have > 2 points)
        tol = 1e-3 if self.is_joint_on_top else -1e-3
        if len(self.polylines[i+1]) > 2:
            if self.is_joint_on_top:
                poly_line_mid = self._polyline_at_mid(i+1, tol)[::-1] + self._extline_at_mid(i+1, tol)
                poly_line_btm = self._polyline_at_btm(i+1)[::-1] + self._extline_at_btm(i+1)
                shapes.append(polyhedron_box_from_vertices(poly_line_btm + poly_line_mid))
            else:
                poly_line_mid = self._polyline_at_mid(i+1, tol)[::-1] + self._extline_at_mid(i+1, tol)
                poly_line_top = self._polyline_at_top(i+1)[::-1] + self._extline_at_top(i+1)
                shapes.append(polyhedron_box_from_vertices(poly_line_mid + poly_line_top))

        if len(self.polylines[(i+3)%4]) > 2:
            if self.is_joint_on_top:
                poly_line_mid = self._polyline_at_mid(i+3, tol)[::-1] + self._extline_at_mid(i+3, tol)
                poly_line_btm = self._polyline_at_btm(i+3)[::-1] + self._extline_at_btm(i+3)
                shapes.append(polyhedron_box_from_vertices(poly_line_btm + poly_line_mid))
            else:
                poly_line_mid = self._polyline_at_mid(i+3, tol)[::-1] + self._extline_at_mid(i+3, tol)
                poly_line_top = self._polyline_at_top(i+3)[::-1] + self._extline_at_top(i+3)
                shapes.append(polyhedron_box_from_vertices(poly_line_mid + poly_line_top))

        return shapes

    def get_clamp_frames(self, beam):
        # type: (Beam) -> list[Frame]
        """Compute the possible frames where the clamp can be attached.
        The clamp frame is located at the opposite side of the face_id.

        Orgin of the frame is located on the surface of that face, at the center point of the joint.
        X Axis is along the length of the beam.
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
        origin = self.get_joint_center_at_solid_side(beam)
        reference_side_wcf = beam.reference_side_wcf()

        forward_clamp = Frame(origin, reference_side_wcf.xaxis, reference_side_wcf.yaxis.scaled(-1))
        backward_clamp = Frame(origin, reference_side_wcf.xaxis.scaled(-1), reference_side_wcf.yaxis)
        return [forward_clamp, backward_clamp]

    def get_joint_center_at_open_side(self, beam):
        # type: (Beam) -> Point
        open_side_face_id = self.face_id
        return beam.get_face_center_line(open_side_face_id).point_from_start(self.center_distance)

    def get_joint_center_at_solid_side(self, beam):
        # type: (Beam) -> Point
        solid_side_face_id = (self.face_id + 1) % 4 + 1
        return beam.get_face_center_line(solid_side_face_id).point_from_start(self.center_distance)

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
        self.is_joint_on_top = not self.is_joint_on_top

    def assembly_tool_types(self, beam_assembly_method):
        # type: (BeamAssemblyMethod) -> list[str]
        # Returns a list of clamps types that can assemble this joint
        from integral_timber_joints.assembly.beam_assembly_method import BeamAssemblyMethod
        clamps = []
        if beam_assembly_method == BeamAssemblyMethod.SCREWED_WITH_GRIPPER:
            return ['SL1', 'SL1_G200']
        elif beam_assembly_method == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
            return ['SL1_G200', 'SL1']       # Preferentially requesting SL1_G200 (this is likely to be assigned to the gripping joint)
        else:
            if self.angle > 24.9 and self.angle < 90.1:
                clamps.append('CL3')
            if self.angle > 89.9 and self.angle < 155.1:
                clamps.append('CL3M')
        return clamps

    @classmethod
    def from_beam_beam_intersection(cls, beam_stay, beam_move, dist_tol=1e-5, coplanar_tol=5e-3, joint_face_id_move=None):
        # type: (Beam, Beam, int, float, float) -> Tuple[JointPolylineLap, JointPolylineLap, Line]
        ''' Compute the intersection between two beams.

        `beam_stay` must be the earlier beam in assembly sequence
        `beam_move` must be the later beam in assembly sequence

        Returns a tuple of [JointHalfLap, JointHalfLap] when a valid joint pair can be found.

        The function will check for beam center-line intersections.

        If no intersection can be found or the two beam are not coplanar,
        Returns a tuple of (None, None, None)
        '''
        # Compute intersection distance, Return None if they don't intersect
        def llx_distance(line1, line2):
            dist_tol = line1.length * 1e-5
            intersection_result = intersection_segment_segment(line1, line2, dist_tol)
            if intersection_result[0] is None:
                # print("Joint Intersection result is none")
                return None
            if distance_point_point(intersection_result[0], intersection_result[1]) > dist_tol:
                # print("Joint Intersection result %s > tol: %s" % (distance_point_point(intersection_result[0], intersection_result[1]), dist_tol))
                return None
            return distance_point_point(intersection_result[0], line1.start)

        # * Find joint distance on center line
        beam_move_center_line = beam_move.get_center_line()
        beam_stay_center_line = beam_stay.get_center_line()
        if beam_move_center_line is None or beam_stay_center_line is None:
            return (None, None, None)

        beam_m_center_distance = llx_distance(beam_move_center_line, beam_stay_center_line)
        beam_s_center_distance = llx_distance(beam_stay_center_line, beam_move_center_line)

        # Find coplanar faces
        face_pairs = beam_move.get_beam_beam_coplanar_face_ids(beam_stay, coplanar_tol)
        if len(face_pairs) == 0:
            return (None, None, None)

        # * Choosing which beam face to put joint on. Taking a optional guide parameter
        beam_m_face_id, beam_s_face_id = face_pairs[0]  # Default
        for id_m, id_s in face_pairs:
            if id_m == joint_face_id_move:
                beam_m_face_id, beam_s_face_id = id_m, id_s
        beam_s_face_id = (beam_s_face_id + 1) % 4 + 1  # Flip joint face on staying beam to opposite side

        # * Find edges on beam_move
        m_top_frnt = beam_move.reference_edge_wcf(beam_m_face_id)
        m_btm_frnt = beam_move.reference_edge_wcf(beam_m_face_id + 1)
        m_btm_back = beam_move.reference_edge_wcf(beam_m_face_id + 2)
        m_top_back = beam_move.reference_edge_wcf(beam_m_face_id + 3)

        # * Find edges on beam_stay
        # Compute which side the beam_stay comes from
        ref_side_m = beam_move.reference_side_wcf(beam_m_face_id)
        ref_side_s = beam_stay.reference_side_wcf(beam_s_face_id)
        if dot_vectors(ref_side_s.yaxis, ref_side_m.xaxis) > 0.0:
            s_btm_near = beam_stay.reference_edge_wcf(beam_s_face_id)
            s_top_near = beam_stay.reference_edge_wcf(beam_s_face_id + 1)
            s_top_farr = beam_stay.reference_edge_wcf(beam_s_face_id + 2)
            s_btm_farr = beam_stay.reference_edge_wcf(beam_s_face_id + 3)
        else:
            s_btm_near = beam_stay.reference_edge_wcf(beam_s_face_id + 3)
            s_top_near = beam_stay.reference_edge_wcf(beam_s_face_id + 2)
            s_top_farr = beam_stay.reference_edge_wcf(beam_s_face_id + 1)
            s_btm_farr = beam_stay.reference_edge_wcf(beam_s_face_id)

        # Compute corner points in WCF
        corner_pts = []
        corner_pts.append(intersection_segment_segment(m_btm_frnt, s_btm_near)[0])
        corner_pts.append(intersection_segment_segment(m_btm_back, s_btm_near)[0])
        corner_pts.append(intersection_segment_segment(m_btm_back, s_btm_farr)[0])
        corner_pts.append(intersection_segment_segment(m_btm_frnt, s_btm_farr)[0])
        corner_pts.append(intersection_segment_segment(m_top_frnt, s_top_near)[0])
        corner_pts.append(intersection_segment_segment(m_top_back, s_top_near)[0])
        corner_pts.append(intersection_segment_segment(m_top_back, s_top_farr)[0])
        corner_pts.append(intersection_segment_segment(m_top_frnt, s_top_farr)[0])

        # Construct Joint object and flip one of them
        joint_m = JointPolylineLap(
            face_id=beam_m_face_id,
            center_distance=beam_m_center_distance,
            top_side_thickness=beam_stay.get_face_height(beam_s_face_id)/2,
            corner_pts=corner_pts,
            is_joint_on_beam_move=True,
            is_joint_on_top=True,
            name='%s-%s' % (beam_move.name, beam_stay.name))
        joint_s = JointPolylineLap(
            face_id=beam_s_face_id,
            center_distance=beam_s_center_distance,
            top_side_thickness=beam_move.get_face_height(beam_m_face_id)/2,
            corner_pts=corner_pts,
            is_joint_on_beam_move=False,
            is_joint_on_top=False,
            name='%s-%s' % (beam_stay.name, beam_move.name))

        # conpute screw center line
        beam_move_center = joint_m.get_joint_center_at_solid_side(beam_move)
        beam_stay_center = joint_s.get_joint_center_at_solid_side(beam_stay)
        screw_line = Line(beam_move_center, beam_stay_center)
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
