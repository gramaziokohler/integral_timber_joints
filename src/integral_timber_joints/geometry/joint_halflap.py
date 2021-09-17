# Joint_lap Object (inheret from Joint class)
#   Joint_lap object are used to model variable angle lap joint.

import math

import compas
from compas.datastructures import Mesh
from compas.geometry import Box, Frame, Point, Line, Projection, Translation, Vector, distance_point_point, intersection_line_line, intersection_segment_segment

from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.geometry.joint import Joint
from integral_timber_joints.geometry.screw import Screw_SL
from integral_timber_joints.geometry.utils import *
from integral_timber_joints.assembly.beam_assembly_method import BeamAssemblyMethod


try:
    from typing import Dict, List, Optional, Tuple, cast, Any

    from integral_timber_joints.process import RobotClampAssemblyProcess
except:
    pass


class JointHalfLap(Joint):
    """
    joint class containing varied joints
    """

    def __init__(self, face_id=1, distance=100, angle=90, length=100, width=100, height=50, thickness=50, name=None):
        """
        :param distance:  double
        :param face_id:   int
        """
        self.face_id = face_id
        self.distance = distance
        self.angle = angle          # angle in degrees
        self.length = length
        self.width = width
        self.height = height
        self._thickness = thickness  # Thickness of the solid part of the beam after boolean.
        self.thru_x_neg = True
        self.thru_x_pos = True
        self.name = name
        self.mesh = None

    @property
    def data(self):
        data = {
            'face_id': self.face_id,
            'distance': self.distance,
            'angle': self.angle,
            'length': self.length,
            'width': self.width,
            'height': self.height,
            'thickness': self.thickness,
            'thru_x_neg': self.thru_x_neg,
            'thru_x_pos': self.thru_x_pos,
            'name': self.name,
        }
        return data

    @classmethod
    def from_data(cls, data):
        """Construct a Joint object from structured data.
        This class method must be overridden by an inherited class.
        """
        joint = cls()
        joint.face_id = data['face_id']
        joint.distance = data['distance']
        joint.angle = data['angle']
        joint.length = data['length']
        joint.width = data['width']
        joint.height = data['height']
        joint.thickness = data.get('thickness', 50)
        joint.thru_x_neg = data['thru_x_neg']
        joint.thru_x_pos = data['thru_x_pos']
        joint.name = data['name']
        return joint

    @property
    def thickness(self):
        return self._thickness

    @thickness.setter
    def thickness(self, value):
        self._thickness = value

    @property
    def angled_lead(self):
        # Calculate the lap joint lead distance (relative to width) caused by angled joint
        # Distance on X axis (length) between point 0 and 1.
        # Positive lead if angle > 90, Negative lead if angle < 90
        return math.tan(math.radians(self.angle - 90)) * self.width

    @property
    def angled_length(self):
        # Calculates the length of the lap opening.
        # Distance on X axis (length) between point 1 and 2. or point 0 and 3.
        # The value is equal to self.length when angle = 90 degrees.
        return self.length / math.cos(math.radians(self.angle - 90))

    @property
    def distance_at_center(self):
        return self.distance + self.angled_lead / 2 + self.angled_length / 2

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
            diff = value - self.thickness
            self.thickness += diff
            self.height -= diff
            return
        raise KeyError("%s is invalid for JointHalfLap" % key)

    # #####################
    # Joint Shape
    # #####################

    def get_feature_shapes(self, BeamRef):
        # type: (Beam) -> list[Mesh]
        """Compute the negative shape of the joint.
        Parameters
        ----------
        BeamRef -> integral_timber_joint.geometry.Beam
            The Beam object this joint is attached to
        Returns
        -------
        object
            A compas.Mesh

        """
        OVERSIZE = 10.0

        # Get face_frame from Beam (the parent Beam)
        face_frame = BeamRef.reference_side_wcf(self.face_id)

        # The 8 corners of a box:

        vertices = []
        vertices.append(Point(self.distance, 0, -self.height))  # point on -x -y -z
        vertices.append(Point(self.distance + self.angled_lead, self.width, -self.height))  # point on -x +y -z
        vertices.append(Point(self.distance + self.angled_lead + self.angled_length, self.width, -self.height))  # point on +x +y -z
        vertices.append(Point(self.distance + self.angled_length, 0, -self.height))  # point on -x +y -z

        # add the last 4 points related to the height and tolerance
        vertices.append(vertices[0] + Point(0, 0, self.height + OVERSIZE))  # relative to point 0
        vertices.append(vertices[1] + Point(0, 0, self.height + OVERSIZE))  # relative to point 1
        vertices.append(vertices[2] + Point(0, 0, self.height + OVERSIZE))  # relative to point 2
        vertices.append(vertices[3] + Point(0, 0, self.height + OVERSIZE))  # relative to point 3

        # create mesh and add offset
        move_vertex_from_neighbor(vertices, [0, 3, 7, 4], [1, 2, 6, 5], OVERSIZE)
        move_vertex_from_neighbor(vertices, [1, 2, 6, 5], [0, 3, 7, 4], OVERSIZE)

        shape = polyhedron_box_from_vertices(vertices)
        shape = face_frame.to_world_coordinates(shape)

        return [shape]

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

        reference_side_wcf = beam.reference_side_wcf((self.face_id - 1 + 2) % 4 + 1)
        origin = reference_side_wcf.to_world_coordinates(Point(
            self.distance + self.angled_lead / 2 + self.angled_length / 2,
            beam.get_face_height(self.face_id) / 2,
            0))

        forward_clamp = Frame(origin, reference_side_wcf.xaxis, reference_side_wcf.yaxis.scaled(-1))
        backward_clamp = Frame(origin, reference_side_wcf.xaxis.scaled(-1), reference_side_wcf.yaxis)
        return [forward_clamp, backward_clamp]

    def get_joint_center_at_open_side(self, beam):
        # type: (Beam) -> Point
        center_point_in_ocf = Point(self.distance_at_center, self.width / 2, 0)

        # Get face_frame from Beam (the parent Beam)
        face_frame = beam.reference_side_wcf(self.face_id)

        return face_frame.to_world_coordinates(center_point_in_ocf)

    def get_joint_center_at_solid_side(self, beam):
        # type: (Beam) -> Point
        center_point_in_ocf = Point(self.distance_at_center, self.width / 2, 0)

        # Get face_frame from Beam (the parent Beam)
        face_frame = beam.reference_side_wcf((self.face_id + 1) % 4 + 1)

        return face_frame.to_world_coordinates(center_point_in_ocf)

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

    def assembly_tool_types(self, beam_assembly_method):
        # type: (BeamAssemblyMethod) -> list[str]
        # Returns a list of clamps types that can assemble this joint
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
    def from_beam_beam_intersection(cls, beam_stay, beam_move, face_choice=0, dist_tol=1e-5, coplanar_tol=5e-3):
        # type: (Beam, Beam, int, float, float) -> Tuple[JointHalfLap, JointHalfLap, Line]
        ''' Compute the intersection between two beams.

        `beam_stay` must be the earlier beam in assembly sequence
        `beam_move` must be the later beam in assembly sequence

        Returns a tuple of [JointHalfLap, JointHalfLap] when a valid joint pair can be found.

        The function will check for beam center-line intersections.

        If no intersection can be found or the two beam are not coplanar,
        Returns a tuple of (None, None, None)
        '''

        # Find coplanar faces
        face_pairs = beam_move.get_beam_beam_coplanar_face_ids(beam_stay, coplanar_tol)
        if len(face_pairs) == 0:
            return (None, None, None)
        beam_m_face_id, beam_s_face_id = face_pairs[face_choice]

        # Find front and back edges
        beam1_frnt_edge = beam_move.reference_edge_wcf(beam_m_face_id)
        beam1_back_edge = beam_move.reference_edge_wcf(beam_m_face_id - 1)
        beam2_frnt_edge = beam_stay.reference_edge_wcf(beam_s_face_id)
        beam2_back_edge = beam_stay.reference_edge_wcf(beam_s_face_id - 1)

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

        d1f2f = llx_distance(beam1_frnt_edge, beam2_frnt_edge)
        d1f2b = llx_distance(beam1_frnt_edge, beam2_back_edge)
        d2f1f = llx_distance(beam2_frnt_edge, beam1_frnt_edge)
        d2f1b = llx_distance(beam2_frnt_edge, beam1_back_edge)
        # Handles the case where the intersecion fails.
        if any(v is None for v in [d1f2f, d1f2b, d2f1f, d2f1b]):
            return (None, None, None)

        beam_m_distance = min(d1f2f, d1f2b)
        beam_s_distance = min(d2f1f, d2f1b)

        # Compute angle
        def llx_llx_angle(line1, line2, angled_line):
            intersection_result1 = intersection_line_line(line1, angled_line)[0]
            intersection_result2 = intersection_line_line(line2, angled_line)[0]
            v1 = Vector.from_start_end(intersection_result1, line1.start)
            v2 = Vector.from_start_end(intersection_result1, intersection_result2)
            return v1.angle(v2)

        beam_m_angle = math.degrees(llx_llx_angle(beam1_frnt_edge, beam1_back_edge, beam2_frnt_edge))
        beam_s_angle = math.degrees(llx_llx_angle(beam2_frnt_edge, beam2_back_edge, beam1_frnt_edge))

        # Construct Joint object and flip one of them
        joint_m = JointHalfLap(
            face_id=beam_m_face_id,
            distance=beam_m_distance,
            angle=beam_m_angle,
            length=beam_stay.get_face_width(beam_s_face_id),
            width=beam_move.get_face_width(beam_m_face_id),
            height=beam_move.get_face_height(beam_m_face_id)/2,
            thickness=beam_stay.get_face_height(beam_s_face_id)/2,
            name='%s-%s' % (beam_move.name, beam_stay.name))
        joint_s = JointHalfLap(
            face_id=beam_s_face_id,
            distance=beam_s_distance,
            angle=beam_s_angle,
            length=beam_move.get_face_width(beam_m_face_id),
            width=beam_stay.get_face_width(beam_s_face_id),
            height=beam_stay.get_face_height(beam_s_face_id)/2,
            thickness=beam_move.get_face_height(beam_m_face_id)/2,
            name='%s-%s' % (beam_stay.name, beam_move.name))
        joint_s.swap_faceid_to_opposite_face()

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
