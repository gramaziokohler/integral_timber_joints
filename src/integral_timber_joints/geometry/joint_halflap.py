# Joint_lap Object (inheret from Joint class)
#   Joint_lap object are used to model variable angle lap joint.

import math

import compas
from compas.datastructures import Mesh
from compas.geometry import Box, Frame, Point, Vector, distance_point_point, intersection_line_line
from compas.geometry import intersection_segment_segment

from integral_timber_joints.geometry.joint import Joint


class Joint_halflap(Joint):
    """
    joint class containing varied joints
    """

    def __init__(self, face_id, distance, angle, length, width, height, name=None):
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
        self.thru_x_neg = True
        self.thru_x_pos = True
        self.name = name
        self.mesh = None

        # #Perform initial calculation of the mesh (except when this is an empty object)
        # if frame is not None:
        #     self.update_joint_mesh()

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
        TOLEARNCE = 10.0

        # Get face_frame from Beam (the parent Beam)
        face_frame = BeamRef.reference_side_wcf(self.face_id)  # type: compas.datastructures.Mesh

        # Compute beam boolean box location
        box_frame_origin = face_frame.to_world_coordinates([(self.distance), self.height / 2 - TOLEARNCE / 2, self.length / 2])
        box_frame = Frame(box_frame_origin, face_frame.xaxis, face_frame.yaxis)

        # The box vertex order is not well documented:
        angled_length = self.length / math.cos(math.radians(self.angle - 90))
        vertices = []
        vertices.append(Point(self.distance, 0, -self.height))  # point on -x -y -z
        vertices.append(Point(self.distance + self.angled_lead, self.width, -self.height))  # point on -x +y -z
        vertices.append(Point(self.distance + self.angled_lead + self.angled_length, self.width, -self.height))  # point on +x +y -z
        vertices.append(Point(self.distance + self.angled_length, 0, -self.height))  # point on -x +y -z

        # moving the trim box to -X and +X direction for easier boolean
        vector_0_1 = Vector.from_start_end(vertices[0], vertices[1]).unitized().scaled(TOLEARNCE)
        vector_2_3 = Vector.from_start_end(vertices[2], vertices[3]).unitized().scaled(TOLEARNCE)
        if self.thru_x_neg:
            vertices[0] = vertices[0] - vector_0_1
        if self.thru_x_pos:
            vertices[1] = vertices[1] + vector_0_1
        if self.thru_x_pos:
            vertices[2] = vertices[2] - vector_2_3
        if self.thru_x_neg:
            vertices[3] = vertices[3] + vector_2_3

        # add the last 4 points related to the height and tolerance
        vertices.append(vertices[0] + Point(0, 0, self.height + TOLEARNCE))  # relative to point 0
        vertices.append(vertices[3] + Point(0, 0, self.height + TOLEARNCE))  # relative to point 3
        vertices.append(vertices[2] + Point(0, 0, self.height + TOLEARNCE))  # relative to point 2
        vertices.append(vertices[1] + Point(0, 0, self.height + TOLEARNCE))  # relative to point 1

        box = Box(Frame.worldXY(), 1, 1, 1)
        boolean_box_mesh = Mesh.from_vertices_and_faces(vertices, box.faces)
        boolean_box_mesh = face_frame.to_world_coordinates(boolean_box_mesh)

        # Draw boolean box and assign to self.mesh
        self.mesh = boolean_box_mesh
        return self.mesh

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
    def clamp_types(self):
        # Returns a list of clamps types that can assemble this joint
        clamps = []
        if self.angle > 24.9 and self.angle < 90.1:
            clamps.append('CL3')
        if self.angle > 89.9 and self.angle < 155.1:
            clamps.append('CL3M')

        return clamps


def Joint_halflap_from_beam_beam_intersection(beam1, beam2, face_choice=0, dist_tol=1e-5, coplanar_tol=5e-3):
    ''' Compute the intersection between two beams.
    Returns a tuple of [Joint_halflap, Joint_halflap] when a valid joint pair can be found.

    The function will check for beam center-line intersections.

    If no intersection can be found or the two beam are not coplanar,
    Returns a tuple of [None, None]
    '''
    # type: (Beam, Beam): -> Tuple[Joint_halflap, Joint_halflap]

    # Find coplanar faces
    face_pairs = beam1.get_beam_beam_coplanar_face_ids(beam2, coplanar_tol)
    if len(face_pairs) == 0:
        return (None, None)
    beam1_face_id, beam2_face_id = face_pairs[face_choice]

    # Find front and back edges
    beam1_frnt_edge = beam1.reference_edge_wcf(beam1_face_id)
    beam1_back_edge = beam1.reference_edge_wcf(beam1_face_id - 1)
    beam2_frnt_edge = beam2.reference_edge_wcf(beam2_face_id)
    beam2_back_edge = beam2.reference_edge_wcf(beam2_face_id - 1)

    # Compute intersection distance, Return None if they don't intersect
    def llx_distance(line1, line2):
        intersection_result = intersection_segment_segment(line1, line2, dist_tol)
        if intersection_result is None :
            return None
        return distance_point_point(intersection_result, line1.start)

    beam1_distance = min(llx_distance(beam1_frnt_edge, beam2_frnt_edge), llx_distance(beam1_frnt_edge, beam2_back_edge))
    beam2_distance = min(llx_distance(beam2_frnt_edge, beam1_frnt_edge), llx_distance(beam2_frnt_edge, beam1_back_edge))

    # Handles the case where the intersecion fails.
    if beam1_distance is None or beam2_distance is None:
        return (None, None)

    # Compute angle
    def llx_llx_angle(line1, line2, angled_line):
        intersection_result1 = intersection_line_line(line1, angled_line)[0]
        intersection_result2 = intersection_line_line(line2, angled_line)[0]
        v1 = Vector.from_start_end(intersection_result1, line1.start)
        v2 = Vector.from_start_end(intersection_result1, intersection_result2)
        return v1.angle(v2)

    beam1_angle = math.degrees(llx_llx_angle(beam1_frnt_edge, beam1_back_edge, beam2_frnt_edge))
    beam2_angle = math.degrees(llx_llx_angle(beam2_frnt_edge, beam2_back_edge, beam1_frnt_edge))

    # Construct Joint object and flip one of them
    joint1 = Joint_halflap(beam1_face_id, beam1_distance, beam1_angle, beam2.get_face_width(beam2_face_id), beam1.get_face_width(
        beam1_face_id), beam1.get_face_height(beam1_face_id)/2, name='%s-%s' % (beam1.name, beam2.name))
    joint2 = Joint_halflap(beam2_face_id, beam2_distance, beam2_angle, beam1.get_face_width(beam1_face_id), beam2.get_face_width(
        beam2_face_id), beam2.get_face_height(beam2_face_id)/2, name='%s-%s' % (beam2.name, beam1.name))
    joint2.swap_faceid_to_opposite_face()

    return (joint1, joint2)


if __name__ == "__main__":
    import compas
    import tempfile
    import os

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
