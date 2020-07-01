# Joint_lap Object (inheret from Joint class)
#   Joint_lap object are used to model variable angle lap joint.

import math
import compas
from compas.datastructures import Mesh
from compas.geometry import Box
from compas.geometry import Frame, Vector, Point

from compas.geometry import distance_point_point, intersection_line_line

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

        # Constants
        self.clamp_types = ['CL3']  # The clamp type capable of assembling this joint

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
        vertices.append(Point(self.distance + self.angle_lead, self.width, -self.height))  # point on -x +y -z
        vertices.append(Point(self.distance + self.angle_lead + angled_length, self.width, -self.height))  # point on +x +y -z
        vertices.append(Point(self.distance + angled_length, 0, -self.height))  # point on -x +y -z

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

        Parameters
        ----------
        beam : Beam
            Beam Object

        Returns
        -------
        list(Frame)
            Frames in WCF
        """
        # print "Dist%s" % self.distance
        face_frame = beam.get_face_frame(self.face_id)
        origin = face_frame.to_world_coordinates([self.distance, beam.get_face_height(self.face_id), beam.get_face_width(self.face_id)/2])
        # print origin
        forward_clamp = Frame(origin, face_frame.xaxis, face_frame.zaxis)
        backward_clamp = Frame(origin, face_frame.xaxis.scaled(-1), face_frame.zaxis.scaled(-1))
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

        # Distance point change by angle_lead distance
        self.distance = self.distance + self.angle_lead
        # Angle flip
        self.angle = 180 - self.angle

    @property
    def angle_lead(self):
        # Calculate the lap joint lead distance (relative to width) caused by angled joint
        # Distance on X axis between point 0 and 1
        # Positive lead if angle > 90, Negative lead if angle < 90
        return math.tan(math.radians(self.angle - 90)) * self.width


def Joint_halflap_from_beam_beam_intersection(beam1, beam2, face_choice=0):
    # type: (Beam, Beam): -> Tuple[Joint_halflap, Joint_halflap]

    # Find coplanar faces
    face_pairs = beam1.get_beam_beam_coplanar_face_ids(beam2)
    if len(face_pairs) == 0:
        return (None, None)
    beam1_face_id, beam2_face_id = face_pairs[face_choice]

    # Find front and back edges
    beam1_frnt_edge = beam1.reference_edge_wcf(beam1_face_id)
    beam1_back_edge = beam1.reference_edge_wcf(beam1_face_id - 1)
    beam2_frnt_edge = beam2.reference_edge_wcf(beam2_face_id)
    beam2_back_edge = beam2.reference_edge_wcf(beam2_face_id - 1)

    # Compute intersection distance
    def llx_distance(line1, line2):
        intersection_result = intersection_line_line(line1, line2)[0]
        return distance_point_point(intersection_result, line1.start)

    beam1_distance = min(llx_distance(beam1_frnt_edge, beam2_frnt_edge), llx_distance(beam1_frnt_edge, beam2_back_edge))
    beam2_distance = min(llx_distance(beam2_frnt_edge, beam1_frnt_edge), llx_distance(beam2_frnt_edge, beam1_back_edge))

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
