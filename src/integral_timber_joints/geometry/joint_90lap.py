# This file originates from Keerthana Udaykumar's <kudaykum@student.ethz.ch> master thesis on `Timber Grammar` in Fall 2019
# Pulled from https://github.com/KEERTHANAUDAY/timber_grammar on 2019-10-09 commit: d389364304f3740e652dbe4e85291402a2df0636
# This file is there on modified by Victor Leung <leung@arch.eth.ch> for use in Integral Timber Joint project

# Joint_90lap Object (inheret from Joint class)
#   Joint_90lap object are used to model 90 degree lap joint.

import math
import compas
from compas.datastructures import Mesh
from compas.geometry import Box
from compas.geometry import Frame

from compas.geometry import distance_point_point, intersection_line_line

from integral_timber_joints.geometry.joint import Joint

__all__ = ['Joint_90lap']


class Joint_90lap(Joint):
    """
    joint class containing varied joints
    """
    def __init__(self, distance,face_id, length, width, height, name = None):

        """
        :param distance:  double
        :param face_id:   int
        """
        self.distance = distance
        self.name = name
        self.face_id = face_id
        self.length = length
        self.width = width
        self.height = height
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
        face_frame = BeamRef.get_face_frame(self.face_id) # type: compas.datastructures.Mesh

        # Compute beam boolean box location
        box_frame_origin = face_frame.to_world_coords([(self.distance), self.height / 2 - TOLEARNCE / 2 , self.length / 2])
        box_frame = Frame(box_frame_origin, face_frame.xaxis, face_frame.yaxis)

        # Compute 3 Box dimensions
        box_x = self.width
        box_y = self.height + TOLEARNCE
        box_z = self.length + 2 * TOLEARNCE

        # Draw Boolean Box
        boolean_box = Box(box_frame, box_x, box_y, box_z)
        boolean_box_mesh = Mesh.from_vertices_and_faces(boolean_box.vertices, boolean_box.faces)

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
        #print "Dist%s" % self.distance
        face_frame = beam.face_frame(self.face_id)
        origin = face_frame.to_world_coords([self.distance, beam.face_height(self.face_id), beam.face_width(self.face_id)/2])
        #print origin
        forward_clamp =  Frame(origin, face_frame.xaxis, face_frame.zaxis)
        backward_clamp =  Frame(origin,face_frame.xaxis.scaled(-1), face_frame.zaxis.scaled(-1))
        return [forward_clamp, backward_clamp]

    #This method is not generalizable and should be removed in future
    def get_assembly_direction(self, beam):
        '''
        Returns the only possible assembly direction.
        '''
        #print "Dist%s" % self.distance
        face_frame = beam.face_plane(self.face_id)
        return face_frame.normal.scaled(-1 * beam.face_height(self.face_id))

def Joint_90lap_from_beam_beam_intersection(beam1, beam2, face_choice = 0):
    # type: (Beam, Beam): -> Tuple[Joint_90lap, Joint_90lap]

    # Find coplanar faces
    face_pairs = beam1.get_beam_beam_coplanar_face_ids(beam2)
    if len(face_pairs) == 0: return (None, None)

    # Compute Centerline Intersection Distances
    f1, f2 = face_pairs[face_choice]
    f2 = (f2 + 2 - 1) % 4 + 1
    c1 = beam1.get_face_center_line(f1)
    c2 = beam2.get_face_center_line(f2)
    intersection_line = intersection_line_line(c1,c2)
    dist1 = distance_point_point(intersection_line[0], c1[0])
    dist2 = distance_point_point(intersection_line[1], c2[0])

    # Construct Joint object
    #print ("Creating 2 Joints: Beam=%s,Face=%s,Distance=%s, Beam=%s,Face=%s,Distance=%s" % (bid_1, f1, dist1, bid_2, f2, dist2))
    joint1 = Joint_90lap(dist1, f1, beam1.get_face_width(f1), beam2.get_face_width(f2), beam1.get_face_height(f1)/2, name = '%s-%s'%(beam1.name, beam2.name))
    joint2 = Joint_90lap(dist2, f2, beam2.get_face_width(f2), beam1.get_face_width(f1), beam2.get_face_height(f2)/2, name = '%s-%s'%(beam2.name, beam1.name))

    return (joint1, joint2)


if __name__ == "__main__":
    import compas
    import tempfile
    import os

    #Test to create Joint_90lap object. Serialize and deserialize.
    #j.data and q.data should have the same value

    #Create Joint object
    from compas.geometry import Frame
    joint = Joint_90lap(180,1,50,100,100)
    print (joint.data)

    #Save Joint to Json
    joint.to_json(os.path.join(tempfile.gettempdir(), "joint.json"),pretty=True)

    #Load saved Joint Object
    loaded_joint = Joint_90lap.from_json(os.path.join(tempfile.gettempdir(), "joint.json"))

    #Assert that the two Joint objects are different objects
    assert (joint is not loaded_joint)

    print("Test 1: Comparing two beam data dictionary:")
    assert (joint.data == loaded_joint.data)
    if (joint.data == loaded_joint.data):
        print("Correct")
    else:
        print("Incorrect")

    print (joint.data)




