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

from integral_timber_joints.datastructures.joint import Joint

__all__ = ['Joint_90lap']


class Joint_90lap(Joint):
    """
    joint class containing varied joints
    """
    def __init__(self, distance,face_id, length, width, height):

        """
        :param distance:  double
        :param face_id:   int
        """
        self.distance = distance
        self.face_id = face_id
        self.length = length
        self.width = width
        self.height = height
        self.mesh = None

        # #Perform initial calculation of the mesh (except when this is an empty object)
        # if frame is not None:
        #     self.update_joint_mesh()


    @property
    def data(self):
        """dict : A data dict representing all internal persistant data for serialisation.
        The dict has the following structure:
        * 'type'            => string (name of the class)
        * 'distance'        => double
        * 'face_id'         => int
        * 'length'           => double
        * 'width'           => double
        * 'height'          => double
        * 'mesh'            => dict of compas.mesh.to_data()
        """
        data = {
            'type'      : self.__class__.__name__, #Keep this line for deserialization
            'distance'  : self.distance,
            'face_id'   : self.face_id,
            'length'    : self.length,
            'width'     : self.width,
            'height'    : self.height,
            'mesh'      : None,
            }
        if (self.mesh is not None) :
            data['mesh'] = self.mesh.to_data()
        return data

    @data.setter
    def data(self, data):
        self.distance    = data.get('distance') or None
        self.face_id    = data.get('face_id') or None
        self.length      = data.get('length') or None
        self.width      = data.get('width') or None
        self.height     = data.get('height') or None
        self.mesh       = None
        if (data.get('mesh') is not None): self.mesh = compas.datastructures.Mesh.from_data(data.get('mesh'))

    @classmethod
    def from_data(cls,data):
        new_object = cls(None,None,None,None,None) # Only this line needs to be updated
        new_object.data = data
        return new_object

    def update_joint_mesh(self, BeamRef):
        """Compute the negative mesh volume of the joint.

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
        face_frame = BeamRef.face_frame(self.face_id)

        box_frame_origin = face_frame.represent_point_in_global_coordinates([(self.distance - 50), TOLEARNCE * -1.0, TOLEARNCE * -1.0])
        box_frame = Frame(box_frame_origin, face_frame.xaxis, face_frame.yaxis)

        # Compute 3 Box dimensions
        box_x = self.length
        box_y = self.width + TOLEARNCE
        box_z = self.height + 2 * TOLEARNCE

        # Draw Boolean Box
        boolean_box = Box(box_frame, box_x, box_y, box_z)
        boolean_box_mesh = Mesh.from_vertices_and_faces(boolean_box.vertices, boolean_box.faces)

        # Draw boolean box and assign to self.mesh
        self.mesh = boolean_box_mesh
        return self.mesh


if __name__ == "__main__":
    import compas
    import tempfile
    import os

    #Test to create Joint_90lap object. Serialize and deserialize.
    #j.data and q.data should have the same value

    #Create Joint object
    from compas.geometry.primitives import Frame
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




