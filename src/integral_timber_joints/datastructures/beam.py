# This file originates from Keerthana Udaykumar's <kudaykum@student.ethz.ch> master thesis on `Timber Grammar` in Fall 2019
# Pulled from https://github.com/KEERTHANAUDAY/timber_grammar on 2019-10-09 commit: d389364304f3740e652dbe4e85291402a2df0636
# This file is there on modified by Victor Leung <leung@arch.eth.ch> for use in Integral Timber Joint project

#
# Beam Object
#   Beam objects are used to model a linear beam object that
#   It holds a list of Joint objects
#

import math
import compas
from compas.geometry import Box
from compas.datastructures import Mesh
from compas.datastructures import mesh_bounding_box
from compas.geometry import Frame
from compas.geometry import Plane
from compas.geometry import Translation
from compas.geometry import distance_point_plane
import json

from .joint import Joint
from .utils import create_id

from compas.rpc import Proxy
import sys

__all__ = ['Beam']

class Beam(object):

    def __init__(self, frame, length, width, height, name ):
        """ initialization

            :frame:           base plane for the beam
            :length:          the length along the local x-axis (Typically the long side)
            :width:           the length along the local z-axis (Short Side of Side 1 and Side 3)
            :height:          the length along the local y-axis (Short Side of Side 2 and Side 4)
            :name(optional):  UUID generated frim id_generator as name for each mesh

        """
        self.frame = frame
        self.length = length
        self.width = width
        self.height = height
        self.name = name
        # self.face_frame = None
        self.mesh = None
        self.joints = []

        #Perform initial calculation of the mesh if this object is not empty
        if frame is not None:
            self.update_mesh()


    @property
    def data(self):
        """dict : A data dict representing all internal persistant data for serialisation.
        The dict has the following structure:
        * 'type'            => string (name of the class)
        * 'frame'           => dict of compas.frame.to_data()
        * 'length'          => double
        * 'width'           => double
        * 'height'          => double
        * 'mesh'            => dict of compas.mesh.to_data()
        * 'joints'          => list of dict of Joint.to_data()
        * 'name'            => string
        """
        data = {
            'type'      : self.__class__.__name__, #Keep this line for deserialization
            'frame'     : None,
            'length'    : self.length,
            'width'     : self.width,
            'height'    : self.height,
            'name'      : self.name,
            'mesh'      : None,
            'joints'    : [joint.to_data() for joint in self.joints],
            }
        if (self.frame is not None) :
            data['frame'] = self.frame.to_data()
        if (self.mesh is not None) :
            data['mesh'] = self.mesh.to_data()
        return data

    @data.setter
    def data(self, data):
        """ data setter for data dictionary
        """
        self.frame      = None
        if (data.get('frame') is not None): self.frame = compas.geometry.Frame.from_data(data.get('frame'))
        self.length      = data.get('length') or None
        self.width      = data.get('width') or None
        self.height     = data.get('height') or None
        self.name       = data.get('name') or None
        self.mesh       = None
        if (data.get('mesh') is not None): self.mesh = compas.datastructures.Mesh.from_data(data.get('mesh'))
        for joint_data in data.get('joints') :
            self.joints.append(Joint.from_data(joint_data))

    @classmethod
    def from_data(cls,data):
        """Construct a Beam object from structured data.
        Parameters
        ----------
        data : dict
            The data dictionary.

        Returns
        -------
        object
            An object of the type Beam

        Note
        ----
        This constructor method is meant to be used in conjuction with the
        corresponding *to_data* method.

        """

        new_object = cls(None,None,None,None,None) # Only this line needs to be updated
        new_object.data = data
        if new_object.mesh is None:
            new_object.update_mesh()
        return new_object


    def to_data(self):
        """Returns a dictionary of structured data representing the data structure.
        Actual implementation is in @property def data(self)

        Returns
        -------
        dict
            The structured data.

        Note
        ----
        This method produces the data that can be used in conjuction with the
        corresponding *from_data* class method.

        """
        return self.data

    @classmethod
    def from_json(cls, filepath):
        """Construct a datastructure from structured data contained in a json file.

        Parameters
        ----------
        filepath : str
            The path to the json file.

        Returns
        -------
        object
            An object of the type of ``cls``.

        Note
        ----
        This constructor method is meant to be used in conjuction with the
        corresponding *to_json* method.

        """
        # PATH = os.path.abspath(os.path.join(filepath, '..', 'data'))
        with open(filepath, 'r') as fp:
            data = json.load(fp)
        print('Json Loaded: Type=', data['type'])

        assert data['type'] ==  cls.__name__ , "Deserialized object type: %s is not equal to %s." % (data['type'] , cls.__name__)
        return cls.from_data(data)

    def to_json(self, filepath, pretty=False):
        """Serialise the structured data representing the data structure to json.

        Parameters
        ----------
        filepath : str
            The path to the json file.

        """
        # PATH = os.path.abspath(os.path.join(filepath, '..', 'data'))
        with open(filepath, 'w+') as fp:
            if pretty:
                json.dump(self.data, fp, sort_keys=True, indent=4)
            else:
                json.dump(self.data, fp)

    #Here is where the functions of the class begins
    def update_mesh(self):
        """Computes the beam geometry with boolean difference of all joints.

        Returns
        -------
        compas.datastructures.Mesh
        The beam mesh with joint geoemtry removed

        Note
        ----------
        self.mesh is updated.
        """

        meshes = []
        #First mesh in the list is the uncut beam mesh
        self.mesh = self.draw_uncut_mesh()
        if len(self.joints) > 0 :
            meshes.append(self.mesh)

            #Extract mesh objects from each joint in the beam
            for joint in self.joints:
                meshes.append(joint.mesh)

            #Calls trimesh to perform boolean
            self.mesh = self.trimesh_proxy_subtract_multiple(meshes) #why am i giving joint.mesh and not joint, isn't trimesh_proxy_subtract a classmethod?

        self.mesh.name = self.name

        return self.mesh

    def Get_distancefromBeamYZFrame(self,placed_point):
        """Computes the distance from selected point to Beam YZ_Plane(face_id = 0)
        Parameters:
        ----------
        BeamRef: Beam Object
        placed_point: Point3d
        Return:
        ------
        distance (double)
        """
        YZ_Plane = self.face_plane(0)
        dist = distance_point_plane(placed_point,YZ_Plane)
        return dist

    def center_point_at_beam_start(self):
        """Computes the centroid of a beam
        ----------
        Return:
        ------
        point[list]
        """
        return self.frame.represent_point_in_global_coordinates([0,self.width/2,self.height/2])

    # Here we compute the face_frame of the beam
    def face_frame(self,face_id):
        """Computes the frame of the selected face
        ----------
        face_id: (int) ID of selected face of Beam

        Return:
        ------
        compas Frame
        """
        if face_id == 1:
            return self.frame.copy()
        if face_id == 2:
            new_origin = self.frame.represent_point_in_global_coordinates([0,self.height,0])
            return Frame(new_origin,self.frame.xaxis, self.frame.normal)
        if face_id == 3:
            new_origin = self.frame.represent_point_in_global_coordinates([0,self.height,self.width])
            return Frame(new_origin,self.frame.xaxis, self.frame.yaxis * -1.0)
        if face_id == 4:
            new_origin = self.frame.represent_point_in_global_coordinates([0,0,self.width])
            return Frame(new_origin,self.frame.xaxis, self.frame.normal * -1.0)
        else:
            raise IndexError('face_id index out of range')

    def face_plane(self,plane_id):
        """Computes the plane of each face of a beam
        ----------
        plane_id: (int) ID of plane

        Return:
        ------
        compas plane
        """

        origin_frame = self.frame.copy()
        if plane_id == 0:
            plane = Plane(origin_frame.point, origin_frame.xaxis)

        elif plane_id == 1:
            face_1 = self.face_frame(1).copy()
            plane = Plane(face_1.point, face_1.yaxis)

        elif plane_id == 2:
            face_2 = self.face_frame(2).copy()
            plane = Plane(face_2.point, face_2.yaxis)

        elif plane_id == 3:
            face_3 = self.face_frame(3).copy()
            plane = Plane(face_3.point, face_3.yaxis)

        elif plane_id == 4:
            face_4 = self.face_frame(4).copy()
            plane = Plane(face_4.point, face_4.yaxis)

        elif plane_id == 5: #horizontal plane that at the center of the beam
            center_point = self.center_point_at_beam_start()
            plane = Plane(center_point, self.frame.normal)

        elif plane_id == 6: #vertical plane that at the center of the beam
            center_point = self.center_point_at_beam_start()
            plane = Plane(center_point, self.frame.yaxis)

        return plane

    def neighbour_face_plane(self,plane_id):
        """Computes the adjacent planes of a plane
        ----------
        plane_id: (int) ID of plane

        Return:
        ------
        compas plane
        """
        if plane_id == 1:
            return [self.face_plane(2),self.face_plane(4)]
        if plane_id == 2:
            return [self.face_plane(3),self.face_plane(1)]
        if plane_id == 3:
            return [self.face_plane(4),self.face_plane(2)]
        if plane_id == 4:
            return [self.face_plane(1),self.face_plane(3)]

    def draw_uncut_mesh(self):
        """Computes and returns the beam geometry.

        Returns
        -------
        compas.datastructures.Mesh
        The beam mesh without joint geoemtry

        """
        box = Box(self.frame, self.length,self.width,self.height)
        box_mesh = Mesh.from_vertices_and_faces(box.vertices, box.faces)
        return box_mesh

    # def draw_cut_match_mesh(self,match_beam_mesh):

    #     for joint in self.joints:
    #         self.mesh = self.trimesh_proxy_subtract(match_beam_mesh,joint.mesh)
    #     return self.mesh

    @classmethod #hence does not rely on the instance of the Beam class, inout of the type is enough
    def trimesh_proxy_subtract(cls,mesh_a,mesh_b):
        """Computes boolean through trimesh by calling compas proxy.

        Returns
        -------
        compas.datastructures.Mesh

        """
        # with Proxy(package='Trimesh_proxy',python=python_exe_path) as f:
        with Proxy(package='timber_grammar.Trimesh_proxy') as f:
            result = f.trimesh_subtract(mesh_a, mesh_b)
            result_mesh = Mesh.from_data(result['value'])
        return result_mesh

    @classmethod #hence does not rely on the instance of the Beam class, inout of the type is enough
    def trimesh_proxy_subtract_multiple(cls,meshes):
        """Computes boolean through trimesh by calling compas proxy.

        Returns
        -------
        compas.datastructures.Mesh

        """
        with Proxy(package='timber_grammar.Trimesh_proxy') as f:
            result = f.trimesh_subtract_multiple(meshes)
            result_mesh = Mesh.from_data(result['value'])
        return result_mesh

        # with Proxy(package='Trimesh_proxy',python=python_exe_path) as f:
        #     result = f.trimesh_subtract_multiple(meshes)
        #     result_mesh = Mesh.from_data(result['value'])
        # return result_mesh

    @classmethod
    def debug_get_dummy_beam(cls, include_joint=False):
        from compas.geometry.primitives import Frame
        from compas.geometry.primitives import Point
        from compas.geometry.primitives import Vector
        from .joint_90lap import Joint_90lap
        #Create Beam object
        beam = cls(Frame(Point(0, 0, 0), Vector(0, 1, 0), Vector(0, 0, 1)),1000,100,150,"dummy_beam_1")
        #Create Joint object
        if (include_joint):
            beam.joints.append(Joint_90lap(100,3,100,100,50))
            #Update mesh - Boolean the joints from Mesh
            beam.joints[0].update_joint_mesh(beam)

        beam.update_mesh()
        return beam



if __name__ == '__main__':
    import compas
    import tempfile
    import os

    #Test 1 : Beam data to be saved and loaded and the two should be the same.
    beam = Beam.debug_get_dummy_beam()

    print("Test 1: Beam Data Save and Load to JSON")

    #Save Beam to Json
    beam.to_json(os.path.join(tempfile.gettempdir(), "beam.json"),pretty=True)

    #Load saved Beam Object
    loaded_beam = Beam.from_json(os.path.join(tempfile.gettempdir(), "beam.json"))

    #Assert that the two beam objects are different objects
    assert (beam is not loaded_beam)

    print("Test 1: Comparing two beam data dictionary:")
    assert (beam.data == loaded_beam.data)
    if (beam.data == loaded_beam.data):
        print("Correct")
    else:
        print("Incorrect")

    print("-- -- -- -- -- -- -- --")


    #Test 2 : Beam with Joint data attached and saved and loaded.
    print("Test 2: Beam with Joint Data Attached")

    #Create some joints on the beam
    beam = Beam.debug_get_dummy_beam(include_joint=True)

    #Save Beam with the appended Joint to Json
    beam.to_json(os.path.join(tempfile.gettempdir(), "beam.json"),pretty=True)

    #Load the saved Beam
    loaded_beam = Beam.from_json(os.path.join(tempfile.gettempdir(), "beam.json"))

    print("Test 2: Comparing two beam data dictionary:")
    assert (beam.data == loaded_beam.data)
    if (beam.data == loaded_beam.data):
        print("Correct")
    else:
        print("Incorrect")

    print("-- -- -- -- -- -- -- --")

    print("Test 3: Print out dummy beam (with Joint) data")
    print (beam.data)

    print("-- -- -- -- -- -- -- --")
