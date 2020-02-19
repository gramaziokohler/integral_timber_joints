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
from compas.geometry import Point
from compas.geometry import Line
from compas.geometry import Transformation
from compas.geometry import Translation
from compas.geometry import distance_point_plane
from compas.geometry import is_point_on_plane
import json

from integral_timber_joints.datastructures.joint import Joint
from integral_timber_joints.datastructures.utils import create_id



import sys

__all__ = ['Beam']

class Beam(object):

    def __init__(self, frame, length, width, height, name = None):
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
        if (name == None): name = create_id()
        self.name = name
        # self.face_frame = None
        self.mesh = None
        self.joints = []

        #Perform initial calculation of the mesh if this object is not empty
        if frame is not None:
            self.update_mesh()

    def __str__(self):
        return "Beam (%s)" % self.name   

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
        self.length      = data.get('length')
        self.width      = data.get('width')
        self.height     = data.get('height')
        self.name       = data.get('name')
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

    @classmethod
    def from_mesh(cls, raw_mesh, orientation_frame):
        """Construct a beam from a mesh.

        Performs bounding box calculation on the mesh and create a rectangular beam.
        
        Parameters
        ----------
        raw_mesh : :class:`Mesh`
            Compas Mesh datastructure describing the beam. Notches will be ignored.
        orientation_frame : :class:`Frame`
            Orientation frame for computing bounding box of the beam.
            X Axis +ve = Length
            Y Axis +ve = Height
            Z Axis +ve = Width

        Returns
        -------
        :class:`Beam`
            New instance of Beam.
        """

        # transform mesh to oriented frame and perform bounding box
        
        T = Transformation.from_frame_to_frame(orientation_frame, Frame.worldXY())
        transformed_mesh = raw_mesh.transformed(T)
        bbox = mesh_bounding_box(transformed_mesh)

        # transform back
        origin_point = Point(*bbox[0])
        origin_point.transform(T.inverse())
        bbox_size = Point(*bbox[6]) - Point(*bbox[0])
        
        length = bbox_size[0]
        height = bbox_size[1]
        width = bbox_size[2]
        
        xaxis = orientation_frame.xaxis
        yaxis = orientation_frame.yaxis
        beam_frame = Frame(origin_point,xaxis,yaxis)

        beam = cls(beam_frame, length, width, height, None )
        beam.mesh = raw_mesh

        return beam
        
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
                if joint.mesh == None:
                    joint.update_joint_mesh(self)
                meshes.append(joint.mesh)

            #Calls trimesh to perform boolean

            from compas.rpc import Proxy
            trimesh_proxy = Proxy(package='compas_trimesh')
            #print(meshes)
            result = trimesh_proxy.trimesh_subtract_multiple(meshes)
            self.mesh = Mesh.from_data(result['value'])

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

    def center_point_at_beam_start(self, length = 0):
        """Computes the centroid of a beam
        ----------
        Return:
        ------
        point[list]
        """
        return self.frame.to_world_coords([length,self.width/2,self.height/2])

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
            new_origin = self.frame.to_world_coords([0,self.height,0])
            return Frame(new_origin,self.frame.xaxis, self.frame.normal)
        if face_id == 3:
            new_origin = self.frame.to_world_coords([0,self.height,self.width])
            return Frame(new_origin,self.frame.xaxis, self.frame.yaxis * -1.0)
        if face_id == 4:
            new_origin = self.frame.to_world_coords([0,0,self.width])
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

    def face_width(self,face_id):
        """Gets the width of the selected face
        Corrisponds to the dimension of the beam which is the width of the selected face.
        ----------
        plane_id: (int) ID of plane

        Return:
        ------
        Face 1 and 3 = beam.height
        Face 2 and 4 = beam.width
        
        """
        if face_id == 1:
            return self.height
        if face_id == 2:
            return self.width
        if face_id == 3:
            return self.height
        if face_id == 4:
            return self.width
        else:
            raise IndexError()

    def face_height(self,face_id):
        """Gets the height (depth) of the selected face.
        Corrisponds to the dimension of the beam which is normal to the selected face.
        ----------
        plane_id: (int) ID of plane

        Return:
        ------
        Face 1 and 3 = beam.width
        Face 2 and 4 = beam.height
        
        """
        if face_id == 1:
            return self.width
        if face_id == 2:
            return self.height
        if face_id == 3:
            return self.width
        if face_id == 4:
            return self.height
        else:
            raise IndexError()

            
    def center_line(self):
        start = self.frame.to_world_coords([0, self.width/2, self.height/2])
        end = self.frame.to_world_coords([self.length, self.width/2, self.height/2])
        return Line(start, end)

    def face_center_line(self, face_id):
        start = self.face_frame(face_id).to_world_coords([0, 0, self.face_width(face_id)/2])
        end = self.face_frame(face_id).to_world_coords([self.length, 0, self.face_width(face_id)/2])
        return Line(start, end)

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
        # Compas Box origin is at the center of the box
        box_center_point = self.frame.to_world_coords(Point(self.length/2, self.width/2, self.height/2))
        box_center_frame = Frame(box_center_point, self.frame.xaxis, self.frame.yaxis)
        box = Box(box_center_frame, self.length,self.width,self.height)

        # Convert Box to Mesh
        box_mesh = Mesh.from_vertices_and_faces(box.vertices, box.faces)
        return box_mesh

    def add_joint(self, joint, update_mesh = True):
        """Add a Joint to the current Beam and triggers mesh update
        ----------
        joint: Joint object

        Return:
            None
        """

        self.joints.append(joint)
        if update_mesh:
            joint.update_joint_mesh(self)
            self.update_mesh()

    def copy(self):
        cls = type(self)
        return cls.from_data(self.to_data())

    @classmethod
    def beam_beam_coplanar(cls,beam1, beam2):
        """
        Computes the faces that are coplanar between two beams
        Returns:
            List of [Tuples (face_id on Beam 1, face_id on Beam 2)]
        """
        ffx = []
        for i in range(1,5):
            p1 = beam1.face_plane(i)
            for j in range(1,5):
                p2 = beam2.face_plane(j)
                if (is_point_on_plane(p1.point,p2) and is_point_on_plane(p2.point,p1)):
                    #print ("Intersection Beam1.Face%s Beam2.Face%s " %(i , j))
                    ffx.append((i,j))
        return ffx   

    @classmethod
    def debug_get_dummy_beam(cls, include_joint=False):
        from compas.geometry.primitives import Frame
        from compas.geometry.primitives import Point
        from compas.geometry.primitives import Vector
        from integral_timber_joints.datastructures.joint_90lap import Joint_90lap
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

    print("Test4: Test Copy class")
    beam_copied = beam.copy()

    if (id(beam) != id(beam_copied)):
        print("Copy has different ID - Correct")
    else:
        print("Copy same ID - Incorrect")

    if (id(beam.joints[0]) != id(beam_copied.joints[0])):
        print("Copy Beam.Joints has different ID - Correct")
    else:
        print("Copy Beam.Joints same ID - Incorrect")


    if (beam.data == beam_copied.data):
        print("Copy data is same - Correct")
    else:
        print("Copy data is not same - Incorrect")
    print("-- -- -- -- -- -- -- --")
