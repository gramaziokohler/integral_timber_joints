# This file originates from Keerthana Udaykumar's <kudaykum@student.ethz.ch> master thesis on `Timber Grammar` in Fall 2019
# Pulled from https://github.com/KEERTHANAUDAY/timber_grammar on 2019-10-09 commit: d389364304f3740e652dbe4e85291402a2df0636
# This file is there on modified by Victor Leung <leung@arch.eth.ch> for use in Integral Timber Joint project

#
# Beam Object
#   Beam objects are used to model a linear beam object that
#   It holds a list of Joint objects
#

import math
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

from integral_timber_joints.geometry.joint import Joint
from integral_timber_joints.geometry.utils import create_id

from compas_ghpython.artists import MeshArtist

from copy import deepcopy

import sys

__all__ = ['Beam']

class Beam(object):

    def __init__(self, frame, length, width, height, name = None):
        """
        initialization
        ----------------
            frame:      compas.geometry.Frame           base plane for the beam
            length:     int, float      the length along the local x-axis (Typically the long side)
            width:      int, float      the length along the local z-axis (Short Side of Side 1 and Side 3)
            height:     int, float      the length along the local y-axis (Short Side of Side 2 and Side 4)
            name:       str(optional):  UUID generated from id_generator as name for each mesh

        """
        if (frame is None): frame = Frame.worldXY()
        self.frame = frame.copy()
        self.length = float(length)     # type: float
        self.width = float(width)       # type: float
        self.height = float(height)     # type: float
        if (name == None): name = create_id()
        self.name = name                # type: str
        self.cached_mesh = None         # type: compas.datastructures.Mesh


        # Attribute for animation visualization
        self.is_visible = True # Decides if the self.draw_visuals() draw or not.
        self.is_attached = True # Decides if the self.draw_visuals() draws it differently.
        self.current_location = frame.copy() # Not Serialized
        self.storage_location = frame.copy() # Not Serialized
        self.grasp_frame = Frame.worldXY() # Not Serialized # Local Coords


    # -----------------------
    # Constructors
    # -----------------------

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

    # -----------------------
    # Str Repr Copy
    # -----------------------

    def __str__(self):
        return "Beam (%s)" % self.name

    # -----------------------
    # Computed Properity
    # -----------------------

    def get_point_distance_from_beam_start(self, placed_point):
        """Computes the distance from selected point to Beam YZ_Plane(face_id = 0)
        Parameters:
        ----------
        BeamRef: Beam Object
        placed_point: Point3d
        Return:
        ------
        distance (double)
        """
        YZ_Plane = self.get_face_plane(0)
        dist = distance_point_plane(placed_point,YZ_Plane)
        return dist

    def get_center_point_at_beam_start(self, length = 0):
        """Computes the centroid of a beam
        ----------
        Return:
        ------
        point[list]
        """
        return self.frame.to_world_coords([length,self.width/2,self.height/2])

    def get_face_frame(self,face_id):
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
            new_origin = self.frame.to_world_coords([0,self.width,0])
            return Frame(new_origin,self.frame.xaxis, self.frame.normal)
        if face_id == 3:
            new_origin = self.frame.to_world_coords([0,self.width,self.height])
            return Frame(new_origin,self.frame.xaxis, self.frame.yaxis * -1.0)
        if face_id == 4:
            new_origin = self.frame.to_world_coords([0,0,self.height])
            return Frame(new_origin,self.frame.xaxis, self.frame.normal * -1.0)
        else:
            raise IndexError('face_id index out of range')

    def get_face_plane(self,plane_id):
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
            face_1 = self.get_face_frame(1).copy()
            plane = Plane(face_1.point, face_1.yaxis)

        elif plane_id == 2:
            face_2 = self.get_face_frame(2).copy()
            plane = Plane(face_2.point, face_2.yaxis)

        elif plane_id == 3:
            face_3 = self.get_face_frame(3).copy()
            plane = Plane(face_3.point, face_3.yaxis)

        elif plane_id == 4:
            face_4 = self.get_face_frame(4).copy()
            plane = Plane(face_4.point, face_4.yaxis)

        elif plane_id == 5: #horizontal plane that at the center of the beam
            center_point = self.get_center_point_at_beam_start()
            plane = Plane(center_point, self.frame.normal)

        elif plane_id == 6: #vertical plane that at the center of the beam
            center_point = self.get_center_point_at_beam_start()
            plane = Plane(center_point, self.frame.yaxis)

        return plane

    def get_face_width(self,face_id):
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

    def get_face_height(self,face_id):
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

    def get_center_line(self):
        start = self.frame.to_world_coords([0, self.width/2, self.height/2])
        end = self.frame.to_world_coords([self.length, self.width/2, self.height/2])
        return Line(start, end)

    def get_face_center_line(self, face_id):
        start = self.get_face_frame(face_id).to_world_coords([0, 0, self.get_face_width(face_id)/2])
        end = self.get_face_frame(face_id).to_world_coords([self.length, 0, self.get_face_width(face_id)/2])
        return Line(start, end)

    def get_neighbour_face_plane(self,plane_id):
        """Computes the adjacent planes of a plane
        ----------
        plane_id: (int) ID of plane

        Return:
        ------
        compas plane
        """
        if plane_id == 1:
            return [self.get_face_plane(2),self.get_face_plane(4)]
        if plane_id == 2:
            return [self.get_face_plane(3),self.get_face_plane(1)]
        if plane_id == 3:
            return [self.get_face_plane(4),self.get_face_plane(2)]
        if plane_id == 4:
            return [self.get_face_plane(1),self.get_face_plane(3)]

    # -----------------------
    # Geometrical
    # -----------------------

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
        from compas.datastructures import mesh_quads_to_triangles
        box_mesh = Mesh.from_vertices_and_faces(box.vertices, box.faces) # type: compas.datastructures.Mesh
        mesh_quads_to_triangles(box_mesh)
        return box_mesh

    def update_cached_mesh(self, beam_features = []):
        """Computes the beam geometry with boolean difference of all joints and features.
        This is manually triggered.
        Parameters
        ----------
        beam_features: list(BeamFeature)
            Objects that implements the BeamFeature ABC

        Returns
        -------
        compas.datastructures.Mesh
            The beam mesh with joint geoemtry removed

        Note
        ----------
        self.cached_mesh is updated.
        features object need to implement the get_feature_mesh(beam)->Mesh function.
        """

        meshes = []
        #First mesh in the list is the uncut beam mesh
        self.cached_mesh = self.draw_uncut_mesh()

        if len(beam_features) > 0 :
            meshes.append(self.cached_mesh)

            #Compute the negative meshes from the features
            for feature in beam_features:
                negative_mesh = feature.get_feature_mesh(self)
                meshes.append(negative_mesh)

            #Calls trimesh to perform boolean
            from compas.rpc import Proxy
            trimesh_proxy = Proxy(package='compas_trimesh')
            #print(meshes)
            result = trimesh_proxy.trimesh_subtract_multiple(meshes)
            self.cached_mesh = Mesh.from_data(result['value'])

        self.cached_mesh.name = self.name + "_mesh"

        return self.cached_mesh


    def set_state(self, state_dict):
        '''
        This function serves animation / viauslization purpose only
        '''
        # state is simply a frame
        self.current_location = state_dict['current_location'].copy()   # Frame
        self.is_visible = state_dict['is_visible']                      # Boolean
        self.is_attached = state_dict['is_attached']                      # Boolean

    def get_state(self):
        '''
        This function serves animation / viauslization purpose only
        '''
        state_dict = {}
        state_dict['current_location'] = self.current_location.copy()
        state_dict['is_visible'] = self.is_visible
        state_dict['is_attached'] = self.is_attached


        return state_dict

    def draw_visuals(self):
        '''
        This function serves animation / viauslization purpose only
        '''
        if not self.is_visible: return []
        # Create the transformation
        T = Transformation.from_frame_to_frame(self.frame, self.current_location)
        # Create a copy of the mesh, transformed.
        transformed_mesh = self.cached_mesh.transformed(T)
        # Use a mesh artist to draw everything
        artist = MeshArtist(transformed_mesh)
        visual = artist.draw_mesh()

        # We should have only one mesh per beam object
        visuals = [visual]
        return visuals

    def copy(self):
        return deepcopy(self)

    # -----------------------
    # Intersection
    # -----------------------

    def get_beam_beam_coplanar_face_ids(self, neighbour_beam):
        # type: (Beam, Beam): List[Tuple[str,str]]
        """
        Computes the faces that are coplanar between two beams
        Returns:
            List of [Tuples (face_id on self, face_id on neighbour_beam)]
        """
        ffx = []
        for i in range(1,5):
            p1 = self.get_face_plane(i)
            for j in range(1,5):
                p2 = neighbour_beam.get_face_plane(j)
                if (is_point_on_plane(p1.point,p2) and is_point_on_plane(p2.point,p1)):
                    #print ("Intersection self.Face%s Beam2.Face%s " %(i , j))
                    ffx.append((i,j))
        return ffx

    @classmethod
    def debug_get_dummy_beam(cls, include_joint=False):
        from compas.geometry import Frame
        from compas.geometry import Point
        from compas.geometry import Vector
        from integral_timber_joints.geometry.joint_90lap import Joint_90lap
        #Create Beam object
        beam = cls(Frame(Point(0, 0, 0), Vector(0, 1, 0), Vector(0, 0, 1)),1000,100,150,"dummy_beam_1")
        #Create Joint object
        if (include_joint):
            return (beam, Joint_90lap(100,3,100,100,50,"dummy_joint"))
        else:
            return beam


if __name__ == '__main__':
    pass
