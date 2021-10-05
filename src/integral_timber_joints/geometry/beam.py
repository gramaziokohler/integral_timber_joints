# This file originates from Keerthana Udaykumar's <kudaykum@student.ethz.ch> master thesis on `Timber Grammar` in Fall 2019
# Pulled from https://github.com/KEERTHANAUDAY/timber_grammar on 2019-10-09 commit: d389364304f3740e652dbe4e85291402a2df0636
# This file is there on modified by Victor Leung <leung@arch.eth.ch> for use in Integral Timber Joint project

#
# Beam Object
#   Beam objects are used to model a linear beam object that
#   It holds a list of Joint objects
#
try:
    from typing import Dict, List, Optional, Tuple

    from integral_timber_joints.process.state import ObjectState
except:
    pass

import math

import compas
from compas.datastructures import Mesh, Network, mesh_bounding_box
from compas.geometry import Box, Point, Transformation, Translation, distance_point_plane, is_point_on_plane
from compas.geometry import Frame, Line, Plane, Vector


from integral_timber_joints.geometry.joint import Joint
from integral_timber_joints.geometry.utils import create_id

if compas.IPY:
    try:
        from compas_ghpython.artists import MeshArtist
    except:
        pass


from copy import deepcopy

__all__ = ['Beam']


class Beam(Network):

    def __init__(self, frame=None, length=1000, width=100, height=100, name=None):
        """
        initialization
        ----------------
            frame:      compas.geometry.Frame           base plane for the beam
            length:     int, float      the length along the local x-axis (Typically the long side)
            width:      int, float      the length along the local z-axis (Short Side of Side 1 and Side 3)
            height:     int, float      the length along the local y-axis (Short Side of Side 2 and Side 4)
            name:       str(optional):  UUID generated from id_generator as name for each mesh

        """
        super(Beam, self).__init__()
        if (frame is None):
            frame = Frame.worldXY()
        # self.frame = frame.copy()       # type: Frame
        self.attributes['frame'] = frame.copy()
        self.attributes['current_frame'] = frame.copy()
        self.attributes['length'] = float(length)
        self.attributes['width'] = float(width)
        self.attributes['height'] = float(height)

        if (name == None):
            name = create_id()
        self.attributes['name'] = name
        self.attributes['cached_mesh'] = None

    # -----------------------
    # Properity access
    # -----------------------

    @property
    def name(self):
        # type: () -> str
        return self.attributes['name']

    @name.setter
    def name(self, value):
        # type: (str) -> None
        self.attributes['name'] = str(value)

    @property
    def length(self):
        # type: () -> float
        return self.attributes['length']

    @length.setter
    def length(self, value):
        # type: (float) -> None
        self.attributes['length'] = float(value)

    @property
    def width(self):
        # type: () -> float
        return self.attributes['width']

    @width.setter
    def width(self, value):
        # type: (float) -> None
        self.attributes['width'] = float(value)

    @property
    def height(self):
        # type: () -> float
        return self.attributes['height']

    @height.setter
    def height(self, value):
        # type: (float) -> None
        self.attributes['height'] = float(value)

    @property
    def frame(self):
        # type: () -> Frame
        """Final position of the beam as-designed """
        return self.attributes['frame']

    @frame.setter
    def frame(self, value):
        # type: (Frame) -> None
        self.attributes['frame'] = value

    @property
    def current_frame(self):
        # type: () -> Frame
        """An extrinsic state of the beam in a different location other than the final location"""
        return self.attributes['current_frame']

    @current_frame.setter
    def current_frame(self, value):
        # type: (Frame) -> None
        self.attributes['current_frame'] = value

    @property
    def cached_mesh(self):
        # type: () -> Mesh
        """Beam mesh at the designed (final) location in WCF.
        """
        return self.attributes['cached_mesh']

    @cached_mesh.setter
    def cached_mesh(self, value):
        # type: (Mesh) -> None
        self.attributes['cached_mesh'] = value

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
        beam_frame = Frame(origin_point, xaxis, yaxis)

        beam = cls(beam_frame, length, width, height, None)
        beam.mesh = raw_mesh

        return beam

    @classmethod
    def from_centerline(cls, centerline, guide_vector, width, height):
        # type(Line, Vector, float, float): -> Beam
        '''Create a Beam form center line and width , height.
        The direction of the line is X axis of beam.
        The beam's y axis aligns to the guide vector.
        Note that the guide_vecto cannot be parallel to the centerline
        '''

        # Automatic beam_frame
        length = centerline.length

        xaxis = Vector.from_start_end(centerline.start, centerline.end)
        zaxis = xaxis.cross(guide_vector)
        yaxis = zaxis.cross(xaxis)

        # compute origin point
        origin_point = (centerline.start
                        + yaxis.unitized().scaled(height/-2)
                        + zaxis.unitized().scaled(width/-2)
                        )
        beam_frame = Frame(origin_point, xaxis, yaxis)

        beam = cls(beam_frame, length, width, height, None)
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
        dist = distance_point_plane(placed_point, YZ_Plane)
        return dist

    def get_center_point_at_beam_start(self, length=0):
        """Computes the center point of a beam on Reference Side 5 in WCF
        ----------
        Return:
        ------
        point[list]
        """
        return self.frame.to_world_coordinates([length, self.height/2, self.width/2])

    def get_center_point(self):
        """Computes the centroid of a beam in WCF
        ----------
        Return:
        ------
        point[list]
        """
        return self.frame.to_world_coordinates([self.length/2, self.height/2, self.width/2])

    def get_face_frame(self, face_id):
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
            new_origin = self.frame.to_world_coordinates([0, self.height, 0])
            return Frame(new_origin, self.frame.xaxis, self.frame.normal)
        if face_id == 3:
            new_origin = self.frame.to_world_coordinates([0, self.height, self.width])
            return Frame(new_origin, self.frame.xaxis, self.frame.yaxis * -1.0)
        if face_id == 4:
            new_origin = self.frame.to_world_coordinates([0, 0, self.width])
            return Frame(new_origin, self.frame.xaxis, self.frame.normal * -1.0)
        else:
            raise IndexError('face_id index out of range')

    def get_face_plane(self, plane_id):
        # type: (int) -> Plane
        """Computes the plane of each face of a beam in WCF
        ----------
        plane_id: (int) ID of plane

        Return:
        ------
        compas plane
        """
        plane = None

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

        elif plane_id == 5:  # horizontal plane that at the center of the beam
            center_point = self.get_center_point_at_beam_start()
            plane = Plane(center_point, self.frame.normal)

        elif plane_id == 6:  # vertical plane that at the center of the beam
            center_point = self.get_center_point_at_beam_start()
            plane = Plane(center_point, self.frame.yaxis)

        return plane

    def get_face_width(self, face_id):
        """Gets the width of the selected face
        Corrisponds to the dimension of the beam which is the width of the selected face.
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

    def get_face_height(self, face_id):
        """Gets the height (depth) of the selected face.
        Corrisponds to the dimension of the beam which is normal to the selected face.
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

    def get_center_line(self):
        # type: () -> Line
        start = self.frame.to_world_coordinates([0, self.height/2, self.width/2])
        end = self.frame.to_world_coordinates([self.length, self.height/2, self.width/2])
        return Line(start, end)

    def get_face_center_line(self, face_id):
        start = self.get_face_frame(face_id).to_world_coordinates([0, 0, self.get_face_width(face_id)/2])
        end = self.get_face_frame(face_id).to_world_coordinates([self.length, 0, self.get_face_width(face_id)/2])
        return Line(start, end)

    def get_neighbor_face_plane(self, plane_id):
        """Computes the adjacent planes of a plane
        ----------
        plane_id: (int) ID of plane

        Return:
        ------
        compas plane
        """
        if plane_id == 1:
            return [self.get_face_plane(2), self.get_face_plane(4)]
        if plane_id == 2:
            return [self.get_face_plane(3), self.get_face_plane(1)]
        if plane_id == 3:
            return [self.get_face_plane(4), self.get_face_plane(2)]
        if plane_id == 4:
            return [self.get_face_plane(1), self.get_face_plane(3)]

    # -------------------------
    # BTLx Defined Properities
    # -------------------------

    def reference_side_ocf(self, side_id):
        # type: (int) -> Frame
        """ Returns the Coordinate Frame of a reference side as defined in BTLx 1.1

        Reference to OCF
        """
        if side_id == 1:
            return Frame(self.corner_ocf(1), Vector.Xaxis(), Vector.Zaxis())
        if side_id == 2:
            return Frame(self.corner_ocf(2), Vector.Xaxis(), Vector.Yaxis().scaled(-1))
        if side_id == 3:
            return Frame(self.corner_ocf(3), Vector.Xaxis(), Vector.Zaxis().scaled(-1))
        if side_id == 4:
            return Frame(self.corner_ocf(4), Vector.Xaxis(), Vector.Yaxis())
        if side_id == 5:
            return Frame(self.corner_ocf(1), Vector.Zaxis(), Vector.Yaxis())
        if side_id == 6:
            return Frame(self.corner_ocf(6), Vector.Zaxis(), Vector.Yaxis().scaled(-1))
        raise IndexError("side_id only accepts (int) 1 - 6")

    def reference_side_wcf(self, side_id):
        # type: (int) -> Frame
        """ Returns the Coordinate Frame of a reference side as defined in BTLx 1.1
        Reference to WCF
        """
        T = Transformation.from_frame(self.frame)
        return self.reference_side_ocf(side_id).transformed(T)

    def reference_edge_ocf(self, edge_id, wrap_edge_id=True):
        """Returns the reference edge as defined in BTLx 1.1.
        For example Reference Edge 1 corrispond to the X axis of Reference Side 1.
        Line is drawn from beam start side to beam end side.
        Line coordinate in beam object coordinate frame (OCF)

        Valid edge_id are from (1 to 4), values outside will be wrapped if wrap_edge_id = True

        Returns
        -------
        compas.geometry.Line
        """

        if wrap_edge_id:
            edge_id = (edge_id - 1) % 4 + 1

        if edge_id in range(1, 5):
            return Line(self.corner_ocf(edge_id), self.corner_ocf(edge_id + 4))

        # In case of wrap_edge_id = False
        raise IndexError("edge_id only accepts (int) 1 - 4 if wrap_edge_id = False")

    def reference_edge_wcf(self, edge_id, wrap_edge_id=True):
        # type: (int, bool) -> Line
        """Returns the reference edge as defined in BTLx 1.1.
        For example Reference Edge 1 corrispond to the X axis of Reference Side 1.
        Line is drawn from beam start side to beam end side.
        Line coordinate in world coordinate frame (WCF)

        Valid edge_id are from (1 to 4), values outside will be wrapped if wrap_edge_id = True

        Returns
        -------
        compas.geometry.Line
        """
        return self.frame.to_world_coordinates(self.reference_edge_ocf(edge_id, wrap_edge_id))

    def ending_reference_side_ocf(self, side_id):
        # type: (int) -> Frame
        """ Returns the Coordinate Frame of a reference side similar to reference_side_ocf()
        but on the ending side of the beam.

        Reference to OCF
        """
        if side_id == 1:
            return Frame(self.corner_ocf(8), Vector.Xaxis().scaled(-1), Vector.Zaxis().scaled(-1))
        if side_id == 2:
            return Frame(self.corner_ocf(5), Vector.Xaxis().scaled(-1), Vector.Yaxis())
        if side_id == 3:
            return Frame(self.corner_ocf(6), Vector.Xaxis().scaled(-1), Vector.Zaxis())
        if side_id == 4:
            return Frame(self.corner_ocf(7), Vector.Xaxis().scaled(-1), Vector.Yaxis().scaled(-1))
        raise IndexError("side_id only accepts (int) 1 - 4")

    def ending_reference_side_wcf(self, side_id):
        # type: (int) -> Frame
        """ Returns the Coordinate Frame of a reference side similar to reference_side_ocf()
        but on the ending side of the beam.

        Reference to WCF
        """
        T = Transformation.from_frame(self.frame)
        return self.ending_reference_side_ocf(side_id).transformed(T)


    # -------------------------
    # Transformation
    # -------------------------

    def transform(self, transformation):
        """Transform the frame and current frame of the beam.
        This transforms the beam's frame within the WCF"""
        self.frame.transform(transformation)
        if self.current_frame is not None:
            self.current_frame.transform(transformation)
        if self.cached_mesh is not None:
            self.cached_mesh.transform(transformation)

    # -------------------------------
    # Extended Set of Geo Properities
    # -------------------------------

    def corner_ocf(self, corner):
        # type: (int) -> Point
        """Corner 1 to 4 corrisponds to the Start Point of Reference Edge 1 to 4
        Corner 5 to 8 corrisponds to the End Point of Reference Edge 1 to 4
        """
        if corner == 1:
            return Point(0, 0, 0)
        if corner == 2:
            return Point(0, self.height, 0)
        if corner == 3:
            return Point(0, self.height, self.width)
        if corner == 4:
            return Point(0, 0, self.width)

        if corner >= 5 and corner <= 8:
            return self.corner_ocf(corner - 4) + Point(self.length, 0, 0)

        raise IndexError("corner only accepts (int) 1 - 8")

    def corner_wcf(self, corner):
        # type: (int) -> Point
        """Corner 1 to 4 corrisponds to the Start Point of Reference Edge 1 to 4
        Corner 5 to 8 corrisponds to the End Point of Reference Edge 1 to 4
        """
        return self.frame.to_world_coordinates(self.corner_ocf(corner))

    def draw_uncut_mesh(self):
        """Computes and returns the beam geometry.

        Returns
        -------
        compas.datastructures.Mesh
            The beam mesh without joint geoemtry

        """
        # Compas Box origin is at the center of the box
        box_center_point = self.frame.to_world_coordinates(Point(self.length/2, self.height/2, self.width/2))
        box_center_frame = Frame(box_center_point, self.frame.xaxis, self.frame.yaxis)
        box = Box(box_center_frame, self.length, self.height, self.width)

        # Convert Box to Mesh
        # from compas.datastructures import mesh_quads_to_triangles
        box_mesh = Mesh.from_vertices_and_faces(box.vertices, box.faces)  # type: compas.datastructures.Mesh
        # mesh_quads_to_triangles(box_mesh)
        return box_mesh

    def remove_cached_mesh(self):
        # type: () -> None
        self.cached_mesh = None

    def set_state(self, state):
        # type: (ObjectState) -> None
        '''Set the state of the beam from an ObjectState
        Only current_frame is set.
        '''
        assert state.current_frame is not None
        self.current_frame = state.current_frame

    def get_state(self):
        # type: () -> ObjectState
        ''' Get an ObjectState object representing the Beam.
        Only current_frame is set.
        '''
        state = ObjectState()
        state.current_frame = self.current_frame.copy()
        return state

    def draw_visuals(self):
        '''
        This function serves animation / viauslization purpose only
        '''
        if not self.is_visible:
            return []
        # Create the transformation
        T = Transformation.from_frame_to_frame(self.frame, self.current_frame)
        # Create a copy of the mesh, transformed.
        transformed_mesh = self.cached_mesh.transformed(T)
        # Use a mesh artist to draw everything
        artist = MeshArtist(transformed_mesh)
        visual = artist.draw_mesh()

        # We should have only one mesh per beam object
        visuals = [visual]
        return visuals

    def draw_state(self, state, robot_world_transform=None):
        # type: (ObjectState, Optional[Transformation]) -> List[Mesh]
        """Returns the collision meshe(s) of the Beam (typically 1) for a given state.

        State Change
        ------------
        Calling this function sets the beam's state to the given state.
        If desired, call `beam.get_state()` to backup the current state.

        Two transformation is performed:
        - `state.current_frame` if not `None` (typically it is None for EnvironmentMesh)
        - `robot_world_transform` if not `None`
        """
        self.set_state(state)

        # The cached_mesh is located at self.frame
        T = Transformation.from_frame_to_frame(self.frame, state.current_frame)
        transformed_mesh = self.cached_mesh.transformed(T)  # type: Mesh

        if robot_world_transform is not None:
            assert isinstance(robot_world_transform, Transformation)
            transformed_mesh = transformed_mesh.transform(robot_world_transform)

        return [transformed_mesh]

    def copy(self):
        return deepcopy(self)

    # -----------------------
    # Gripper / Clamp
    # -----------------------

    def grasp_frame_ocf(self, side_id, dist_from_start):
        # type: (int, float) -> Frame
        ''' Returns the Grasp Frame according to side_id and dist_from_start.
        Grasp Frame Origin coincide with the center line on the selected surface.
        Grasp Frame X Axis is aligned to the beam's X Axis
        Grasp Frame Z Axis is pointing to the center of the beam
        Reference to OCF
        '''
        ref_side_ocf = self.reference_side_ocf(side_id)
        T = Transformation.from_frame(ref_side_ocf)
        new_origin = Point(dist_from_start, self.get_face_width(side_id) / 2, 0).transformed(T)
        return Frame(new_origin, ref_side_ocf.xaxis, ref_side_ocf.yaxis.scaled(-1.0))
    # -----------------------
    # Intersection
    # -----------------------

    def get_beam_beam_coplanar_face_ids(self, neighbor_beam, tol=0.005):
        # type: (Beam, float) -> list[tuple[str,str]]
        """
        Computes the faces that are coplanar between two beams
        Returns:
            List of [Tuples (face_id on self, face_id on neighbor_beam)]
        """
        ffx = []
        for i in range(1, 5):
            p1 = self.get_face_plane(i)
            for j in range(1, 5):
                p2 = neighbor_beam.get_face_plane(j)
                if (is_point_on_plane(p1.point, p2, tol) and is_point_on_plane(p2.point, p1, tol)):
                    #print ("Intersection self.Face%s Beam2.Face%s " %(i , j))
                    ffx.append((i, j))
        return ffx

    @classmethod
    def debug_get_dummy_beam(cls, include_joint=False):
        from compas.geometry import Frame, Point, Vector

        from integral_timber_joints.geometry.joint_90lap import Joint_90lap

        # Create Beam object
        beam = cls(Frame(Point(0, 0, 0), Vector(0, 1, 0), Vector(0, 0, 1)), 1000, 100, 150, "dummy_beam_1")
        # Create Joint object
        if (include_joint):
            return (beam, Joint_90lap(100, 3, 100, 100, 50, "dummy_joint"))
        else:
            return beam


if __name__ == '__main__':
    pass
