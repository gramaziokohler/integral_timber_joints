# This file originates from Keerthana Udaykumar's <kudaykum@student.ethz.ch> master thesis on `Timber Grammar` in Fall 2019
# Pulled from https://github.com/KEERTHANAUDAY/timber_grammar on 2019-10-09 commit: d389364304f3740e652dbe4e85291402a2df0636
# This file is there on modified by Victor Leung <leung@arch.eth.ch> for use in Integral Timber Joint project

# Joint Object (Abstract class)
#   Joint object are used to model a physical connection between two beams.
#   It holds semantic data (such as the location of the joint)
#
#   It can be used to generate the geometry of a boolean object for beam visualization.
#   It can be used to generate BTL processing object for BTL output
#   It is refefenced, and serialized through the Beam object


import json

from compas.datastructures import Mesh
from compas.geometry import Cylinder, Polyhedron, Shape

try:
    from integral_timber_joints.geometry.beam import Beam
except:
    pass

class Joint(object):

    def __init__(self):
        """Handel the creation of sub class objects
        """
        pass

    def to_data(self):
        """Returns a dictionary of structured data representing the data structure.
        Actual implementation is in @property def data(self) of the inherited class
        Returns
        -------
        dict
            The structured data.
        Note
        ----
        This method produces the data that can be used in conjuction with the
        corresponding *from_data* class method.
        """
        assert hasattr(self, 'data'), "Inherited class %s do not have data attribute" % self.__class__.__name__
        return self.data

    def get_feature_shapes(self, BeamRef):
        # type: (Beam) -> list[Shape]
        """Compute the negative shapes of the joint.

        Returns
        -------
        list[Shape]

        Note
        ----
        This function needs to be implemented by inhereted class

        """
        raise NotImplementedError

    def get_feature_meshes(self, BeamRef):
        # type: (Beam) -> list[Mesh]
        """Compute the negative mesh volume of the joint.

        The default implementation of this function is to get feeatures from
        `get_feature_shapes()` and convert them to meshes using `shape.to_vertices_and_faces()`

        Returns
        -------
        list[Mesh]

        Note
        ----
        This function needs to be implemented by inhereted class
        """
        shapes = self.get_feature_shapes(BeamRef)
        # for shape in shapes:
        #     if isinstance (shape, Cylinder):
        #         Mesh.from_vertices_and_faces(* shape.to_vertices_and_faces(u=16))
        #     if isinstance (shape, Polyhedron)

        return [Mesh.from_vertices_and_faces(* shape.to_vertices_and_faces()) for shape in shapes]

    def get_clamp_frames(self, beam):
        # type: (Beam) -> list[Frame]
        raise NotImplementedError

if __name__ == "__main__":
    # Test to create a inherited joint object, serialize and deserialize it.
    from compas.geometry import Frame

    from integral_timber_joints.geometry.joint_90lap import Joint_90lap

    j = Joint_90lap(Frame.worldXY(), 1, 50, 100, 100)
    j.to_json("n.json", pretty=True)
    print(j)
    print(j.data)
    q = Joint.from_json("n.json")
    print(q)
    print(q.data)
