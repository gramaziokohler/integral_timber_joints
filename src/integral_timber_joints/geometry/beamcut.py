# Beamcut Object (Abstract class)
#   Beamcut object are used to model the beam-end termination of a beam
#   It holds semantic data (such as a trimming plane for a single cut)
#
#   It can be used to generate the geometry of a boolean object for beam visualization.
#   It can be used to generate BTL processing object for BTL output
from compas.datastructures import Mesh
from compas.geometry import Cylinder, Polyhedron, Shape, Transformation

from integral_timber_joints.geometry.beam import Beam


class Beamcut(object):

    def __init__(self):
        """Handel the creation of sub class objects
        """
        pass

    def get_feature_meshes(self, BeamRef):
        # type: (Beam) -> list[Mesh]
        """Compute the negative mesh volume of the joint.

        Returns
        -------
        object
            A compas.Mesh

        Note
        ----
        This function needs to be implemented by inhereted class
        The self.mesh should be updated with the new mesh

        """
        raise NotImplementedError

    def to_data(self):
        """Simpliest way to get this class serialized.
        """
        return self.data

    @classmethod
    def from_data(cls, data):
        """Construct a Movement from structured data. Subclass must add their properity to
        the data properity.
        """
        beamcut = cls()
        beamcut.data = data
        return beamcut

    def transform_(self, transformation):
        # type: (Transformation) -> None
        """Transforming the beamcut object in WCF.
        Typically called by assembly.transform when initiated by user."""
        raise NotImplementedError

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
        # type: (Beam) -> list(Mesh)
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
        return [Mesh.from_vertices_and_faces(* shape.to_vertices_and_faces()) for shape in shapes]

    @property
    def data(self):
        raise NotImplementedError

    @data.setter
    def data(self, data):
        raise NotImplementedError


if __name__ == "__main__":
    pass
