# Beamcut Object (Abstract class)
#   Beamcut object are used to model the beam-end termination of a beam
#   It holds semantic data (such as a trimming plane for a single cut)
#
#   It can be used to generate the geometry of a boolean object for beam visualization.
#   It can be used to generate BTL processing object for BTL output

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

    @property
    def data(self):
        raise NotImplementedError

    @data.setter
    def data(self, data):
        raise NotImplementedError


if __name__ == "__main__":
    pass
