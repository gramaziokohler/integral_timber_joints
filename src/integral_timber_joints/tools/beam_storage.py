from compas.geometry.primitives.frame import Frame


class BeamStorage(object):

    def __init__(self, frame=None, y_count=5, y_spacing=140, z_spacing=140):
        # type: (Frame, int, float, float) -> None
        """Frame should have X pointing along beam length and Z pointing to world Z"""
        self.frame = frame          # type: (Frame) # Frame where the
        self.y_count = y_count
        self.y_spacing = y_spacing
        self.z_spacing = z_spacing

    def to_data(self):
        """Simpliest way to get this class serialized.
        """
        return self.data

    @classmethod
    def from_data(cls, data):
        """Construct a Movement from structured data. Subclass must add their properity to
        the data properity.
        """
        beamstorage = cls()
        beamstorage.data = data
        return beamstorage

    @property
    def data(self):
        data = {}
        data['frame'] = self.frame
        data['y_count'] = self.y_count
        data['y_spacing'] = self.y_spacing
        data['z_spacing'] = self.z_spacing
        return data

    @data.setter
    def data(self, data):
        self.frame = data.get('frame', Frame.worldXY())
        self.y_count = data.get('y_count', 5)
        self.y_spacing = data.get('y_spacing', 140)
        self.z_spacing = data.get('z_spacing', 140)

    def get_storage_frame(self, beam_seq, total_beam_count = 0):
        # type(int) -> Frame
        """Get the storage frame of a particular beam based on the sequence number (zero start)
        The algorithm is a simple Y first and then Z. 

        The returned frame have X pointing along the beam length and
        Z pointing to world Up. You can align the grasp face's face_frame such
        that the beam is stored in the same orientation with the gripping direction.
        and optionally compensate the depth of the beam by moving the beam up.
        """
        # Reverse the order (since we pick form the top)
        if total_beam_count > 0 :
            beam_seq = total_beam_count - beam_seq - 1

        y = (beam_seq % self.y_count)
        z = beam_seq // self.y_count

        y_offset = y * self.y_spacing
        z_offset = z * self.z_spacing

        transform_vector = self.frame.yaxis.unitized().scaled(y_offset) + self.frame.zaxis.unitized().scaled(z_offset)
        return Frame(self.frame.point + transform_vector, self.frame.xaxis.copy(), self.frame.yaxis.copy())
