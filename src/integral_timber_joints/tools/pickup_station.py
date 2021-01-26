class PickupStation (object):

    def __init__(self, alignment_frame = None, pickup_retract_vector = None):
        # Frame (in WCF) for the beam to align to
        self.alignment_frame = alignment_frame

        # Vector for the robot to move (in WCF) after picking up beams
        self.pickup_retract_vector = pickup_retract_vector

        self.collision_meshes = []

        # Point on the beam (relative to the gripper_grasp_face) for alignment purpose
        self.align_face_X0 = True
        self.align_face_Y0 = True
        self.align_face_Z0 = True

    def to_data(self):
        return self.data

    @classmethod
    def from_data(cls, data):
        station = cls()
        station.data = data
        return station

    @property
    def data(self):
        data = {}
        data['alignment_frame'] = self.alignment_frame
        data['pickup_retract_vector'] = self.pickup_retract_vector
        data['collision_meshes'] = self.collision_meshes
        data['align_face_X0'] = self.align_face_X0
        data['align_face_Y0'] = self.align_face_Y0
        data['align_face_Z0'] = self.align_face_Z0
        return data

    @data.setter
    def data(self, data):
        self.alignment_frame = data.get('alignment_frame', None)
        self.pickup_retract_vector = data.get('pickup_retract_vector', None)
        self.collision_meshes = data.get('collision_meshes', [])
        self.align_face_X0 = data.get('align_face_X0', True)
        self.align_face_Y0 = data.get('align_face_Y0', True)
        self.align_face_Z0 = data.get('align_face_Z0', True)

class StackedPickupStation (PickupStation):

    def __init__(self, alignment_frame = None, pickup_retract_vector = None):
        # Frame (in WCF) for the beam to align to
        super(StackedPickupStation, self).__init__(alignment_frame, pickup_retract_vector)
        self.number_of_rows = 4
        self.hori_spacing = 100
        self.vert_spacing = 100

    @property
    def data(self):
        data = super(StackedPickupStation, self).to_data()
        data['number_of_rows'] = self.number_of_rows
        data['hori_spacing'] = self.hori_spacing
        data['vert_spacing'] = self.vert_spacing
        return data
    
    @data.setter
    def data(self, data):
        super(StackedPickupStation, type(self)).data.fset(self, data)
        self.number_of_rows = data.get('number_of_rows', 4)
        self.hori_spacing = data.get('hori_spacing', 100)
        self.vert_spacing = data.get('vert_spacing', 100)

    @classmethod
    def from_data(cls, data):
        station = cls()
        station.data = data
        return station
