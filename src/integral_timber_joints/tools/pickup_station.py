from integral_timber_joints.process.dependency import ComputationalResult


class PickupStation (object):

    def __init__(self, alignment_frame=None, pickup_retract_vector=None):
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

    def __init__(self, alignment_frame=None, pickup_retract_vector=None):
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


class GripperAlignedPickupStation(PickupStation):
    """Gripper aligned Pickup station have a consistent gripper location. 
    The location of the beam changes in response to grasp pose.

    """

    def compute_pickup_frame(self, process, beam_id):
        # type: (integral_timber_joints.process.RobotClampAssemblyProcess, str) -> None
        """ Compute 'assembly_wcf_pickup' alignment frame
        by aligning the grasp frame to the given pickup_station_frame.
        The effect is that the gripper stays at the same place.

        compute_gripper_grasp_pose should be run before hand to set 'gripper_tcp_in_ocf'

        Side Effect
        -----------
        beam_attribute 'assembly_wcf_pickup' will be set.

        """
        from compas.geometry.transformations.transformation import Transformation
        from compas.geometry.primitives.frame import Frame

        # Grasp
        gripper_tcp_in_ocf = process.assembly.get_beam_attribute(beam_id, 'gripper_tcp_in_ocf')
        assert gripper_tcp_in_ocf is not None
        ocf_from_gripper_tcp = Transformation.from_frame(gripper_tcp_in_ocf)
        gripper_tcp_from_ocf = ocf_from_gripper_tcp.inverse()

        # Gripper position in world
        world_from_gripper_tcp = Transformation.from_frame(self.alignment_frame)

        # Transformation
        world_from_ocf = Frame.from_transformation(world_from_gripper_tcp * gripper_tcp_from_ocf)
        process.assembly.set_beam_attribute(beam_id, 'assembly_wcf_pickup', world_from_ocf)

        return True
