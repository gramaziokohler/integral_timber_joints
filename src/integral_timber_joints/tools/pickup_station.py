from compas_fab.robots import Configuration
from compas.data import Data
try:
    from typing import Dict, List, Optional, Tuple, cast

    from integral_timber_joints.process import RobotClampAssemblyProcess
except:
    pass


class PickupStation (Data):

    def __init__(self, alignment_frame=None, pickup_retract_vector=None):
        # Frame (in WCF) for the beam to align to
        self.alignment_frame = alignment_frame

        # Vector for the robot to move (in WCF) after picking up beams
        self.pickup_retract_vector = pickup_retract_vector
        # Optional configuration for a static pickup robot configuration. This will be applied to all gripper types.
        self.beam_pickup_configuration = None  # type: Configuration

        self.collision_meshes = []

        # Point on the beam (relative to the gripper_grasp_face) for alignment purpose
        self.align_face_X0 = True
        self.align_face_Y0 = True
        self.align_face_Z0 = True

    def get_robot_config_at_pickup(self, gripper_id):
        # type: (str) -> Configuration
        return self.beam_pickup_configuration

    @property
    def data(self):
        data = {}
        data['alignment_frame'] = self.alignment_frame
        data['pickup_retract_vector'] = self.pickup_retract_vector
        data['beam_pickup_configuration'] = self.beam_pickup_configuration
        data['collision_meshes'] = self.collision_meshes
        data['align_face_X0'] = self.align_face_X0
        data['align_face_Y0'] = self.align_face_Y0
        data['align_face_Z0'] = self.align_face_Z0
        return data

    @data.setter
    def data(self, data):
        self.alignment_frame = data.get('alignment_frame', None)
        self.pickup_retract_vector = data.get('pickup_retract_vector', None)
        self.beam_pickup_configuration = data.get('beam_pickup_configuration', None)
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
        data = super(StackedPickupStation, self).data
        data['number_of_rows'] = self.number_of_rows
        data['hori_spacing'] = self.hori_spacing
        data['vert_spacing'] = self.vert_spacing
        return data

    @data.setter
    def data(self, data):
        PickupStation.data.fset(self, data)
        self.number_of_rows = data.get('number_of_rows', 4)
        self.hori_spacing = data.get('hori_spacing', 100)
        self.vert_spacing = data.get('vert_spacing', 100)


class GripperAlignedPickupStation(PickupStation):
    """Gripper aligned Pickup station have a consistent gripper location.
    The location of the beam changes in response to grasp pose.

    """

    def __init__(self, default_pickup_frame=None, pickup_retract_vector=None):
        # Frame (in WCF) for the beam to align to
        super(GripperAlignedPickupStation, self).__init__(alignment_frame=default_pickup_frame, pickup_retract_vector = pickup_retract_vector)
        self.gripper_tcp_frame_at_pickup = {}
        self.robot_config_at_pickup = {}

    @property
    def data(self):
        data = super(GripperAlignedPickupStation, self).data
        data['gripper_tcp_frame_at_pickup'] = self.gripper_tcp_frame_at_pickup
        data['robot_config_at_pickup'] = self.robot_config_at_pickup
        return data

    @data.setter
    def data(self, data):
        PickupStation.data.fset(self, data)
        self.gripper_tcp_frame_at_pickup = data.get('gripper_tcp_frame_at_pickup', {})
        self.robot_config_at_pickup = data.get('robot_config_at_pickup', {})

    def get_gripper_tcp_frame_at_pickup(self, gripper_id):
        if gripper_id in self.gripper_tcp_frame_at_pickup:
            return self.gripper_tcp_frame_at_pickup[gripper_id]
        else:
            return self.alignment_frame

    def get_robot_config_at_pickup(self, gripper_id):
        if gripper_id in self.robot_config_at_pickup:
            return self.robot_config_at_pickup[gripper_id]
        else:
            return None

    def compute_pickup_frame(self, process, beam_id):
        # type: (RobotClampAssemblyProcess, str) -> None
        """ Compute 'assembly_wcf_pickup' alignment frame
        by aligning the grasp frame to the given pickup_station_frame.
        The effect is that the gripper stays at the same place.

        `gripper_tcp_in_ocf` and 'gripper_id` should be set before hand to set 'gripper_tcp_in_ocf'

        Side Effect
        -----------
        beam_attribute 'assembly_wcf_pickup' will be set.

        """
        from compas.geometry.primitives.frame import Frame
        from compas.geometry.transformations.transformation import Transformation

        # Grasp
        gripper_tcp_in_ocf = process.assembly.get_beam_attribute(beam_id, 'gripper_tcp_in_ocf')
        assert gripper_tcp_in_ocf is not None
        ocf_from_gripper_tcp = Transformation.from_frame(gripper_tcp_in_ocf)
        gripper_tcp_from_ocf = ocf_from_gripper_tcp.inverse()

        # Gripper position in world
        gripper_id = process.assembly.get_beam_attribute(beam_id, 'gripper_id')
        world_from_gripper_tcp = Transformation.from_frame(self.get_gripper_tcp_frame_at_pickup(gripper_id))

        # Transformation
        world_from_ocf = Frame.from_transformation(world_from_gripper_tcp * gripper_tcp_from_ocf)
        process.assembly.set_beam_attribute(beam_id, 'assembly_wcf_pickup', world_from_ocf)

        return True
