class PickupStation (object):
    def __init__(self, alignment_frame, pickup_retract_vector):
        # Frame (in WCF) for the beam to align to
        self.alignment_frame = alignment_frame

        # Vector for the robot to move (in WCF) after picking up beams
        self.pickup_retract_vector = pickup_retract_vector

        self.collision_mesh = None

        # Point on the beam (relative to the gripper_grasp_face) for alignment purpose
        self.align_face_X0 = True
        self.align_face_Y0 = True
        self.align_face_Z0 = True
