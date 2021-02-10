from compas.geometry import Frame


class ObjectState(object):
    """ Base class of a state object.
    Note that the State object should exist in a state dictionary where the key refers to an object id.

    `State.current_frame` 
    - A frame describing the loaction of an object. `None` can be used to represent an un-transformed state.

    `State.kinematic_config` 
    - A dictionary to describe the kinematic configuration of a RobotModel or ToolModel (such as Clamp and Gripper)
    - The dictionary uses `joint names` as keys and `values` in degrees or millimeters.
    - `None` can be used for Beams and Env Objects that does not have kinematic state

    `State.attached_to_robot` 
    - Bool value describing if an object is attached to the robot. 
    - Note that the ToolChanger is always attached to the robot's flange. (Level 1)
    - Gripper or Clamp can be attached to a robot's ToolChanger. (Level 2)
    - Beam can only be attached to a Gripper or Clamp (Level 3)

    `State.attached_to_joint` 
    - joint_id tuple[str,str] describing if a Clamp is hanging onto a beam passively.
    - This is used when a Clamp is left at a Beam or if a Screwdriver is attached to a flying Beam


    """

    def __init__(self):
        self.current_frame = None  # type: Frame
        self.kinematic_config = None  # type: Frame
        self.attached_to_robot = False  # type: bool
        self.attached_to_joint = None  # type: tuple[str,str]

    def to_data(self):
        """Simpliest way to get this class serialized.
        """
        return self.data

    @classmethod
    def from_data(cls, data):
        """Construct a Movement from structured data. Subclass must add their properity to
        the data properity.
        """
        state = cls()
        state.data = data
        return state

    @property
    def data(self):
        data = {
            'current_frame': self.current_frame,
            'kinematic_config': self.kinematic_config,
            'attached_to_robot': self.attached_to_robot,
            'attached_to_joint': self.attached_to_joint,
        }
        return data

    @data.setter
    def data(self, data):
        self.current_frame = data['current_frame']
        self.kinematic_config = data['kinematic_config']
        self.attached_to_robot = data['attached_to_robot']
        self.attached_to_joint = data['attached_to_joint']
