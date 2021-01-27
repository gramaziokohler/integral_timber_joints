from compas.datastructures import Mesh
from compas.geometry import Frame, Transformation

try:
    from typing import List, Dict, Tuple, Optional
    from integral_timber_joints.process.state import ObjectState
except:
    pass

class EnvironmentModel(Mesh):
    """Each EnvironmentModel is a Mesh object.
    Use mesh with no convex hull if possible.

    draw_state(ObjectState, global_transform) function can draw
    """
    def __init__(self, name = None):
        super(EnvironmentModel, self). __init__()
        self.name = name # type: Frame
    
    @property
    def name(self):
        """str : The name of the EnvironmentModel."""
        return self.attributes['name']

    @name.setter
    def name(self, value):
        self.attributes['name'] = value

    def draw_state(self, state, robot_world_transform = None):
        # type: (ObjectState, Optional[Transformation]) -> List[Mesh] 
        """Returns the collision meshes of the EnvironmentModel (typically 1) for a given state.
        
        Two transformation is performed:
        - `state.current_frame` if not `None` (typically it is None for EnvironmentMesh)
        - `robot_world_transform` if not `None`
        """

        if state.current_frame is not None:
            T = Transformation.from_frame_to_frame(Frame.worldXY(), state.current_frame)
        else:
            T = Transformation() # Identity Matrix

        transformed_mesh = self.transformed(T)

        if robot_world_transform is not None:
            assert isinstance(robot_world_transform, Transformation)
            transformed_mesh = transformed_mesh.transform(robot_world_transform)
        
        return [transformed_mesh]