import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext
import re
from integral_timber_joints.rhino.process_artist import ProcessArtist
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.process import ComputationalResult
from integral_timber_joints.assembly import BeamAssemblyMethod

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh

try:
    from typing import Dict, List, Optional, Tuple

    from integral_timber_joints.process import RFLPathPlanner
except:
    pass

from compas.geometry import Transformation, Frame

def get_current_selected_scene_state(self, override_attached_objects_with_fk = True):
    # type: (ProcessArtist, bool) -> SceneState
    """
    Return the currently selected SceneState

    if `override_attached_objects_with_fk` is true, and if robot config is defined,
    the frame of the attached objects will be overridden by the FK result of the robot.

    Note state_id = 1 is referring to end of the first (0) movement.
    """
    state_id = self.selected_state_id
    scene = None
    # Short circuit for returning the initial state.
    # No override_attached_objects_with_fk will be performed.
    if state_id == 0:
        scene =  self.process.initial_state
        return scene


    movement = self.process.movements[state_id - 1]  # type: RoboticMovement
    scene =  self.process.get_movement_end_scene(movement)

    process = self.process
    if override_attached_objects_with_fk:
        if ('robot', 'c') in scene and scene[('robot', 'c')] is not None:
            print ("override_attached_objects_with_fk")
            from copy import deepcopy
            scene = deepcopy(scene)

            # * Compute FK
            configuration = scene[('robot', 'c')]  # type: Configuration
            fk_flange_frame = process.robot_model.forward_kinematics(configuration.scaled(1000), process.ROBOT_END_LINK)
            print (fk_flange_frame)
            t_world_from_flange = Transformation.from_frame(fk_flange_frame)

            # * Set attached objects, use `t_flange_from_attached_objects` in Movement
            object_id = 'tool_changer'
            print (object_id)
            print (scene[(object_id, 'f')])
            scene[(object_id, 'f')] = fk_flange_frame
            print (scene[(object_id, 'f')])

            for object_id, t_flange_from_attached_objects in zip(movement.attached_objects, movement.t_flange_from_attached_objects):
                print (object_id)
                # print (t_flange_from_attached_objects)
                print (scene[(object_id, 'f')])
                t_world_from_object = t_world_from_flange * t_flange_from_attached_objects
                scene[(object_id, 'f')] = Frame.from_transformation(t_world_from_object)
                print (scene[(object_id, 'f')])

    return scene

if __name__ == '__main__':
    process = get_process()
    artist = get_process_artist()
    artist.delete_state(redraw=True)

    scene = get_current_selected_scene_state(artist)
    artist.draw_state(scene=scene, redraw=True)
    artist.draw_sweep_trajectory(scene=scene, redraw=False)