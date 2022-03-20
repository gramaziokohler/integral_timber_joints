import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext
import re
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointHalfLap
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist
from integral_timber_joints.rhino.process_artist import ProcessArtist, RobotClampAssemblyProcess, Assembly
from integral_timber_joints.process import RoboticMovement, ObjectState
from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh


from compas_rhino.utilities import draw_polylines
from compas.geometry import Frame, Transformation, Cylinder, Point, transform_points, transpose_matrix, multiply_matrices

try:
    from typing import Any, Dict, List, Optional, Tuple, Type
except:
    pass


if __name__ == '__main__':
    process = get_process()
    artist = get_process_artist()
    movements = process.movements
    robot = process.robot_model
    # print (len(movements))
    # for movement in movements:
    #     if isinstance(movement, RoboticMovement):
    #         if movement.target_configuration is not None:
    #             print (movement.movement_id, movement.tag)
    #             target_frame = movement.target_frame
    #             target_frame_from_FK = robot.forward_kinematics(movement.target_configuration.scaled(1000), process.ROBOT_END_LINK)
    #             print ("tg_frame: %s" % target_frame)
    #             print ("fk_frame: %s" % target_frame_from_FK)

    movement = process.get_movement_by_movement_id("A308_M2")
    print (movement)
    end_state = process.get_movement_end_scene(movement)
    print ("%s End State" % movement.movement_id)
    print (end_state[('robot', 'c')])
    print (robot.forward_kinematics(end_state[('robot', 'c')].scaled(1000), process.ROBOT_END_LINK))
    print (end_state[('robot', 'f')])
    print (end_state[('tool_changer', 'f')])

    movement = process.get_movement_by_movement_id("A311_M0")
    start_state = process.get_movement_start_scene(movement)
    print ("%s Start State" % movement.movement_id)
    print (start_state[('robot', 'c')])
    print (start_state[('robot', 'f')])
    print (start_state[('tool_changer', 'f')])

    end_state = process.get_movement_end_scene(movement)
    print ("%s End State" % movement.movement_id)
    print (end_state[('robot', 'c')])
    print (robot.forward_kinematics(end_state[('robot', 'c')].scaled(1000), process.ROBOT_END_LINK))
    print (end_state[('robot', 'f')])
    print (end_state[('tool_changer', 'f')])

    movement = process.get_movement_by_movement_id("A311_M1")
    start_state = process.get_movement_start_scene(movement)
    print ("%s Start State" % movement.movement_id)
    print (start_state[('robot', 'f')])
    print (start_state[('tool_changer', 'f')])
