import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext
import re
import os

from integral_timber_joints.rhino.process_artist import ProcessArtist
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none, get_activedoc_process_path
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.process import ComputationalResult
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.process import Movement, RobotClampAssemblyProcess, RoboticMovement

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh

try:
    from typing import Dict, List, Optional, Tuple

    from integral_timber_joints.process import RFLPathPlanner
except:
    pass

from compas.geometry import Transformation, Frame

if __name__ == '__main__':
    process = get_process()
    artist = get_process_artist()
    from integral_timber_joints.rhino.visualize_trajectory import load_selected_external_movment_if_exist

    # * Load all external movements
    for i, movement in enumerate(process.movements):
        print ("Loading Movement #%s" % i)

        external_movement_path = os.path.join(get_activedoc_process_path(), '..\\results')
        process.load_external_movement(external_movement_path, movement, subdir='movements', verbose=False)


    # * Check Final trajectory point and

    for i, movement in enumerate(process.movements):
        # if i > 500:
        #     continue

        if not isinstance(movement, RoboticMovement):
            continue

        if movement.trajectory is None:
            print ("Movement #%s (%s) is RoboticMovement but no Trajectory" % (i, movement.movement_id))
            continue

        # * Forward Kinematics from robot Config
        configuration = movement.trajectory.points[-1]
        configuration = process.robot_initial_config.merged(configuration)
        trajectory_frame = process.robot_model.forward_kinematics(configuration.scaled(1000), process.ROBOT_END_LINK)

        # * Compare with Robot Target Frame
        target_frame = movement.target_frame
        distance = target_frame.point.distance_to_point(trajectory_frame.point)
        distance_threshold = 0.5 # mm scale
        if distance> distance_threshold:
            print ("Movement #%s (%s) Target to FK distance : %smm" % (i, movement.movement_id, distance))
            print (" - %s" % (movement.tag) )

        # * Compare with tool_changer Target Frame
        target_frame = process.get_movement_end_scene(movement)['tool_changer', 'f']
        distance = target_frame.point.distance_to_point(trajectory_frame.point)
        distance_threshold = 0.5 # mm scale
        if distance> distance_threshold:
            print ("Movement #%s (%s) Target to FK distance : %smm" % (i, movement.movement_id, distance))
            print (" - %s" % (movement.tag) )

        # * Compare Robot Target Frame and Tool Changer Frame
        # target_frame_tc = process.get_movement_end_scene(movement)['tool_changer', 'f']
        # target_frame_rob = process.get_movement_end_scene(movement)['robot', 'f']
        # distance = target_frame_tc.point.distance_to_point(target_frame_rob.point)
        # distance_threshold = 0.01 # mm scale
        # if distance> distance_threshold:
        #     print ("Movement #%s (%s) Rob Target to TC Target : %smm" % (i, movement.movement_id, distance))
        #     print (" - %s" % (movement.tag) )

