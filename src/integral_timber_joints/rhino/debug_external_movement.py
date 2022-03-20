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

def print_movement(index):
    movement = process.movements[index]
    if isinstance(movement, RoboticMovement):
        print ("#%s (%s) %s" % (index, movement.movement_id, movement.tag))
        print (" - %s"% movement.target_frame)

def print_ext_movement(index):
    external_movement_path = os.path.join(get_activedoc_process_path(), '..\\results')
    from compas.utilities import DataDecoder
    import json
    movement = process.movements[index]
    movement_path = os.path.join(external_movement_path, movement.get_filepath(subdir='movements'))
    with open(movement_path, 'r') as f:
        movement = json.load(f, cls=DataDecoder)

        if isinstance(movement, RoboticMovement):
            print ("#%s (%s) %s" % (index, movement.movement_id, movement.tag))
            print (" - %s"% movement.target_frame)

if __name__ == '__main__':
    process = get_process()
    artist = get_process_artist()
    from integral_timber_joints.rhino.visualize_trajectory import load_selected_external_movment_if_exist


    print_movement(1100)
    print_movement(1101)
    print_movement(1102)
    print_movement(1029)
    print_movement(1030)
    print_movement(1031)

    print_ext_movement(1100)
    print_ext_movement(1101)
    print_ext_movement(1102)
    print_ext_movement(1029)
    print_ext_movement(1030)
    print_ext_movement(1031)
