import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext
import re
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



if __name__ == '__main__':
    process = get_process()
    artist = get_process_artist()

    key = ('b1', 'f')
    initial_scene = process.initial_state
    print(initial_scene.object_state_dict[key])

    scene1 = process.get_movement_end_scene(process.movements[0])
    print(scene1.object_state_dict[key])

    scene2 = process.get_movement_end_scene(process.movements[1])
    print(scene1.object_state_dict[key])

    print (artist.state_visualization_current_state[key[0]])
