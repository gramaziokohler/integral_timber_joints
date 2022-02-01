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

if __name__ == '__main__':
    process = get_process()
    artist = get_process_artist()
    movement_i = 1030
    movement = process.movements[movement_i]
    print (movement.movement_id)

