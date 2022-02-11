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

