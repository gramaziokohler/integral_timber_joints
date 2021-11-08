from integral_timber_joints.geometry import JointHalfLap
from integral_timber_joints.process import RobotClampAssemblyProcess, RoboticMovement, ObjectState


from compas.geometry import Frame, Transformation, Cylinder, Point, transform_points, transpose_matrix, multiply_matrices
from compas.datastructures import Mesh, mesh_weld
from compas.data import DataDecoder

try:
    from typing import Any, Dict, List, Optional, Tuple, Type
except:
    pass

import json

json_path = r"C:\Users\leungp\Documents\GitHub\integral_timber_joints\external\itj_design_study\211010_CantiBox\CantiBoxRight_process.json"
movement_path = r"C:\Users\leungp\Documents\GitHub\integral_timber_joints\external\itj_design_study\211010_CantiBox\results"

with open(json_path, 'r') as f:
    process = json.load(f, cls=DataDecoder) #type: RobotClampAssemblyProcess
# result = process.load_external_movement(process_folder_path=movement_path,movement=process.get_movement_by_movement_id("A1_M2"), verbose=True)
result = process.load_external_movements(process_folder_path=movement_path, verbose=True)
print (result)

if __name__ == '__main__':
    pass

