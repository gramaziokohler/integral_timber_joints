import jsonpickle

from integral_timber_joints.process.algorithms import *

import time

json_path_out = "examples/process_design_example/frame_ortho_lap_joints_no_rfl_pathplan.json"


#########################################################################
# Load process from file
#########################################################################

f = open(json_path_out, 'r')
json_str = f.read()
print ("json_str len:" , len(json_str))
process = jsonpickle.decode(json_str, keys=True)  # type: RobotClampAssemblyProcess
f.close()

assembly = process.assembly # For convinence

debug_print_process_actions_movements(process)
