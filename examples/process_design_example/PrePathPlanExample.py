import jsonpickle

from integral_timber_joints.process.algorithms import *

import time

json_path_in = "examples/process_design_example/frame_ortho_lap_joints_no_rfl_process.json"
json_path_out = "examples/process_design_example/frame_ortho_lap_joints_no_rfl_prepathplan.json"

#########################################################################
# Load process from file
#########################################################################

f = open(json_path_in, 'r')
json_str = f.read()
process = jsonpickle.decode(json_str, keys=True)  # type: RobotClampAssemblyProcess
f.close()

#########################################################################
# Pre computation
#########################################################################

# process.assign_clamp_type_to_joints()
# for beam_id in process.assembly.sequence:
#     process.compute_clamp_attachapproach_attachretract(beam_id, verbose = True)

#########################################################################
# From process.sequence, create actions (high level actions)
#########################################################################


ms = time.time()*1000.0
create_actions_from_sequence(process, verbose=False)
print("> create_actions_from_sequence(process)\n    Time: %sms" % (time.time()*1000.0 - ms))

ms = time.time()*1000.0
optimize_actions_place_pick_gripper(process, verbose=False)
print("> optimize_actions_place_pick_gripper(process)\n    Time: %sms" % (time.time()*1000.0 - ms))

ms = time.time()*1000.0
assign_tools_to_actions(process, verbose=False)
print("> assign_tools_to_actions(process)\n    Time: %sms" % (time.time()*1000.0 - ms))

ms = time.time()*1000.0
optimize_actions_place_pick_clamp(process, verbose=False)
print("> optimize_actions_place_pick_clamp(process)\n    Time: %sms" % (time.time()*1000.0 - ms))


#########################################################################
# Loop through each action, create movements (low level movements)
#########################################################################

ms = time.time()*1000.0
create_movements_from_actions(process)
print("> create_movements_from_actions(process)\n    Time: %sms" % (time.time()*1000.0 - ms))

#########################################################################
# List out actions and movements
#########################################################################

# debug_print_process_actions_movements(process)

#########################################################################
# Save process from file
#########################################################################

ms = time.time()*1000.0
f = open(json_path_out, 'w')
f.write(jsonpickle.encode(process, keys=True))
f.close()
print("> Saving Process to %s \n    Time: %sms" % (json_path_out, time.time()*1000.0 - ms))
