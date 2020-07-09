import jsonpickle

from integral_timber_joints.process.algorithms import *

import time

json_path_in = "examples/path_planning_wip/halflap_structure_process.json"
json_path_out = "examples/path_planning_wip/halflap_structure_prepathplan.json"
movement_log_file_out = "examples/path_planning_wip/halflap_structure_movements.txt"

#########################################################################
# Load process from file
#########################################################################
ms = time.time()*1000.0
f = open(json_path_in, 'r')
json_str = f.read()
print ("json_str len:" , len(json_str))
process = jsonpickle.decode(json_str, keys=True)  # type: RobotClampAssemblyProcess
f.close()
print("> Loading Process from %s\n    Time: %sms" % (json_path_in, time.time()*1000.0 - ms))

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

debug_print_process_actions_movements(process, movement_log_file_out)

#########################################################################
# Save process from file
#########################################################################

ms = time.time()*1000.0
f = open(json_path_out, 'w')
json_str = jsonpickle.encode(process, keys=True)
print ("json_str len:" , len(json_str))
f.write(json_str)
f.close()
