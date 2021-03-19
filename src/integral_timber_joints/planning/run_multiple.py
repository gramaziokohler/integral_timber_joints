from integral_timber_joints.planning.run import *
from integral_timber_joints.process.robot_clamp_assembly_process import *

process = parse_process("C:\\Users\\leungp\\Documents\\GitHub\\integral_timber_joints\\external\\itj_design_study\\210128_RemodelFredPavilion\\twelve_pieces_process.json") # type: RobotClampAssemblyProcess


client, robot, _ = load_RFL_world(True)
full_start_conf = to_rlf_robot_full_conf(R11_INTER_CONF_VALS, R12_INTER_CONF_VALS)
client.set_robot_configuration(robot, full_start_conf)

# start_state = process.get_movement_start_state(m)
# set_state(client, robot, process, start_state, initialize=True,
#     options={'debug' : False, 'include_env' : True, 'reinit_tool' : False})

set_state(client, robot, process, process.initial_state, initialize=True,
    options={'debug' : False, 'include_env' : True, 'reinit_tool' : False})

m = process.movements[4]
compute_movement(client, robot, process, m, diagnosis=True)

visualize_movement_trajectory(client, robot, process, m, step_sim=True)



m = process.movements[7]
compute_movement(client, robot, process, process.movements[5], diagnosis=True)
propagate_states(process, [m], process.movements)

process.get_movement_summary_by_beam_id('b0')

# Plan Free
m = process.movements[5]
compute_movement(client, robot, process, m, diagnosis=True)
propagate_states(process, [m], process.movements)
visualize_movement_trajectory(client, robot, process, m, step_sim=True)

propagate_states(process, [m], process.movements)
