import jsonpickle

from integral_timber_joints.process.algorithms import *

import time

json_path_in = "examples/process_design_example/frame_ortho_lap_joints_no_rfl_prepathplan.json"
json_path_out = "examples/process_design_example/frame_ortho_lap_joints_no_rfl_pathplan.json"
pickle_path_out = "examples/process_design_example/frame_ortho_lap_joints_no_rfl_pathplan.pickle"

ros_planner_ip = "192.168.43.28"

#########################################################################
# Connect to path planning backend and initialize robot parameters
#########################################################################

pp = RFLPathPlanner(None)
pp.connect_to_ros_planner(ros_planner_ip)
robot = pp.robot # type: compas_fab.robots.Robot

pp.current_configuration = pp.rfl_timber_start_configuration()

#########################################################################
# Load process from file
#########################################################################

f = open(json_path_in, 'r')
json_str = f.read()
print ("json_str len:" , len(json_str))
process = jsonpickle.decode(json_str, keys=True)  # type: RobotClampAssemblyProcess
f.close()

assembly = process.assembly # For convinence

#########################################################################
# Static Collision Mesh
#########################################################################

# Add static collision mesh to planning scene
from compas.geometry import Scale
pp.remove_collision_mesh("PickupStation")
#pp.append_collision_mesh(process.pickup_station.collision_mesh.transformed(Scale.from_factors([0.001]*3)), "PickupStation")
pp.append_collision_mesh(process.pickup_station.collision_mesh, "PickupStation")

# TODO: FLoor and other meshes

#########################################################################
# Sequential path planning
#########################################################################
t = time.time()
for ia, action in enumerate(process.actions):
    if ia not in list(range(0, 10)): continue
    seq_n = action.seq_n
    # print ("Seq(%s) Acion (%s) - %s \n" % (seq_n, ia, action))

    for im, movement in enumerate(action.movements):
        print ("Seq(%s) Act (%s) Mov (%s) - %s" % (seq_n, ia, im, movement))

        if isinstance(movement, RoboticMovement):
            # Add already built beams to planning scene
            already_built_beam_ids = assembly.get_already_built_beams(assembly.sequence[seq_n])

            # Attach Tool and Beam to robot

            # Prepare Starting Config and Goal constraints
            # start_configuration = Configuration.from_revolute_values([0, 4.295, 0, -3.327, 4.755, 0.])
            trajectory = pp.plan_motion(movement.target_frame, free_motion = True, verbose = True)

            if trajectory:
                print("> > Motion Planned (%s pts, %.1f secs)" % (len(trajectory.points), trajectory.time_from_start))
                print("> > Last Point: %s" % trajectory.points[-1])

                # Assign Last frame of the path to next configuration.
                last_configuration = robot.merge_group_with_full_configuration(trajectory.points[-1], pp.current_configuration, pp.planning_group)
                pp.current_configuration = last_configuration

                # Save trajectory to movement
                # movement.trajectory = trajectory
            else:
                print("> > Motion Plan Fail !!!")
        else:
            print("> > No Robotic Motion")
        print("")
print("> Total Path Plan Time: %.2fs" % (time.time() - t))

pp.ros_client.close()

#########################################################################
# Save Results
#########################################################################

# f = open(json_path_out, 'w')
# json_str = jsonpickle.encode(process, keys=True)
# print ("json_str len:" , len(json_str))
# f.write(json_str)
# f.close()

import pickle
with open(pickle_path_out, 'wb') as f:
    pickle.dump(process, f, protocol=2)
