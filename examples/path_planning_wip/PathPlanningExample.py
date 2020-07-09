import jsonpickle

from integral_timber_joints.process.algorithms import *

import time

json_path_in = "examples/path_planning_wip/timber_box_prepathplan.json"
json_path_out = "examples/path_planning_wip/timber_box_pathplan.json"
# replan = True # If Replan is true, the previous pathplanned process file is loaded.
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
# Sequentially copy movement target_frame to next movement source_frame
#########################################################################

_source_frame = None
for ia, action in enumerate(process.actions):
    for im, movement in enumerate(action.movements):
        if isinstance(movement, RoboticMovement):
            movement.source_frame = _source_frame
            _source_frame = movement.target_frame

#########################################################################
# Sequential path planning
#########################################################################
last_configuration = pp.rfl_timber_start_configuration()

t = time.time()
for ia, action in enumerate(process.actions):
    # if ia not in list(range(0, 20)): continue
    seq_n = action.seq_n
    act_n = action.act_n

    for im, movement in enumerate(action.movements):
        print ("Seq(%s) Act (%s) Mov (%s) - %s" % (seq_n, act_n, im, movement))

        if isinstance(movement, RoboticMovement):
            # Add already built beams to planning scene
            # beam_id = assembly.sequence[seq_n]
            # already_built_beam_ids = assembly.get_already_built_beams(beam_id)
            # pp.remove_collision_mesh('already_built_beams')
            # for already_beam_id in already_built_beam_ids:
            #     pp.append_collision_mesh(assembly.beam(already_beam_id).cached_mesh, 'already_built_beams')

            # Attach Tool and Beam to robot

            # Prepare Starting Configuration
            if last_configuration is not None:
                # retrive last planned trajectory's last config
                start_configuration = last_configuration
            else:
                # Previous planning failed, compute config based on source_frame
                # Source frame is created in the beginning of this file.
                start_configuration = pp.ik_solution(movement.source_frame, verbose=True)

            # Perform planning if start_configuration is not None.
            if start_configuration is not None:
                trajectory = pp.plan_motion(
                    movement.target_frame,
                    start_configuration = start_configuration,
                    free_motion = True,
                    verbose = True)
            else:
                trajectory = None

            # Post Planning
            if trajectory and trajectory.fraction == 1.0:
                # Path planning succeeded
                print("> > Motion Planned (%s pts, %.1f secs)" % (len(trajectory.points), trajectory.time_from_start))
                print("> > Last Point: %s" % trajectory.points[-1])

                # Assign Last frame of the path to next configuration.
                last_configuration = robot.merge_group_with_full_configuration(trajectory.points[-1], pp.current_configuration, pp.planning_group)

                # Save trajectory to movement
                movement.trajectory = trajectory
            else:
                # Path planning failed.
                print("> > Motion Plan Fail !!!")
                last_configuration = None
        else:
            print("> > No Robotic Motion")
        print("")
print("> Total Path Plan Time: %.2fs" % (time.time() - t))

pp.ros_client.close()

#########################################################################
# Save Results
#########################################################################

f = open(json_path_out, 'w')
json_str = jsonpickle.encode(process, keys=True)
print ("out json_str len:" , len(json_str))
f.write(json_str)
f.close()
