from itertools import product
import pybullet_planning as pp

from integral_timber_joints.planning.robot_setup import load_RFL_world, GANTRY_ARM_GROUP
from integral_timber_joints.planning.state import set_state, gantry_base_generator
from integral_timber_joints.process import RoboticMovement


def get_ik_solutions(process, movement_index, options={}):
    # TODO how to recover the client and robot if one is running already?
    all_movements = process.movements
    movement = process.movements[movement_index]

    if not isinstance(movement, RoboticMovement):
        print('{} not an robotic movement, return None'.format(movement.short_summary))
        return None

    viewer = options.get('viewer', False)
    reachable_range = options.get('reachable_range', (0.2, 2.8))
    ik_gantry_attempts = options.get('ik_gantry_attempts', 100)

    # if not pp.is_connected():
    # * Connect to path planning backend and initialize robot parameters
    client, robot, _ = load_RFL_world(viewer=viewer)
    process.set_initial_state_robot_config(process.robot_initial_config)
    set_state(client, robot, process, process.initial_state, initialize=True,
        options={'debug' : False, 'include_env' : True, 'reinit_tool' : False})

    # start_scene = process.get_movement_start_scene(movement)
    # set_state(client, robot, process, start_scene)

    end_scene = process.get_movement_end_scene(movement)
    set_state(client, robot, process, end_scene)

    flange_frame = end_scene[('robot', 'f')].copy()
    assert flange_frame is not None
    # convert to meter
    flange_frame.point *= 1e-3

    temp_name = '_tmp'
    for o1_name, o2_name in movement.allowed_collision_matrix:
        o1_bodies = client._get_bodies('^{}$'.format(o1_name))
        o2_bodies = client._get_bodies('^{}$'.format(o2_name))
        for parent_body, child_body in product(o1_bodies, o2_bodies):
            client.extra_disabled_collision_links[temp_name].add(
                ((parent_body, None), (child_body, None))
            )

    # TODO use track ik or ikfast here
    # sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
    conf = None
    # * sample from a ball near the pose
    gantry_base_gen_fn = gantry_base_generator(client, robot, flange_frame, reachable_range=reachable_range, scale=1.0)
    with pp.HideOutput():
        for _, base_conf in zip(range(ik_gantry_attempts), gantry_base_gen_fn):
            # TODO a more formal gantry_base_from_world_base
            conf = client.inverse_kinematics(robot, flange_frame, group=GANTRY_ARM_GROUP,
                                             options={'avoid_collisions': False})
            if conf is not None and not client.check_collisions(robot, conf, options=options):
                break
        else:
            client.disconnect() # This is necessary to cleanly exit the script. Otherwise it corrupt the rpc server.
            raise RuntimeError('no attach conf found after {} attempts.'.format(ik_gantry_attempts))

    # if temp_name in client.extra_disabled_collision_links:
    #     del client.extra_disabled_collision_links[temp_name]
    client.disconnect()

    return conf


if __name__ == "__main__":
    import os
    import sys
    import time
    import json
    import datetime
    import integral_timber_joints
    from integral_timber_joints.process import RobotClampAssemblyProcess
    from compas.utilities import DataDecoder

    def load_process(json_path):
        # type: (str) -> RobotClampAssemblyProcess
        exist = os.path.exists(json_path)
        if exist:
            with open(json_path, 'r') as f:
                process = json.load(f, cls=DataDecoder)
                c_time = datetime.datetime.fromtimestamp(os.path.getmtime(json_path))
                print("Process loaded from: %s (%i Beams). File last modified on %s." % (os.path.basename(json_path), len(list(process.assembly.beams())), c_time))
                return process
        else:
            print("json_path not exist")

    def get_ik_no_proxy(process, state_id, viewer):
        options = {}
        options = {'viewer': viewer}
        result = get_ik_solutions(process, state_id - 1, options)
        print("IK result: %s" % result)

    def get_ik_via_proxy(process, state_id):
        from compas.rpc import Proxy
        rhino_interface = Proxy('integral_timber_joints.planning.rhino_interface')
        try:
            result = rhino_interface.get_ik_solutions(process, state_id - 1)
        except:
            result = None
        print("IK result: %s" % result)

    ##########################

    path_to_json = os.path.realpath(os.path.join(os.path.dirname(integral_timber_joints.__file__), '..', '..', 'external', 'itj_design_study', '210605_ScrewdriverTestProcess', 'nine_pieces_process.json'))
    process = load_process(path_to_json) # type: RobotClampAssemblyProcess

    # Which state to get IK
    # ---------------------
    # state_id = 225 # Only TC attached, result ok (no proxy) , result ok (via Proxy)
    # state_id = 226 # Only TC touch with Tool, result ok (no proxy) , result ok (via Proxy)
    state_id = 229 # Linear retract 1 after Tool Attached, result None
    # state_id = 230 # Linear retract 2 after Tool Attached, result None
    # state_id = 233 # Linear Approach 2 of 2 to attach CL3 ('c1') to structure., result None
    # state_id = 236 # Linear Retract after attaching CL3 ('c1') on structure, result None (Sphere sampling error?)
    state_id = 249 # Free Move reach Storage Approach Frame of PG500 ('g1'), to get tool. result ok (no proxy), ok (via Proxy)


    get_ik_no_proxy(process, state_id, True)
    # get_ik_via_proxy(process, state_id)