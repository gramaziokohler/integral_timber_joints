import time
from itertools import product
from termcolor import cprint
import pybullet_planning as pp
from pybullet_planning.interfaces.env_manager.user_io import wait_if_gui

from compas.robots import Configuration
from compas_fab_pychoreo.conversions import pose_from_frame

from integral_timber_joints.process import RoboticMovement
from integral_timber_joints.planning.robot_setup import load_RFL_world, GANTRY_ARM_GROUP
from integral_timber_joints.planning.state import set_state, gantry_base_generator
from integral_timber_joints.planning.stream import _get_sample_bare_arm_ik_fn


def get_ik_solutions(process, movement_index, options=None):
    # TODO how to recover the client and robot if one is running already?
    options = options or {}
    movement = process.movements[movement_index]
    if not isinstance(movement, RoboticMovement):
        return (False, None, 'Not an robotic movement')

    viewer = options.get('viewer', False)
    debug = options.get('debug', False)
    reachable_range = options.get('reachable_range', (0.2, 2.5))
    ik_gantry_attempts = options.get('ik_gantry_attempts', 10)

    start_time = time.time()

    # * pass in client and robot if resuing a client
    reuse_client = False
    client = options.get('client', None)
    robot = options.get('robot', None)
    if client and robot and pp.is_connected():
        reuse_client = True
    else:
        # * Connect to path planning backend and initialize robot parameters
        # ! initial state must be set to make robot R12 conf correct
        client, robot, _ = load_RFL_world(viewer=viewer)
        process.set_initial_state_robot_config(process.robot_initial_config)
        set_state(client, robot, process, process.initial_state, initialize=True,
            options={'include_env' : True, 'reinit_tool' : False})

    end_scene = process.get_movement_end_scene(movement)
    set_state(client, robot, process, end_scene, options=options)

    flange_frame = end_scene[('robot', 'f')].copy()
    if flange_frame is None:
        client.disconnect()
        return (False, None, 'No robot target frame is specified in the end scene.')
    flange_frame.point *= 1e-3

    temp_name = '_tmp'
    for o1_name, o2_name in movement.allowed_collision_matrix:
        o1_bodies = client._get_bodies('^{}$'.format(o1_name))
        o2_bodies = client._get_bodies('^{}$'.format(o2_name))
        for parent_body, child_body in product(o1_bodies, o2_bodies):
            client.extra_disabled_collision_links[temp_name].add(
                ((parent_body, None), (child_body, None))
            )

    # TODO use trac-ik  here
    sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)

    conf = None
    success = False
    # print(flange_frame)
    # pp.draw_pose(pose_from_frame(flange_frame, scale=1))

    gantry_base_gen_fn = gantry_base_generator(client, robot, flange_frame, reachable_range=reachable_range, scale=1.0, options=options)
    with pp.LockRenderer(not debug):
        # * sample from a ball near the pose
        for attempt, base_conf in zip(range(ik_gantry_attempts), gantry_base_gen_fn):
            # * pybullet gradient-based IK
            conf = client.inverse_kinematics(robot, flange_frame, group=GANTRY_ARM_GROUP,
                                             options={'avoid_collisions': True})
            # conf = None
            if not conf:
                # * bare-arm IKfast sampler
                arm_conf_vals = sample_ik_fn(pose_from_frame(flange_frame, scale=1))
                # in total, 8 ik solutions for the 6-axis arm
                for ik_iter, arm_conf_val in enumerate(arm_conf_vals):
                    if arm_conf_val is None:
                        continue
                    conf = Configuration(list(base_conf.joint_values) + list(arm_conf_val),
                        gantry_arm_joint_types, gantry_arm_joint_names)
                    if not client.check_collisions(robot, conf, options=options):
                        msg = 'IK found with IKFast after {}/{} gantry attempts - {} ik attempts / total {} IKFast solutions | total time {:.3f}'.format(
                            attempt+1, ik_gantry_attempts, ik_iter, len(arm_conf_vals), pp.elapsed_time(start_time))
                        success = True
                        # ! break the ikfast sol loop
                        break
                if success:
                    # ! break the gantry sampling loop
                    break
            else:
                # pb finds a solution
                success = True
                msg = 'IK found with pb-IK after {}/{} gantry attempts | total time {:.3f}'.format(
                    attempt+1, ik_gantry_attempts, pp.elapsed_time(start_time))
                # ! break the gantry sampling loop
                break
        else:
            msg = 'no IK solotion found after {} gantry attempts | total time {:.3f}'.format(ik_gantry_attempts, pp.elapsed_time(start_time))

    if debug and success:
        client.set_robot_configuration(robot, conf)
        cprint((success, conf.joint_values, msg), 'green' if success else 'red')
        wait_if_gui('Conf found!')

    if temp_name in client.extra_disabled_collision_links:
        del client.extra_disabled_collision_links[temp_name]
    if not reuse_client:
        client.disconnect()

    return (success, conf, msg)


if __name__ == "__main__":
    pass
