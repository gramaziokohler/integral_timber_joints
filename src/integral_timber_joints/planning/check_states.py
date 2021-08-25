import os, time
import argparse
from termcolor import cprint
from itertools import product, combinations
import numpy as np

from compas_fab.robots import Robot
from compas.robots import Joint

from pybullet_planning import wait_if_gui, wait_for_user, has_gui
from pybullet_planning import get_distance, draw_collision_diagnosis, expand_links, get_all_links, \
    link_from_name, pairwise_link_collision_info, get_name, get_link_name, WorldSaver

from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from compas_fab_pychoreo.utils import compare_configurations

from integral_timber_joints.planning.parsing import parse_process, get_process_path
from integral_timber_joints.planning.robot_setup import load_RFL_world, to_rlf_robot_full_conf, \
    R11_INTER_CONF_VALS, R12_INTER_CONF_VALS, GANTRY_ARM_GROUP
from integral_timber_joints.planning.state import set_state
from integral_timber_joints.planning.utils import print_title

from integral_timber_joints.process import RoboticMovement, RobotClampAssemblyProcess

###########################################
def check_state_collisions_among_objects(client: PyChoreoClient, robot : Robot, process: RobotClampAssemblyProcess,
    scene_from_object: dict, options=None):
    options = options or {}
    debug = options.get('debug', False)

    # * update state
    set_state(client, robot, process, scene_from_object, options=options)
    if debug:
        client._print_object_summary()

    in_collision = client.check_attachment_collisions(options)
    if scene_from_object[process.robot_config_key]:
        pychore_collision_fn = PyChoreoConfigurationCollisionChecker(client)
        in_collision |= pychore_collision_fn.check_collisions(robot, scene_from_object[process.robot_config_key], options=options)
    if debug:
        wait_for_user()
    return in_collision

###########################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='nine_pieces_process.json', # pavilion_process.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='.', # pavilion.json
                        help='subdir of the process file, default to `.`. Popular use: `YJ_tmp`, `<time stamp>`')
    parser.add_argument('--plan_summary', action='store_true', help='Give a summary of currently found plans.')
    parser.add_argument('--verify_plan', action='store_true', help='Collision check found trajectories.')
    #
    parser.add_argument('--seq_i', type=int, default=0, help='sequence index to start from, default to 0.')
    parser.add_argument('--batch_run', action='store_true', help='Batch run. Will turn `--seq_i` as run from.')
    parser.add_argument('--id_only', default=None, type=str, help='Compute only for movement with a specific tag, e.g. `A54_M0`.')
    #
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('--reinit_tool', action='store_true', help='Regenerate tool URDFs.')
    parser.add_argument('--debug', action='store_true', help='debug mode.')
    parser.add_argument('--traj_collision', action='store_false', help='check trajectory collisions.')
    args = parser.parse_args()
    print('Arguments:', args)
    print('='*10)

    process = parse_process(args.problem, subdir=args.problem_subdir)

    result_path = get_process_path(args.problem, subdir=args.problem_subdir)
    if args.plan_summary:
        ext_movement_path = os.path.dirname(result_path)
        cprint('Loading external movements from {}'.format(ext_movement_path), 'cyan')
        unfound_beams = []
        # process.load_external_movements(ext_movement_path)
        for i, beam_id in enumerate(process.assembly.sequence):
            print('='*10)
            cprint('({}) Beam #{}:'.format(i, beam_id), 'cyan')
            b_movements = process.get_movements_by_beam_id(beam_id)
            all_found = True
            for movement in b_movements:
                movement_path = os.path.join(ext_movement_path, movement.filepath)
                if not os.path.exists(movement_path):
                    cprint('{} not found | {}'.format(movement.movement_id, movement.short_summary), 'red')
                    all_found = False
            if all_found:
                if len(b_movements) > 0:
                    cprint('({}) Beam #{} all found!'.format(i, beam_id), 'green')
                    movement_path = os.path.join(ext_movement_path, b_movements[-1].filepath)
                    print("   created: %s" % time.ctime(os.path.getctime(movement_path)))
                    print("   last modified: %s" % time.ctime(os.path.getmtime(movement_path)))
                else:
                    cprint('({}) Beam #{} empty movement list!'.format(i, beam_id), 'yellowjk')
            else:
                unfound_beams.append((i, beam_id))
        print('Unfound beams: {}'.format(unfound_beams))
        return

    if args.verify_plan:
        ext_movement_path = os.path.dirname(result_path)
        cprint('Loading external movements from {}'.format(ext_movement_path), 'cyan')
        process.load_external_movements(ext_movement_path)

    # * Connect to path planning backend and initialize robot parameters
    client, robot, _ = load_RFL_world(viewer=args.viewer)

    # set all other unused robot
    full_start_conf = to_rlf_robot_full_conf(R11_INTER_CONF_VALS, R12_INTER_CONF_VALS)
    client.set_robot_configuration(robot, full_start_conf)

    process.set_initial_state_robot_config(process.robot_initial_config)
    set_state(client, robot, process, process.initial_state, initialize=True,
        options={'debug' : False, 'reinit_tool' : args.reinit_tool})
    # * collision sanity check for the initial conf
    assert not client.check_collisions(robot, full_start_conf, options={'diagnosis':True})

    joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    joint_types = robot.get_joint_types_by_names(joint_names)
    joint_jump_threshold = {jt_name : 0.1 \
            if jt_type in [Joint.REVOLUTE, Joint.CONTINUOUS] else 0.1 \
            for jt_name, jt_type in zip(joint_names, joint_types)}

    options = {
        # * collision checking tolerance, in meter, peneration distance bigger than this number will be regarded as in collision
        'distance_threshold' : 0.0025,
        # * buffering distance, If the distance between objects exceeds this maximum distance, no points may be returned.
        'max_distance' : 0.0,
        # * If target_configuration is different from the target_frame by more that this amount at flange center, a warning will be raised.
        'frame_jump_tolerance' : 0.0012,
        'diagnosis' : True,
        'debug' : args.debug,
    }

    full_seq_len = len(process.assembly.sequence)
    assert args.seq_i >= 0 and args.seq_i < full_seq_len, 'seq_i out of range!'
    if args.batch_run:
        beam_ids = [process.assembly.sequence[si] for si in range(args.seq_i, full_seq_len)]
    elif args.id_only:
        beam_ids = [process.get_beam_id_from_movement_id(args.id_only)]
    else:
        beam_ids = [process.assembly.sequence[args.seq_i]]

    joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)

    movement_need_fix = []
    for beam_id in beam_ids:
        seq_i = process.assembly.sequence.index(beam_id)
        if not args.id_only:
            print('='*20)
            cprint('(Seq#{}) Beam {}'.format(seq_i, beam_id), 'yellow')
        if args.debug:
            process.get_movement_summary_by_beam_id(beam_id)

        all_movements = process.get_movements_by_beam_id(beam_id)
        for i, m in enumerate(all_movements):
            if args.id_only and m.movement_id != args.id_only:
                continue
            start_state = process.get_movement_start_scene(m)
            end_state = process.get_movement_end_scene(m)
            start_conf = process.get_movement_start_robot_config(m)
            # if start_conf:
            #     start_conf.joint_names = joint_names
            end_conf = process.get_movement_end_robot_config(m)
            # if end_conf:
            #     end_conf.joint_names = joint_names

            in_collision = False
            joint_flip = False
            no_traj = False
            m_index = process.movements.index(m)
            print('-'*10)
            print_title('(MovementIndex={}) (Seq#{}-#{}) {}'.format(m_index, seq_i, i, m.short_summary))

            if isinstance(m, RoboticMovement):
                # movement-specific ACM
                temp_name = '_tmp'
                for o1_name, o2_name in m.allowed_collision_matrix:
                    o1_bodies = client._get_bodies('^{}$'.format(o1_name))
                    o2_bodies = client._get_bodies('^{}$'.format(o2_name))
                    for parent_body, child_body in product(o1_bodies, o2_bodies):
                        client.extra_disabled_collision_links[temp_name].add(
                            ((parent_body, None), (child_body, None))
                        )

                cprint('Start State:', 'blue')
                start_in_collision = check_state_collisions_among_objects(client, robot, process, start_state, options=options)
                in_collision |= start_in_collision
                cprint('Start State in collision: {}.'.format(start_in_collision), 'red' if start_in_collision else 'green')
                print('#'*20)

                if args.verify_plan:
                    pychore_collision_fn = PyChoreoConfigurationCollisionChecker(client)
                    if m.trajectory:
                        # print(client._print_object_summary())
                        prev_conf = start_conf
                        for conf_id, jpt in enumerate(list(m.trajectory.points) + [end_conf]):
                            if not jpt.joint_names:
                                jpt.joint_names = joint_names
                            if args.traj_collision:
                                # with WorldSaver():
                                in_collision |= pychore_collision_fn.check_collisions(robot, jpt, options=options)
                                # in_collision |= client.check_sweeping_collisions(robot, prev_conf, jpt, options=options)

                            if prev_conf and compare_configurations(jpt, prev_conf, joint_jump_threshold, verbose=True):
                                print('Up | traj point #{}/{}'.format(conf_id, len(m.trajectory.points)))
                                print('='*10)
                                joint_flip |= True
                            prev_conf = jpt
                        cprint('Trajectory in trouble: in_collision {} | joint_flip {}'.format(in_collision, joint_flip), 'red' if in_collision or joint_flip else 'green')
                        if in_collision or joint_flip:
                            if args.debug:
                                wait_for_user()
                    else:
                        no_traj = True
                        cprint('No trajectory found!', 'red')
                        # wait_for_user()
                    print('#'*20)

                cprint('End State:', 'blue')
                end_in_collision = check_state_collisions_among_objects(client, robot, process, end_state, options=options)
                in_collision |= end_in_collision
                cprint('End State in collision: {}.'.format(end_in_collision), 'red' if end_in_collision else 'green')
                print('#'*20)

                if temp_name in client.extra_disabled_collision_links:
                    del client.extra_disabled_collision_links[temp_name]
            else:
                # check joint consistency
                if start_conf and end_conf:
                    joint_flip |= compare_configurations(start_conf, end_conf, joint_jump_threshold, verbose=True)
                    if joint_flip:
                        cprint('Joint conf not consistent!'.format(m.short_summary), 'red')
                        # wait_for_user()
                else:
                    no_traj = True
                    cprint('Start found: {} | End conf found: {}.'.format(start_conf is not None, end_conf is not None), 'yellow')
                    # wait_for_user()

            if in_collision or joint_flip or no_traj:
                movement_need_fix.append(m)

    print('='*20)
    if len(movement_need_fix) == 0:
        cprint('Congrats, check state done!', 'green')
    else:
        cprint('Movements that requires care and love:', 'yellow')
        for fm in movement_need_fix:
            cprint(fm.short_summary)
    print('='*20)

    client.disconnect()

if __name__ == '__main__':
    main()
