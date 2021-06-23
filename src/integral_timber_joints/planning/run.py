import os
import time
import numpy as np
import argparse
import json
import sys
from os import path

from termcolor import cprint, colored
from copy import copy, deepcopy
from compas.utilities import DataEncoder

from compas.robots import Joint
from pybullet_planning import wait_if_gui, wait_for_user, LockRenderer, WorldSaver, HideOutput

from integral_timber_joints.planning.parsing import parse_process, save_process_and_movements, get_process_path, save_process
from integral_timber_joints.planning.robot_setup import load_RFL_world, to_rlf_robot_full_conf, \
    R11_INTER_CONF_VALS, R12_INTER_CONF_VALS, GANTRY_ARM_GROUP
from integral_timber_joints.planning.utils import notify, print_title
from integral_timber_joints.planning.state import set_state
from integral_timber_joints.planning.visualization import visualize_movement_trajectory
from integral_timber_joints.planning.load_save_process import recompute_action_states
from integral_timber_joints.planning.solve import get_movement_status, MovementStatus, compute_selected_movements

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement

# * Need now
# TODO replay viz from file

# * Next steps
# TODO further smoothing transit/transfer trajectories
# TODO use linkstatistics joint weight and resolutions
# TODO backtrack in case of subsequent sampling cannot find a solution (linear movement with one end specified)

##############################################

def compute_movements_for_beam_id(client, robot, process, beam_id, args, options=None):
    # if args.verbose:
    #     print_title('0) Before planning')
    #     process.get_movement_summary_by_beam_id(beam_id)
        # wait_for_user()

    with LockRenderer(not args.debug) as lockrenderer:
        options['lockrenderer'] = lockrenderer
        # TODO loop and backtrack
        # TODO have to find a way to recover movements
        all_movements = process.get_movements_by_beam_id(beam_id)
        altered_movements = []
        with HideOutput(): #args.verbose
            if args.id_only is None:
                if not args.free_motion_only:
                    success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 1, [RoboticLinearMovement, RoboticClampSyncLinearMovement],
                        [MovementStatus.neither_done, MovementStatus.one_sided],
                        options=options, viz_upon_found=args.viz_upon_found, diagnosis=args.diagnosis)
                    if not success:
                        return False
                    else:
                        altered_movements.extend(altered_ms)
                        if args.save_now:
                            save_process_and_movements(args.problem, process, altered_ms, overwrite=False,
                                include_traj_in_process=False, save_temp=args.save_temp)

                    # TODO if fails remove the related movement's trajectory and try again
                    success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 0, [RoboticLinearMovement],
                        [MovementStatus.one_sided],
                        options=options, viz_upon_found=args.viz_upon_found, diagnosis=args.diagnosis, write_now=args.save_now)
                    if not success:
                        return False
                    else:
                        altered_movements.extend(altered_ms)

                    # since adjacent "neither_done" states will change to `one_sided` and get skipped
                    # which will cause adjacent linear movement joint flip problems (especially for clamp placements)
                    # Thus, when solving `neither_done`, we solve for both `neither_done` and `one_sided` sequentially
                    # The movement statuses get changed on the fly.
                    success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 0, [RoboticLinearMovement],
                        [MovementStatus.neither_done, MovementStatus.one_sided],
                        options=options, viz_upon_found=args.viz_upon_found, diagnosis=args.diagnosis, write_now=args.save_now)
                    if not success:
                        return False
                    else:
                        altered_movements.extend(altered_ms)

                # Ideally, all the free motions should have both start and end conf specified.
                # one_sided is used to sample the start conf if none is given (especially when `arg.problem_dir = 'YJ_tmp'` is not used).
                success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 0, [RoboticFreeMovement],
                    [MovementStatus.both_done, MovementStatus.one_sided] if not args.free_motion_only else \
                        [MovementStatus.both_done, MovementStatus.one_sided], #, MovementStatus.has_traj],
                    options=options, viz_upon_found=args.viz_upon_found, diagnosis=args.diagnosis, write_now=args.save_now)
                if not success:
                    print('No success for free motions')
                    return False
                else:
                    altered_movements.extend(altered_ms)

                # * export computed movements
                if not args.save_now and args.write:
                    save_process_and_movements(args.problem, process, altered_movements, overwrite=False,
                        include_traj_in_process=False, save_temp=args.save_temp)
            else:
                cprint('Computing only for {}'.format(args.id_only), 'yellow')
                options['movement_id_filter'] = [args.id_only]
                chosen_m = process.get_movement_by_movement_id(args.id_only)
                # * if linear movement and has both end specified, ask user keep start or end
                if not args.propagate_only and get_movement_status(process, chosen_m, [RoboticLinearMovement]) \
                    in [MovementStatus.has_traj, MovementStatus.both_done]:
                    chosen_m.trajectory = None
                    keep_end = int(input("Keep start or end conf? Enter 0 for start, 1 for end. 2 for abandoning both."))
                    if keep_end == 0:
                        # keep start
                        end_state = process.get_movement_end_state(chosen_m)
                        end_state['robot'].kinematic_config = None
                    elif keep_end == 1:
                        start_state = process.get_movement_start_state(chosen_m)
                        start_state['robot'].kinematic_config = None
                    elif keep_end == 2:
                        start_state = process.get_movement_start_state(chosen_m)
                        start_state['robot'].kinematic_config = None
                        end_state = process.get_movement_end_state(chosen_m)
                        end_state['robot'].kinematic_config = None
                        assert get_movement_status(process, chosen_m, [RoboticLinearMovement]) in [MovementStatus.neither_done]
                    if keep_end != 2:
                        assert get_movement_status(process, chosen_m, [RoboticLinearMovement]) in [MovementStatus.one_sided]

                success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 0, [],
                    None, options=options, viz_upon_found=args.viz_upon_found, diagnosis=args.diagnosis, \
                    write_now=args.write, plan_impacted=args.plan_impacted)
                if not success:
                    return False

    # * final visualization
    if args.watch:
        print('='*20)
        print_title('Visualize results')
        set_state(client, robot, process, process.initial_state)
        for m in all_movements:
            visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim)

    if args.verbose:
        notify('A plan has been found for beam id {}!'.format(beam_id))
    return True

#################################

def set_initial_state(client, robot, process, disable_env=False, reinit_tool=True):
    # set all other unused robot
    full_start_conf = to_rlf_robot_full_conf(R11_INTER_CONF_VALS, R12_INTER_CONF_VALS)
    client.set_robot_configuration(robot, full_start_conf)
    # wait_if_gui('Pre Initial state.')
    process.initial_state['robot'].kinematic_config = process.robot_initial_config
    try:
        set_state(client, robot, process, process.initial_state, initialize=True,
            options={'debug' : False, 'include_env' : not disable_env, 'reinit_tool' : reinit_tool})
    except:
        cprint('Recomputing Actions and States', 'cyan')
        recompute_action_states(process, False)
        set_state(client, robot, process, process.initial_state, initialize=True,
            options={'debug' : False, 'include_env' : not disable_env, 'reinit_tool' : reinit_tool})
    # # * collision sanity check
    # assert not client.check_collisions(robot, full_start_conf, options={'diagnosis':True})

#################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='pavilion_process.json', # twelve_pieces_process.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='.', # pavilion.json
                        help='subdir of the process file, default to `.`. Popular use: `results`')
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    #
    parser.add_argument('--seq_i', default=0, type=int, help='individual step to plan.')
    parser.add_argument('--batch_run', action='store_true', help='Batch run. Will turn `--seq_i` as run from.')
    #
    parser.add_argument('--id_only', default=None, type=str, help='Compute only for movement with a specific tag, e.g. `A54_M0`.')
    parser.add_argument('--free_motion_only', action='store_true', help='Only compute free motions.')
    parser.add_argument('--propagate_only', action='store_true', help='Only do state propagation and impacted movement planning.')
    parser.add_argument('--plan_impacted', action='store_true', help='impacted movement planning.')
    #
    parser.add_argument('--write', action='store_true', help='Write output json.')
    parser.add_argument('--save_now', action='store_true', help='Save immediately upon found.')
    parser.add_argument('--recompute_action_states', action='store_true', help='Recompute actions and states for the process.')
    parser.add_argument('--load_external_movements', action='store_true', help='Load externally saved movements into the parsed process, default to False.')
    #
    parser.add_argument('--watch', action='store_true', help='Watch computed trajectories in the pybullet GUI.')
    parser.add_argument('--debug', action='store_true', help='Debug mode')
    parser.add_argument('--step_sim', action='store_true', help='Pause after each conf viz.')
    parser.add_argument('--verbose', action='store_false', help='Print out verbose. Defaults to True.')
    parser.add_argument('--diagnosis', action='store_true', help='Diagnosis mode')
    #
    parser.add_argument('--disable_env', action='store_true', help='Disable environment collision geometry.')
    parser.add_argument('--reinit_tool', action='store_true', help='Regenerate tool URDFs.')
    parser.add_argument('--save_temp', action='store_true', help='Save a temporary process file. Defaults to False.')
    parser.add_argument('--viz_upon_found', action='store_true', help='Viz found traj immediately after found. Defaults to False.')
    parser.add_argument('--use_stored_seed', action='store_true', help='Use stored seed. Defaults to False.')
    #
    parser.add_argument('--low_res', action='store_true', help='Run the planning with low resolutions. Defaults to True.')
    parser.add_argument('--max_distance', default=0.0, type=float, help='Buffering distance for collision checking, larger means safer. Defaults to 0.')
    args = parser.parse_args()
    print('Arguments:', args)
    print('='*10)

    # * Connect to path planning backend and initialize robot parameters
    client, robot, _ = load_RFL_world(viewer=args.viewer or args.diagnosis or args.watch or args.step_sim)

    #########
    # * Load process and recompute actions and states
    process = parse_process(args.problem, subdir=args.problem_subdir)
    result_path = get_process_path(args.problem, subdir='results')
    if len(process.movements) == 0:
        cprint('No movements found in process, trigger recompute actions.', 'red')
        args.recompute_action_states = True
    if args.recompute_action_states:
        cprint('Recomputing Actions and States', 'cyan')
        recompute_action_states(process, False)

    # force load external if only planning for the free motions
    args.load_external_movements = args.load_external_movements or args.free_motion_only or args.id_only is not None
    if args.load_external_movements:
        ext_movement_path = os.path.dirname(result_path)
        cprint('Loading external movements from {}'.format(ext_movement_path), 'cyan')
        process.load_external_movements(ext_movement_path)

    if args.recompute_action_states:
        save_process(process, result_path)
        cprint('Recomputed process saved to %s' % result_path, 'green')
    #########

    joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    joint_types = robot.get_joint_types_by_names(joint_names)
    # 0.1 rad = 5.7 deg
    joint_jump_threshold = {jt_name : np.pi/6 \
            if jt_type in [Joint.REVOLUTE, Joint.CONTINUOUS] else 0.1 \
            for jt_name, jt_type in zip(joint_names, joint_types)}
    options = {
        'debug' : args.debug,
        'diagnosis' : args.diagnosis,
        'low_res' : args.low_res,
        'distance_threshold' : 0.0012,
        'frame_jump_tolerance' : 0.0012,
        'verbose' : args.verbose,
        'problem_name' : args.problem,
        'use_stored_seed' : args.use_stored_seed,
        'jump_threshold' : joint_jump_threshold,
        'max_distance' : args.max_distance,
        'propagate_only' : args.propagate_only,
        # until Trajectory json is fixed...
        'joint_names' : robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP),
    }

    set_initial_state(client, robot, process, disable_env=args.disable_env, reinit_tool=args.reinit_tool)

    full_seq_len = len(process.assembly.sequence)
    assert args.seq_i < full_seq_len and args.seq_i >= 0
    if args.batch_run:
        beam_ids = [process.assembly.sequence[si] for si in range(args.seq_i, full_seq_len)]
    elif args.id_only:
        beam_ids = [process.get_beam_id_from_movement_id(args.id_only)]
    else:
        # only one
        beam_ids = [process.assembly.sequence[args.seq_i]]

    beam_attempts = 10 if args.batch_run else 100
    for beam_id in beam_ids:
        print('-'*20)
        s_i = process.assembly.sequence.index(beam_id)
        print('({}) Beam#{}'.format(s_i, beam_id))
        for i in range(beam_attempts):
            print('\n\n\n')
            print('#'*20)
            print('-- iter #{}'.format(i))
            if compute_movements_for_beam_id(client, robot, process, beam_id, args, options=options):
                cprint('Beam #{} plan found after {} iters!'.format(beam_id, i+1), 'cyan')
                break
            client.disconnect()

            print('\n\n\n')
            client, robot, _ = load_RFL_world(viewer=args.viewer or args.diagnosis or args.watch or args.step_sim)
            process = parse_process(args.problem, subdir='results')
            set_initial_state(client, robot, process, disable_env=args.disable_env, reinit_tool=False)
        else:
            cprint('Beam #{} plan not found after {} attempts.'.format(beam_id, beam_attempts), 'red')

    client.disconnect()

if __name__ == '__main__':
    main()
