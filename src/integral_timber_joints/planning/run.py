import os
import time
import numpy as np
import argparse
import json
import sys
from os import path
from enum import Enum, unique

from termcolor import cprint, colored
from copy import copy, deepcopy

from pybullet_planning import wait_if_gui, wait_for_user, LockRenderer, WorldSaver
from pybullet_planning import set_random_seed, set_numpy_seed, elapsed_time
import ikfast_abb_irb4600_40_255

from integral_timber_joints.planning.parsing import parse_process, save_process_and_movements
from integral_timber_joints.planning.robot_setup import load_RFL_world, to_rlf_robot_full_conf, \
    R11_INTER_CONF_VALS, R12_INTER_CONF_VALS, GANTRY_ARM_GROUP, BARE_ARM_GROUP
from integral_timber_joints.planning.utils import notify, print_title
from integral_timber_joints.planning.stream import set_state, compute_free_movement, compute_linear_movement, \
    _get_sample_bare_arm_ik_fn
from integral_timber_joints.planning.state import set_state
from integral_timber_joints.planning.visualization import visualize_movement_trajectory
from integral_timber_joints.planning.load_save_process import recompute_action_states

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement, RoboticClampSyncLinearMovement

# * Need now
# TODO replay viz from file

# * Next steps
# TODO further smoothing transit/transfer trajectories
# TODO use linkstatistics joint weight and resolutions
# TODO backtrack in case of subsequent sampling cannot find a solution (linear movement with one end specified)

###########################################

@unique
class MovementStatus(Enum):
    incorrect_type = 0
    has_traj = 1
    one_sided = 2
    both_done = 3
    neither_done = 4

###########################################

def compute_movement(client, robot, process, movement, options=None, diagnosis=False):
    if not isinstance(movement, RoboticMovement):
        return None
    options = options or {}
    # * low_res mode is used to quickly get a feeling of the planning problem
    low_res = options.get('low_res', False)
    verbose = options.get('verbose', True)
    if verbose:
        cprint(movement.short_summary, 'cyan')

    traj = None
    if isinstance(movement, RoboticLinearMovement):
        lm_options = options.copy()
        lm_options.update({
            'max_step' : 0.01, # interpolation step size, in meter
            'distance_threshold':0.002, # collision checking tolerance, in meter
            'gantry_attempts' : 500,  # gantry attempt matters more
            'cartesian_attempts' : 5, # boosting up cartesian attempt here does not really help
            'reachable_range' : (0.2, 2.8), # circle radius for sampling gantry base when computing IK
            # -------------------
            'planner_id' : 'IterativeIK',
            'cartesian_move_group' : GANTRY_ARM_GROUP,
            # -------------------
            # 'planner_id' : 'LadderGraph',
            # 'ik_function' : _get_sample_bare_arm_ik_fn(client, robot),
            # 'cartesian_move_group' : BARE_ARM_GROUP,
            })
        traj = compute_linear_movement(client, robot, process, movement, lm_options, diagnosis)
    elif isinstance(movement, RoboticClampSyncLinearMovement):
        lm_options = options.copy()
        # * interpolation step size, in meter
        lm_options.update({
            'max_step' : 0.02, # interpolation step size, in meter
            'distance_threshold':0.0025, # collision checking tolerance, in meter
            'gantry_attempts' : 500,  # gantry attempt matters more
            'cartesian_attempts' : 5, # boosting up cartesian attempt here does not really help, ladder graph only needs one attemp
            'reachable_range' : (0.2, 3.0), # circle radius for sampling gantry base when computing IK
            # -------------------
            'planner_id' : 'IterativeIK',
            'cartesian_move_group' : GANTRY_ARM_GROUP,
            # -------------------
            # 'planner_id' : 'LadderGraph',
            # 'ik_function' : _get_sample_bare_arm_ik_fn(client, robot),
            # 'cartesian_move_group' : BARE_ARM_GROUP,
            })
        traj = compute_linear_movement(client, robot, process, movement, lm_options, diagnosis)
    elif isinstance(movement, RoboticFreeMovement):
        joint_resolutions = 1.0 if low_res else 0.05
        fm_options = options.copy()
        fm_options.update({
            'rrt_restarts' : 200,
            'rrt_iterations' : 200,
            'smooth_iterations': 100,
            'resolutions' : joint_resolutions,
            'max_step' : 0.01,
            })
        traj = compute_free_movement(client, robot, process, movement, fm_options, diagnosis)
    else:
        raise ValueError()

    if traj is not None:
        # update start/end states
        movement.trajectory = traj
        movement.path_from_link = traj.path_from_link
        start_state = process.get_movement_start_state(movement)
        start_state['robot'].kinematic_config = traj.points[0]
        end_state = process.get_movement_end_state(movement)
        end_state['robot'].kinematic_config = traj.points[-1]
        return True
    else:
        if verbose:
            notify('Planning fails! Go back to the command line now!')
            wait_for_user('Planning fails, press Enter to continue. Try exit and running again - may the Luck be with you next time :)')
        return False

def propagate_states(process, selected_movements, all_movements, options=None):
    verbose = options.get('verbose', False)
    for target_m in selected_movements:
        if not isinstance(target_m, RoboticMovement) or target_m.trajectory is None:
            continue
        m_id = all_movements.index(target_m)
        target_start_state = process.get_movement_start_state(target_m)
        target_end_state = process.get_movement_end_state(target_m)
        target_start_conf = target_start_state['robot'].kinematic_config
        target_end_conf = target_end_state['robot'].kinematic_config
        if verbose:
            print('~'*5)
            print('\tPropagate states for ({}) : {}'.format(m_id, target_m.short_summary))
        # * backward fill all adjacent (-1) movements
        back_id = m_id-1
        while back_id > 0 and all_movements[back_id].planning_priority == -1:
            back_m = all_movements[back_id]
            back_start_state = process.get_movement_start_state(back_m)
            back_end_state = process.get_movement_end_state(back_m)
            back_end_conf = back_end_state['robot'].kinematic_config
            if verbose:
                if back_end_conf is not None and \
                    not back_end_conf.close_to(target_start_conf, tol=1e-3):
                        cprint('Backward Prop: Start conf not coincided - max diff {}'.format(back_end_conf.max_difference(target_start_conf)), 'red')
                        # notify('Warning! Go back to the command line now!')
                        # wait_for_user()
                print('\t- past (backward): ({}) {}'.format(colored(back_id, 'green'), back_m.short_summary))
            back_end_state['robot'].kinematic_config = target_start_conf
            back_start_state['robot'].kinematic_config = target_start_conf
            back_id -= 1
        # * forward fill all adjacent (-1) movements
        forward_id = m_id+1
        while forward_id < len(all_movements) and all_movements[forward_id].planning_priority == -1:
            forward_m = all_movements[forward_id]
            forward_start_state = process.get_movement_start_state(forward_m)
            forward_end_state = process.get_movement_end_state(forward_m)
            forward_start_conf = forward_start_state['robot'].kinematic_config
            if verbose:
                if forward_start_conf is not None and \
                    not forward_start_conf.close_to(target_end_conf, tol=1e-3):
                        cprint('Forward Prop: End conf not coincided - max diff {}'.format(back_end_conf.max_difference(target_end_conf)), 'red')
                        # notify('Warning! Go back to the command line now!')
                        # wait_for_user()
                print('\t- future (forward): ({}) {}'.format(colored(forward_id, 'green'), forward_m.short_summary))
            forward_start_state['robot'].kinematic_config = target_end_conf
            forward_end_state['robot'].kinematic_config = target_end_conf
            forward_id += 1
    # end loop selected_movements
    return all_movements

def get_movement_status(process, m, movement_types, verbose=True):
    """get the movement's current status, see the `MovementStatus` class

    Parameters
    ----------
    process : [type]
    m : Movement
    movement_types : list(Movement)
        A list of movement class types, so this function returns `MovementStatus.incorrect_type`
        if the given `m` does not fall into any of the given types.

    Returns
    -------
    MovementStatus.xxx
    """
    if not isinstance(m, RoboticMovement) or all([not isinstance(m, mt) for mt in movement_types]):
        return MovementStatus.incorrect_type
    has_start_conf = process.movement_has_start_robot_config(m)
    has_end_conf = process.movement_has_end_robot_config(m)
    has_traj = m.trajectory is not None
    # special warning
    if not isinstance(m, RoboticFreeMovement) and \
            has_start_conf and has_end_conf and not has_traj:
        cprint('{} has both start, end conf specified, but no traj computed. This is BAD!!'.format(m.short_summary), 'yellow')
        notify('Warning! Go back to the command line now!')
        # wait_for_user()
    # imply(has_traj, has_start_conf and has_end_conf)
    assert not has_traj or (has_start_conf and has_end_conf)
    if has_traj:
        return MovementStatus.has_traj
    else:
        if has_start_conf ^ has_end_conf:
            return MovementStatus.one_sided
        elif has_start_conf and has_end_conf:
            return MovementStatus.both_done
        elif not has_start_conf and not has_end_conf:
            return MovementStatus.neither_done

def compute_selected_movements(client, robot, process, beam_id, priority, movement_types, movement_statuses, options=None,
        viz_upon_found=False, diagnosis=False):
    """Compute trajectories for movements specified by a certain criteria.

    Parameters
    ----------
    client : [type]
    robot : compas_fab Robot
    process : Process
    beam_id : int
        beam id
    priority : int
        -1, 0 or 1
    movement_types : list(Movement subclass)
        list of `MovementStatus` to filter, e.g. [RoboticLiearMovement, ...]
    movement_statuses : list(MovementStatus.xx)
        list of `MovementStatus` to filter, e.g. [MovementStatus.one_sided, ...]
    options : dict, optional
        planning options, by default None
    viz_upon_found : bool, optional
        visualize trajectory immediately upon found, by default False
    step_sim : bool, optional
        step conf-by-conf if viz_upon_found == True, by default False
    """
    verbose = options.get('verbose', False)
    if verbose:
        print('='*20)
        print_title('* compute {} (priority {}, status {})'.format([mt.__name__ for mt in movement_types], priority,
            movement_statuses))
    all_movements = process.get_movements_by_beam_id(beam_id)
    selected_movements = process.get_movements_by_planning_priority(beam_id, priority)
    for m in selected_movements:
        altered_movements = []
        if get_movement_status(process, m, movement_types) in movement_statuses:
            m_id = all_movements.index(m)
            if verbose:
                print('-'*10)
                print('({})'.format(m_id))
            if compute_movement(client, robot, process, m, options, diagnosis):
                altered_movements.append(m)
                if viz_upon_found:
                    with WorldSaver():
                        visualize_movement_trajectory(client, robot, process, m, step_sim=True)
            else:
                # break
                return False
        # * propagate to -1 movements
        propagate_states(process, altered_movements, all_movements, options=options)
    if verbose:
        print('\n\n')
        process.get_movement_summary_by_beam_id(beam_id)
    return True

#################################

def compute_movements_for_beam_id(client, robot, process, beam_id, args, options=None):
    if args.verbose:
        print_title('0) Before planning')
        process.get_movement_summary_by_beam_id(beam_id)
        # wait_for_user()

    with LockRenderer(not args.debug) as lockrenderer:
        options['lockrenderer'] = lockrenderer
        # TODO loop and backtrack
        # TODO have to find a way to recover movements
        all_movements = process.get_movements_by_beam_id(beam_id)

        if not compute_selected_movements(client, robot, process, beam_id, 1, [RoboticLinearMovement, RoboticClampSyncLinearMovement],
            [MovementStatus.neither_done, MovementStatus.one_sided],
            options=options, viz_upon_found=args.viz_upon_found, diagnosis=args.diagnosis):
            return False

        # TODO if fails remove the related movement's trajectory and try again
        if not compute_selected_movements(client, robot, process, beam_id, 0, [RoboticLinearMovement],
            [MovementStatus.one_sided],
            options=options, viz_upon_found=args.viz_upon_found, diagnosis=args.diagnosis):
            return False

        # since adjacent "neither_done" states will change to `one_sided` and get skipped
        # which will cause adjacent linear movement joint flip problems (especially for clamp placements)
        # Thus, when solving `neither_done`, we solve for both `neither_done` and `one_sided` sequentially
        # The movement statuses get changed on the fly.
        if not compute_selected_movements(client, robot, process, beam_id, 0, [RoboticLinearMovement],
            [MovementStatus.neither_done, MovementStatus.one_sided],
            options=options, viz_upon_found=args.viz_upon_found, diagnosis=args.diagnosis):
            return False

        # Ideally, all the free motions should have both start and end conf specified.
        # one_sided is used to sample the start conf if none is given (especially when `arg.problem_dir = 'YJ_tmp'` is not used).
        if not compute_selected_movements(client, robot, process, beam_id, 0, [RoboticFreeMovement],
            [MovementStatus.both_done, MovementStatus.one_sided],
            options=options, viz_upon_found=args.viz_upon_found, diagnosis=args.diagnosis):
            return False

    # * export computed movements
    if args.write:
        save_process_and_movements(args.problem, process, all_movements, overwrite=False,
            include_traj_in_process=False, save_temp=args.save_temp)

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
    parser.add_argument('-p', '--problem', default='twelve_pieces_process.json', # pavilion.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='.', # pavilion.json
                        help='subdir of the process file, default to `.`. Popular use: `YJ_tmp`, `<time stamp>`')
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('--seq_i', default=0, type=int, help='individual step to plan. Set value to -1 to run all.')
    parser.add_argument('--write', action='store_true', help='Write output json.')
    parser.add_argument('--recompute_action_states', action='store_true', help='Recompute actions and states for the process.')
    parser.add_argument('--watch', action='store_true', help='Watch computed trajectories in the pybullet GUI.')
    parser.add_argument('--debug', action='store_true', help='Debug mode')
    parser.add_argument('--verbose', action='store_false', help='Print out verbose. Defaults to True.')
    parser.add_argument('--diagnosis', action='store_true', help='Diagnosis mode')
    parser.add_argument('--step_sim', action='store_true', help='Pause after each conf viz.')
    parser.add_argument('--disable_env', action='store_true', help='Disable environment collision geometry.')
    parser.add_argument('--view_states', action='store_true', help='Visualize states.')
    parser.add_argument('--reinit_tool', action='store_true', help='Regenerate tool URDFs.')
    parser.add_argument('--save_temp', action='store_true', help='Save a temporary process file. Defaults to False.')
    parser.add_argument('--viz_upon_found', action='store_true', help='Viz found traj immediately after found. Defaults to False.')
    parser.add_argument('--low_res', action='store_true', help='Run the planning with low resolutions. Defaults to True.')
    parser.add_argument('--seed', default=0, help='Random seed.')
    args = parser.parse_args()
    print('Arguments:', args)
    print('='*10)

    initial_time = time.time()
    # * random seed setup
    # seed = hash(time.time())
    # seed = args.seed
    # set_random_seed(seed)
    # set_numpy_seed(seed)

    options = {
        'debug' : args.debug,
        'low_res' : args.low_res,
        'distance_threshold' : 0.0012,
        'frame_jump_tolerance' : 0.0012,
        'verbose' : args.verbose,
    }

    # * Connect to path planning backend and initialize robot parameters
    client, robot, _ = load_RFL_world(viewer=args.viewer or args.diagnosis or args.view_states or args.watch or args.step_sim)
    # * Load process and recompute actions and states
    process = parse_process(args.problem, subdir=args.problem_subdir)
    if args.recompute_action_states:
        cprint('Recomputing Actions and States', 'cyan')
        recompute_action_states(process, False)
    set_initial_state(client, robot, process, disable_env=args.disable_env, reinit_tool=args.reinit_tool)

    beam_ids = list(process.assembly.sequence) if args.seq_i < 0 else [process.assembly.sequence[args.seq_i]]
    beam_attempts = 5
    for s_i, beam_id in enumerate(beam_ids):
        print('-'*20)
        print('({}) Beam#{}'.format(s_i, beam_id))
        for i in range(beam_attempts):
            print('-- iter #{}'.format(i))
            if compute_movements_for_beam_id(client, robot, process, beam_id, args, options=options):
                cprint('Beam #{} plan found after {} iters!'.format(beam_id, i+1), 'cyan')
                break
            client.disconnect()
            client, robot, _ = load_RFL_world(viewer=args.viewer or args.diagnosis or args.view_states or args.watch or args.step_sim)
            process = parse_process(args.problem, subdir=args.problem_subdir)
            set_initial_state(client, robot, process, disable_env=args.disable_env, reinit_tool=False)
        else:
            cprint('Beam #{} plan not found after {} attempts.'.format(beam_id, beam_attempts), 'red')

    print('Total time:', elapsed_time(initial_time))
    client.disconnect()

if __name__ == '__main__':
    main()
