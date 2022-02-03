import time
import logging
from enum import Enum, unique
from copy import deepcopy
from collections import defaultdict
from turtle import back
from compas_fab.backends.pybullet.utils import LOG
from termcolor import colored
from tqdm import tqdm

from pybullet_planning import set_random_seed, set_numpy_seed, elapsed_time, get_random_seed
from pybullet_planning import wait_if_gui, wait_for_user, WorldSaver

from compas_fab.robots import Robot
from compas_fab_pychoreo.utils import is_configurations_close

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement
from integral_timber_joints.process import RobotClampAssemblyProcess, Movement
from integral_timber_joints.planning.stream import compute_free_movement, compute_linear_movement
from integral_timber_joints.planning.utils import notify, print_title, LOGGER
from integral_timber_joints.planning.robot_setup import GANTRY_ARM_GROUP
from integral_timber_joints.planning.visualization import visualize_movement_trajectory

try:
    # This is for type hint in VSCode/Pylance IDE while compatible with IronPython
    from typing import Dict, List, Optional, Tuple, Type, Any
    from compas_fab.backends import PyBulletClient
except:
    pass

###########################################

@unique
class MovementStatus(Enum):
    incorrect_type = 0
    correct_type = 1
    has_traj = 2
    one_sided = 3
    both_done = 4
    neither_done = 5

def get_movement_status(process, m, movement_types, verbose=True, check_type_only=False):
    # type: (RobotClampAssemblyProcess, Movement, List[Type], bool, bool) -> MovementStatus
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
    # if not isinstance(m, RoboticMovement):
    if all([not isinstance(m, mt) for mt in movement_types]):
        return MovementStatus.incorrect_type
    if check_type_only:
        return MovementStatus.correct_type
    has_start_conf = process.movement_has_start_robot_config(m)
    has_end_conf = process.movement_has_end_robot_config(m)
    has_traj = m.trajectory is not None
    # special warning
    if not isinstance(m, RoboticFreeMovement) and \
            has_start_conf and has_end_conf and not has_traj:
        LOGGER.error(colored('{} has both start, end conf specified, but no traj computed. This is BAD!!'.format(m.short_summary), 'yellow'))
        notify('Warning! Go back to the command line now!')
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

###########################################

def compute_movement(client, robot, process, movement, options=None, diagnosis=False):
    # type: (PyBulletClient, Robot, RobotClampAssemblyProcess, Movement, Dict, Dict) -> bool
    """Compute trajectory for a single movement
    """
    if not isinstance(movement, RoboticMovement):
        return None
    options = options or {}
    # * low_res mode is used to quickly get a feeling of the planning problem
    verbose = options.get('verbose', True)
    joint_compare_tolerances = options.get('joint_compare_tolerances', {})
    if verbose:
        LOGGER.debug(colored(movement.short_summary, 'cyan'))

    # use_stored_seed = options.get('use_stored_seed', False)
    # set seed stored in the movement
    # if use_stored_seed:
    #     seed = movement.seed
    #     assert seed is not None, 'No meaningful seed saved in the movement.'
    # else:
    #     seed = get_random_seed()
    #     movement.seed = seed
    # set_random_seed(seed)
    # set_numpy_seed(seed)

    # * custom limits
    traj = None
    if isinstance(movement, RoboticLinearMovement):
        lm_options = options.copy()
        lm_options.update({
            'max_step' : 0.01, # interpolation step size, in meter
            'cartesian_attempts' : 1, # boosting up cartesian attempt here does not really help
            # -------------------
            'planner_id' : 'IterativeIK',
            'cartesian_move_group' : GANTRY_ARM_GROUP,
            # -------------------
            # 'planner_id' : 'LadderGraph',
            # 'ik_function' : _get_sample_bare_arm_ik_fn(client, robot),
            # 'cartesian_move_group' : BARE_ARM_GROUP,
            })
        traj = compute_linear_movement(client, robot, process, movement, lm_options, diagnosis)
    elif isinstance(movement, RoboticClampSyncLinearMovement) or \
         isinstance(movement, RobotScrewdriverSyncLinearMovement):
        #  'reorient' in movement.short_summary:
        lm_options = options.copy()
        # * interpolation step size, in meter
        lm_options.update({
            'max_step' : 0.02, # interpolation step size, in meter
            'cartesian_attempts' : 1, # boosting up cartesian attempt here does not really help, ladder graph only needs one attemp
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
        fm_options = options.copy()
        fm_options.update({
            'rrt_restarts' : 2, #20,
            'smooth_iterations': None, # ! smoothing will be done in postprocessing
            # -------------------
            'max_step' : 0.01, # interpolation step size, in meter, used in buffering motion
            })
        traj = compute_free_movement(client, robot, process, movement, fm_options, diagnosis)
    else:
        LOGGER.critical("Unrecognized movement type {}".format(movement))
        return None

    if traj is not None:
        # update start/end states
        prev_robot_conf = process.get_movement_start_robot_config(movement)
        if prev_robot_conf is not None and not is_configurations_close(prev_robot_conf, traj.points[0], options=options):
            LOGGER.error('Planned trajectory\'s first conf does not agree with the previous movement\'s end conf! Planning fails.')
            return False
        movement.trajectory = traj
        process.set_movement_start_robot_config(movement, traj.points[0])
        process.set_movement_end_robot_config(movement, traj.points[-1])
        return True
    else:
        notify('Planning fails! Go back to the command line now!')
        return False

def compute_selected_movements(client, robot, process, beam_id, priority, movement_types=None, movement_statuses=None, options=None,
        viz_upon_found=False, diagnosis=False, check_type_only=False):
    # type: (PyBulletClient, Robot, RobotClampAssemblyProcess, int, int, List[Type], List[MovementStatus], Dict, bool, bool, bool) -> Tuple[bool, List]

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
        None if no filter is applied.
    options : dict, optional
        planning options, by default None
    viz_upon_found : bool, optional
        visualize trajectory immediately upon found, by default False
    step_sim : bool, optional
        step conf-by-conf if viz_upon_found == True, by default False
    """
    verbose = options.get('verbose', False)
    m_attempts = options.get('movement_planning_reattempts', 1)
    movement_id_filter = options.get('movement_id_filter', [])
    movement_types = movement_types or []
    if len(movement_id_filter) > 0:
        selected_movements = []
        for mid in movement_id_filter:
            if mid.startswith('A'):
                chosen_m = process.get_movement_by_movement_id(mid)
            else:
                chosen_m = process.movements[int(mid)]
            selected_movements.append(chosen_m)
        if verbose:
            LOGGER.debug('='*20)
            print_title('* compute movement ids: {}'.format(movement_id_filter))
    else:
        selected_movements = process.get_movements_by_planning_priority(beam_id, priority)
        if verbose:
            LOGGER.debug('='*20)
            print_title('* compute {} (priority {}, status {})'.format([mt.__name__ for mt in movement_types], priority,
                movement_statuses))

    total_altered_movements = []
    # no filter applied if movement_statuses is None
    filtered_movements = [m for m in selected_movements \
        if movement_statuses is None or \
           any([get_movement_status(process, m, movement_types, check_type_only=check_type_only).value == m_st.value
                for m_st in movement_statuses])]

    pbar = tqdm(total=len(filtered_movements), desc=f'{beam_id}-priority {priority}')
    for m in filtered_movements:
        # TODO move propagate here
        altered_movements = []
        m_id = process.movements.index(m)
        if verbose:
            LOGGER.debug('-'*10)
            LOGGER.debug('({})'.format(m_id))

        options['samplig_order_counter'] += 1
        archived_start_conf = process.get_movement_start_robot_config(m)
        archived_end_conf = process.get_movement_end_robot_config(m)

        for attempt in range(m_attempts):
            start_time = time.time()
            plan_success = compute_movement(client, robot, process, m, options, diagnosis)
            plan_time = elapsed_time(start_time)
            # * log planning profile
            if 'profiles' in options:
                if m_id not in options['profiles']:
                    options['profiles'][m_id] = defaultdict(list)
                options['profiles'][m_id]['movement_id'] = [m.movement_id]
                options['profiles'][m_id]['plan_time'].append(plan_time)
                options['profiles'][m_id]['plan_success'].append(plan_success)

            if plan_success:
                altered_movements.append(m)
                if viz_upon_found:
                    with WorldSaver():
                        visualize_movement_trajectory(client, robot, process, m, step_sim=True)
                break
            else:
                process.set_movement_start_robot_config(m, archived_start_conf)
                process.set_movement_end_robot_config(m, archived_end_conf)
        else:
            # TODO backtracking
            pbar.close()
            LOGGER.info('No plan found for {} after {} attempts. {}'.format(m.movement_id, m_attempts, m.short_summary))
            return False, []

        # * propagate to -1 movements
        altered_new_movements, impact_movements = propagate_states(process, altered_movements, options=options)
        altered_movements.extend(altered_new_movements)
        total_altered_movements.extend(altered_movements)
        # update progress bar
        pbar.update(1)

    pbar.close()
    return True, total_altered_movements

###########################################

def propagate_states(process, selected_movements, options=None):
    # type: (RobotClampAssemblyProcess, List[Movement], Dict) -> Tuple[List[Movement], List[Movement]]
    """Returns two lists of movements:
        - altered_movements: non-robotic movements with conf updated
        - impacted_movements: robotic movements with start / end conf updated
    """
    options = options or {}
    verbose = options.get('verbose', False)
    joint_compare_tolerances = options.get('joint_compare_tolerances', {})
    altered_movements = []
    impact_movements = []
    all_movements = process.movements
    for target_m in selected_movements:
        if not isinstance(target_m, RoboticMovement) or target_m.trajectory is None:
            # skip non-robotic movement or robotic movement without a trajectory planned
            continue
        m_id = all_movements.index(target_m)
        target_start_conf = process.get_movement_start_robot_config(target_m)
        target_end_conf = process.get_movement_end_robot_config(target_m)
        if verbose:
            LOGGER.debug('\tPropagate states for ({}) : {}'.format(colored(m_id, 'cyan'), target_m.short_summary))

        # * backward fill all adjacent (-1) movements
        back_id = m_id-1
        while back_id >= 0:
            back_m = all_movements[back_id]
            back_end_conf = process.get_movement_end_robot_config(back_m)
            if isinstance(back_m, RoboticMovement):
                if back_end_conf is not None:
                    # double check if configuration agrees
                    if not is_configurations_close(back_end_conf, target_start_conf, options=options):
                        LOGGER.error("Back propagation configruation disagree! {} /\ {}.".format(back_m, target_m))
                else:
                    # * write end conf to robot movement with no end conf
                    m_symbol = '$ Impacted'
                    m_color = 'yellow'
                    LOGGER.debug('\t{} (backward): ({}) {}'.format(m_symbol, colored(back_id, m_color), back_m.short_summary))
                    process.set_movement_end_robot_config(back_m, target_start_conf)
                    impact_movements.append(back_m)
                # * break if encountering a robot movement
                break
            else:
                # * propagate to Nonrobotic movement
                if verbose:
                    m_symbol = '- Altered'
                    m_color = 'green'
                    LOGGER.debug('\t{} (backward): ({}) {}'.format(m_symbol, colored(back_id, m_color), back_m.short_summary))
                process.set_movement_end_robot_config(back_m, target_start_conf)
                altered_movements.append(back_m)
                back_id -= 1

        # * forward fill all adjacent (+1) movements
        forward_id = m_id+1
        while forward_id < len(all_movements):
            forward_m = all_movements[forward_id]
            if isinstance(forward_m, RoboticMovement):
                if forward_m.trajectory is not None:
                    # double check if configuration agrees
                    if not is_configurations_close(target_end_conf, forward_m.trajectory.points[0], options=options):
                        LOGGER.error("Forward propagation configruation disagree! {} /\ {}.".format(forward_m, target_m))
                # * break if encountering a robot movement
                break
            else:
                # * propagate to Nonrobotic movement
                if verbose:
                    m_symbol = '- Altered'
                    m_color = 'green'
                    LOGGER.debug('\t{} (forward): ({}) {}'.format(m_symbol, colored(forward_id, m_color), forward_m.short_summary))
                # for non-robotic movement start_conf = end_conf, so only need to write end conf here
                process.set_movement_end_robot_config(forward_m, target_end_conf)
                altered_movements.append(forward_m)
                forward_id += 1
    # end loop selected_movements
    return altered_movements, impact_movements

