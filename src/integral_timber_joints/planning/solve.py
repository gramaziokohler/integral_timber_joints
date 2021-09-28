import time
from enum import Enum, unique
from copy import deepcopy
from collections import defaultdict
from termcolor import cprint, colored

from pybullet_planning import set_random_seed, set_numpy_seed, elapsed_time, get_random_seed
from pybullet_planning import wait_if_gui, wait_for_user, LockRenderer, WorldSaver

from compas_fab.robots import Robot
from compas_fab_pychoreo.utils import compare_configurations

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement
from integral_timber_joints.process import RobotClampAssemblyProcess, Movement
from integral_timber_joints.planning.stream import compute_free_movement, compute_linear_movement
from integral_timber_joints.planning.state import set_state
from integral_timber_joints.planning.utils import notify, print_title
from integral_timber_joints.planning.robot_setup import GANTRY_ARM_GROUP, R11_JOINT_RESOLUTIONS, R11_JOINT_WEIGHTS, \
    MAIN_ROBOT_ID, get_gantry_robot_custom_limits
from integral_timber_joints.planning.parsing import save_process_and_movements
from integral_timber_joints.planning.visualization import visualize_movement_trajectory

try:
    # This is for type hint in VSCode/Pylance IDE while compatible with IronPython
    from typing import Dict, List, Optional, Tuple, Type, Any
    from compas_fab.backends import PyBulletClient
except:
    pass

GANTRY_ATTEMPTS = 100

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

###########################################

def compute_movement(client, robot, process, movement, options=None, diagnosis=False):
    # type: (PyBulletClient, Robot, RobotClampAssemblyProcess, Movement, Dict, Dict) -> bool
    if not isinstance(movement, RoboticMovement):
        return None
    options = options or {}
    # * low_res mode is used to quickly get a feeling of the planning problem
    low_res = options.get('low_res', False)
    verbose = options.get('verbose', True)
    if verbose:
        cprint(movement.short_summary, 'cyan')

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
    custom_limits = get_gantry_robot_custom_limits(MAIN_ROBOT_ID)
    options['custom_limits'] = custom_limits

    traj = None
    if isinstance(movement, RoboticLinearMovement):
        lm_options = options.copy()
        lm_options.update({
            'max_step' : 0.01, # interpolation step size, in meter
            'distance_threshold':0.0025, # collision checking tolerance, in meter
            'gantry_attempts' : GANTRY_ATTEMPTS,  # gantry attempt matters more
            'cartesian_attempts' : 1, # boosting up cartesian attempt here does not really help
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
    elif isinstance(movement, RoboticClampSyncLinearMovement) or \
         isinstance(movement, RobotScrewdriverSyncLinearMovement) or \
         'reorient' in movement.short_summary:
        lm_options = options.copy()
        # * interpolation step size, in meter
        lm_options.update({
            'max_step' : 0.02, # interpolation step size, in meter
            'distance_threshold':0.0025, # collision checking tolerance, in meter
            'gantry_attempts' : GANTRY_ATTEMPTS,  # gantry attempt matters more
            'cartesian_attempts' : 1, # boosting up cartesian attempt here does not really help, ladder graph only needs one attemp
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
        resolution_ratio = 10.0 if low_res else 1.0
        fm_options = options.copy()
        fm_options.update({
            'rrt_restarts' : 2, #20,
            'rrt_iterations' : 400, # ! value < 100 might not find solutions even if there is one
            'smooth_iterations': None, # ! Done: smoothing in postprocessing
            'joint_resolutions' : R11_JOINT_RESOLUTIONS*resolution_ratio,
            'joint_weights' : R11_JOINT_WEIGHTS,
            # -------------------
            'distance_threshold':0.0025, # collision checking tolerance, in meter
            'gantry_attempts' : GANTRY_ATTEMPTS,  # gantry attempt matters more
            'max_step' : 0.01, # interpolation step size, in meter, used in buffering motion
            'reachable_range' : (0.2, 3.0), # circle radius for sampling gantry base when computing IK
            })
        traj = compute_free_movement(client, robot, process, movement, fm_options, diagnosis)
    else:
        raise ValueError()

    if traj is not None:
        # update start/end states
        movement.trajectory = traj
        process.set_movement_start_robot_config(movement, traj.points[0])
        process.set_movement_end_robot_config(movement, traj.points[-1])
        # # ! update attached objecs' current frame at the end state
        # client.set_robot_configuration(robot, end_state[process.robot_config_key])
        # for object_id, object_state in end_state.items():
        #     if object_state.attached_to_robot:
        #         # convert back to millimeter
        #         object_state.current_frame = list(client.get_object_frame('^{}$'.format(object_id), scale=1e3).values())[0]
        return True
    else:
        if verbose:
            notify('Planning fails! Go back to the command line now!')
            # wait_for_user('Planning fails, press Enter to continue. Try exit and running again - may the Luck be with you next time :)')
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
    propagate_only = options.get('propagate_only', False)
    m_attempts = options.get('movement_planning_reattempts', 1)
    movement_id_filter = options.get('movement_id_filter', [])
    all_movements = process.get_movements_by_beam_id(beam_id)
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
            print('='*20)
            print_title('* compute movement ids: {}'.format(movement_id_filter))
    else:
        selected_movements = process.get_movements_by_planning_priority(beam_id, priority)
        if verbose:
            print('='*20)
            print_title('* compute {} (priority {}, status {})'.format([mt.__name__ for mt in movement_types], priority,
                movement_statuses))

    total_altered_movements = []
    for m in selected_movements:
        altered_movements = []
        if movement_statuses is None or \
            any([get_movement_status(process, m, movement_types, check_type_only=check_type_only).value - m_st.value == 0 for m_st in movement_statuses]):
            m_id = all_movements.index(m)
            if verbose:
                print('-'*10)
                print('({})'.format(m_id))

            if not propagate_only:
                options['samplig_order_counter'] += 1
                archived_start_conf = process.get_movement_start_robot_config(m)
                archived_end_conf = process.get_movement_end_robot_config(m)

                for attempt in range(m_attempts):
                    # if verbose:
                    #     print('Movement planning attempt {}'.format(attempt))
                    start_time = time.time()
                    plan_success = compute_movement(client, robot, process, m, options, diagnosis)
                    plan_time = elapsed_time(start_time)
                    if 'profiles' in options:
                        if m_id not in options['profiles']:
                            options['profiles'][m_id] = defaultdict(list)
                        options['profiles'][m_id]['movement_id'] = [m.movement_id]
                        options['profiles'][m_id]['plan_time'].append(plan_time)
                        options['profiles'][m_id]['plan_success'].append(plan_success)
                        options['profiles'][m_id]['sample_order'].append(options['samplig_order_counter'])

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
                    cprint('No plan found for {} after {} attempts! {}'.format(m.movement_id, m_attempts, m.short_summary), 'red')
                    # continue
                    # break
                    return False, []
            else:
                traj = m.trajectory
                process.set_movement_start_robot_config(m, traj.points[0])
                process.set_movement_end_robot_config(m, traj.points[-1])
                altered_movements.append(m)
        else:
            continue

        # * propagate to -1 movements
        altered_new_movements, impact_movements = propagate_states(process, altered_movements, all_movements, options=options)
        altered_movements.extend(altered_new_movements)
        total_altered_movements.extend(altered_movements)
    # if verbose:
    #     print('\n\n')
    #     process.get_movement_summary_by_beam_id(beam_id)
    return True, total_altered_movements

###########################################

def propagate_states(process, selected_movements, all_movements, options=None):
    # type: (RobotClampAssemblyProcess, List[Movement], List[Movement], Dict) -> Tuple[List[Movement], List[Movement]]
    options = options or {}
    verbose = options.get('verbose', False)
    jump_threshold = options.get('jump_threshold', {})
    joint_names = options.get('joint_names', [])
    altered_movements = []
    # movements that needs to be recomputed
    impact_movements = []
    for target_m in selected_movements:
        if not isinstance(target_m, RoboticMovement) or target_m.trajectory is None:
            continue
        m_id = all_movements.index(target_m)
        target_start_conf = process.get_movement_start_robot_config(target_m)
        target_end_conf = process.get_movement_end_robot_config(target_m)
        if verbose:
            print('~'*5)
            print('\tPropagate states for ({}) : {}'.format(colored(m_id, 'cyan'), target_m.short_summary))
        # * backward fill all adjacent (-1) movements
        back_id = m_id-1
        while back_id >= 0:
            back_m = all_movements[back_id]
            if isinstance(back_m, RoboticMovement) and back_m.trajectory:
                back_end_conf = back_m.trajectory.points[-1]
            else:
                back_end_conf = process.get_movement_end_robot_config(back_m)
            # print('----')
            # print('Back: {} | back_end_conf {}'.format(back_m.short_summary, back_end_conf))
            # # print(compare_configurations(back_end_conf, target_start_conf, jump_threshold, fallback_tol=1e-3, verbose=verbose))
            # print(get_movement_status(process, back_m, [RoboticMovement]))

            # if back_m.planning_priority == -1:
            if not isinstance(back_m, RoboticMovement):
                if verbose:
                    if back_end_conf and compare_configurations(back_end_conf, target_start_conf, jump_threshold, verbose=False):
                        cprint('Backward Prop: Start conf not coincided', 'red')
                            # notify('Warning! Go back to the command line now!')
                            # wait_for_user()
                    if verbose:
                        print('\t- Altered (backward): ({}) {}'.format(colored(back_id, 'green'), back_m.short_summary))
                process.set_movement_end_robot_config(back_m, target_start_conf)
                altered_movements.append(back_m)
                back_id -= 1
            # get_movement_status(process, back_m, [RoboticMovement]) in [MovementStatus.has_traj, MovementStatus.both_done] and \
            elif back_m.trajectory is None or back_end_conf is None or \
                compare_configurations(back_end_conf, target_start_conf, jump_threshold, verbose=False):
                process.set_movement_end_robot_config(back_m, target_start_conf)
                impact_movements.append(back_m)
                if verbose:
                    print('\t$ Impacted (backward): ({}) {}'.format(colored(back_id, 'yellow'), back_m.short_summary))
                break
            else:
                break
        # * forward fill all adjacent (-1) movements
        forward_id = m_id+1
        while forward_id < len(all_movements):
            forward_m = all_movements[forward_id]
            if isinstance(forward_m, RoboticMovement) and forward_m.trajectory:
                forward_start_conf = forward_m.trajectory.points[0]
            else:
                forward_start_conf = process.get_movement_start_robot_config(forward_m)
            # if all_movements[forward_id].planning_priority == -1:
            if not isinstance(forward_m, RoboticMovement):
                if verbose:
                    if forward_start_conf and compare_configurations(forward_start_conf, target_end_conf, jump_threshold,
                        verbose=False):
                        cprint('Forward Prop: End conf not coincided', 'red')
                        # notify('Warning! Go back to the command line now!')
                        # wait_for_user()
                    print('\t- Altered (forward): ({}) {}'.format(colored(forward_id, 'green'), forward_m.short_summary))
                process.set_movement_end_robot_config(forward_m, target_end_conf)
                altered_movements.append(forward_m)
                forward_id += 1
            # elif get_movement_status(process, forward_m, [RoboticMovement]) in [MovementStatus.has_traj, MovementStatus.both_done] and \
            #     compare_configurations(forward_start_conf, target_end_conf, jump_threshold, fallback_tol=1e-3, verbose=False):
            elif forward_m.trajectory is None or forward_start_conf is None or \
                compare_configurations(forward_start_conf, target_end_conf, jump_threshold, verbose=False):
                impact_movements.append(forward_m)
                if verbose:
                    print('\t$ Impacted (forward): ({}) {}'.format(colored(forward_id, 'yellow'), forward_m.short_summary))
                break
            else:
                break
    # end loop selected_movements
    return altered_movements, impact_movements

