import time
from enum import Enum, unique
from collections import defaultdict
from termcolor import colored
from tqdm import tqdm

from pybullet_planning import set_random_seed, set_numpy_seed, elapsed_time, get_random_seed
from pybullet_planning import wait_if_gui, wait_for_user, WorldSaver

from compas_fab.robots import Robot
from compas_fab_pychoreo.utils import is_configurations_close

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement
from integral_timber_joints.process import RobotClampAssemblyProcess, Movement
from integral_timber_joints.planning.stream import compute_free_movement, compute_linear_movement, compute_free_movement_with_waypoints
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
    has_traj = 2
    one_sided = 3
    both_done = 4
    neither_done = 5


def get_movement_status(process, m):
    # type: (RobotClampAssemblyProcess, Movement) -> MovementStatus
    """get the movement's current status, see the `MovementStatus` class

    Parameters
    ----------
    process : [type]
    m : Movement

    Returns
    -------
    MovementStatus.xxx
    """
    # if not isinstance(m, RoboticMovement):
    has_start_conf = process.movement_has_start_robot_config(m)
    has_end_conf = process.movement_has_end_robot_config(m)
    has_traj = m.trajectory is not None

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
    verbose = options.get('verbose', True)
    force_linear_to_free_movement = options.get('force_linear_to_free_movement', False)
    if verbose:
        LOGGER.debug(colored(movement.short_summary, 'cyan'))

    use_stored_seed = options.get('use_stored_seed', False)
    # * set seed stored in the movement
    seed = None
    if use_stored_seed:
        seed = movement.seed
        if seed is None:
            LOGGER.warning(f'No meaningful seed saved in movement {movement.movement_id}.')
        LOGGER.debug(f'using seed {seed}')
    if seed is None:
        seed = hash(time.time())
        movement.seed = seed
    set_random_seed(seed)
    set_numpy_seed(seed)

    if force_linear_to_free_movement:
        orig_movement = movement
        movement = RoboticFreeMovement.from_data(orig_movement.data)
        for action in process.actions:
            for i in range(len(action.movements)):
                if action.movements[i] == orig_movement:
                    action.movements[i] = movement

    LOGGER.debug("Number of intermediate_planning_waypoint: %s" % len(movement.intermediate_planning_waypoint))
    LOGGER.debug("intermediate_planning_waypoint: %s" % movement.intermediate_planning_waypoint)

    # * custom limits
    traj = None
    if isinstance(movement, RoboticLinearMovement):
        lm_options = options.copy()
        lm_options.update({
            'max_step': movement.planning_linear_step_distance_m or 0.01,  # interpolation step size, in meter
            'cartesian_attempts': 1,  # boosting up cartesian attempt here does not really help
            # -------------------
            'planner_id': 'IterativeIK',
            'cartesian_move_group': GANTRY_ARM_GROUP,
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
            'max_step': movement.planning_linear_step_distance_m or 0.02,  # interpolation step size, in meter
            'cartesian_attempts': 1,  # boosting up cartesian attempt here does not really help, ladder graph only needs one attemp
            # -------------------
            'planner_id': 'IterativeIK',
            'cartesian_move_group': GANTRY_ARM_GROUP,
            # -------------------
            # 'planner_id' : 'LadderGraph',
            # 'ik_function' : _get_sample_bare_arm_ik_fn(client, robot),
            # 'cartesian_move_group' : BARE_ARM_GROUP,
        })
        traj = compute_linear_movement(client, robot, process, movement, lm_options, diagnosis)

    elif isinstance(movement, RoboticFreeMovement) and len(movement.intermediate_planning_waypoint) > 0:
        fm_options = options.copy()
        fm_options.update({
            'rrt_restarts': 2,  # 20,
            'smooth_iterations': None,  # ! smoothing will be done in postprocessing
            # -------------------
            'max_step': 0.005,  # interpolation step size, in meter, used in buffering motion
        })
        traj = compute_free_movement_with_waypoints(client, robot, process, movement, fm_options, diagnosis)

    elif isinstance(movement, RoboticFreeMovement):
        fm_options = options.copy()
        fm_options.update({
            'rrt_restarts': 2,  # 20,
            'smooth_iterations': None,  # ! smoothing will be done in postprocessing
            # -------------------
            'max_step': 0.005,  # interpolation step size, in meter, used in buffering motion
        })
        traj = compute_free_movement(client, robot, process, movement, fm_options, diagnosis)
    else:
        LOGGER.critical("Unrecognized movement type {}".format(movement))
        return None

    if traj is not None:
        if force_linear_to_free_movement:
            orig_movement.trajectory = movement.trajectory
            for action in process.actions:
                for i in range(len(action.movements)):
                    if action.movements[i] == movement:
                        action.movements[i] = orig_movement
            movement = orig_movement

        prev_robot_conf = process.get_movement_start_robot_config(movement)
        if prev_robot_conf is not None and not is_configurations_close(prev_robot_conf, traj.points[0], options=options):
            LOGGER.error('Planned trajectory\'s first conf does not agree with the previous movement\'s end conf! Planning fails.')
            return False
        process.set_movement_trajectory(movement, traj)
        return True
    else:
        notify('Planning fails! Go back to the command line now!')
        return False


def compute_selected_movements(client, robot,
                               process: RobotClampAssemblyProcess,
                               beam_id: str,
                               planning_priority_filter: List[int] = None,
                               movement_type_filter: List[Type] = None,
                               movement_status_filter: List[MovementStatus] = None,
                               has_no_trajectory : bool = True,
                               options:  Dict = None,
                               viz_upon_found: bool = False,
                               diagnosis: bool = False,
                               ) -> Tuple[bool, List[Movement]]:
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
    movement_id_filter = options.get('movement_id_filter', None)
    if movement_id_filter == []:
        movement_id_filter = None

    computed_movements = []

    while (True):
        if beam_id is None:
            selected_movements = process.movements
        else:
            selected_movements = process.get_movements_by_beam_id(beam_id)  # type: Movement

        selected_movements = [m for m in selected_movements if isinstance(m, RoboticMovement)]

        # * Filter by already computed movement
        selected_movements = [m for m in selected_movements if m not in computed_movements]

        # * Filter by no trajectory
        if has_no_trajectory:
            selected_movements = [m for m in selected_movements if m.trajectory is None]

        # * Filter by movement_id
        if movement_id_filter is not None:
            # * Unify movement_id filter to be actual movement_id, not index.
            for i in range(len(movement_id_filter)):
                if movement_id_filter[i].isdigit():
                    movement_id_filter[i] = process.movements[int(movement_id_filter[i])].movement_id
            # * Filter
            selected_movements = [m for m in selected_movements if m.movement_id in movement_id_filter]

        # * Filter by planning priority
        if planning_priority_filter is not None:
            selected_movements = [m for m in selected_movements if m.planning_priority in planning_priority_filter]

        # * Filter by movement type
        if movement_type_filter is not None:
            selected_movements = [m for m in selected_movements if type(m) in movement_type_filter]

        # * Filter by movement_status
        if movement_status_filter is not None:
            selected_movements = [m for m in selected_movements if get_movement_status(process, m) in movement_status_filter]

        if len(selected_movements) == 0 :
            break
        # * Plan all selected movements
        for m in selected_movements:
            m_id = process.movements.index(m)
            start_time = time.time()
            plan_success = compute_movement(client, robot, process, m, options, diagnosis)
            plan_time = elapsed_time(start_time)
            if 'profiles' in options:
                # * log planning profile
                if m_id not in options['profiles']:
                    options['profiles'][m_id] = defaultdict(list)
                options['profiles'][m_id]['movement_id'] = [m.movement_id]
                options['profiles'][m_id]['plan_time'].append(plan_time)
                options['profiles'][m_id]['plan_success'].append(plan_success)

            if not plan_success:
                LOGGER.info('No plan found for {} | {}'.format(m.movement_id, m.short_summary))
                return False, []

            if viz_upon_found:
                with WorldSaver():
                    visualize_movement_trajectory(client, robot, process, m, step_sim=True)
            computed_movements.append(m)

    # Debug message
    debug_message = ""
    if movement_id_filter is not None:
        debug_message += "movement_id_filter = {}, ".format(movement_id_filter)
    if planning_priority_filter is not None:
        debug_message += "planning_priority_filter = {}, ".format(planning_priority_filter)
    if movement_type_filter is not None:
        debug_message += "movement_type_filter = {}, ".format([t.__name__ for t in movement_type_filter])
    if movement_status_filter is not None:
        debug_message += "movement_status_filter = {}, ".format(movement_status_filter)
    debug_message += "has_no_trajectory = {}, ".format(has_no_trajectory)
    computed_movements_id = [m.movement_id for m in computed_movements]
    LOGGER.debug("compute_selected_movements({}) solved {} Movements: {}".format(debug_message, len(computed_movements), computed_movements_id))

    return True, computed_movements
