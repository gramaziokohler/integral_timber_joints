import os, time
import logging
import argparse
from termcolor import cprint, colored
from itertools import product, combinations
from tqdm import tqdm

from compas.robots import Joint
from compas.geometry import distance_point_point
from compas_fab.robots import Robot

import pybullet_planning as pp
from pybullet_planning import wait_if_gui, wait_for_user
from pybullet_planning import  link_from_name

from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from compas_fab_pychoreo.utils import is_configurations_close, is_frames_close, does_configurations_jump
from compas_fab_pychoreo.utils import LOGGER as PYCHOREO_LOGGER
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose

from integral_timber_joints.planning.parsing import parse_process, get_process_path
from integral_timber_joints.planning.robot_setup import load_RFL_world, GANTRY_ARM_GROUP, get_tolerances
from integral_timber_joints.planning.state import set_state
from integral_timber_joints.planning.utils import print_title, FRAME_TOL, color_from_success, beam_ids_from_argparse_seq_n, LOGGER

from integral_timber_joints.process import RoboticMovement, RobotClampAssemblyProcess

###########################################
def check_state_collisions_among_objects(client: PyChoreoClient, robot : Robot, process: RobotClampAssemblyProcess,
    scene_state: dict, options=None):
    options = options or {}
    debug = options.get('debug', False)

    # * update state
    set_state(client, robot, process, scene_state, options=options)

    # * check collisions among the list of attached objects and obstacles in the scene.
    # This includes collisions between:
    #     - each pair of (attached object, obstacle)
    #     - each pair of (attached object 1, attached object 2)
    in_collision = client.check_attachment_collisions(options)
    if scene_state[process.robot_config_key]:
        pychore_collision_fn = PyChoreoConfigurationCollisionChecker(client)
        in_collision |= pychore_collision_fn.check_collisions(robot, scene_state[process.robot_config_key], options=options)
    # if debug:
    if debug and in_collision:
        client._print_object_summary()
        wait_for_user('Collision checked: {}'.format(in_collision))
    return in_collision

def check_FK_consistency(client, robot, process, scene_state, options=None):
    debug = options.get('debug', False)

    robot_uid = client.get_robot_pybullet_uid(robot)
    flange_link_name = robot.get_end_effector_link_name(group=GANTRY_ARM_GROUP)

    robot_config = scene_state[process.robot_config_key]
    if robot_config is not None:
        client.set_robot_configuration(robot, robot_config)
        tool_link = link_from_name(robot_uid, flange_link_name)
        FK_tool_frame = frame_from_pose(pp.get_link_pose(robot_uid, tool_link), scale=1)
        if scene_state[('robot', 'f')] is not None:
            given_robot_frame = scene_state[('robot', 'f')].copy()
            given_robot_frame.point *= 1e-3 # in meter
            if not is_frames_close(given_robot_frame, FK_tool_frame, options=options):
                center_dist = distance_point_point(given_robot_frame.point, FK_tool_frame.point)
                msg = 'Robot FK tool pose and current frame diverge: {:.5f} (m)'.format(center_dist)
                if debug:
                    wait_for_user('FK consistency checked: {}'.format(msg))
                return (False, msg)
        return (True, 'Given robot\'s FK agrees with the given robot_frame.')
    else:
        return (True, 'No robot config specified.')

###########################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--design_dir', default='210605_ScrewdriverTestProcess',
                        help='problem json\'s containing folder\'s name.')
    parser.add_argument('--problem', default='nine_pieces_process.json', # pavilion_process.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='.', # pavilion.json
                        help='subdir of the process file, default to `.`. Popular use: `results`, `YJ_tmp`')
    #
    parser.add_argument('--plan_summary', action='store_true', help='Give a summary of currently found plans.')
    parser.add_argument('--verify_plan', action='store_true', help='Collision check found trajectories.')
    parser.add_argument('--traj_collision', action='store_true', help='Check trajectory collisions, not check states.')
    #
    parser.add_argument('--seq_n', nargs='+', type=int, help='Zero-based index according to the Beam sequence in process.assembly.sequence. If only provide one number, `--seq_n 1`, we will only plan for one beam. If provide two numbers, `--seq_n start_id end_id`, we will plan from #start_id UNTIL #end_id. If more numbers are provided. By default, all the beams will be checked.')
    parser.add_argument('--movement_id', default=None, type=str, help='Compute only for movement with a specific tag, e.g. `A54_M0`.')
    #
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('--reinit_tool', action='store_true', help='Regenerate tool URDFs.')
    parser.add_argument('--debug', action='store_true', help='debug mode.')
    args = parser.parse_args()

    logging_level = logging.DEBUG if args.debug else logging.INFO
    LOGGER.setLevel(logging_level)
    PYCHOREO_LOGGER.setLevel(logging.DEBUG)

    # log_folder = os.path.dirname(get_process_path(args.design_dir, args.problem, subdir='results'))
    # log_path = os.path.join(log_folder, 'check_state.log')
    # file_handler = logging.FileHandler(filename=log_path, mode='w')
    # formatter = logging.Formatter('%(asctime)s | %(name)s | %(levelname)s | %(message)s')
    # file_handler.setFormatter(formatter)
    # file_handler.setLevel(logging.WARN)
    # LOGGER.addHandler(file_handler)
    LOGGER.info("planning.run.py started with args: %s" % args)

    process = parse_process(args.design_dir, args.problem, subdir=args.problem_subdir)

    result_path = get_process_path(args.design_dir, args.problem, subdir=args.problem_subdir)
    # * print out a summary of all the movements to check which one hasn't been solved yet
    if args.plan_summary:
        ext_movement_path = os.path.dirname(result_path)
        LOGGER.info('Loading external movements from {}'.format(ext_movement_path))
        unfound_beams = []
        # process.load_external_movements(ext_movement_path)
        for i, beam_id in enumerate(process.assembly.sequence):
            LOGGER.debug('='*10)
            LOGGER.debug('({}) Beam #{}:'.format(i, beam_id))
            b_movements = process.get_movements_by_beam_id(beam_id)
            all_found = True
            for movement in b_movements:
                movement_path = os.path.join(ext_movement_path, movement.get_filepath())
                if not os.path.exists(movement_path):
                    LOGGER.warning('{} not found | {}'.format(movement.movement_id, movement.short_summary))
                    all_found = False
            if all_found:
                if len(b_movements) > 0:
                    LOGGER.debug('({}) Beam #{} all found!'.format(i, beam_id))
                    movement_path = os.path.join(ext_movement_path, b_movements[-1].get_filepath())
                    LOGGER.debug("\tcreated: \t%s" % time.ctime(os.path.getctime(movement_path)))
                    LOGGER.debug("\tlast modified: \t%s" % time.ctime(os.path.getmtime(movement_path)))
                else:
                    LOGGER.error('({}) Beam #{} empty movement list!'.format(i, beam_id))
            else:
                unfound_beams.append((i, beam_id))
        LOGGER.info('Unfound beams: {}'.format(unfound_beams))
        return

    # * if verifying existing solved movement, load from external movements
    if args.verify_plan:
        ext_movement_path = os.path.dirname(result_path)
        LOGGER.info('Loading external movements from {}'.format(ext_movement_path))
        process.load_external_movements(ext_movement_path)

    # * Connect to path planning backend and initialize robot parameters
    client, robot, _ = load_RFL_world(viewer=args.viewer)

    process.set_initial_state_robot_config(process.robot_initial_config)
    set_state(client, robot, process, process.initial_state, initialize=True,
        options={'debug' : False, 'reinit_tool' : args.reinit_tool})
    # * collision sanity check for the initial conf
    if process.robot_initial_config is not None:
        assert not client.check_collisions(robot, process.robot_initial_config, options={'diagnosis':True})

    options = {
        # * collision checking tolerance, in meter, peneration distance bigger than this number will be regarded as in collision
        'collision_distance_threshold' : 0.0012,
        'diagnosis' : True,
        'debug' : args.debug,
        # turn on verbose will make compas_fab_pychoreo to print the
        # joint compare details in DEBUG channel (in a separate logger)
        'verbose' : True,
    }
    # ! frame, conf compare, joint flip tolerances are set here
    options.update(get_tolerances(robot))

    all_movements = process.movements
    beam_ids = beam_ids_from_argparse_seq_n(process, args.seq_n, args.movement_id)
    chosen_movements = []
    for beam_id in beam_ids:
        actions = process.assembly.get_beam_attribute(beam_id, 'actions')
        for action in actions:
            for i, m in enumerate(action.movements):
                global_movement_id = all_movements.index(m)
                # * filter by movement id, support both integer-based index and string id
                if args.movement_id is not None and \
                    (m.movement_id != args.movement_id and args.movement_id != str(global_movement_id)):
                    continue
                # * skip non-robotic movement
                if not isinstance(m, RoboticMovement):
                    continue
                chosen_movements.append(m)

    movements_need_fix = []
    failure_reasons = []
    for m in tqdm(chosen_movements):
        start_state = process.get_movement_start_scene(m)
        end_state = process.get_movement_end_scene(m)
        start_conf = process.get_movement_start_robot_config(m)
        end_conf = process.get_movement_end_robot_config(m)

        in_collision = False
        joint_flip = False
        no_traj = False
        fk_disagree = False
        m_index = process.movements.index(m)
        LOGGER.debug('-'*10)
        print_title('(MovementIndex={}) {}'.format(m_index, m.short_summary), log_level='debug')

        # * Movement-specific ACM
        temp_name = '_tmp'
        for o1_name, o2_name in m.allowed_collision_matrix:
            o1_bodies = client._get_bodies('^{}$'.format(o1_name))
            o2_bodies = client._get_bodies('^{}$'.format(o2_name))
            for parent_body, child_body in product(o1_bodies, o2_bodies):
                client.extra_disabled_collision_links[temp_name].add(
                    ((parent_body, None), (child_body, None))
                )

        # LOGGER.debug('Start State:')
        # start_in_collision = check_state_collisions_among_objects(client, robot, process, start_state, options=options)
        # in_collision |= start_in_collision
        # if start_in_collision:
        #     LOGGER.error(m.short_summary)
        #     LOGGER.error('Start State in collision: {}.'.format(start_in_collision))
        # start_fk_agree, msg = check_FK_consistency(client, robot, process, start_state, options)
        # fk_disagree |= not start_fk_agree
        # if not start_fk_agree:
        #     LOGGER.error(m.short_summary)
        #     LOGGER.error('Start State FK consistency: {} | {}'.format(start_fk_agree, msg))
        # LOGGER.debug('#'*20)

        # * verify found trajectory's collisions and joint flips
        if args.verify_plan:
            pychore_collision_fn = PyChoreoConfigurationCollisionChecker(client)
            if m.trajectory:
                # print(client._print_object_summary())
                prev_conf = start_conf
                for conf_id, jpt in enumerate(list(m.trajectory.points) + [end_conf]):
                    if args.traj_collision:
                        # with WorldSaver():
                        # * traj-point collision checking
                        in_collision |= pychore_collision_fn.check_collisions(robot, jpt, options=options)
                        # * prev-conf~conf polyline collision checking
                        in_collision |= client.check_sweeping_collisions(robot, prev_conf, jpt, options=options)

                    if prev_conf and does_configurations_jump(jpt, prev_conf, options=options):
                        LOGGER.error(m.short_summary)
                        LOGGER.error('\t traj point #{}/{}'.format(conf_id, len(m.trajectory.points)))
                        joint_flip |= True
                    prev_conf = jpt
                LOGGER.debug(colored('Trajectory in trouble: in_collision {} | joint_flip {}'.format(in_collision, joint_flip), 'red' if in_collision or joint_flip else 'green'))
                if in_collision or joint_flip:
                    if args.debug:
                        wait_for_user()
            else:
                no_traj = True
                LOGGER.warning('No trajectory found!', 'red')
            LOGGER.debug('#'*20)

        # LOGGER.debug('End State:')
        # end_in_collision = check_state_collisions_among_objects(client, robot, process, end_state, options=options)
        # in_collision |= end_in_collision
        # if end_in_collision:
        #     LOGGER.error(m.short_summary)
        #     LOGGER.error('End State in collision: {}.'.format(end_in_collision))
        # end_fk_agree, msg = check_FK_consistency(client, robot, process, end_state, options)
        # fk_disagree |= not end_fk_agree
        # if not end_fk_agree:
        #     LOGGER.error(m.short_summary)
        #     LOGGER.error('End State FK consistency: {} | {}'.format(end_fk_agree, msg))
        # LOGGER.debug('#'*20)

        if temp_name in client.extra_disabled_collision_links:
            del client.extra_disabled_collision_links[temp_name]

        checks = [in_collision, joint_flip, no_traj, fk_disagree]
        if any(checks):
            movements_need_fix.append(m)
            report_msg = 'in_collision: {} | joint_flip: {} | no_traj: {} | fk_disagree: {}'.format(*[colored(c, color=color_from_success(not c)) for c in checks])
            failure_reasons.append(report_msg)

    LOGGER.debug('='*20)
    if len(movements_need_fix) == 0:
        LOGGER.info(colored('Congrats, check state done!', 'green'))
    else:
        LOGGER.info(colored('Movements that requires care and love:', 'yellow'))
        for fm, reason in zip(movements_need_fix, failure_reasons):
            global_movement_id = all_movements.index(fm)
            LOGGER.info('(MovementIndex={}) {}: {}'.format(global_movement_id, fm.short_summary, reason))
    LOGGER.debug('='*20)

    client.disconnect()

if __name__ == '__main__':
    main()
