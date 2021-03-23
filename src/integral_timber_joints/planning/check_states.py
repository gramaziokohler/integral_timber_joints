import os, time
import argparse
from termcolor import cprint
from itertools import product, combinations

from compas_fab.robots import Robot
from pybullet_planning import wait_if_gui, wait_for_user, has_gui
from pybullet_planning import get_distance, draw_collision_diagnosis, expand_links, get_all_links, \
    link_from_name, pairwise_link_collision_info, get_name, get_link_name
from compas_fab_pychoreo.client import PyChoreoClient

from integral_timber_joints.planning.parsing import parse_process, get_process_path
from integral_timber_joints.planning.robot_setup import load_RFL_world, to_rlf_robot_full_conf, \
    R11_INTER_CONF_VALS, R12_INTER_CONF_VALS
from integral_timber_joints.planning.state import set_state
from integral_timber_joints.planning.utils import print_title

from integral_timber_joints.process import RoboticMovement, RobotClampAssemblyProcess

###########################################
def check_state_collisions_among_objects(client: PyChoreoClient, robot : Robot, process: RobotClampAssemblyProcess,
    state_from_object: dict, options=None):
    options = options or {}
    debug = options.get('debug', False)

    # * update state
    set_state(client, robot, process, state_from_object, options=options)
    if debug:
        client._print_object_summary()

    in_collision = client.check_attachment_collisions(options)
    if debug:
        wait_for_user()
    return in_collision

###########################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='twelve_pieces_process.json', # pavilion_process.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='.', # pavilion.json
                        help='subdir of the process file, default to `.`. Popular use: `YJ_tmp`, `<time stamp>`')
    parser.add_argument('--plan_summary', action='store_true', help='Give a summary of currently found plans.')
    parser.add_argument('--start_from_id', type=int, default=0, help='sequence index to start from, default to 0 (i.e. check all).')
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('--reinit_tool', action='store_true', help='Regenerate tool URDFs.')
    parser.add_argument('--debug', action='store_true', help='debug mode.')
    args = parser.parse_args()
    print('Arguments:', args)
    print('='*10)

    process = parse_process(args.problem, subdir=args.problem_subdir)

    result_path = get_process_path(args.problem, subdir='results')
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

    assert args.start_from_id >= 0 and args.start_from_id < len(process.assembly.sequence), 'start_from_id out of range!'

    # * Connect to path planning backend and initialize robot parameters
    client, robot, _ = load_RFL_world(viewer=args.viewer)

    # set all other unused robot
    full_start_conf = to_rlf_robot_full_conf(R11_INTER_CONF_VALS, R12_INTER_CONF_VALS)
    client.set_robot_configuration(robot, full_start_conf)

    process.initial_state['robot'].kinematic_config = process.robot_initial_config
    set_state(client, robot, process, process.initial_state, initialize=True,
        options={'debug' : False, 'reinit_tool' : args.reinit_tool})
    # * collision sanity check for the initial conf
    assert not client.check_collisions(robot, full_start_conf, options={'diagnosis':True})

    options = {
        # * collision checking tolerance, in meter, peneration distance bigger than this number will be regarded as in collision
        'distance_threshold' : 0.0012,
        # * buffering distance, If the distance between objects exceeds this maximum distance, no points may be returned.
        'max_distance' : 0.0,
        # * If target_configuration is different from the target_frame by more that this amount at flange center, a warning will be raised.
        'frame_jump_tolerance' : 0.0012,
        'diagnosis' : True,
        'debug' : args.debug,
    }

    for seq_i in range(args.start_from_id, len(process.assembly.sequence)):
        beam_id = process.assembly.sequence[seq_i]
        print('='*20)
        cprint('(Seq#{}) Beam {}'.format(seq_i, beam_id), 'yellow')
        all_movements = process.get_movements_by_beam_id(beam_id)
        for i, m in enumerate(all_movements):
            if isinstance(m, RoboticMovement):
                start_state = process.get_movement_start_state(m)
                end_state = process.get_movement_end_state(m)

                # movement-specific ACM
                temp_name = '_tmp'
                for o1_name, o2_name in m.allowed_collision_matrix:
                    o1_bodies = client._get_bodies('^{}$'.format(o1_name))
                    o2_bodies = client._get_bodies('^{}$'.format(o2_name))
                    for parent_body, child_body in product(o1_bodies, o2_bodies):
                        client.extra_disabled_collision_links[temp_name].add(
                            ((parent_body, None), (child_body, None))
                        )

                print('-'*10)
                print_title('(Seq#{}-#{}) {}'.format(seq_i, i, m.short_summary))

                cprint('Start State:', 'blue')
                in_collision = check_state_collisions_among_objects(client, robot, process, start_state, options=options)
                cprint('Start State in collision: {}.'.format(in_collision), 'red' if in_collision else 'green')
                print('#'*20)

                cprint('End State:', 'blue')
                in_collision = check_state_collisions_among_objects(client, robot, process, end_state, options=options)
                cprint('End State in collision: {}.'.format(in_collision), 'red' if in_collision else 'green')
                print('#'*20)

                if temp_name in client.extra_disabled_collision_links:
                    del client.extra_disabled_collision_links[temp_name]

    wait_for_user('Congrats, check state done! Enter to exit.')

    client.disconnect()

if __name__ == '__main__':
    main()
