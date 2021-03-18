import argparse
from termcolor import cprint
from itertools import product, combinations

from compas_fab.robots import Robot
from pybullet_planning import wait_if_gui, wait_for_user, has_gui
from pybullet_planning import get_distance, draw_collision_diagnosis, expand_links, get_all_links, \
    link_from_name, pairwise_link_collision_info, get_name, get_link_name
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.utils import values_as_list

from integral_timber_joints.planning.parsing import parse_process
from integral_timber_joints.planning.robot_setup import load_RFL_world, to_rlf_robot_full_conf, \
    R11_INTER_CONF_VALS, R12_INTER_CONF_VALS
from integral_timber_joints.planning.state import set_state

from integral_timber_joints.process import RoboticMovement, RobotClampAssemblyProcess

###########################################
def check_state_collisions_among_objects(client: PyChoreoClient, robot : Robot, process: RobotClampAssemblyProcess,
    state_from_object: dict, options=None):
    options = options or {}
    # collision checking tolerance, in meter, peneration distance bigger than this number will be regarded as in collision
    distance_threshold = options.get('distance_threshold', 0.0)
    # buffering distance, If the distance between objects exceeds this maximum distance, no points may be returned.
    max_distance = options.get('max_distance', 0.0)
    built_beam_ids = options.get('built_beam_ids', [])
    option_disabled_link_names = options.get('extra_disabled_collisions', set())
    movement_info = options.get('movement_info', '')

    # * update state
    set_state(client, robot, process, state_from_object, options=options)

    # * ignored ACM
    extra_disabled_collision_names = values_as_list(client.extra_disabled_collision_links)
    extra_disabled_collisions = set()
    for bpair in list(extra_disabled_collision_names) + list(option_disabled_link_names):
        b1, b1link_name = bpair[0]
        b2, b2link_name = bpair[1]
        b1_links = get_all_links(b1) if b1link_name is None else [link_from_name(b1, b1link_name)]
        b2_links = get_all_links(b2) if b2link_name is None else [link_from_name(b2, b2link_name)]
        for b1_link, b2_link in product(b1_links, b2_links):
            extra_disabled_collisions.add(
                ((b1, b1_link), (b2, b2_link))
                )

    # * construct (body, body) pairs
    object_ids = list(state_from_object.keys()) + list(process.environment_models.keys())
    check_body_pairs = []
    for object1, object2 in combinations(object_ids, 2):
        # * ignore robot
        # * don't check among environment objects
        if (object1.startswith('r') or object2.startswith('r')) or \
           (object1.startswith('e') and object2.startswith('e')) or \
           (object1 in built_beam_ids and object2 in built_beam_ids) or \
           (object1 not in built_beam_ids and object2.startswith('e')) or \
           (object2 not in built_beam_ids and object1.startswith('e')):
            continue
        object1_bodies = client._get_bodies('^{}$'.format(object1))
        object2_bodies = client._get_bodies('^{}$'.format(object2))
        for b1, b2 in product(object1_bodies, object2_bodies):
            check_body_pairs.append((b1,b2))

    # * construct ((body, link), (body, link)) pairs
    check_body_link_pairs = []
    for body1, body2 in check_body_pairs:
        body1, links1 = expand_links(body1)
        body2, links2 = expand_links(body2)
        if body1 == body2:
            continue
        for bb_links in product(links1, links2):
            bbll_pair = ((body1, bb_links[0]), (body2, bb_links[1]))
            if bbll_pair not in extra_disabled_collisions and bbll_pair[::-1] not in extra_disabled_collisions:
                check_body_link_pairs.append(bbll_pair)

    # * perform checking
    name_from_body_id = client._name_from_body_id
    for (body1, link1), (body2, link2) in check_body_link_pairs:
        # print((body1, link1), (body2, link2))
        collision_msgs = pairwise_link_collision_info(body1, link1, body2, link2, max_distance)
        in_collision = False
        for u_cr in collision_msgs:
            if get_distance(u_cr[5], u_cr[6]) > distance_threshold:
                in_collision = True
                break
        if in_collision:
            cprint('~'*10, 'yellow')
            print('Extra disabled collision links:')
            for (b1,l1), (b2,l2) in list(extra_disabled_collisions):
                b1_name = name_from_body_id[b1] if b1 in name_from_body_id else get_name(b1)
                b2_name = name_from_body_id[b2] if b2 in name_from_body_id else get_name(b2)
                print('\t({}-{}), ({}-{})'.format(b1_name,get_link_name(b1,l1),b2_name,get_link_name(b2, l2)))
            # print('~~~')
            cprint('{}'.format(movement_info), 'blue')
            # client._print_object_summary()
            draw_collision_diagnosis(collision_msgs, body_name_from_id=client._name_from_body_id, viz_all=False)
            if not has_gui():
                wait_for_user()
            print('~'*5)


###########################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='twelve_pieces_process.json', # pavilion.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='.', # pavilion.json
                        help='subdir of the process file, default to `.`. Popular use: `YJ_tmp`, `<time stamp>`')
    parser.add_argument('--start_from_id', type=int, default=0, help='sequence index to start from, default to 0 (i.e. check all).')
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('--write', action='store_true', help='Write output json.')
    parser.add_argument('--reinit_tool', action='store_true', help='Regenerate tool URDFs.')
    args = parser.parse_args()
    print('Arguments:', args)
    print('='*10)

    process = parse_process(args.problem, subdir=args.problem_subdir)
    assert args.start_from_id >= 0 and args.start_from_id < len(process.assembly.sequence), 'start_from_id out of range!'

    # * Connect to path planning backend and initialize robot parameters
    client, robot, _ = load_RFL_world(viewer=args.viewer)

    # set all other unused robot
    full_start_conf = to_rlf_robot_full_conf(R11_INTER_CONF_VALS, R12_INTER_CONF_VALS)
    client.set_robot_configuration(robot, full_start_conf)

    process.initial_state['robot'].kinematic_config = process.robot_initial_config
    set_state(client, robot, process, process.initial_state, initialize=True,
        options={'debug' : False, 'reinit_tool' : args.reinit_tool})
    # * collision sanity check
    assert not client.check_collisions(robot, full_start_conf, options={'diagnosis':True})

    for seq_i in range(args.start_from_id, len(process.assembly.sequence)):
        beam_id = process.assembly.sequence[seq_i]
        built_beam_ids = [process.assembly.sequence[i] for i in range(0,seq_i)]
        print('='*20)
        cprint('(Seq#{}) Beam {}'.format(seq_i, beam_id), 'yellow')
        all_movements = process.get_movements_by_beam_id(beam_id)
        for i, m in enumerate(all_movements):
            if isinstance(m, RoboticMovement):
                start_state = process.get_movement_start_state(m)
                end_state = process.get_movement_end_state(m)
                extra_disabled_collision_links = set()
                for o1_name, o2_name in m.allowed_collision_matrix:
                    o1_bodies = client._get_bodies('^{}$'.format(o1_name))
                    o2_bodies = client._get_bodies('^{}$'.format(o2_name))
                    for parent_body, child_body in product(o1_bodies, o2_bodies):
                        extra_disabled_collision_links.add(
                            ((parent_body, None), (child_body, None))
                            )

                print('-'*10)
                cprint('(Seq#{}-#{}) {}'.format(seq_i, i, m.short_summary), 'cyan')
                check_state_collisions_among_objects(client, robot, process, start_state,
                    options={
                        'extra_disabled_collisions' : extra_disabled_collision_links,
                        'built_beam_ids' : built_beam_ids,
                        'movement_info' : m.short_summary,
                        })
                cprint('Start State Done.')
                print('####')
                check_state_collisions_among_objects(client, robot, process, end_state,
                    options={
                        'extra_disabled_collisions' : extra_disabled_collision_links,
                        'built_beam_ids' : built_beam_ids,
                        'movement_info' : m.short_summary,
                        })
                cprint('End State Done.')
                print('####')

    wait_for_user('Enter to exit.')

    client.disconnect()

if __name__ == '__main__':
    main()
