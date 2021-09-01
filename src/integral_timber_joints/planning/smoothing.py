import argparse


#################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--design_dir', default='210605_ScrewdriverTestProcess',
                        help='problem json\'s containing folder\'s name.')
    parser.add_argument('--problem', default='pavilion_process.json', # twelve_pieces_process.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='.',
                        help='subdir of the process file, default to `.`. Popular use: `results`')
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    #
    parser.add_argument('--seq_i', default=0, type=int, help='individual step to plan.')
    parser.add_argument('--batch_run', action='store_true', help='Batch run. Will turn `--seq_i` as run from.')
    #
    parser.add_argument('--id_only', default=None, type=str, help='Compute only for movement with a specific tag, e.g. `A54_M0`.')
    #
    parser.add_argument('--write', action='store_true', help='Write output json.')
    parser.add_argument('--load_external_movements', action='store_true', help='Load externally saved movements into the parsed process, default to False.')
    #
    parser.add_argument('--debug', action='store_true', help='Debug mode')
    parser.add_argument('--step_sim', action='store_true', help='Pause after each conf viz.')
    parser.add_argument('--verbose', action='store_false', help='Print out verbose. Defaults to True.')
    parser.add_argument('--diagnosis', action='store_true', help='Diagnosis mode')

    args = parser.parse_args()
    print('Arguments:', args)
    print('='*10)
    if args.id_only is not None:
        args.solve_mode = 'id_only'

    # * Connect to path planning backend and initialize robot parameters
    client, robot, _ = load_RFL_world(viewer=args.viewer or args.diagnosis or args.watch or args.step_sim)

    #########
    # * Load process and recompute actions and states
    process = parse_process(args.problem, subdir=args.problem_subdir)
    result_path = get_process_path(args.design_dir, args.problem, subdir='results')

    # Double check entire solution is valid
    for beam_id in process.assembly.sequence:
        if not process.dependency.beam_all_valid(beam_id):
            process.dependency.compute_all(beam_id)
            assert process.dependency.beam_all_valid(beam_id)

    # force load external if only planning for the free motions
    args.load_external_movements = args.load_external_movements or args.solve_mode == 'free_motion_only' or args.solve_mode == 'id_only'
    if args.load_external_movements:
        ext_movement_path = os.path.dirname(result_path)
        cprint('Loading external movements from {}'.format(ext_movement_path), 'cyan')
        process.load_external_movements(ext_movement_path)

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
        'distance_threshold' : 0.0012, # in meter
        'frame_jump_tolerance' : 0.0012, # in meter
        'verbose' : args.verbose,
        'problem_name' : args.problem,
        'use_stored_seed' : args.use_stored_seed,
        'jump_threshold' : joint_jump_threshold,
        'max_distance' : args.max_distance,
        'propagate_only' : args.solve_mode == 'propagate_only',
        # until Trajectory json is fixed...
        'joint_names' : robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP),
    }

    set_initial_state(client, robot, process, disable_env=args.disable_env, reinit_tool=args.reinit_tool)

    full_seq_len = len(process.assembly.sequence)
    assert args.seq_i < full_seq_len and args.seq_i >= 0
    if args.batch_run:
        beam_ids = [process.assembly.sequence[si] for si in range(args.seq_i, full_seq_len)]
    elif args.solve_mode == 'id_only':
        beam_ids = [process.get_beam_id_from_movement_id(args.id_only)]
    else:
        # only one
        beam_ids = [process.assembly.sequence[args.seq_i]]

    for beam_id in beam_ids:
        print('-'*20)
        s_i = process.assembly.sequence.index(beam_id)
        print('({}) Beam#{}'.format(s_i, beam_id))

        # if compute_movements_for_beam_id(client, robot, process, beam_id, args, options=options):
        #     cprint('Beam #{} plan found after {} iters!'.format(beam_id, i+1), 'cyan')
        #     break

    client.disconnect()

if __name__ == '__main__':
    main()

