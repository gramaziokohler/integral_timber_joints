######################################

def _archived_get_action_ik_fn(client, process, robot, options=None):
    options = options or {}
    debug = options.get('debug', False)
    verbose = options.get('verbose', False)
    reachable_range = options.get('reachable_range', (0.2, 2.5))
    ik_gantry_attempts = options.get('ik_gantry_attempts', 10)

    def ik_fn(object_name, object_pose, grasp, fluents=[]):

        # * set state
        # pp.wait_if_gui('IK: Before assign fluent state')
        assign_fluent_state(client, robot, process, fluents)
        # print(client._print_object_summary())
        # pp.wait_if_gui('IK: After assign fluent state')

        state = SceneState(process)
        object_frame = object_pose.copy()
        object_frame.point *= 1e-3
        state[(object_name, 'f')] = object_frame
        state[(object_name, 'g')] = grasp
        world_from_object = pose_from_frame(object_frame)
        robot_flange_from_attached_obj = pose_from_transformation(grasp, scale=1e-3)
        flange_frame = frame_from_pose(pp.multiply(world_from_object, pp.invert(robot_flange_from_attached_obj)))

        # TODO use trac-ik here
        sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
        gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
        gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)

        # print(flange_frame)
        # pp.draw_pose(pose_from_frame(flange_frame, scale=1))

        # * ACM setup
        temp_name = '_tmp'
        allowed_collision_matrix = [('tool_changer', object_name)]
        if object_name == 'g1':
            # TODO fix this hand-coded index...
            allowed_collision_matrix.extend([
                (object_name, 'e47'),
                (object_name, 'e48'),
                (object_name, 'e53'),
                (object_name, 'e54'),
                ])

        for o1_name, o2_name in allowed_collision_matrix:
            o1_bodies = client._get_bodies('^{}$'.format(o1_name))
            o2_bodies = client._get_bodies('^{}$'.format(o2_name))
            for parent_body, child_body in product(o1_bodies, o2_bodies):
                client.extra_disabled_collision_links[temp_name].add(
                    ((parent_body, None), (child_body, None))
                )

        start_time = time.time()
        gantry_base_gen_fn = gantry_base_generator(client, robot, flange_frame, reachable_range=reachable_range, scale=1.0, options=options)
        with pp.LockRenderer(not debug):
            # * sample from a ball near the pose
            for attempt, base_conf in zip(range(ik_gantry_attempts), gantry_base_gen_fn):
                # * pybullet gradient-based IK
                conf = client.inverse_kinematics(robot, flange_frame, group=GANTRY_ARM_GROUP,
                                                 options=options)
                if not conf:
                    # * bare-arm IKfast sampler
                    arm_conf_vals = sample_ik_fn(pose_from_frame(flange_frame, scale=1))
                    # in total, 8 ik solutions for the 6-axis arm
                    for ik_iter, arm_conf_val in enumerate(arm_conf_vals):
                        if arm_conf_val is None:
                            continue
                        conf = Configuration(list(base_conf.joint_values) + list(arm_conf_val),
                            gantry_arm_joint_types, gantry_arm_joint_names)
                        if not client.check_collisions(robot, conf, options=options):
                            if verbose:
                                msg = 'IK found with IKFast after {}/{} gantry attempts - {} ik attempts / total {} IKFast solutions | total time {:.3f}'.format(
                                    attempt+1, ik_gantry_attempts, ik_iter, len(arm_conf_vals), pp.elapsed_time(start_time))
                                cprint(msg, 'green')

                            if temp_name in client.extra_disabled_collision_links:
                                del client.extra_disabled_collision_links[temp_name]
                            return (conf,)
                else:
                    if verbose:
                        msg = 'IK found with pb-IK after {}/{} gantry attempts | total time {:.3f}'.format(
                            attempt+1, ik_gantry_attempts, pp.elapsed_time(start_time))
                        cprint(msg, 'green')

                    if temp_name in client.extra_disabled_collision_links:
                        del client.extra_disabled_collision_links[temp_name]
                    return (conf,)
        if verbose:
            msg = 'no IK solotion found after {} gantry attempts | total time {:.3f}'.format(ik_gantry_attempts, pp.elapsed_time(start_time))
            cprint(msg, 'yellow')

        if temp_name in client.extra_disabled_collision_links:
            del client.extra_disabled_collision_links[temp_name]
        return None

    return ik_fn
