import os
import numpy as np
from pybullet_planning.interfaces.env_manager.pose_transformation import multiply
from pybullet_planning.interfaces.env_manager.user_io import wait_if_gui
from termcolor import cprint
from copy import copy, deepcopy
from itertools import product, chain

from compas.geometry import distance_point_point, Transformation
from compas.geometry.primitives.frame import Frame
from compas.datastructures.mesh.triangulation import mesh_quads_to_triangles
from compas_fab.robots import AttachedCollisionMesh, Configuration, CollisionMesh, Robot

from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo.client import PyChoreoClient

import pybullet_planning as pp
from pybullet_planning import GREY
from pybullet_planning import LockRenderer, HideOutput, load_pybullet, wait_for_user
from pybullet_planning import get_sample_fn, link_from_name, joint_from_name, link_from_name, get_link_pose
from pybullet_planning import uniform_pose_generator

from integral_timber_joints.planning.robot_setup import MAIN_ROBOT_ID, GANTRY_ARM_GROUP, GANTRY_Z_LIMIT
from integral_timber_joints.planning.robot_setup import to_rlf_robot_full_conf, \
    R11_INTER_CONF_VALS, R12_INTER_CONF_VALS
from integral_timber_joints.planning.robot_setup import get_gantry_control_joint_names
from integral_timber_joints.planning.visualization import color_from_object_id
from integral_timber_joints.planning.parsing import PLANNING_DATA_DIR
from integral_timber_joints.planning.utils import FRAME_TOL
from integral_timber_joints.process import SceneState
from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotClampAssemblyProcess, Movement, ObjectState
from typing import Dict, Tuple, Any

##############################


def gantry_base_generator(client: PyChoreoClient, robot: Robot, flange_frame: Frame,
        reachable_range=(0., 2.5), scale=1.0, spherical_sampling=True):
    robot_uid = client.get_robot_pybullet_uid(robot)
    flange_pose = pose_from_frame(flange_frame, scale=scale)

    sorted_gantry_joint_names = get_gantry_control_joint_names(MAIN_ROBOT_ID)
    gantry_arm_joint_types = robot.get_joint_types_by_names(sorted_gantry_joint_names)
    gantry_z_joint = joint_from_name(robot_uid, sorted_gantry_joint_names[2])

    gantry_z_sample_fn = get_sample_fn(robot_uid, [gantry_z_joint], custom_limits={gantry_z_joint: GANTRY_Z_LIMIT})

    gantry_base_from_world = pp.get_relative_pose(robot_uid,
        link_from_name(robot_uid, 'world'), link_from_name(robot_uid, 'x_rail'))
    # ? gantry_base_from_flange = gantry_base_from_world * world_from_flange
    gantry_base_from_flange = multiply(gantry_base_from_world, flange_pose)
    base_gen_fn = uniform_pose_generator(robot_uid, gantry_base_from_flange, reachable_range=reachable_range)

    while True:
        x, y, _ = next(base_gen_fn)
        # x joint: lower="0.0" upper="37.206"
        # y joint: lower="-9.65" upper="0.0"
        y *= -1
        z, = gantry_z_sample_fn()

        gantry_xyz_vals = [x, y, z]
        gantry_base_conf = Configuration(gantry_xyz_vals, gantry_arm_joint_types, sorted_gantry_joint_names)
        client.set_robot_configuration(robot, gantry_base_conf)

        yield gantry_base_conf


def set_state(client: PyChoreoClient, robot: Robot, process: RobotClampAssemblyProcess, scene: SceneState, initialize=False, scale=1e-3, options=None):
    """Set the pybullet client to a given scene state
    """
    options = options or {}
    debug = options.get('debug', False)
    verbose = options.get('verbose', True)
    include_env = options.get('include_env', True)
    reinit_tool = options.get('reinit_tool', False)
    frame_jump_tolerance = options.get('frame_jump_tolerance', FRAME_TOL*1e3)
    reachable_range = options.get('reachable_range') or (0.2, 2.8)

    # robot needed for creating attachments
    robot_uid = client.get_robot_pybullet_uid(robot)
    flange_link_name = robot.get_end_effector_link_name(group=GANTRY_ARM_GROUP)

    with LockRenderer(not debug):
        # * Robot and Tool Changer
        robot_config = scene[process.robot_config_key]
        if robot_config is not None:
            client.set_robot_configuration(robot, robot_config)
            tool_link = link_from_name(robot_uid, flange_link_name)
            FK_tool_frame = frame_from_pose(get_link_pose(robot_uid, tool_link), scale=1/scale)
            # wait_if_gui()
            # TODO the client FK function is not working
            # FK_tool_frame = client.forward_kinematics(robot, robot_config, group=GANTRY_ARM_GROUP,
            #     options={'link' : flange_link_name})
            # FK_tool_frame.point *= 1/scale
            if scene[('robot', 'f')] is None:
                # print('Overwrite: ', FK_tool_frame)
                scene[('robot', 'f')] = FK_tool_frame
            else:
                # consistency check
                robot_frame = scene[('robot', 'f')]
                if not robot_frame.__eq__(FK_tool_frame, tol=frame_jump_tolerance*scale):
                    if (1e-3*distance_point_point(robot_frame.point, FK_tool_frame.point) > frame_jump_tolerance):
                        if verbose:
                            msg = 'Robot FK tool pose and current frame diverge: {:.5f} (m)'.format(1e-3*distance_point_point(robot_frame.point, FK_tool_frame.point))
                            cprint(msg, 'yellow')
                            cprint('!!! Overwriting the current_frame {} by the given robot conf\'s FK {} | robot conf {}. Please confirm this.'.format(
                                robot_frame.point, FK_tool_frame.point, robot_config.joint_values
                            ))
                            wait_if_gui()
                    scene[('robot', 'f')] = FK_tool_frame

            if initialize:
                # update tool_changer's current_frame
                # ! change if tool_changer has a non-trivial grasp pose
                scene['tool_changer', 'f'] = FK_tool_frame

        # * Environment meshes
        if initialize and include_env:
            for name, _mesh in process.environment_models.items():
                # if name == 'e27':
                mesh = _mesh.copy()
                mesh_quads_to_triangles(mesh)
                cm = CollisionMesh(mesh, name)
                cm.scale(scale)
                client.add_collision_mesh(cm, {'color': GREY})

        # * Beams
        for beam_id in process.assembly.beam_ids():
            if initialize:
                color = GREY
                # ! notice that the notch geometry will be convexified in pybullet
                mesh = process.assembly.beam(beam_id).mesh.copy()
                mesh_quads_to_triangles(mesh)
                cm = CollisionMesh(mesh, beam_id)
                cm.scale(scale)
                # add mesh to environment at origin
                client.add_collision_mesh(cm)

            # * Setting Frame
            if scene[beam_id, 'f'] is not None:
                current_frame = copy(scene[beam_id, 'f'])
                current_frame.point *= scale
                # * set pose according to state
                client.set_object_frame('^{}$'.format(beam_id), current_frame, options={'color': color_from_object_id(beam_id)})

        # * Tools and Tool changer
        for tool_id in chain(['tool_changer'], process.tool_ids):
            tool = process.robot_toolchanger if tool_id == 'tool_changer' else process.tool(tool_id)
            if initialize:
                urdf_path = tool.get_urdf_path(PLANNING_DATA_DIR)
                if reinit_tool or not os.path.exists(urdf_path):
                    tool.save_as_urdf(PLANNING_DATA_DIR, scale=1e-3, triangulize=True)
                    cprint('Tool {} ({}) URDF generated to {}'.format(tool.type_name, tool_id, urdf_path), 'green')
                with HideOutput():
                    tool_robot = load_pybullet(urdf_path, fixed_base=False)
                client.collision_objects[tool_id] = [tool_robot]

            # * Setting Frame
            if scene[tool_id, 'f'] is not None:
                current_frame = copy(scene[tool_id, 'f'])
                current_frame.point *= scale
                # * set pose according to state
                client.set_object_frame('^{}$'.format(tool_id), current_frame, options={'color': color_from_object_id(tool_id)})

            if tool_id != 'tool_changer':
                # * Setting Kinematics
                # this might be in millimeter, but that's not related to pybullet's business (if we use static meshes)
                tool._set_kinematic_state(scene[tool_id, 'c'])
                tool_conf = tool.current_configuration.scaled(1e-3)
                tool_bodies = client._get_bodies('^{}$'.format(tool_id))
                for b in tool_bodies:
                    client._set_body_configuration(b, tool_conf)

        # * attachment management
        for object_id in chain(process.assembly.sequence, process.tool_ids, ['tool_changer']):
            wildcard = '^{}$'.format(object_id)
            # -1 if not attached and not collision object
            # 0 if collision object, 1 if attached
            status = client._get_body_status(wildcard)
            if scene[(object_id, 'a')] != True:
                # ! these are not attached objects
                # * demote attachedCM to collision_objects
                if status == 1:
                    client.detach_attached_collision_mesh(object_id, {'wildcard': wildcard})
            else:
                # ! these are attached objects
                # * promote to attached_collision_object if needed
                if status == 0:
                    # * attached collision objects haven't been added
                    assert initialize or ('robot', 'f') in scene
                    # object_from_flange = get_object_from_flange(scene, object_id)
                    flange_frame = scene[('robot', 'f')].copy()
                    object_frame = scene[(object_id, 'f')].copy()
                    # convert to meter
                    flange_frame.point *= scale
                    object_frame.point *= scale
                    robot_flange_from_tool = Transformation.from_frame_to_frame(object_frame, flange_frame)

                    # * create attachments
                    wildcard = '^{}$'.format(object_id)
                    collision_object_names = client._get_collision_object_names(wildcard)

                    # touched_links is only for the adjacent Robot links
                    touched_links = []
                    attached_child_link_name = None # default to use BASE_LINK if None
                    if object_id == 'tool_changer':
                        # tool changer
                        attached_child_link_name = process.robot_toolchanger.get_base_link_name()
                        touched_links = ['{}_tool0'.format(MAIN_ROBOT_ID), '{}_link_6'.format(MAIN_ROBOT_ID)]
                    elif object_id in process.tool_ids:
                        # tools
                        attached_child_link_name = process.tool(object_id).get_base_link_name()
                    else:
                        # beams
                        attached_child_link_name = None # process.tool(object_id).get_base_link_name()

                    for name in collision_object_names:
                        # a faked AttachedCM since we are not adding a new mesh, just promoting collision meshes to AttachedCMes
                        client.add_attached_collision_mesh(
                            AttachedCollisionMesh(CollisionMesh(None, name),
                                                  flange_link_name, touch_links=touched_links),
                            options={'robot': robot,
                                     'attached_child_link_name': attached_child_link_name,
                                     'parent_link_from_child_link_transformation' : robot_flange_from_tool,
                                     })

                    # * attachments disabled collisions
                    # list of bodies that should not collide with the current object_id
                    extra_disabled_bodies = []
                    if object_id in process.assembly.sequence:
                        # beam is the current object, iterating through all the tools to see which one is attached
                        # and disable collisions between them
                        attached_gripper_id = None
                        for gripper in process.grippers:
                            if scene[gripper.name, 'a']:
                                attached_gripper_id = gripper.name
                                break
                        assert attached_gripper_id is not None, 'At least one gripper should be attached to the robot when the beam is attached.'
                        extra_disabled_bodies = client._get_bodies('^{}$'.format(attached_gripper_id))
                    elif object_id in process.tool_ids:
                        # tool_changer and a tool
                        extra_disabled_bodies = client._get_bodies('^{}$'.format('tool_changer'))

                    for name in collision_object_names:
                        at_bodies = client._get_attached_bodies('^{}$'.format(name))
                        assert len(at_bodies) > 0
                        for parent_body, child_body in product(extra_disabled_bodies, at_bodies):
                            client.extra_disabled_collision_links[name].add(
                                ((parent_body, None), (child_body, None))
                            )
            # end if attached_to_robot

#################################

def set_initial_state(client, robot, process, disable_env=False, reinit_tool=True, debug=False):
    # set all other unused robot
    # full_start_conf = to_rlf_robot_full_conf(R11_INTER_CONF_VALS, R12_INTER_CONF_VALS)
    # client.set_robot_configuration(robot, full_start_conf)
    process.set_initial_state_robot_config(process.robot_initial_config)
    try:
        set_state(client, robot, process, process.initial_state, initialize=True,
            options={'debug' : debug, 'include_env' : not disable_env, 'reinit_tool' : reinit_tool})
    except:
        cprint('Recomputing Actions and States', 'cyan')
        for beam_id in process.assembly.beam_ids():
            process.dependency.compute_all(beam_id)
        set_state(client, robot, process, process.initial_state, initialize=True,
            options={'debug' : debug, 'include_env' : not disable_env, 'reinit_tool' : reinit_tool})
    # # * collision sanity check
    # assert not client.check_collisions(robot, full_start_conf, options={'diagnosis':True})

