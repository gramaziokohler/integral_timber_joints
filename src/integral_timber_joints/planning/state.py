import os
import logging
import numpy as np
from pybullet_planning.interfaces.env_manager.pose_transformation import multiply
from copy import copy
from itertools import product, chain

from compas.geometry import distance_point_point, Transformation, allclose
from compas.geometry.primitives.frame import Frame
from compas.datastructures import Mesh
from compas.datastructures.mesh.triangulation import mesh_quads_to_triangles
from compas_fab.robots import AttachedCollisionMesh, Configuration, CollisionMesh, Robot

from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.utils import is_frames_close

import pybullet_planning as pp
from pybullet_planning import GREY
from pybullet_planning import LockRenderer, HideOutput, load_pybullet, wait_for_user
from pybullet_planning import get_sample_fn, link_from_name, joint_from_name, link_from_name, get_link_pose
from pybullet_planning import uniform_pose_generator

from integral_timber_joints.planning.robot_setup import MAIN_ROBOT_ID, GANTRY_ARM_GROUP, GANTRY_Z_LIMIT
from integral_timber_joints.planning.robot_setup import get_gantry_control_joint_names
from integral_timber_joints.planning.visualization import color_from_object_id
from integral_timber_joints.planning.parsing import PLANNING_DATA_DIR
from integral_timber_joints.planning.utils import FRAME_TOL, LOGGER
from integral_timber_joints.planning.cgal_utils import cgal_split_long_edges, HAS_CGAL
from integral_timber_joints.process import SceneState
from integral_timber_joints.process import  RobotClampAssemblyProcess

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
        # the y joint goes -9.65 to 0.0 so we need to negate it
        y *= -1
        z, = gantry_z_sample_fn()

        gantry_xyz_vals = [x, y, z]
        gantry_base_conf = Configuration(gantry_xyz_vals, gantry_arm_joint_types, sorted_gantry_joint_names)
        client.set_robot_configuration(robot, gantry_base_conf)

        yield gantry_base_conf


def set_state(client: PyChoreoClient, robot: Robot, process: RobotClampAssemblyProcess, scene: SceneState,
    initialize=False, scale=1e-3, options=None):
    """Set the pybullet client to a given scene state
    """
    options = options or {}
    debug = options.get('debug', False)
    verbose = options.get('verbose', True)
    include_env = options.get('include_env', True)
    reinit_tool = options.get('reinit_tool', False)
    # if 0.0, split_long_edges is turned off
    # ! this is ONLY applied to beams, NOT tools
    mesh_split_long_edge_max_length = options.get('mesh_split_long_edge_max_length', 0.0)

    # robot needed for creating attachments
    robot_uid = client.get_robot_pybullet_uid(robot)
    # flange_link_name = robot.get_end_effector_link_name(group=GANTRY_ARM_GROUP)
    flange_link_name = process.ROBOT_END_LINK

    with LockRenderer(not debug):
        # * Robot
        robot_config = scene[process.robot_config_key]
        if robot_config is not None:
            client.set_robot_configuration(robot, robot_config)
            tool_link = link_from_name(robot_uid, flange_link_name)
            FK_tool_frame = frame_from_pose(get_link_pose(robot_uid, tool_link))
            # TODO the client FK function is not working
            # FK_tool_frame = client.forward_kinematics(robot, robot_config, group=GANTRY_ARM_GROUP,
            #     options={'link' : flange_link_name})
            if scene[('robot', 'f')] is None:
                LOGGER.error('Robot frame is not set in scene!')
                return False
            else:
                # consistency check
                robot_frame = scene[('robot', 'f')].copy()
                robot_frame.point *= scale # convert to meter
                if not is_frames_close(robot_frame, FK_tool_frame, options=options):
                    msg = 'set_state: Robot FK tool pose and current frame diverge: {} (m)'.format(distance_point_point(robot_frame.point, FK_tool_frame.point))
                    LOGGER.error(msg)
                    return False

        # * Environment meshes
        if initialize and include_env:
            for name, _mesh in process.environment_models.items():
                mesh = _mesh.copy()
                mesh_quads_to_triangles(mesh)
                cm = CollisionMesh(mesh, name)
                cm.scale(scale)
                client.add_collision_mesh(cm, {'color': GREY})

        # * Beams
        for beam_id in process.assembly.beam_ids():
            if initialize:
                # ! notice that the notch geometry will be convexified in pybullet
                mesh = process.assembly.get_beam_mesh_in_ocf(beam_id).copy()
                # still in millimeter
                mesh_quads_to_triangles(mesh)

                if HAS_CGAL and mesh_split_long_edge_max_length > 1e-7:
                    V, F = mesh.to_vertices_and_faces()
                    # max_length is in millimeter
                    new_mesh_V_F = cgal_split_long_edges(V, F, max_length=mesh_split_long_edge_max_length, verbose=verbose)
                    mesh = Mesh.from_vertices_and_faces(*new_mesh_V_F)

                cm = CollisionMesh(mesh, beam_id)
                # this scale modifies the mesh attributes directly
                cm.scale(scale)
                # add mesh to environment at origin
                client.add_collision_mesh(cm)

            # * Setting Frame
            if scene[(beam_id, 'f')] is not None:
                current_frame = copy(scene[(beam_id, 'f')])
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
                    LOGGER.info('Tool {} ({}) URDF generated to {}'.format(tool.type_name, tool_id, urdf_path))
                with HideOutput():
                    tool_robot = load_pybullet(urdf_path, fixed_base=False)
                client.collision_objects[tool_id] = [tool_robot]

            # * Setting Frame
            if scene[(tool_id, 'f')] is not None:
                current_frame = copy(scene[(tool_id, 'f')])
                current_frame.point *= scale
                # * set pose according to state
                client.set_object_frame('^{}$'.format(tool_id), current_frame, options={'color': color_from_object_id(tool_id)})
            # else:
            #     LOGGER.error("Object {} frame not set!".format(tool_id))
            #     return False

            if tool_id != 'tool_changer' and scene[tool_id, 'c']:
                # * Setting Kinematics
                # this might be in millimeter, but that's not related to pybullet's business (if we use static meshes)
                tool._set_kinematic_state(scene[(tool_id, 'c')])
                tool_conf = tool.current_configuration.scaled(1e-3)
                tool_bodies = client._get_bodies('^{}$'.format(tool_id))
                for b in tool_bodies:
                    client._set_body_configuration(b, tool_conf)

        # * attachment management
        for object_id in chain(process.assembly.sequence, process.tool_ids, ['tool_changer']):
            wildcard = '^{}$'.format(object_id)
            object_names, status = client.get_object_names_and_status(wildcard)
            if scene[(object_id, 'a')] == False:
                # ! these are not attached objects
                # * demote attachedCM to collision_objects
                if status == 'attached_object':
                    client.detach_attached_collision_mesh(object_id, {'wildcard': wildcard})
            elif scene[(object_id, 'a')] == True:
                # ! these are attached objects
                if status == 'not_exist':
                    assert False, 'Object set object id ({}) | body names: {} as attached in scene but object not added to the scene!'.format(object_id, object_names)
                if scene[(object_id, 'g')] is None:
                    # * derive grasp transformation from FK and object frame
                    assert initialize or ('robot', 'f') in scene
                    # object_from_flange = get_object_from_flange(scene, object_id)
                    flange_frame = scene[('robot', 'f')].copy()
                    object_frame = scene[(object_id, 'f')].copy()
                    # convert to meter
                    flange_frame.point *= scale
                    object_frame.point *= scale

                    # * Derive transformation robot_flange_from_tool for attachment
                    t_world_object = Transformation.from_frame(object_frame)
                    t_world_robot = Transformation.from_frame(flange_frame)
                    robot_flange_from_attached_obj = t_world_robot.inverse() * t_world_object
                else:
                    # ! used only in pddlstream, grasp given
                    robot_flange_from_attached_obj = scene[(object_id, 'g')].copy()
                    for k in range(3):
                        robot_flange_from_attached_obj[k,3] *= 1e-3

                # touched_links is only for the adjacent Robot links
                touched_robot_links = []
                attached_object_base_link_name = None # default to use BASE_LINK if None
                if object_id == 'tool_changer':
                    # tool changer
                    attached_object_base_link_name = process.robot_toolchanger.get_base_link_name()
                    touched_robot_links = ['{}_tool0'.format(MAIN_ROBOT_ID), '{}_link_6'.format(MAIN_ROBOT_ID)]
                elif object_id in process.tool_ids:
                    # tools
                    attached_object_base_link_name = process.tool(object_id).get_base_link_name()
                else:
                    # beams
                    attached_object_base_link_name = None # process.tool(object_id).get_base_link_name()

                # * create attachments
                for name in object_names:
                    # a faked AttachedCM since we are not adding a new mesh, just promoting collision meshes to AttachedCMes
                    # if attachedCM already exist, update it's grasp and other info.
                    client.add_attached_collision_mesh(
                        AttachedCollisionMesh(CollisionMesh(None, name),
                                              flange_link_name, touch_links=touched_robot_links),
                        options={'robot': robot,
                                 'attached_child_link_name': attached_object_base_link_name,
                                 'parent_link_from_child_link_transformation' : robot_flange_from_attached_obj,
                                 })

                # ! update extra allowed collision regardless of status
                # * extra attachments disabled collisions (beyond just disabling attachment parent-child)
                # list of bodies that should not collide with the current object_id
                extra_disabled_bodies = []
                if object_id in process.assembly.sequence:
                    # ! disable collisions between the beam and gripper
                    # beam is the current object, iterating through all the tools to see which one is attached
                    # and disable collisions between them
                    tool_count = 0
                    for tool_id in process.tool_ids:
                        if scene[(tool_id, 'a')]:
                            extra_disabled_bodies.extend(client._get_bodies('^{}$'.format(tool_id)))
                            tool_count += 1
                    if tool_count == 0:
                        LOGGER.error('At least one tool should be attached to the robot when the beam is attached.')
                        return False

                elif object_id in process.tool_ids:
                    # ! disable collisions between the tool_changer and a tool
                    extra_disabled_bodies.extend(client._get_bodies('^{}$'.format('tool_changer')))

                for name in object_names:
                    at_bodies = client._get_attached_bodies('^{}$'.format(name))
                    if len(at_bodies) == 0:
                        LOGGER.error('Object {} has no pybullet body associated!'.format(name))
                        return False
                    for parent_body, child_body in product(extra_disabled_bodies, at_bodies):
                        client.extra_disabled_collision_links[name].add(
                            ((parent_body, None), (child_body, None))
                        )
            else:
                # * status == 'not_exist'
                LOGGER.error('Object set as attached in scene but object not added to the scene!')
                return False
    return True

#################################

def set_initial_state(client, robot, process, initialize=True, options=None):
    options = options or {}
    process.set_initial_state_robot_config(process.robot_initial_config)
    try:
        return set_state(client, robot, process, process.initial_state, initialize=initialize,
            options=options)
    except:
        LOGGER.info('Recomputing Actions and States')
        for beam_id in process.assembly.beam_ids():
            process.dependency.compute_all(beam_id)
        return set_state(client, robot, process, process.initial_state, initialize=initialize,
            options=options)
    # # * collision sanity check
    # assert not client.check_collisions(robot, full_start_conf, options={'diagnosis':True})
