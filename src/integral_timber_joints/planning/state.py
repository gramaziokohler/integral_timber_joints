from external.pybullet_planning.src.pybullet_planning.interfaces.env_manager.user_io import wait_for_user
import os
from termcolor import cprint
from copy import copy, deepcopy
from itertools import product

from compas.geometry import distance_point_point
from compas.geometry.primitives.frame import Frame
from compas.datastructures.mesh.triangulation import mesh_quads_to_triangles
from compas_fab.robots import AttachedCollisionMesh, Configuration, CollisionMesh, Robot

from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo.client import PyChoreoClient

from pybullet_planning import GREY
from pybullet_planning import LockRenderer, HideOutput, load_pybullet
from pybullet_planning import get_sample_fn, link_from_name, joint_from_name, link_from_name, get_link_pose
from pybullet_planning import uniform_pose_generator

from integral_timber_joints.planning.robot_setup import MAIN_ROBOT_ID, GANTRY_ARM_GROUP, GANTRY_Z_LIMIT
from integral_timber_joints.planning.robot_setup import get_gantry_control_joint_names
from integral_timber_joints.planning.visualization import color_from_object_id
from integral_timber_joints.planning.parsing import PLANNING_DATA_DIR
from integral_timber_joints.planning.utils import FRAME_TOL
from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotClampAssemblyProcess, Movement, ObjectState
from typing import Dict

##############################


def gantry_base_generator(client: PyChoreoClient, robot: Robot, flange_frame: Frame, reachable_range=(0.2, 2.5), scale=1.0):
    robot_uid = client.get_robot_pybullet_uid(robot)
    flange_pose = pose_from_frame(flange_frame, scale=scale)

    sorted_gantry_joint_names = get_gantry_control_joint_names(MAIN_ROBOT_ID)
    gantry_arm_joint_types = robot.get_joint_types_by_names(sorted_gantry_joint_names)
    gantry_z_joint = joint_from_name(robot_uid, sorted_gantry_joint_names[2])

    gantry_z_sample_fn = get_sample_fn(robot_uid, [gantry_z_joint], custom_limits={gantry_z_joint: GANTRY_Z_LIMIT})
    base_gen_fn = uniform_pose_generator(robot_uid, flange_pose, reachable_range=reachable_range)

    while True:
        # TODO a more formal gantry_base_from_world_base
        x, y, _ = next(base_gen_fn)
        y *= -1
        z, = gantry_z_sample_fn()
        gantry_xyz_vals = [x, y, z]
        gantry_base_conf = Configuration(gantry_xyz_vals, gantry_arm_joint_types, sorted_gantry_joint_names)
        client.set_robot_configuration(robot, gantry_base_conf)
        yield gantry_base_conf


def set_state(client: PyChoreoClient, robot: Robot, process: RobotClampAssemblyProcess, state_from_object: Dict[str, ObjectState], initialize=False, scale=1e-3, options=None):
    """Set the pybullet client to a given state

    Parameters
    ----------
    client : [type]
        [description]
    robot : [type]
        [description]
    process : [type]
        [description]
    state_from_object : dict(object_id : state)
        state dictionary for objects in the scene
    initialize : bool, optional
        [description], by default False
    scale : [type], optional
        [description], by default 1e-3
    options : [type], optional
        [description], by default None

    Raises
    ------
    RuntimeError
        [description]
    """
    options = options or {}
    ik_gantry_attempts = options.get('ik_gantry_attempts') or 5000
    debug = options.get('debug', False)
    include_env = options.get('include_env', True)
    reinit_tool = options.get('reinit_tool', False)
    frame_jump_tolerance = options.get('frame_jump_tolerance', FRAME_TOL*1e3)
    reachable_range = options.get('reachable_range') or (0.2, 2.8)

    # robot needed for creating attachments
    robot_uid = client.get_robot_pybullet_uid(robot)
    flange_link_name = robot.get_end_effector_link_name(group=GANTRY_ARM_GROUP)

    with LockRenderer(not debug):
        # * Do robot first
        robot_state = state_from_object['robot']
        if robot_state.kinematic_config is not None:
            client.set_robot_configuration(robot, robot_state.kinematic_config)
            tool_link = link_from_name(robot_uid, flange_link_name)
            # in millimeter
            FK_tool_frame = frame_from_pose(get_link_pose(robot_uid, tool_link), scale=1/scale)
            # perform FK
            if robot_state.current_frame is None:
                robot_state.current_frame = FK_tool_frame
            else:
                if not robot_state.current_frame.__eq__(FK_tool_frame, tol=frame_jump_tolerance):
                    if (1e-3*distance_point_point(robot_state.current_frame.point, FK_tool_frame.point) > frame_jump_tolerance):
                        msg = 'Robot FK tool pose and current frame diverge: {:.5f} (m)'.format(1e-3*distance_point_point(robot_state.current_frame.point, FK_tool_frame.point))
                        cprint(msg, 'yellow')
                        cprint('!!! Overwriting the current_frame by the given robot conf\'s FK. Please confirm this.')
                        wait_for_user()
                    robot_state.current_frame = FK_tool_frame

            if initialize:
                # update tool_changer's current_frame
                # ! change if tool_changer has a non-trivial grasp pose
                state_from_object['tool_changer'].current_frame = FK_tool_frame

        # * environment meshes
        if initialize and include_env:
            for name, _mesh in process.environment_models.items():
                # if name == 'e27':
                mesh = _mesh.copy()
                mesh_quads_to_triangles(mesh)
                cm = CollisionMesh(mesh, name)
                cm.scale(scale)
                client.add_collision_mesh(cm, {'color': GREY})

        for object_id, object_state in state_from_object.items():
            # print('====')
            # print('{} : {}'.format(object_id, object_state))
            if object_id.startswith('robot'):
                continue
            obj = process.get_object_from_id(object_id)
            if initialize:
                # * create each object in the state dictionary
                # create objects in pybullet, convert mm to m
                color = GREY
                if object_id.startswith('b'):
                    # ! notice that the notch geometry will be convexified in pybullet
                    mesh = obj.mesh.copy()
                    mesh_quads_to_triangles(mesh)
                    cm = CollisionMesh(mesh, object_id)
                    cm.scale(scale)
                    # add mesh to environment at origin
                    client.add_collision_mesh(cm)
                else:
                    urdf_path = obj.get_urdf_path(PLANNING_DATA_DIR)
                    if reinit_tool or not os.path.exists(urdf_path):
                        obj.save_as_urdf(PLANNING_DATA_DIR, scale=1e-3, triangulize=True)
                        cprint('Tool {} ({}) URDF generated to {}'.format(object_id, obj.name, urdf_path), 'green')
                    with HideOutput():
                        tool_robot = load_pybullet(urdf_path, fixed_base=False)
                    client.collision_objects[object_id] = [tool_robot]
            # end if initialize

            if object_state.current_frame is not None:
                current_frame = copy(object_state.current_frame)
                current_frame.point *= scale
                # * set pose according to state
                client.set_object_frame('^{}$'.format(object_id), current_frame, options={'color': color_from_object_id(object_id)})

            if object_state.kinematic_config is not None:
                assert object_id.startswith('c') or object_id.startswith('g')
                # this might be in millimeter, but that's not related to pybullet's business (if we use static meshes)
                obj._set_kinematic_state(object_state.kinematic_config)
                tool_conf = obj.current_configuration.scaled(1e-3)
                tool_bodies = client._get_bodies('^{}$'.format(object_id))
                for b in tool_bodies:
                    client._set_body_configuration(b, tool_conf)

            # * attachment management
            wildcard = '^{}$'.format(object_id)
            # -1 if not attached and not collision object
            # 0 if collision object, 1 if attached
            status = client._get_body_status(wildcard)
            if not object_state.attached_to_robot:
                # * demote attachedCM to collision_objects
                if status == 1:
                    client.detach_attached_collision_mesh(object_id, {'wildcard': wildcard})
            else:
                # * promote to attached_collision_object if needed
                if status == 0:
                    # * attached collision objects haven't been added
                    assert initialize or state_from_object['robot'].current_frame is not None
                    # object_from_flange = get_object_from_flange(state_from_object, object_id)
                    flange_frame = deepcopy(state_from_object['robot'].current_frame)
                    object_frame = deepcopy(state_from_object[object_id].current_frame)
                    flange_frame.point *= scale
                    object_frame.point *= scale

                    # * sample from a ball near the pose
                    gantry_base_gen_fn = gantry_base_generator(client, robot, flange_frame, reachable_range=reachable_range, scale=1.0)
                    with HideOutput():
                        for _, base_conf in zip(range(ik_gantry_attempts), gantry_base_gen_fn):
                            # TODO a more formal gantry_base_from_world_base
                            conf = client.inverse_kinematics(robot, flange_frame, group=GANTRY_ARM_GROUP,
                                                             options={'avoid_collisions': False})
                            if conf is not None:
                                break
                        else:
                            raise RuntimeError('no attach conf found for {} after {} attempts.'.format(object_state, ik_gantry_attempts))
                    client.set_robot_configuration(robot, conf)
                    # wait_if_gui('Conf set for attachment')

                    # * create attachments
                    wildcard = '^{}$'.format(object_id)
                    names = client._get_collision_object_names(wildcard)
                    # touched_links is only for the adjacent Robot links
                    touched_links = ['{}_tool0'.format(MAIN_ROBOT_ID), '{}_link_6'.format(MAIN_ROBOT_ID)] \
                        if object_id.startswith('t') else []
                    # TODO auto derive from URDF link tree
                    attached_child_link_name = None
                    if object_id.startswith('t'):
                        attached_child_link_name = 'toolchanger_base'
                    elif object_id.startswith('g') or object_id.startswith('c'):
                        attached_child_link_name = 'gripper_base'
                    for name in names:
                        # a faked AttachedCM since we are not adding a new mesh, just promoting collision meshes to AttachedCMes
                        client.add_attached_collision_mesh(AttachedCollisionMesh(CollisionMesh(None, name),
                                                                                 flange_link_name, touch_links=touched_links), options={'robot': robot, 'attached_child_link_name': attached_child_link_name})

                    # * attachments disabled collisions
                    extra_disabled_bodies = []
                    if object_id.startswith('b'):
                        # gripper and beam
                        g_id = None
                        for o_id, o_st in state_from_object.items():
                            if o_id.startswith('g') and o_st.attached_to_robot:
                                g_id = o_id
                                break
                        assert g_id is not None, 'At least one gripper should be attached to the robot when the beam is attached.'
                        extra_disabled_bodies = client._get_bodies('^{}$'.format(g_id))
                    elif object_id.startswith('c') or object_id.startswith('g'):
                        # tool_changer and gripper
                        extra_disabled_bodies = client._get_bodies('^{}$'.format('tool_changer'))
                    for name in names:
                        at_bodies = client._get_attached_bodies('^{}$'.format(name))
                        assert len(at_bodies) > 0
                        for parent_body, child_body in product(extra_disabled_bodies, at_bodies):
                            client.extra_disabled_collision_links[name].add(
                                ((parent_body, None), (child_body, None))
                            )
            # end if attached_to_robot
