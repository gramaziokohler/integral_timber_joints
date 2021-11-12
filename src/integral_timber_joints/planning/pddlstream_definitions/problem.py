import os
import random
from integral_timber_joints.planning.pddlstream_definitions import ITJ_PDDLSTREAM_DEF_DIR
from integral_timber_joints.planning import load_pddlstream
from integral_timber_joints.assembly.beam_assembly_method import BeamAssemblyMethod
from integral_timber_joints.planning.pddlstream_definitions.stream import get_ik_fn

from pddlstream.utils import read
from pddlstream.language.stream import StreamInfo, PartialInputs, WildOutput, DEBUG
from pddlstream.language.constants import And, PDDLProblem, Equal, print_plan
from pddlstream.language.generator import from_gen_fn, from_fn, from_test

from compas.robots import Configuration
from compas_fab.robots import Trajectory

class EmptyTrajectory(object):
    def __init__(self, tag=''):
        self.tag = tag
    def __repr__(self):
        return 'Traj-{}'.format(self.tag)

class EmptyConfiguration(object):
    def __init__(self, tag=''):
        self.tag = tag
    def __repr__(self):
        return 'Conf-{}'.format(self.tag)

def get_pddlstream_problem(client, process, robot, use_partial_order=True,
    debug=False, reset_to_home=False, consider_transition=False, symbolic_only=False, **kwargs):
    if symbolic_only:
        domain_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'symbolic', 'domain.pddl'))
        stream_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'symbolic', 'stream.pddl'))
    else:
        domain_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'tamp', 'domain.pddl'))
        stream_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'tamp', 'stream.pddl'))

    init = []

    home_conf = EmptyConfiguration('Home')
    constant_map = {}

    init.extend([
        ('RobotToolChangerEmpty',),
    ])
    if consider_transition:
        init.extend([
        ('ConsiderTransition',),
        ('RobotConf', home_conf),
        ('RobotAtConf', home_conf),
        ('CanFreeMove',),
        ])

    # * Elements
    toolchanger = process.robot_toolchanger
    flange_from_toolchanger_base = toolchanger.t_t0cf_from_tcf
    beam_seq = process.assembly.sequence
    for e in beam_seq:
        f_world_from_beam_pickup = process.assembly.get_beam_attribute(e, 'assembly_wcf_pickup')
        f_world_from_beam_final = process.assembly.get_beam_attribute(e, 'assembly_wcf_final')

        # * get beam grasp
        # ? different gripper might have different grasp for a beam?
        t_gripper_tcf_from_beam = process.assembly.get_t_gripper_tcf_from_beam(e)
        beam_gripper_id = process.assembly.get_beam_attribute(e, "gripper_id")
        beam_gripper = process.tool(beam_gripper_id)
        flange_from_beam = flange_from_toolchanger_base * beam_gripper.t_t0cf_from_tcf * t_gripper_tcf_from_beam

        init.extend([
            ('Element', e),
            ('IsElement', e),
            ('RackPose', e, f_world_from_beam_pickup),
            ('Pose', e, f_world_from_beam_pickup),
            ('ElementGoalPose', e, f_world_from_beam_final),
            ('Pose', e, f_world_from_beam_final),
            ('Grasp', e, flange_from_beam),
        ])
        if process.assembly.get_assembly_method(e) == BeamAssemblyMethod.GROUND_CONTACT:
            init.extend([
                ('Grounded', e),
                ])

    for j in process.assembly.joint_ids():
        for k0, k1 in [(0,1), (1,0)]:
            init.extend([
                ('Joint', j[k0], j[k1]),
                ('NoToolAtJoint', j[k0], j[k1]),
            ])

    if use_partial_order:
        for e1, e2 in zip(beam_seq[:-1], beam_seq[1:]):
            init.append(('Order', e1, e2))

    # * Clamps
    for c in process.clamps:
        tool_storage_frame = c.tool_storage_frame
        clamp_grasp = toolchanger.t_t0cf_from_tcf
        init.extend([
            ('Clamp', c.name),
            ('IsClamp', c.name),
            ('IsTool', c.name),
            ('RackPose', c.name, tool_storage_frame),
            ('Pose', c.name, tool_storage_frame),
            ('AtPose', c.name, tool_storage_frame),
            ('ToolNotOccupiedOnJoint', c.name),
            ('Grasp', c.name, clamp_grasp),
        ])
    # ! assign tool type to joints
    for j in process.assembly.joint_ids():
        joint_clamp_type = process.assembly.get_joint_attribute(j, 'tool_type')
        clamp_wcf_final = process.get_tool_t0cf_at(j, 'clamp_wcf_final')
        for c in process.clamps:
            if c.type_name == joint_clamp_type:
                init.extend([
                    ('JointToolTypeMatch', j[0], j[1], c.name),
                    ('JointToolTypeMatch', j[1], j[0], c.name),
                    ('Pose', c.name, clamp_wcf_final),
                    ('ClampPose', c.name, clamp_wcf_final),
                    ('JointPose', j[0], j[1], clamp_wcf_final),
                    ('JointPose', j[1], j[0], clamp_wcf_final),
                ])

    # * Grippers
    for g in process.grippers:
        tool_storage_frame = g.tool_storage_frame
        gripper_grasp = toolchanger.t_t0cf_from_tcf
        init.extend([
            ('Gripper', g.name),
            ('IsGripper', g.name),
            ('IsTool', g.name),
            ('RackPose', g.name, tool_storage_frame),
            ('Pose', g.name, tool_storage_frame),
            ('AtPose', g.name, tool_storage_frame),
            ('Grasp', g.name, gripper_grasp),
        ])
    # * gripper type
    for beam_id in beam_seq:
        beam_gripper_type = process.assembly.get_beam_attribute(beam_id, "gripper_type")
        for g in process.grippers:
            if g.type_name == beam_gripper_type:
                init.append(('GripperToolTypeMatch', beam_id, g.name))

    if debug:
        stream_map = DEBUG
    else:
        stream_map = {
            'inverse-kinematics':  from_fn(get_ik_fn(client, process, robot, **kwargs)),
            # 'sample-move': from_fn(lambda conf1, conf2: (EmptyTrajectory(),)),
            # 'sample-pick-tool': from_fn(lambda obj: (EmptyConfiguration(), EmptyConfiguration(), EmptyTrajectory())),
            # 'sample-place-tool': from_fn(lambda obj: (EmptyConfiguration(), EmptyConfiguration(), EmptyTrajectory())),
            # 'sample-pick-element': from_fn(lambda obj: (EmptyConfiguration(), EmptyConfiguration(), EmptyTrajectory())),
            # 'sample-place-element': from_fn(lambda obj: (EmptyConfiguration(), EmptyConfiguration(), EmptyTrajectory())),
            # 'test-cfree': from_test(lambda *args: not collision_test(*args)),
        }

    goal_literals = []
    goal_literals.extend(('Assembled', e) for e in beam_seq)
    if reset_to_home:
        goal_literals.extend(('AtRack', t.name) for t in list(process.clamps) + list(process.grippers))
        if consider_transition:
            goal_literals.append(('RobotAtConf', home_conf))
    goal = And(*goal_literals)

    pddlstream_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)
    return pddlstream_problem
