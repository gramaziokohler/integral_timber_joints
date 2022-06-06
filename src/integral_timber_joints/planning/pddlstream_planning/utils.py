from functools import partial
from termcolor import cprint, colored

from compas.geometry import Transformation, Frame
from compas_fab.robots import Trajectory
from compas.robots.configuration import Configuration

from pddlstream.utils import str_from_object
from pddlstream.language.conversion import obj_from_pddl
from pddlstream.language.constants import is_plan, DurativeAction, Action, StreamAction, FunctionAction
#
from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.downward import get_problem, task_from_domain_problem

from integral_timber_joints.process.action import LoadBeamAction, PickGripperFromStorageAction, PickBeamWithGripperAction, PickClampFromStorageAction, PlaceClampToStructureAction, BeamPlacementWithClampsAction, PlaceGripperToStorageAction, PlaceClampToStorageAction, PickClampFromStructureAction, BeamPlacementWithoutClampsAction, AssembleBeamWithScrewdriversAction,  RetractGripperFromBeamAction, PickScrewdriverFromStorageAction, PlaceScrewdriverToStorageAction, ManaulAssemblyAction, OperatorAttachScrewdriverAction, DockWithScrewdriverAction, RetractScrewdriverFromBeamAction, GenericGripperApproachBeamPickupAction, CloseGripperOnBeamAction
from integral_timber_joints.planning.utils import LOGGER

##########################################

ITJ_ACTION_CLASS_FROM_PDDL_ACTION_NAME  = {
    # ! (beam_id, gripper_id)
    'pick_beam_with_gripper' : PickBeamWithGripperAction,
    'generic_gripper_approach_beam_pickup' : GenericGripperApproachBeamPickupAction,
    'close_gripper_on_beam' : CloseGripperOnBeamAction,

    'retract_gripper_from_beam' : RetractGripperFromBeamAction,
    'beam_placement_without_clamp' : BeamPlacementWithoutClampsAction,
    # ! (beam_id, joint_ids, gripper_id, clamp_ids | screwdriver_ids)
    'beam_placement_with_clamps' : BeamPlacementWithClampsAction,
    'assemble_beam_with_screwdrivers' : AssembleBeamWithScrewdriversAction,
    # ! (tool_type, tool_id)
    'pick_tool_from_rack' : {
        'g' : PickGripperFromStorageAction,
        'c' : PickClampFromStorageAction,
        's' : PickScrewdriverFromStorageAction,
    },
    'place_tool_at_rack' : {
        'g' : PlaceGripperToStorageAction,
        'c' : PlaceClampToStorageAction,
        's' : PlaceScrewdriverToStorageAction,
    },
    # ! (joint_id, tool_type, tool_id)
    'place_clamp_to_structure' : PlaceClampToStructureAction,
    'pick_clamp_from_structure' : PickClampFromStructureAction,
    # ! (joint_id, tool_position='screwdriver_assembled_attached', tool_type, tool_id)
    'dock_with_screwdriver' : DockWithScrewdriverAction,
    # ! (beam_id, joint_id, tool_id)
    'retract_screwdriver_from_beam' : RetractScrewdriverFromBeamAction,
    # ! manual ops: no sampler needed
    'operator_load_beam' : LoadBeamAction,
    # (beam_id)
    'manaul_assemble_scaffold' : ManaulAssemblyAction,
    # (beam_id)
    'operator_attach_screwdriver' : OperatorAttachScrewdriverAction,
    # (beam_id, joint_id, screwdriver.type_name, tool_id, 'assembly_wcf_screwdriver_attachment_pose')
}

##########################################

def print_pddl_task_object_names(pddl_problem):
    evaluations, goal_exp, domain, externals = parse_problem(
        pddl_problem, unit_costs=True)
    problem = get_problem(evaluations, goal_exp, domain, unit_costs=True)
    task = task_from_domain_problem(domain, problem)
    LOGGER.debug('='*10)
    for task_obj, pddl_object in sorted(
            zip(task.objects, map(lambda x: obj_from_pddl(x.name), task.objects)),
            key=lambda x: int(x[0].name.split('v')[1])):
        LOGGER.debug('{} : {}'.format(task_obj.name, colored_str_from_object(pddl_object.value)))
    LOGGER.debug('='*10)

##########################################

def contains_number(value):
    for character in value:
        if character.isdigit():
            return True
    return False

def colored_str_from_object(obj, show_details=False):
    if not show_details:
        if isinstance(obj, Frame):
            return '(frm)'
        elif isinstance(obj, Transformation):
            return '(tf)'
        elif isinstance(obj, Configuration):
            return colored('(conf)', 'yellow')
        elif isinstance(obj, Action):
            return colored(obj, 'yellow')

    str_rep = str_from_object(obj)
    if contains_number(str_rep):
        return colored(str_rep, 'blue')
    else:
        return colored(str_rep, 'red')

def print_fluents(fluents, show_details=False):
    color_print_fn = partial(colored_str_from_object, show_details=show_details)
    LOGGER.debug('Fluents:')
    for i, fluent in enumerate(sorted(fluents, key=lambda x: x[0])):
        fluent_name = fluent[0]
        args = fluent[1:]
        LOGGER.debug('{:2}) {} {}'.format(i, colored(fluent_name, 'green'), ' '.join(map(color_print_fn, args))))

def print_itj_pddl_plan(plan, show_details=False):
    if not is_plan(plan):
        return
    step = 1
    color_print_fn = partial(colored_str_from_object, show_details=show_details)
    for action in plan:
        if isinstance(action, DurativeAction):
            name, args, start, duration = action
            LOGGER.info('{:.2f} - {:.2f}) {} {}'.format(start, start+duration, name,
                                                  ' '.join(map(str_from_object, args))))
        elif isinstance(action, Action):
            name, args = action
            LOGGER.info('{:2}) {} {}'.format(step, colored(name, 'green'), ' '.join(map(color_print_fn, args))))
            step += 1
        elif isinstance(action, StreamAction):
            name, inputs, outputs = action
            LOGGER.info('    {}({})->({})'.format(name, ', '.join(map(str_from_object, inputs)),
                                            ', '.join(map(str_from_object, outputs))))
        elif isinstance(action, FunctionAction):
            name, inputs = action
            LOGGER.info('    {}({})'.format(name, ', '.join(map(str_from_object, inputs))))
        else:
            raise NotImplementedError(action)


