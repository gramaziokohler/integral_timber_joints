import integral_timber_joints.planning.pddlstream_planning.load_pddlstream
from integral_timber_joints.planning.utils import LOGGER
from integral_timber_joints.planning.pddlstream_planning.utils import print_itj_pddl_plan, print_pddl_task_object_names

from pddlstream.algorithms.meta import solve_restart, solve
from pddlstream.language.temporal import parse_domain
from pddlstream.utils import INF, Verbose, str_from_object, SEPARATOR
from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.conversion import Certificate, Object, \
    transform_plan_args, value_from_evaluation
from pddlstream.language.constants import PDDLProblem, get_function, get_prefix, print_solution, AND, get_args, And, \
    Solution, Or, is_plan
from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, \
    get_action_instances, apply_action, evaluation_from_fd, get_fluents
from pddlstream.algorithms.common import evaluations_from_init
from pddlstream.algorithms.serialized import partition_facts, serialize_goal, apply_actions

###############################################

def solve_serialized_incremental(initial_problem, stream_info={}, unit_costs=False, unit_efforts=False, verbose=True,
                     retain_facts=True, **kwargs):
    # TODO: be careful of CanMove deadends
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = initial_problem
    _, _, domain, streams = parse_problem(
        initial_problem, stream_info, constraints=None, unit_costs=unit_costs, unit_efforts=unit_efforts)
    static_init, _ = partition_facts(domain, init) # might not be able to reprove static_int
    #global_all, global_preimage = [], []
    global_plan = []
    global_cost = 0
    state = list(init)
    goals = serialize_goal(goal)
    # TODO: instead just track how the true init updates
    for i in range(len(goals)):
        # TODO: option in algorithms to pass in existing facts
        for stream in streams:
            stream.reset()
        goal = And(*goals[:i+1])
        LOGGER.info(f'Goal: {str_from_object(goal)}')
        # No strict need to reuse streams because generator functions
        #local_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, state, goal)
        local_problem = PDDLProblem(domain_pddl, constant_map, streams, None, state, goal)
        with Verbose(verbose):
            # solution = solve_focused(local_problem, stream_info=stream_info, unit_costs=unit_costs,
            #                          unit_efforts=unit_efforts, verbose=True, **kwargs)
            solution = solve_incremental(
                problem=local_problem, unit_costs=unit_costs,
                    verbose=True, **kwargs)
                
        print_solution(solution)

        local_plan, local_cost, local_certificate = solution
        if local_plan is None:
            # TODO: replan upon failure
            global_certificate = Certificate(all_facts={}, preimage_facts=None)
            return Solution(None, INF, global_certificate)

        if retain_facts:
            state = local_certificate.all_facts
        else:
            _, fluent_facts = partition_facts(domain, state)
            state = static_init + fluent_facts + local_certificate.preimage_facts # TODO: include functions
        #print('State:', state)
        # TODO: indicate when each fact is used
        # TODO: record failed facts
        global_plan.extend(local_plan)  # TODO: compute preimage of the executed plan
        global_cost += local_cost

        static_state, _ = partition_facts(domain, state)
        #global_all.extend(partition_facts(domain, local_certificate.all_facts)[0])
        #global_preimage.extend(static_state)
        # LOGGER.debug(f'Static: {static_state}')

        state = apply_actions(domain, state, local_plan, unit_costs=unit_costs)
        LOGGER.debug(SEPARATOR)
        #user_input('Continue?')
        # TODO: could also just test the goal here
        # TODO: constrain future plan skeletons

    global_certificate = Certificate(all_facts={}, preimage_facts=None)
    return global_plan, global_cost, global_certificate

