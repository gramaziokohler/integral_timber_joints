from compas.datastructures.network.core.graph import Graph

import integral_timber_joints
from integral_timber_joints.assembly import Assembly

try:
    from typing import Dict, List, Optional, Tuple

    from integral_timber_joints.process import RobotClampAssemblyProcess
except:
    pass


class ComputationalResult(object):
    Invalid = 0
    ValidCanContinue = 1
    ValidCannotContinue = 2
    ValidNoChange = 3
    ValidResults = [1, 2, 3]


class ComputationalDependency(Graph):
    """This dependency graph keeps the parent-child relationships between different
    computation functions in the `Process`. Each function is represented as a node where
    node key = function_name. An (Parent, Child) edge is use to represent the dependency.
    Do not create circular depedency. No protection is in place and it will crash.

    Validity
    --------
    Validity means the function's result is/was computed based on a state that has not changed.
    This avoids costly recomputations when a state is changed that only requires partial computation.


    Functions
    ---------
    Functions that are managed by this dependency must accept only one parameter `beam_id`.
    Functions can assume all upstream dependent functions returned "ok" when it is being called.

    The function must return `ComputationalResult.ValidCanContinue` if the computation can be completed,
    and the result permits dowmstream computation. If downstream is not possible, `ComputationalResult.ValidCannotContinue`
    must be returned. In either case, function will not be recomputed unless upstream function changes things.

    The function can return `ComputationalResult.ValidNoChange` if the computation can be completed
    but no changes are made to the state. This function is not implemented yet.

    External actions (eg. action by users) can set the state of a specific beam_id using `invalidate()` or
    `set_solution_validity() and trigger a down stream invalidation.
    This will not cause the depedent functions for the affected beam
    to be recomputed when next compute() is called.

    """

    def __init__(self, process=None):
        # type: (RobotClampAssemblyProcess) -> None
        """The dependency for RobotClampAssemblyProcess is hardcoded here.
        Nodes are optional, included here as list of functions for overview.
        Every Edge (Parent, Child) contains the relationship where Parent runs before Child.
        """

        super(ComputationalDependency, self).__init__()
        self.process = process

        # Clamp related computation
        self.add_node('assign_clamp_type_to_joints')
        self.add_node('search_valid_clamp_orientation_with_guiding_vector')
        self.add_node('compute_clamp_attachapproach_attachretract_detachapproach')
        self.add_node('compute_clamp_detachretract')
        self.add_node('search_valid_jawapproach_vector_prioritizing_beam_side')
        # Level 1
        # -------
        self.add_edge(
            'assign_clamp_type_to_joints',  # Top level
            'search_valid_clamp_orientation_with_guiding_vector'
        )

        # Level 2
        # -------
        function_names = [
            'compute_clamp_attachapproach_attachretract_detachapproach',
            'compute_clamp_detachretract',
            'search_valid_jawapproach_vector_prioritizing_beam_side',
        ]
        for f in function_names:
            self.add_edge('search_valid_clamp_orientation_with_guiding_vector', f)

        # Gripper Related Computation
        # ---------------------------
        # self.add_node('assign_gripper_to_beam')
        # Layer 1
        # -------
        # self.add_node('compute_gripper_grasp_pose')
        self.add_edge(
            'assign_gripper_to_beam',
            'compute_gripper_grasp_pose'
        )
        # Layer 2
        # -------
        # self.add_node('compute_pickup_location_at_corner_aligning_pickup_location')
        self.add_edge(
            'compute_gripper_grasp_pose',
            'compute_pickup_frame'
        )
        # Layer 3
        # -------
        # Things that are based on compute_pickup_frame
        function_names = [
            'compute_beam_pickupretract',
            'compute_beam_pickupapproach',
            'compute_beam_finalretract',
        ]
        for f in function_names:
            self.add_edge('compute_pickup_frame', f)

        # Compute Actions and Movements
        # -----------------------------
        self.add_edge('assign_clamp_type_to_joints', 'assign_tool_id_to_beam_joints')
        self.add_edge('assign_tool_id_to_beam_joints', 'create_actions_from_sequence')

        # Automatically compute all
        terminal_function_names = [
            # Clamp Computations
            'compute_clamp_attachapproach_attachretract_detachapproach',
            'compute_clamp_detachretract',
            'search_valid_jawapproach_vector_prioritizing_beam_side',
            # Gripper Computations
            'compute_beam_pickupretract',
            'compute_beam_pickupapproach',
            'compute_beam_finalretract',
            # Actions
            'create_actions_from_sequence',
        ]

        for f in terminal_function_names:
            self.add_edge(f, 'create_movements_from_action')

        self.add_edge('create_movements_from_action', 'compute_all')

        if self.process is not None and self.process.assembly is not None:
            for beam_id in self.process.assembly.beam_ids():
                self.update_default_node_attributes({beam_id: ComputationalResult.Invalid})

    def set_solution_validity(self, beam_id, fx, state, invalidate_downstream=True):
        # type: (str, function, int, bool) -> None
        """ Sets the solution's validity to a given value.
        If invalidate_downstream == True (default), all downstream computation will be invalidated.
        """
        fx_name = fx.__name__
        self.node_attribute(fx_name, beam_id, state)
        if invalidate_downstream:
            self.invalidate(beam_id, fx, downstream_only=True)

    def get_solution_validity(self, beam_id, fx):
        # type: (str, function) -> int
        """Checks the given computation if it is valid
        Invalid means a recompute is needed.
        Does not perform recursive check upstream.
        """
        fx_name = fx.__name__
        # Check self
        self_status = self.node_attribute(fx_name, beam_id)
        return self_status

    def get_downstream_fxs(self, fx):
        """ Get downstream fx(s) immediately below the given fx """
        for fx_name in self.neighbors_out(fx.__name__):
            yield getattr(self.process, fx_name)

    def get_uptream_fxs(self, fx):
        """ Get upstream fx(s) immediately below the given fx """
        for fx_name in self.neighbors_in(fx.__name__):
            yield getattr(self.process, fx_name)

    def invalidate(self, beam_id, fx, downstream_only=False):
        """ Recursive function to invalidate the given the function and
        all downstream functions.
        If `downstream_only` == True, only downstream are invalidated.
        """
        if not downstream_only:
            self.set_solution_validity(beam_id, fx, ComputationalResult.Invalid)
        for dfx in self.get_downstream_fxs(fx):
            self.invalidate(beam_id, dfx)

    def compute(self, beam_id, fx, attempt_all_parents_even_failure=False, verbose=True):
        """ Recursively compute all parents of this function and this function.
        Starting from the root of dependency, if a function is not valid, it will be recomputed.
        If any of the computation failed, it will not continue.
        The validity of rest of the dependency chain will be set to False.

        Returns
        -------
        True if all the computations upstream and the specified functions are completed.
        """
        fx_name = fx.__name__
        # Depth first to compute parent solutions first
        for ufx in self.get_uptream_fxs(fx):
            parent_status = self.get_solution_validity(beam_id, ufx)

            # If parent solution is not Valid, try compute it.
            if parent_status not in ComputationalResult.ValidResults:
                parent_status = self.compute(beam_id, ufx, attempt_all_parents_even_failure=attempt_all_parents_even_failure, verbose=verbose)
                # If any parent is invalid despite retry, we stop here
                if parent_status != ComputationalResult.ValidCanContinue:
                    if attempt_all_parents_even_failure:
                        continue  # Except if this flag is True
                    return False

        # If all partents are okay, compute itself if it is not valid
        validity = self.get_solution_validity(beam_id, fx)
        if validity not in ComputationalResult.ValidResults:
            validity = fx(beam_id)
            # Invalidate downsteram results.
            if verbose:
                print("Beam(%s) Dependency compute(%s) Validity = %s" % (beam_id, fx_name, validity))
            self.set_solution_validity(beam_id, fx, validity, invalidate_downstream=True)
        return validity in ComputationalResult.ValidResults

    def debug_print(self, beam_id):
        for key, data in self.nodes(data=True):
            print("%s: %s = %s" % (beam_id, key, data[beam_id]))

    def get_invalid_beam_ids(self):
        invalid_beams = []
        for beam_id in self.process.assembly.sequence:
            if not self.process.dependency.get_solution_validity(beam_id, self.process.compute_all) in ComputationalResult.ValidResults:
                invalid_beams.append(beam_id)
        return invalid_beams
