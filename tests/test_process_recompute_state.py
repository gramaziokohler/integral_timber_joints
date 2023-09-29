
import os
import sys
import time
import json
import datetime
import timeit
from compas.data import DataDecoder, DataEncoder
from integral_timber_joints.process import RobotClampAssemblyProcess


# choose timer to use
if sys.platform.startswith('win'):
    default_timer = time.clock
else:
    default_timer = time.time


def load_process(json_path):
    # type: (str) -> RobotClampAssemblyProcess
    exist = os.path.exists(json_path)
    if exist:
        with open(json_path, 'r') as f:
            process = json.load(f, cls=DataDecoder)
            c_time = datetime.datetime.fromtimestamp(os.path.getmtime(json_path))
            print("Process loaded from: %s (%i Beams). File last modified on %s." % (os.path.basename(json_path), len(list(process.assembly.beams())), c_time))
            return process
    else:
        print("json_path not exist")


def test_compute_states(process):
    # type: (RobotClampAssemblyProcess) -> None

    # Reset Dependency Graph
    from integral_timber_joints.process import ComputationalDependency
    process.attributes['dependency'] = ComputationalDependency(process)
    # Recompute Initial State
    process.recompute_initial_state(verbose=False)
    # Recompute all other stuff
    # Make sure everything is computed and nothing is missing
    for beam_id in process.assembly.sequence:
        process.dependency.compute_all(beam_id, attempt_all_parents_even_failure=True, verbose=True)

    invalid_beams = process.dependency.get_invalid_beam_ids()
    if len(invalid_beams) > 0:
        print("Warning: The following beams are not yet computationally valid: %s" % invalid_beams)
        print("States are not valid.")
        return

    # Assign unique numbers across the entire process file.
    process.assign_unique_action_numbers()

    # Print out some information to user
    diff_count = 0
    for movement in process.movements:
        diff_count += len(movement.state_diff)
    print("Total: %i diffs computed for %i object states." % (diff_count, len(process.initial_state)))
    print("Total: %i Movements in %i Acttions for %i Beams." % (len(process.movements), len(process.actions), len(process.assembly.sequence)))

    # import cProfile
    # cProfile.run('[process.dependency.compute_all(beam_id, verbose=False) for beam_id in process.assembly.sequence]', sort="cumulative")


def save_and_load_process(process):
    s = json.dumps(process, cls=DataEncoder, sort_keys=True)
    p2 = json.loads(s, cls=DataDecoder)


if __name__ == "__main__":
    process = load_process(r"C:\Users\leungp\Documents\GitHub\integral_timber_joints\external\itj_design_study\210419_AnticlasticShelter\shelter_process.json")
    test_compute_states(process)

    # movement = process.get_movements_by_beam_id('b1')[0]
    # scene = process.get_movement_start_scene(movement)

    # s = json.dumps(process, cls=DataEncoder, sort_keys=True)
    # p2 = json.loads(s, cls=DataDecoder)
