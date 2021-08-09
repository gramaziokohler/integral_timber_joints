
import os
import sys
import time
import json
import datetime
import timeit
from compas.utilities import DataDecoder, DataEncoder
from integral_timber_joints.process.algorithms import recompute_initial_state
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
    # Reset Dependency Graph
    from integral_timber_joints.process import ComputationalDependency
    process.attributes['dependency'] = ComputationalDependency(process)
    # Recompute Initial State
    recompute_initial_state(process, verbose=False)
    # Recompute all other stuff
    [process.dependency.compute(beam_id, process.compute_all, verbose=False) for beam_id in process.assembly.sequence]
    # import cProfile
    # cProfile.run('[process.dependency.compute(beam_id, process.compute_all, verbose=False) for beam_id in process.assembly.sequence]', sort="cumulative")


def save_and_load_process(process):
    s = json.dumps(process, cls=DataEncoder, sort_keys=True)
    p2 = json.loads(s, cls=DataDecoder)


if __name__ == "__main__":
    process = load_process(r"C:\Users\leungp\Documents\GitHub\integral_timber_joints\external\itj_design_study\210128_RemodelFredPavilion\twelve_pieces_process.json")
    test_compute_states(process)

    movement = process.get_movements_by_beam_id('b1')[0]
    scene = process.get_movement_start_scene(movement)

    # s = json.dumps(process, cls=DataEncoder, sort_keys=True)
    # p2 = json.loads(s, cls=DataDecoder)
