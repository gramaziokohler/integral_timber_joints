import json
import os
from copy import deepcopy

import rhinoscriptsyntax as rs
from compas.data import DataDecoder, DataEncoder

from integral_timber_joints.assembly import Assembly
# import integral_timber_joints.process as Process
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_activedoc_process_path, get_process, get_process_artist, process_is_none


def save_process_less(process):
    # type: (RobotClampAssemblyProcess) -> bool
    """ Save a process with less beams. Default to 12 beams
    """
    if process_is_none(process):
        return

    json_folder = os.path.dirname(get_activedoc_process_path())

    # Ask user what file name to use
    new_filename = rs.GetString("What file name to use? (do not include _process.json)", "twelve_pieces")
    new_json_path = os.path.join(json_folder, new_filename + "_process.json")

    # Ask user how many beams to save
    beams_to_keep = rs.GetInteger("How many beams to save", 12, 0, len(process.assembly.sequence))

    # Ask user what indent to use
    indent = rs.GetString("What json indent to Use?", "None", ["None", "Two", "Four"])
    if indent == "None":
        indent = None
    elif indent == "Two":
        indent = 2
    elif indent == "Four":
        indent = 4
    else:
        print("Error: Please select between: None / Two / Four")
        print("File Not Saved")
        return

    # Make a copy of process and remove beams
    process_copied = RobotClampAssemblyProcess.from_data(deepcopy(process.data))
    delete_beams_until(process_copied, beams_to_keep)

    process_copied.assign_unique_action_numbers()

    # I havent tried these optimization features so far but worth trying
    # process_copied.optimize_actions_place_pick_gripper()
    # process_copied.optimize_actions_place_pick_clamp()
    # process_copied.create_movements_from_action(verbose=False)

    # Save in new location
    with open(new_json_path, 'w') as f:
        json.dump(process_copied, f, cls=DataEncoder, indent=indent, sort_keys=True)
    print("New process with %s beams saved to: %s" % (beams_to_keep, new_json_path))

    return True


def delete_beams_until(process, beams_to_keep):
    # type: (RobotClampAssemblyProcess, int) -> RobotClampAssemblyProcess
    '''  Deleting until a number of beams are left
    '''
    assembly = process.assembly  # type: Assembly
    beam_ids = assembly.sequence[beams_to_keep:]

    # Figure out neighbours
    neighbors = []
    for beam_id in beam_ids:
        neighbors += assembly.neighbors(beam_id)
    neighbors = set(neighbors) - set(beam_ids)

    # Delete Beams and their joints
    for beam_id in beam_ids:
        assembly.remove_beam(beam_id)

    # Redraw neighbour beams since joints maybe gone.
    for beam_id in neighbors:
        assembly.get_beam_mesh_in_wcf(beam_id, use_cache_if_available=False)

    assert len(list(assembly.beams())) == beams_to_keep
    assert len(assembly.sequence) == beams_to_keep

######################
# Rhino Entry Point
######################
# Below is the functions that get evoked when user press UI Button
# Put this in the Rhino button ! _-RunPythonScript "integral_timber_joints.rhino.load.py"


if __name__ == "__main__":

    # Check if the default json path exist:
    process = get_process()
    if process_is_none(process):
        print("Load json first")
    else:
        save_process_less(process)
