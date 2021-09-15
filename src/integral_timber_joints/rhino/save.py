import json
import os

import rhinoscriptsyntax as rs
from compas.utilities import DataDecoder, DataEncoder

from integral_timber_joints.assembly import Assembly
# import integral_timber_joints.process as Process
from integral_timber_joints.process import RobotClampAssemblyProcess, RoboticMovement
from integral_timber_joints.rhino.load import get_activedoc_process_path, get_process, get_process_artist, process_is_none


def save_process(process):
    # type: (RobotClampAssemblyProcess) -> bool
    """ Load json from the path next to the Rhino File
    """
    if process_is_none(process):
        return

    json_path = get_activedoc_process_path()
    exist = os.path.exists(json_path)

    # * Check if any of the movements have trajectory. If so, do not allow saving
    if any(m.trajectory is not None for m in process.movements if isinstance(m, RoboticMovement)):
        print("-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -")
        print("Error: Some Robotic Movements contain Trajectory. ")
        print("File cannot be Saved")
        print("-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -")
        return


    # Ask user what indent to use
    indent = rs.GetString("What json indent to Use?", "Four", ["None", "Two", "Four"])
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

    with open(json_path, 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=indent, sort_keys=True)

    return True


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
        save_process(process)
