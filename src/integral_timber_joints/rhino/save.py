import json
import os

from compas.utilities import DataDecoder, DataEncoder

from integral_timber_joints.assembly import Assembly
# import integral_timber_joints.process as Process
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_activedoc_process_path, get_process, get_process_artist, process_is_none


def save_process(process):
    # type: (RobotClampAssemblyProcess) -> bool
    """ Load json from the path next to the Rhino File
    """
    if process_is_none(process):
        return

    json_path = get_activedoc_process_path()
    exist = os.path.exists(json_path)
    # rename old file as backup
    if exist:
        # os.rename(json_path, get_activedoc_process_path())
        # f = open(json_path, 'r')
        # json_str = f.read()
        # f.close()
        # # Decode json for Process object
        # process = jsonpickle.decode(json_str, keys=True, classes=[RobotClampAssemblyProcess])
        # sc.sticky["itj_process"] = process
        # success = True
        pass

    with open(json_path, 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=4)

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
