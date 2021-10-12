import datetime
import json
import os

import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext as sc  # type: ignore
import compas
import compas_fab
from compas.utilities import DataDecoder, DataEncoder

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.process_artist import ProcessArtist

compas.PRECISION = '12f'


def get_activedoc_path_no_ext():
    # type: () -> str
    activeDoc = Rhino.RhinoDoc.ActiveDoc  # type: Rhino.RhinoDoc
    path = os.path.splitext(activeDoc.Path)[0]
    return path


def get_activedoc_process_path():
    # type: () -> str
    json_path = get_activedoc_path_no_ext() + "_process.json"
    # print(json_path)
    return json_path


def get_process():
    # type: () -> RobotClampAssemblyProcess
    if "itj_process" in sc.sticky:
        return sc.sticky["itj_process"]
    else:
        return None


def get_process_artist():
    # type: () -> ProcessArtist
    if "itj_process_artist" in sc.sticky:
        return sc.sticky["itj_process_artist"]
    else:
        return None


def process_is_none(process):
    # type: (RobotClampAssemblyProcess) -> bool
    """Print message to user if the process file is None.
    Then Return True
    """
    if process is None:
        print("Warning: Process json file is not yet loaded.")
        return True
    return False


def load_process():
    # type: () -> bool
    """ Load json from the path next to the Rhino File
    """
    json_path = get_activedoc_process_path()
    exist = os.path.exists(json_path)
    if exist:
        with open(json_path, 'r') as f:
            process = json.load(f, cls=DataDecoder) #type: RobotClampAssemblyProcess

        # Also load RFL model from default location
        # process.load_robot_model() # Commented out for lazy loading

        sc.sticky["itj_process"] = process
        success = True
        # Crate new artist
        sc.sticky["itj_process_artist"] = ProcessArtist(process)
    else:
        sc.sticky["itj_process"] = None
        success = False
    return success


def create_new_process():
    # type: () -> RobotClampAssemblyProcess
    assembly = Assembly()
    process = RobotClampAssemblyProcess(assembly)
    sc.sticky["itj_process"] = process
    return process

######################
# Rhino Entry Point
######################
# Below is the functions that get evoked when user press UI Button
# Put this in the Rhino button ! _-RunPythonScript "integral_timber_joints.rhino.load.py"


if __name__ == "__main__":

    # Check if the default json path exist:
    json_path = get_activedoc_process_path()
    exist = os.path.exists(json_path)

    if exist:
        print("Compas Version: %s installed in: %s" % (compas.__version__, compas.__file__))
        print("Compas Fab Version: %s installed in: %s" % (compas_fab.__version__, compas_fab.__file__))
        load_process()
        c_time = datetime.datetime.fromtimestamp(os.path.getmtime(json_path))
        print("Process loaded from: %s (%i Beams). File last modified on %s." % (os.path.basename(json_path), len(list(get_process().assembly.beams())), c_time))

        # Draw beams to canvas
        artist = get_process_artist()
        artist.empty_layers()
        process = get_process()
        for beam_id in process.assembly.beam_ids():
            artist.redraw_interactive_beam(beam_id, force_update=False, redraw=False)
        # Enable Redraw
        rs.EnableRedraw(True)

    else:
        # Ask user if create new process file.
        from Rhino.Input.Custom import GetString
        getOption = GetString()
        getOption.SetCommandPrompt("Process json not found. Create new json? [y]es [n]o")
        getOption.SetDefaultString("y")
        getOption.AcceptNothing(True)

        result = getOption.Get()
        if result == Rhino.Input.GetResult.String:
            if getOption.StringResult().startswith('y'):
                process = create_new_process()
                print("New Process json Created")
                # Crate new artist
                sc.sticky["itj_process_artist"] = ProcessArtist(process)
