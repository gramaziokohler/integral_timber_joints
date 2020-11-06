import Rhino
import rhinoscriptsyntax as rs
from compas_rhino.ui import CommandMenu
from compas_rhino.utilities.objects import get_object_name

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none

def something(process):
    #
    print('something not implemented')

def create_robot_wrist(process):
    # Ask user for a list of input

    # Create object and save that to json

    # Ask user to save it somewhere
    pass

def show_menu(process):
    # Create Menu
    config = {
        'message': 'Create Clamps, Grippers, ToolChanger, RobotWrist:',
        'options': [

            {'name': 'CreateClampCL3', 'action': something
                },
            {'name': 'CreateGripperPG', 'action': something
                },
            {'name': 'CreateToolChanger', 'action': something
                },
            {'name': 'CreateRobotWrist', 'action': create_robot_wrist
                },

        ]

    }

    result = CommandMenu(config).select_action()
    # User cancel command by Escape
    if result is None or 'action' not in result:
        print('Exit Function')
        return Rhino.Commands.Result.Cancel

    # Run the selected command
    result['action'](process)


######################
# Rhino Entry Point
######################
# Below is the functions that get evoked when user press UI Button
# Put this in the Rhino button ! _-RunPythonScript 'integral_timber_joints.rhino.tools_environment.py'
if __name__ == '__main__':
    process = get_process()
    if process_is_none(process):
        print("Load json first")
    else:
        show_menu(process)
