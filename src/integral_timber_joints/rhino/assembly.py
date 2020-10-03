from compas_rhino.ui import CommandMenu

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, process_is_none

######################
# Rhino Entry Point
######################
# Below is the functions that get evoked when user press UI Button
# Put this in the Rhino button ! _-RunPythonScript 'integral_timber_joints.rhino.load.py'

def ui_add_beam_from_lines(process):
    # 
    print ('ui_add_beam_from_lines')

def something(process):
    # 
    print ('something')

def show_menu(process):
    assembly = process.assembly # type: Assembly
    config = {
        'message': 'Assembly contains %i Beams %i Joints:' % (len(list(assembly.beams())), len(list(assembly.joints())) / 2) ,
        'options': [
            {'name': 'AddBeam', 'message': 'Add Beams ...', 'options': [
                {'name': 'From Lines', 'action': ui_add_beam_from_lines},
                {'name': 'Continuous', 'action': something},
                {'name': 'Back', 'action': 'Back'},
            ]},
            {'name': 'DeleteBeam', 'message': 'Delete Beams', 'options': [
                {'name': 'Boundary', 'action': something},
                {'name': 'Continuous', 'action': something},
                {'name': 'Back', 'action': 'Back'},
            ]},
            {'name': 'Exit', 'message': 'Select Faces', 'action': 'Exit'
            }
            ]
    }
    menu = CommandMenu(config)

    while (True):
        action = menu.select_action()['action']
        # print(action)
        if action == 'Exit':
            print ('Exit')
            return
        if action == 'Back':
            continue    
        else:
            action(process)


if __name__ == '__main__':
    process = get_process()
    if process_is_none(process):
        print ("Load json first")
    else:
        show_menu(process)


