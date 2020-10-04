from compas.geometry import Vector
from compas_rhino.ui import CommandMenu
from compas_rhino.utilities.objects import get_lines, select_lines


from compas_rhino.geometry import RhinoLine, RhinoCurve
from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none

import rhinoscriptsyntax as rs

def ui_add_beam_from_lines(process):
    ''' Ask user for line(s) to create new beams.

    '''
    # ask user to pick lines
    # guids = select_lines("Select Lines (no curve or polyline)")
    guids = rs.GetObjects("Select Lines (not curve or polyline)", filter=rs.filter.curve)

    print (guids)
    width = 100
    height = 100
    
    # Create Beams form lines
    beams = []
    assembly = get_process().assembly
    for guid in guids:
        rhinoline = RhinoCurve.from_guid(guid) # RhinoLine is not implemented sigh... RhinoCurve is used.
        centerline = rhinoline.to_compas()
        # Compute guide vector: For vertical lines, guide vector points to world X 
        # Otherwise, guide vector points to Z,
        if centerline.start.z == centerline.start.z:
            guide_vector = Vector(1,0,0) 
        else:
            guide_vector = Vector(0,0,1)
        
        # Create Beam object and add to assembly
        beam = Beam.from_centerline(centerline, guide_vector, width, height)
        beam.name = assembly.get_new_beam_id()
        assembly.add_beam(beam)
        print ('New Beam: %s'% beam.name)
        beams.append(beam)

    print ('%i Beams added to the assembly.' % len(beams))

    # # Draw newly added beams
    artist = get_process_artist()
    for beam in beams:
        beam_id = beam.name
        artist.draw_beam_mesh(beam_id)

def something(process):
    # 
    print ('something')

def show_menu(process):
    assembly = process.assembly # type: Assembly
    config = {
        'message': 'Assembly contains %i Beams %i Joints:' % (len(list(assembly.beams())), len(list(assembly.joints())) / 2) ,
        'options': [
            {'name': 'Finish', 'action': 'Exit'
            },
            {'name': 'AddBeam', 'message': 'Add Beams ...', 'options': [
                {'name': 'Back', 'action': 'Back'},
                {'name': 'FromLines', 'action': ui_add_beam_from_lines},
                {'name': 'FromMesh', 'action': something},
            ]},
            {'name': 'DeleteBeam', 'message': 'Delete Beams', 'options': [
                {'name': 'Back', 'action': 'Back'},
                {'name': 'Boundary', 'action': something},
                {'name': 'Continuous', 'action': something},
            ]},
            ]
    }
    menu = CommandMenu(config)

    while (True):
        result = menu.select_action()
        # User cancel command by Escape
        if result is None or 'action' not in result:
            print ('Exit Function')
            return
        action = result['action']
        # print(action)
        # User click Exit Button
        if action == 'Exit':
            print ('Exit Function')
            return
        if action == 'Back':
            continue    
        else:
            action(process)

######################
# Rhino Entry Point
######################
# Below is the functions that get evoked when user press UI Button
# Put this in the Rhino button ! _-RunPythonScript 'integral_timber_joints.rhino.load.py'

if __name__ == '__main__':
    process = get_process()
    if process_is_none(process):
        print ("Load json first")
    else:
        show_menu(process)


