import Rhino
import rhinoscriptsyntax as rs
from compas.geometry import Vector
from compas_rhino.geometry import RhinoCurve
from compas_rhino.ui import CommandMenu
from compas_rhino.utilities.objects import get_object_name, get_object_names

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.geometry.joint_halflap import Joint_halflap_from_beam_beam_intersection
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions


def ui_add_beam_from_lines(process):
    # type: (RobotClampAssemblyProcess) -> None
    ''' Ask user for line(s) to create new beams.
    '''
    # ask user to pick lines
    # guids = select_lines("Select Lines (no curve or polyline)")
    guids = rs.GetObjects("Select Lines (not curve or polyline)", filter=rs.filter.curve)

    print(guids)
    width = 100
    height = 100

    # Create Beams form lines
    new_beam_ids = []
    affected_neighbours = []
    assembly = process.assembly  # type: Assembly
    for guid in guids:
        rhinoline = RhinoCurve.from_guid(guid)  # RhinoLine is not implemented sigh... RhinoCurve is used.
        centerline = rhinoline.to_compas()
        # Compute guide vector: For vertical lines, guide vector points to world X
        # Otherwise, guide vector points to Z,
        if centerline.start.z == centerline.start.z:
            guide_vector = Vector(1, 0, 0)
        else:
            guide_vector = Vector(0, 0, 1)

        # Create Beam object
        beam_id = assembly.get_new_beam_id()
        beam = Beam.from_centerline(centerline, guide_vector, width, height)
        beam.name = beam_id
        print('New Beam: %s' % beam_id)

        # Add to assembly
        assembly.add_beam(beam)
        new_beam_ids.append(beam_id)

        # Check for new Joints
        for exist_beam in assembly.beams():
            if beam == exist_beam:
                continue
            print('Checking for Joint : %s-%s' % (beam_id, exist_beam.name))
            j1, j2 = Joint_halflap_from_beam_beam_intersection(beam, exist_beam)
            # faces = beam.get_beam_beam_coplanar_face_ids(exist_beam)
            if j1 is not None and j2 is not None:
                print('New Joint : %s-%s' % (beam_id, exist_beam.name))
                assembly.add_joint_pair(j1, j2, beam_id, exist_beam.name)
                affected_neighbours.append(exist_beam.name)
                print('New Joint added')

    for beam_id in set(affected_neighbours + new_beam_ids):
        recompute_dependent_solutions(process, beam_id)

    print('Show Assembly color')
    # Draw newly added beams and neighbours affected by new joint
    show_assembly_color(process, set(affected_neighbours + new_beam_ids), redraw=True)

    print('%i Beams added to the assembly.' % len(new_beam_ids))


def ui_delete_beams(process):
    # type: (RobotClampAssemblyProcess) -> None
    '''Ask User to select beams for deleting.
    Deleting them from Process.Assembly.
    '''
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    # Ask user for input
    guids = rs.GetObjects('Select beams to delete:', custom_filter=get_existing_beams_filter(process))
    if not guids:
        return
    beam_ids = get_object_names(guids)

    # Figure out neighbours
    neighbors = []
    for beam_id in beam_ids:
        neighbors += assembly.neighbors(beam_id)
    neighbors = set(neighbors) - set(beam_ids)
    print('neighbors = %s' % neighbors)

    # Delete Beams and their joints
    for beam_id in beam_ids:
        artist.delete_interactive_beam_visualization(beam_id)
        # Maybe need to delete grippers and other things
        assembly.remove_beam(beam_id)
        print('Beam Removed: %s' % beam_id)

    # Redraw neighbour beams since joints maybe gone.
    show_assembly_color(process, neighbors, redraw=True)

    print('Neighbour Beam Affected: %s' % neighbors)


def ui_flip_beams(process):
    # type: (RobotClampAssemblyProcess) -> None
    '''Ask User to select a beam for flipping the joints' direction
    Only joints to earlier beams are flipped.

    THis functions repeats until users press Enter without selecting beam.
    '''
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    while(True):
        # Ask user for input
        guids = rs.GetObject('Select beams to flip:', custom_filter=get_existing_beams_filter(process))
        if not guids:
            # Quit when user press Enter without selection
            return
        beam_id = get_object_name(guids)

        # Loop though alread_built_neighbors and swap joint
        earlier_neighbors = assembly.get_already_built_neighbors(beam_id)
        for neighbour_id in earlier_neighbors:
            assembly.flip_lap_joint((beam_id, neighbour_id))

        # Update drownstream computation
        assembly.set_beam_attribute(beam_id, 'assembly_wcf_final', None)
        recompute_dependent_solutions(process, beam_id)

        # Update visualization of flipped beam and neighbors
        show_assembly_color(process, earlier_neighbors + [beam_id], redraw=True)

        print('Beam flipped: %s (Neighbours: %s)' % (beam_id, earlier_neighbors))


def something(process):
    #
    print('something')


def ui_seq_move_before(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    # Ask user to pick a beam to move
    guid = rs.GetObject('Select beams to change sequence:', custom_filter=get_existing_beams_filter(process))
    if not guid:
        # Quit when user press Enter without selection
        return
    beam_id_tomove = get_object_name(guid)

    # Ask user to pick a target beam to insert before it
    guid = rs.GetObject('Select beams to place after:', custom_filter=get_existing_beams_filter(process))
    if not guid:
        # Quit when user press Enter without selection
        return
    beam_id_after = get_object_name(guid)

    # Compute the amount of shift and shift_beam_sequence
    get_beam_sequence = assembly.get_beam_sequence
    shift_amount = get_beam_sequence(beam_id_after) - get_beam_sequence(beam_id_tomove)
    if shift_amount < 0:
        shift_amount += 1
    print("shift_amount %s" % shift_amount)
    member_seq_affected_by_swap = assembly.shift_beam_sequence(beam_id_tomove, shift_amount)

    # Redraw affected members
    show_assembly_color(process, member_seq_affected_by_swap)


def show_assembly_color(process, beam_ids=None, redraw=False):
    """Activate assembly menu colour code.
    Problematic beams that cannot be assembled are highlighted
    """
    if beam_ids == None:
        beam_ids = process.assembly.sequence
    artist = get_process_artist()
    rs.EnableRedraw(False)
    for beam_id in beam_ids:
        if redraw:
            artist.redraw_interactive_beam(beam_id)
        if process.assembly.beam_problems(beam_id):
            artist.change_interactive_beam_colour(beam_id, 'warning')
        else:
            artist.change_interactive_beam_colour(beam_id, 'active')
    rs.EnableRedraw(True)


def hide_assembly_color(process, beam_ids=None):
    """Deactivate assembly menu colour code.
    """
    if beam_ids == None:
        beam_ids = process.assembly.sequence
    artist = get_process_artist()
    rs.EnableRedraw(False)
    for beam_id in beam_ids:
        artist.change_interactive_beam_colour(beam_id, 'normal')
    rs.EnableRedraw(True)


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly

    # Activate assembly menu colour code:
    show_assembly_color(process)

    while (True):
        # Create Menu
        config = {
            'message': 'Assembly contains %i Beams %i Joints:' % (len(list(assembly.beams())), len(list(assembly.joints())) / 2),
            'options': [
                {'name': 'Finish', 'action': 'Exit'
                 },
                {'name': 'AddBeam', 'message': 'Add Beams ...', 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'FromLines', 'action': ui_add_beam_from_lines},
                    {'name': 'FromMesh', 'action': something},
                ]},
                {'name': 'DeleteBeam', 'action': ui_delete_beams},
                {'name': 'FlipBeamAssemblyDirection', 'action': ui_flip_beams},
                {'name': 'Sequence', 'message': 'Add Beams ...', 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'Shift', 'action': ui_seq_move_before},
                ]},
            ]

        }

        result = CommandMenu(config).select_action()
        # User cancel command by Escape
        if result is None or 'action' not in result:
            print('Exit Function')
            return Rhino.Commands.Result.Cancel

        action = result['action']
        # print(action)
        # User click Exit Button
        if action == 'Exit':
            print('Exit Function')
            return Rhino.Commands.Result.Cancel
        if action == 'Back':
            continue
        else:
            action(process)

    # Deactivate assembly menu colour code:
    hide_assembly_color(process)

######################
# Rhino Entry Point
######################
# Below is the functions that get evoked when user press UI Button
# Put this in the Rhino button ! _-RunPythonScript 'integral_timber_joints.rhino.load.py'


if __name__ == '__main__':
    process = get_process()
    if process_is_none(process):
        print("Load json first")
    else:
        show_menu(process)
