import Rhino  # type: ignore
import rhinoscriptsyntax as rs
from compas.geometry.primitives.frame import Frame
from compas.geometry.primitives.plane import Plane
from compas_rhino.ui import CommandMenu
from compas_rhino.utilities.objects import get_object_name, get_object_names

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions


def ui_trim_beams(process):
    # type: (RobotClampAssemblyProcess) -> None
    '''Ask User to select beams for adding a beam_cut

    3 points are asked from the user to determine the cut plane.
    The Z-positive / normal direction is removed away.

    THis functions repeats until users press Enter without selecting beam.
    '''
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    while(True):
        # Ask user for input
        guids = rs.GetObjects('Select beam(s) to trim:', custom_filter=get_existing_beams_filter(process))
        if not guids:
            # Quit when user press Enter without selection
            return
        beam_ids = get_object_names(guids)

        # Ask user to input 3 points for defining a plane
        origin = rs.GetPoint("Pick 3 points to define plane. Point 1:")
        if origin is None:
            return
        x_point = rs.GetPoint("Pick 3 points to define plane. Point 2:")
        if x_point is None:
            return
        y_point = rs.GetPoint("Pick 3 points to define plane. Point 3:")
        if y_point is None:
            return
        frame = Frame(origin, x_point - origin, y_point - origin)
        plane = Plane.from_frame(frame)

        # Update drownstream computation
        for beam_id in beam_ids:
            assembly.add_ocf_beam_cut_from_wcf_plane(beam_id, plane.copy())
            print('Trim added. Beam (%s) now contain %i BeamCuts.' % (beam_id, len(assembly.beam_cuts(beam_id))))
            artist.redraw_interactive_beam(beam_id)

        # recompute_dependent_solutions(process, beam_id)
        # anything?


def ui_untrim_beams(process):
    # type: (RobotClampAssemblyProcess) -> None
    '''Removes all the Beamcut objects.
    '''
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    # Ask user for input
    guids = rs.GetObjects('Select beam(s) to untrim. (All End trims will be removed):', custom_filter=get_existing_beams_filter(process))
    if not guids:
        # Quit when user press Enter without selection
        return
    beam_ids = get_object_names(guids)

    # Update drownstream computation
    for beam_id in beam_ids:
        assembly.remove_all_beam_cuts(beam_id)
        print('Trim removed. Beam (%s) now contain %i BeamCuts.' % (beam_id, len(assembly.beam_cuts(beam_id))))
        artist.redraw_interactive_beam(beam_id)

    # recompute_dependent_solutions(process, beam_id)
    # anything?


def show_assembly_beams(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()
    artist.show_interactive_beam_until('')


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly

    # Activate assembly menu colour code:
    show_assembly_beams(process)

    while (True):
        # Create Menu
        config = {
            'message': 'Trim or Untrim Beams:',
            'options': [
                {'name': 'Finish', 'action': 'Exit'
                 },
                {'name': 'TrimBeam', 'action': ui_trim_beams},
                {'name': 'UnTrimBeam', 'action': ui_untrim_beams},
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


if __name__ == '__main__':
    process = get_process()
    if process_is_none(process):
        print("Load json first")
    else:
        show_menu(process)
