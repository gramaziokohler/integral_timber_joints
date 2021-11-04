import Rhino  # type: ignore
import Rhino.Geometry as rg
import rhinoscriptsyntax as rs
from compas.geometry import Frame, Plane, project_point_line, distance_point_point
from compas_rhino.ui import CommandMenu
from compas_rhino.utilities.objects import get_object_name, get_object_names

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry import BeamcutFourCorners, BeamcutPlateSlot
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
            artist.redraw_interactive_beam(beam_id, redraw=False)

        rs.EnableRedraw(True)


def ui_trim_four_corners(process):
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
        choices = ["Start", "End", "Both"]
        side = rs.GetString("Which side to trim corners:", "Both", choices)
        if side is None or side not in choices:
            return
        dist_x = rs.GetReal("Distance on X", number=600, minimum=1, maximum=None)
        if dist_x is None:
            return
        dist_y = rs.GetReal("Distance on Y", number=21.1, minimum=1, maximum=None)
        if dist_y is None:
            return
        dist_z = rs.GetReal("Distance on Z", number=36.6, minimum=1, maximum=None)
        if dist_z is None:
            return

        for beam_id in beam_ids:
            if side == "Start" or side == "Both":
                beamcut = BeamcutFourCorners(True, dist_x=dist_x, dist_y=dist_y, dist_z=dist_z)
                assembly.add_beam_cut(beam_id, beamcut)

            if side == "End" or side == "Both":
                beamcut = BeamcutFourCorners(False, dist_x=dist_x, dist_y=dist_y, dist_z=dist_z)
                assembly.add_beam_cut(beam_id, beamcut)

            print('Trim added. Beam (%s) now contain %i BeamCuts.' % (beam_id, len(assembly.beam_cuts(beam_id))))
            artist.redraw_interactive_beam(beam_id, redraw=False)
        rs.EnableRedraw(True)


def ui_adjust_four_corners(process):
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
        guids = rs.GetObjects('Select beam(s) to adjust End Corner Trim:', custom_filter=get_existing_beams_filter(process))
        if not guids:
            # Quit when user press Enter without selection
            return
        beam_ids = get_object_names(guids)

        # Ask user to input 3 points for defining a plane
        dist_x = rs.GetReal("Distance on X", number=400, minimum=1, maximum=None)
        if dist_x is None:
            return
        dist_y = rs.GetReal("Distance on Y", number=21.1, minimum=1, maximum=None)
        if dist_y is None:
            return
        dist_z = rs.GetReal("Distance on Z", number=36.6, minimum=1, maximum=None)
        if dist_z is None:
            return

        for beam_id in beam_ids:
            for beam_cut in assembly.beam_cuts(beam_id):
                if isinstance(beam_cut, BeamcutFourCorners):
                    beam_cut.dist_x = dist_x
                    beam_cut.dist_y = dist_y
                    beam_cut.dist_z = dist_z
                    print('BeamcutFourCorners on Beam (%s) now have parameters (%s, %s, %s)' % (beam_id, dist_x, dist_y, dist_z))
            artist.redraw_interactive_beam(beam_id, redraw=False)
        rs.EnableRedraw(True)


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


def ui_add_plate_slot(process):
    # type: (RobotClampAssemblyProcess) -> None
    '''Ask User to select beams for adding BeamcutPlateSlot for metal plate

    User can choose:
    - one of the four faces to add.
    - length of the slot (How deep the metal plate)
    - width of the slot (Thickness of metal plate)

    This functions repeats until users press Enter or Esc without selecting beam.
    '''
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()


    while(True):
        # Ask user for input
        guids = rs.GetObjects('Select beam(s) to add plate slot:', custom_filter=get_existing_beams_filter(process))
        if not guids:
            # Quit when user press Enter without selection
            return
        beam_ids = get_object_names(guids)

        # * Ask user to select a point
        beam = assembly.beam(beam_ids[0])
        center_line = beam.get_center_line()

        center_line_r = rs.AddLine(rg.Point3d(* list(center_line.start)), rg.Point3d(* list(center_line.end)))
        pt_r = rs.GetPointOnCurve(center_line_r, "Select Height of the plate")
        rs.DeleteObject(center_line_r)
        projected_point = project_point_line([pt_r.X, pt_r.Y, pt_r.Z], center_line)
        length_from_start = distance_point_point(projected_point, center_line.start)
        length_from_end = distance_point_point(projected_point, center_line.end)

        # * Ask user for input
        options = ["SlotThroughTag", "SlotParallelToTag"]
        option = rs.GetString("Which Direction to slot:", options[0], options)

        on_beam_start = length_from_start < length_from_end

        if option == "SlotThroughTag":
            face_id = 1
        elif option == "SlotParallelToTag":
            face_id = 2
        else:
            print("Input not valid: %s" % option)
            return

        length = min(length_from_start, length_from_end)

        width = rs.GetReal("Width of the slot, aka how thick is the metal plate:?", 8, minimum = 1)
        if width is None:
            return

        # * Remove the end cuts
        for beam_id in beam_ids:
            beam_cut = BeamcutPlateSlot(on_beam_start=on_beam_start, face_id=face_id, length=length, width=width)
            assembly.add_beam_cut(beam_id, beam_cut)

            print('Plate Slot added. Beam (%s) now contain %i BeamCuts.' % (beam_id, len(assembly.beam_cuts(beam_id))))

            # Update drownstream computation
            artist.redraw_interactive_beam(beam_id, redraw=False)

        rs.EnableRedraw(True)

def ui_remove_plate_slot(process):
    # type: (RobotClampAssemblyProcess) -> None
    '''Removes all the Beamcut objects.
    '''
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    # * Ask user for input
    guids = rs.GetObjects('Select beam(s) to untrim. (All End trims will be removed):', custom_filter=get_existing_beams_filter(process))
    if not guids:
        # Quit when user press Enter without selection
        return
    beam_ids = get_object_names(guids)

    # * Remove the end cuts
    for beam_id in beam_ids:
        beam_cuts_to_delete = [beam_cut for beam_cut in assembly.beam_cuts(beam_id) if type(beam_cut) == BeamcutPlateSlot]
        print('%i BeamcutPlateSlot removed. Beam (%s) now contain %i BeamCuts.' % (len(beam_cuts_to_delete), beam_id, len(assembly.beam_cuts(beam_id))))
        for beam_cut in beam_cuts_to_delete:
            assembly.beam_cuts(beam_id).remove(beam_cut)

        artist.redraw_interactive_beam(beam_id, redraw=True)

def ui_rotate_plate_slot(process):
    # type: (RobotClampAssemblyProcess) -> None
    '''Rotate all the Beamcut objects.
    Toggle between face 1 or 2
    '''
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    # * Ask user for input
    guids = rs.GetObjects('Select beam(s) to untrim. (All End trims will be removed):', custom_filter=get_existing_beams_filter(process))
    if not guids:
        # Quit when user press Enter without selection
        return
    beam_ids = get_object_names(guids)

    # * Remove the end cuts
    for beam_id in beam_ids:
        beam_cuts = [beam_cut for beam_cut in assembly.beam_cuts(beam_id) if type(beam_cut) == BeamcutPlateSlot]
        print('%i BeamcutPlateSlot Rotated for beam %s.' % (len(beam_cuts), beam_id))
        for beam_cut in beam_cuts:
            if beam_cut.face_id == 1:
                beam_cut.face_id = 2
            else:
                beam_cut.face_id = 1

        artist.redraw_interactive_beam(beam_id, redraw=True)


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
                {'name': 'TrimFourCorners', 'action': ui_trim_four_corners},
                {'name': 'AdjustFourCorners', 'action': ui_adjust_four_corners},
                {'name': 'EndPlateSlot', 'action': ui_add_plate_slot},
                {'name': 'RotateEPSlot', 'action': ui_rotate_plate_slot},
                {'name': 'RemoveEPSlot', 'action': ui_remove_plate_slot},
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
