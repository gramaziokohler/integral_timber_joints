import Rhino
import rhinoscriptsyntax as rs
from compas_rhino.utilities.objects import get_object_name

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions


def show_sequence_color(process, beam_id):
    # type: (RobotClampAssemblyProcess, str) -> None
    rs.EnableRedraw(False)
    artist = get_process_artist()
    assembly = process.assembly  # type: Assembly
    selected_seq_num = assembly.get_beam_sequence(beam_id)
    for seq, beam_id in enumerate(assembly.sequence):
        # Change color based on problems
        problem = assembly.beam_problems(beam_id)
        if seq == selected_seq_num and not problem:
            artist.change_interactive_beam_colour(beam_id, 'active')
        if seq == selected_seq_num and problem:
            artist.change_interactive_beam_colour(beam_id, 'error')
        if seq < selected_seq_num and not problem:
            artist.change_interactive_beam_colour(beam_id, 'built')
        if seq < selected_seq_num and problem:
            artist.change_interactive_beam_colour(beam_id, 'built_warning')
        if seq > selected_seq_num and not problem:
            artist.change_interactive_beam_colour(beam_id, 'unbuilt')
        if seq > selected_seq_num and problem:
            artist.change_interactive_beam_colour(beam_id, 'unbuilt_warning')
        # Hide unbuilt beams if settings show_unbuilt == False
        if seq > selected_seq_num and (getattr(artist, "show_unbuilt", True) == False):
            artist.hide_interactive_beam(beam_id)
        else:
            artist.show_interactive_beam(beam_id)
        # print out for debug
        if problem:
            print (beam_id, problem)
    rs.EnableRedraw(True)


def show_normal_color_and_unhide(process):
    artist = get_process_artist()
    assembly = process.assembly  # type: Assembly
    rs.EnableRedraw(False)
    for beam_id in assembly.sequence:
        artist.change_interactive_beam_colour(beam_id, 'normal')
        artist.show_interactive_beam(beam_id)
    rs.EnableRedraw(True)


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    artist.selected_beam_id = None

    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt("Select beam")
    go.EnablePreSelect(True, True)

    def select_next():
        """ Function invoked by user to change active element to the next one.
        """
        selected_seq_num = assembly.get_beam_sequence(artist.selected_beam_id)
        selected_seq_num = min(selected_seq_num + 1,  len(assembly.sequence) - 1)
        artist.selected_beam_id = assembly.sequence[selected_seq_num]
        show_sequence_color(process, artist.selected_beam_id)
        go.SetDefaultString("Enter again = Next")

    def select_previous():
        """ Function invoked by user to change active element to the previous one.
        """
        selected_seq_num = assembly.get_beam_sequence(artist.selected_beam_id)
        selected_seq_num = max(selected_seq_num - 1,  0)
        artist.selected_beam_id = assembly.sequence[selected_seq_num]
        show_sequence_color(process, artist.selected_beam_id)
        go.SetDefaultString("Enter again = Previous")

    def move_earlier():
        """ Function invoked by user to move the active element's sequence one step earrlier.
        """
        if assembly.get_beam_sequence(artist.selected_beam_id) == 0:
            return
        member_seq_affected_by_swap = assembly.shift_beam_sequence(artist.selected_beam_id, -1)
        [artist.redraw_interactive_beam(beam_id, force_update=True, redraw = False) for beam_id in member_seq_affected_by_swap]
        show_sequence_color(process, artist.selected_beam_id)
        rs.EnableRedraw(True)
        go.SetDefaultString("Enter again = MoveEarlier")

    def move_later():
        """ Function invoked by user to move the active element's sequence one step later.
        """
        if assembly.get_beam_sequence(artist.selected_beam_id) == len(assembly.sequence) -1:
            return
        member_seq_affected_by_swap = assembly.shift_beam_sequence(artist.selected_beam_id, +1)
        [artist.redraw_interactive_beam(beam_id, force_update=True, redraw = False) for beam_id in member_seq_affected_by_swap]
        show_sequence_color(process, artist.selected_beam_id)
        rs.EnableRedraw(True)
        go.SetDefaultString("Enter again = MoveLater")

    def change_next_element():
        """ Function invoked by user to select an element and move that sequence 
        to after the the active element's.
        """
        original_seq = assembly.get_beam_sequence(artist.selected_beam_id)
        # Ask user for another beam
        go2 = Rhino.Input.Custom.GetObject()
        go2.SetCommandPrompt("Select beam to be the next element in sequence")
        go2.EnablePreSelect(False, False)
        go2.SetCustomGeometryFilter(get_existing_beams_filter(process, exclude_beam_ids=[artist.selected_beam_id]))
        result = go2.Get()
        if result == Rhino.Input.GetResult.Object:
            guid = go2.Object(0).ObjectId
            beam_id = get_object_name(guid)
            picked_seq = assembly.get_beam_sequence(beam_id)
            shift = original_seq - picked_seq
            if shift < 0:
                shift = shift + 1
            member_seq_affected_by_swap = assembly.shift_beam_sequence(beam_id, shift)
            artist.selected_beam_id = beam_id
            [artist.redraw_interactive_beam(beam_id, force_update=True, redraw = False) for beam_id in member_seq_affected_by_swap]
            show_sequence_color(process, artist.selected_beam_id)
            rs.EnableRedraw(True)
        go.SetDefaultString("Enter again = ChangeElementAfterThis")

    def display_show_unbuilt():
        """ Function invoked by user to set displaysettings
        """
        artist.show_unbuilt = True
        show_sequence_color(process, artist.selected_beam_id)

    def display_hide_unbuilt():
        """ Function invoked by user to set displaysettings
        """
        artist.show_unbuilt = False
        show_sequence_color(process, artist.selected_beam_id)

    run_cmd = None  # Variable to remember the last command to allow easy rerun.
    while(True):
        go.SetCustomGeometryFilter(get_existing_beams_filter(process))
        result = go.Get()
        # print (result)

        # Performs function on the previously selected beam
        if result == Rhino.Input.GetResult.Option:
            if go.Option().EnglishName == "Finish":
                show_normal_color_and_unhide(process)
                return Rhino.Commands.Result.Cancel

            if go.Option().EnglishName == "Next":
                run_cmd = select_next

            if go.Option().EnglishName == "Previous":
                run_cmd = select_previous

            if go.Option().EnglishName == "MoveEarlier":
                run_cmd = move_earlier

            if go.Option().EnglishName == "MoveLater":
                run_cmd = move_later

            if go.Option().EnglishName == "PickElementToGoAfterThis":
                run_cmd = change_next_element

            if go.Option().EnglishName == "DisplayShowUnbuilt":
                run_cmd = display_show_unbuilt

            if go.Option().EnglishName == "DisplayHideUnbuilt":
                run_cmd = display_hide_unbuilt

            run_cmd()

        elif result == Rhino.Input.GetResult.Cancel:
            # Cancels the color preview and exit function
            show_normal_color_and_unhide(process)
            return Rhino.Commands.Result.Cancel

        elif result != Rhino.Input.GetResult.Object:
            # Repeat last command
            run_cmd()

        else:
            # User clicked on a beam.
            # Show color preview of the sequence at that point
            guid = go.Object(0).ObjectId
            beam_id = get_object_name(guid)
            show_sequence_color(process, beam_id)

            # Unselect obejcts, otherwwise the function will loop infiniitely
            rs.UnselectAllObjects()

            # In the first run where user selected an active beam, the optinos are added
            if artist.selected_beam_id is None:
                go.AddOption("Next")
                go.AddOption("Previous")
                go.AddOption("MoveEarlier")
                go.AddOption("MoveLater")
                go.AddOption("PickElementToGoAfterThis")
                go.AddOption("DisplayShowUnbuilt")
                go.AddOption("DisplayHideUnbuilt")
                go.AddOption("Finish")
            artist.selected_beam_id = beam_id

        print("Currently showing Beam %i of %i" % (assembly.get_beam_sequence(artist.selected_beam_id) + 1, len(assembly.sequence)))

######################
# Rhino Entry Point
######################
# Below is the functions that get evoked when user press UI Button
# Put this in the Rhino button ! _-RunPythonScript 'integral_timber_joints.rhino.sequence.py'


if __name__ == '__main__':
    process = get_process()
    if process_is_none(process):
        print("Load json first")
    else:
        show_menu(process)
