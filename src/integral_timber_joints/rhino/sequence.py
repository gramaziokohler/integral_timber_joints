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
        problem = assembly.beam_problems(beam_id)
        if seq == selected_seq_num and not problem:
            artist.change_beam_colour(beam_id, 'active')
        if seq == selected_seq_num and problem:
            artist.change_beam_colour(beam_id, 'error')
        if seq < selected_seq_num and not problem:
            artist.change_beam_colour(beam_id, 'built')
        if seq < selected_seq_num and problem:
            artist.change_beam_colour(beam_id, 'built_warning')
        if seq > selected_seq_num and not problem:
            artist.change_beam_colour(beam_id, 'unbuilt')
        if seq > selected_seq_num and problem:
            artist.change_beam_colour(beam_id, 'unbuilt_warning')
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
        selected_seq_num = assembly.get_beam_sequence(artist.selected_beam_id)
        selected_seq_num = min(selected_seq_num + 1,  len(assembly.sequence) - 1)
        artist.selected_beam_id = assembly.sequence[selected_seq_num]
        show_sequence_color(process, artist.selected_beam_id)
        go.SetDefaultString("Enter again = Next")

    def select_previous():
        selected_seq_num = assembly.get_beam_sequence(artist.selected_beam_id)
        selected_seq_num = max(selected_seq_num - 1,  0)
        artist.selected_beam_id = assembly.sequence[selected_seq_num]
        show_sequence_color(process, artist.selected_beam_id)
        go.SetDefaultString("Enter again = Previous")

    def move_earlier():
        if assembly.get_beam_sequence(artist.selected_beam_id) == 0: return 
        member_seq_affected_by_swap = assembly.shift_beam_sequence(artist.selected_beam_id, -1)
        [artist.redraw_beam(beam_id, force_update=True) for beam_id in member_seq_affected_by_swap]
        show_sequence_color(process, artist.selected_beam_id)
        go.SetDefaultString("Enter again = MoveEarlier")

    def move_later():
        if assembly.get_beam_sequence(artist.selected_beam_id) == len(assembly.sequence): return 
        member_seq_affected_by_swap = assembly.shift_beam_sequence(artist.selected_beam_id, +1)
        [artist.redraw_beam(beam_id, force_update=True) for beam_id in member_seq_affected_by_swap]
        show_sequence_color(process, artist.selected_beam_id)
        go.SetDefaultString("Enter again = MoveLater")
      
    def change_next_element():
        original_seq = assembly.get_beam_sequence(artist.selected_beam_id)
        # Ask user for another beam
        go2 = Rhino.Input.Custom.GetObject()
        go2.SetCommandPrompt("Select beam to be the next element in sequence")
        go2.EnablePreSelect(False, False)
        go2.SetCustomGeometryFilter(get_existing_beams_filter(process,exclude_beam_ids = [artist.selected_beam_id]))
        result = go2.Get()
        if result == Rhino.Input.GetResult.Object:
            guid = go2.Object(0).ObjectId
            beam_id = get_object_name(guid)
            picked_seq = assembly.get_beam_sequence(beam_id)
            shift = original_seq - picked_seq
            if shift < 0: shift = shift + 1
            member_seq_affected_by_swap = assembly.shift_beam_sequence(beam_id, shift)
            artist.selected_beam_id = beam_id
            [artist.redraw_beam(beam_id, force_update=True) for beam_id in member_seq_affected_by_swap]
            show_sequence_color(process, artist.selected_beam_id)
        go.SetDefaultString("Enter again = ChangeElementAfterThis")



    run_cmd = None
    while(True):
        go.SetCustomGeometryFilter(get_existing_beams_filter(process))
        result = go.Get()
        # print (result)

        # Performs function on the previously selected beam
        if result == Rhino.Input.GetResult.Option:
            if go.Option().EnglishName == "Cancel":
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

            run_cmd()
            
        elif result == Rhino.Input.GetResult.Cancel:
            # Cancels the color preview and exit function
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
                go.AddOption("Cancel")
            artist.selected_beam_id = beam_id

        print ("Current Beam %i of %i" % (assembly.get_beam_sequence(artist.selected_beam_id) + 1, len(assembly.sequence)))

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
