import Rhino
import rhinoscriptsyntax as rs
from compas_rhino import artists
from compas_rhino.utilities.objects import get_object_name

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions


def show_sequence_color(process, selected_beam_id):
    # type: (RobotClampAssemblyProcess, str) -> None
    rs.EnableRedraw(False)
    artist = get_process_artist()
    assembly = process.assembly

    # Loop through selected beams up to selected beam.
    selected_seq_num = assembly.get_beam_sequence(selected_beam_id)
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

        # Hide unbuilt beams
        if seq >= selected_seq_num:
            artist.hide_interactive_beam(beam_id)
        else:
            artist.show_interactive_beam(beam_id)

        # Print out for debug
        if problem:
            print(beam_id, problem)

        # Change gripper color based on problems
        # To be implemented

        # Show gripper and clamp for the specified position for the seleected beam
        position = 'assembly_wcf_final'
        if beam_id == selected_beam_id:
            artist.show_gripper_at_one_position(beam_id, position)
            artist.show_clamp_at_one_position(beam_id, position)
        else:
            artist.hide_gripper_all_positions(beam_id)
            artist.hide_clamp_all_positions(beam_id)

    # artist.draw_clamps(beam_id, position)

    # artist.draw_gripper_all_positions(selected_beam_id)

    rs.EnableRedraw(True)


def show_normal_color_and_unhide(process):
    artist = get_process_artist()
    assembly = process.assembly  # type: Assembly
    rs.EnableRedraw(False)
    for beam_id in assembly.sequence:
        artist.change_interactive_beam_colour(beam_id, 'normal')
        artist.hide_beam_all_positions(beam_id)
        artist.hide_gripper_all_positions(beam_id)
        artist.hide_clamp_all_positions(beam_id)

        artist.show_interactive_beam(beam_id)
    rs.EnableRedraw(True)


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    artist.selected_beam_id = None
    artist.selected_key_position.final_position()
    artist.show_unbuilt = False

    # Draw all gripper positions if they have not been drawn
    # Initially hide them.
    rs.EnableRedraw(False)
    for beam_id in assembly.sequence:
        artist.draw_beam_all_positions(beam_id)
        artist.hide_beam_all_positions(beam_id)
        artist.draw_gripper_all_positions(beam_id)
        artist.hide_gripper_all_positions(beam_id)
        artist.draw_clamp_all_positions(beam_id)
        artist.hide_clamp_all_positions(beam_id)
    rs.EnableRedraw(True)
    rs.Redraw()

    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt("Select beam")
    go.EnablePreSelect(True, True)

    def select_next():
        """ Function invoked by user to change active element to the next one.
        """
        # Hide the current beam and grippers
        artist.hide_beam_all_positions(artist.selected_beam_id)
        artist.hide_gripper_all_positions(artist.selected_beam_id)
        artist.hide_clamp_all_positions(artist.selected_beam_id)
        # Increment the selected id
        selected_seq_num = assembly.get_beam_sequence(artist.selected_beam_id)
        selected_seq_num = min(selected_seq_num + 1,  len(assembly.sequence) - 1)
        artist.selected_beam_id = assembly.sequence[selected_seq_num]
        artist.selected_key_position.final_position()
        show_sequence_color(process, artist.selected_beam_id)
        go.SetDefaultString("Enter again = Next")

    def select_previous():
        """ Function invoked by user to change active element to the previous one.
        """
        # Hide the current beam and grippers
        artist.hide_beam_all_positions(artist.selected_beam_id)
        artist.hide_gripper_all_positions(artist.selected_beam_id)
        artist.hide_clamp_all_positions(artist.selected_beam_id)
        # Decrement the selected id
        selected_seq_num = assembly.get_beam_sequence(artist.selected_beam_id)
        selected_seq_num = max(selected_seq_num - 1,  0)
        artist.selected_beam_id = assembly.sequence[selected_seq_num]
        artist.selected_key_position.final_position()
        show_sequence_color(process, artist.selected_beam_id)
        go.SetDefaultString("Enter again = Previous")

    def next_key_position():
        artist.selected_key_position.next_position()
        artist.show_beam_at_one_position(artist.selected_beam_id)
        artist.show_gripper_at_one_position(artist.selected_beam_id)
        artist.show_clamp_at_one_position(artist.selected_beam_id)
        go.SetDefaultString("Enter again = NextKeyPosition")

    def prev_key_position():
        artist.selected_key_position.prev_position()
        artist.show_beam_at_one_position(artist.selected_beam_id)
        artist.show_gripper_at_one_position(artist.selected_beam_id)
        artist.show_clamp_at_one_position(artist.selected_beam_id)
        go.SetDefaultString("Enter again = PrevKeyPosition")

    def gripper_changetype():
        """ Function invoked by user to change gripper type
        """
        if len(process.grippers) == 0:
            print("No gripper exist for you to choose. Very bad. Add some grippers first.")
            return

        # List out current gripper for users to choose
        beam_id = artist.selected_beam_id
        current_gripper_type = process.assembly.get_beam_attribute(beam_id, 'gripper_type')
        # print("Current Gripper for Beam(%s) is: %s" % (beam_id, current_gripper_type))

        print("Available Grippers:")
        type_names = []
        for gripper in process.grippers:
            print("- %s : %s" % (gripper.name, gripper.type_name))
            if gripper.type_name not in type_names:
                type_names.append(gripper.type_name)

        # Ask user which gripper to delete
        selected_type_name = rs.GetString("Which gripper to use for Beam(%s)? Current gripper type is: %s" % (beam_id, current_gripper_type), current_gripper_type, type_names)
        # Dont change anything if the selection is the same
        if selected_type_name  == current_gripper_type:
            return
        if selected_type_name in type_names:
            print("Gripper type changed to %s. Recomputing frames and visualization..." % (selected_type_name))
            process.assembly.set_beam_attribute(beam_id, 'gripper_type', selected_type_name)

        process.dependency.invalidate(beam_id, process.assign_gripper_to_beam, downstream_only=True)
        process.dependency.compute(beam_id, process.compute_all, attempt_all_parents_even_failure=True)

        # Redraw Visualization
        artist.draw_beam_all_positions(beam_id, delete_old=True)
        # artist.hide_beam_all_positions(beam_id)
        artist.draw_gripper_all_positions(beam_id, delete_old=True)
        # artist.hide_gripper_all_positions(beam_id)
        show_sequence_color(process, beam_id)


    def gripper_move_pos():
        """ Function invoked by user to move grasp pose.
        """
        beam_id = artist.selected_beam_id
        process.adjust_gripper_pos(beam_id, +30)
        process.dependency.compute(beam_id, process.compute_all)
        artist.draw_gripper_all_positions(beam_id, delete_old=True)
        show_sequence_color(process, beam_id)

    def gripper_move_neg():
        """ Function invoked by user to move grasp pose.
        """
        beam_id = artist.selected_beam_id
        process.adjust_gripper_pos(beam_id, -30)
        process.dependency.compute(beam_id, process.compute_all)
        artist.draw_gripper_all_positions(beam_id, delete_old=True)
        show_sequence_color(process, beam_id)

    def gripper_follow_ass_dir():
        """ Function invoked by user to change grasp face.
        """
        beam_id = artist.selected_beam_id
        process.search_grasp_face_from_joint_assembly_direction(beam_id)
        process.dependency.compute(beam_id, process.compute_all)
        artist.draw_gripper_all_positions(beam_id, delete_old=True)
        show_sequence_color(process, beam_id)

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

            if go.Option().EnglishName == "cNextKeyPosition":
                run_cmd = next_key_position

            if go.Option().EnglishName == "vPrevKeyPosition":
                run_cmd = prev_key_position

            if go.Option().EnglishName == "GripperMovePos":
                run_cmd = gripper_move_pos

            if go.Option().EnglishName == "GripperMoveNeg":
                run_cmd = gripper_move_neg

            if go.Option().EnglishName == "GripperFollowAsemblyDirection":
                run_cmd = gripper_follow_ass_dir

            if go.Option().EnglishName == "GripperType":
                run_cmd = gripper_changetype

            # if go.Option().EnglishName == "Autocompute":
            #     run_cmd = auto_compute

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

            # If this is not the first run, hide previous beam positions
            if artist.selected_beam_id is not None:
                # print("Hiding Beam %s" % artist.selected_beam_id)
                artist.hide_beam_all_positions(artist.selected_beam_id)
                artist.hide_gripper_all_positions(artist.selected_beam_id)

            # In the first run where user selected an active beam, the optinos are added
            if artist.selected_beam_id is None:
                go.AddOption("Finish")
                go.AddOption("Next")
                go.AddOption("Previous")
                go.AddOption("cNextKeyPosition")
                go.AddOption("vPrevKeyPosition")
                go.AddOption("GripperMovePos")
                go.AddOption("GripperMoveNeg")
                go.AddOption("GripperFollowAsemblyDirection")
                go.AddOption("GripperType")

            artist.selected_beam_id = beam_id

        print("Showing Beam(%s) (%i of %i) at Position(%s) (%i of %i)" % (
            artist.selected_beam_id,
            assembly.get_beam_sequence(artist.selected_beam_id) + 1,
            len(assembly.sequence),
            artist.selected_key_position.current_pos_name,
            artist.selected_key_position.current_pos_num + 1,
            artist.selected_key_position.total_pos_number,
        ))

        artist.show_beam_at_one_position(artist.selected_beam_id)
        artist.show_gripper_at_one_position(artist.selected_beam_id)
        artist.show_clamp_at_one_position(artist.selected_beam_id)


######################
# Test
######################

def compute_positions(process, verbose=True):
    # type: (RobotClampAssemblyProcess, bool) -> None
    assembly = process.assembly

    from compas.geometry import Vector

    # Hot fix for old process file that does have the right default settings
    process.assembly.update_default_node_attributes({'design_guide_vector_jawapproach': Vector(1, 1, 1)})
    process.assembly.update_default_node_attributes({'design_guide_vector_grasp': Vector(1, 1, 1)})
    process.assembly.update_default_node_attributes({'design_guide_vector_grasp': Vector(0, 0, 1)})

    for beam_id in assembly.sequence:
        process.dependency.compute(beam_id, process.compute_all, attempt_all_parents_even_failure=True)


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
        compute_positions(process)  # Todo: make it not recompute everything
        show_menu(process)
