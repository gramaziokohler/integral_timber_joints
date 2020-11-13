import Rhino
from compas_rhino import artists
import rhinoscriptsyntax as rs
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

    # Draw beams up to selected beam.
    selected_seq_num = assembly.get_beam_sequence(selected_beam_id)
    for seq, beam_id in enumerate(assembly.sequence):

        # Change color based on problems
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

        # Hide unbuilt beams if settings show_unbuilt == False
        if seq > selected_seq_num and (getattr(artist, "show_unbuilt", True) == False):
            artist.hide_beam(beam_id)
        else:
            artist.show_beam(beam_id)

        # print out for debug
        if problem:
            print(beam_id, problem)
    
        # Show gripper for the specified position for the seleected beam
        position = 'assembly_wcf_final'
        if beam_id == selected_beam_id:
            artist.show_gripper_at_one_position(beam_id, position)
        else:
            artist.hide_gripper_all_positions(beam_id)
    # artist.draw_clamps(beam_id, position)

    # artist.draw_gripper_all_positions(selected_beam_id)

    rs.EnableRedraw(True)


def show_normal_color_and_unhide(process):
    artist = get_process_artist()
    assembly = process.assembly  # type: Assembly
    rs.EnableRedraw(False)
    for beam_id in assembly.sequence:
        artist.change_beam_colour(beam_id, 'normal')
        artist.show_beam(beam_id)
        artist.hide_gripper_all_positions(beam_id)
    rs.EnableRedraw(True)


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    artist.selected_beam_id = None
    artist.show_unbuilt = False

    # Draw all gripper positions, initially hide them.
    rs.EnableRedraw(False)
    for beam_id in assembly.sequence:
        artist.draw_gripper_all_positions(beam_id)
        artist.hide_gripper_all_positions(beam_id)
    rs.EnableRedraw(True)
    rs.Redraw()
        
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

            # In the first run where user selected an active beam, the optinos are added
            if artist.selected_beam_id is None:
                go.AddOption("Next")
                go.AddOption("Previous")
                go.AddOption("Finish")
            artist.selected_beam_id = beam_id

        print("Currently showing Beam %i of %i" % (assembly.get_beam_sequence(artist.selected_beam_id) + 1, len(assembly.sequence)))


######################
# Test
######################

def compute_positions(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly

    from compas.geometry import Vector

    # Hot fix for old process file that does have the right default settings
    process.assembly.update_default_node_attributes({'design_guide_vector_jawapproach': Vector(1, 1, 1)})
    process.assembly.update_default_node_attributes({'design_guide_vector_grasp': Vector(1, 1, 1)})
    process.assembly.update_default_node_attributes({'design_guide_vector_grasp': Vector(0, 0, 1)})

    ###########################
    # Clamp related computation
    ###########################

    # Assign clamp to joints
    process.assign_clamp_type_to_joints()
    # Search clamp attachment position

    for beam_id in assembly.sequence:
        # Search clamp attachment position, this sets clamp_wcf_final
        process.search_valid_clamp_orientation_with_guiding_vector(beam_id)
        # Compute three key poositions for the clamp
        process.compute_clamp_attachapproach_attachretract_detachapproach(beam_id, verbose=False)
        # Compute clamp detach key positions.  
        process.compute_clamp_detachretract(beam_id)
        # Compute how the beam slides out. This depended on the clamp orientation.
        process.search_valid_jawapproach_vector_prioritizing_beam_side(beam_id)

    #############################
    # Gripper related computation
    #############################

    for beam_id in assembly.sequence:
        beam = assembly.beam(beam_id)

        # Assign gripper based on beam length
        process.assign_gripper_to_beam(beam_id, verbose=True)
        # Compute gripper grasp pose
        process.compute_default_gripper_grasp_pose(beam_id)

    #################
    # Pickup location
    #################
    
    if process.pickup_station is not None:
        for beam_id in assembly.sequence:
            # Compute storage location
            process.compute_storage_location_at_corner_aligning_pickup_location(beam_id)

            # Set pickup vector from pickup station
            design_guide_vector_storage_pickup = process.pickup_station.pickup_retract_vector
            assembly.set_beam_attribute(beam_id, "design_guide_vector_storage_pickup", design_guide_vector_storage_pickup)

            # Compute storageretract position. Based on pickup vector.
            process.compute_beam_storageretract(beam_id)

            # Compute storageapproach position. Based on gripper approach direction.
            process.compute_beam_storageapproach(beam_id)

    else:
        print ("Pickup station is not defined, cannot compute pickup poses.")


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
        compute_positions(process)
        show_menu(process)
