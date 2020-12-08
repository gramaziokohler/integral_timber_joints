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

    # Loop through selected beams up to selected beam.
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

        # Hide unbuilt beams
        if seq > selected_seq_num:
            artist.hide_beam(beam_id)
        else:
            artist.show_beam(beam_id)

        # Print out for debug
        if problem:
            print(beam_id, problem)

        # Change gripper color based on problems
        # To be implemented

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

    # Draw all gripper positions if they have not been drawn
    # Initially hide them.
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

    # def gripper_changetype():
    #     """ Function invoked by user to move grasp pose.
    #     """
    #     beam_id = artist.selected_beam_id
    #     process.adjust_gripper_pos(beam_id, +30)
    #     artist.draw_gripper_all_positions(beam_id, delete_old=True)

    #     beam = assembly.beam(artist.selected_beam_id)
        
    #     # Change clamp type
    #     assembly.set_beam_attribute(beam_id, "gripper_type", gripper_type)
        
    #     # Recompute best clamp face and update grasp frame
    #     gripper_grasp_face = process.search_grasp_face_from_guide_vector_dir(beam_id)
    #     gripper_grasp_dist_from_start = assembly.get_beam_attribute(beam_id, "gripper_grasp_dist_from_start")
    #     gripper_tcp_in_ocf = beam.grasp_frame_ocf(gripper_grasp_face, gripper_grasp_dist_from_start)
    #     assembly.set_beam_attribute(beam_id, "gripper_tcp_in_ocf", gripper_tcp_in_ocf)
        
    #     # Recompute Pickup Alignment at Pickup Station
    #     process.compute_storage_location_at_corner_aligning_pickup_location(beam_id)
        
    #     # Recompute approach and retract regarding gripper 
    #     process.compute_beam_storageapproach_and_finalretract(beam_id)
    #     process.compute_beam_storageretract(beam_id)

    def gripper_move_pos():
        """ Function invoked by user to move grasp pose.
        """
        beam_id = artist.selected_beam_id
        process.adjust_gripper_pos(beam_id, +30)
        process.dependency.compute(beam_id, process.compute_gripper_grasp_pose)
        artist.draw_gripper_all_positions(beam_id, delete_old=True)
        show_sequence_color(process, beam_id)
    
    def gripper_move_neg():
        """ Function invoked by user to move grasp pose.
        """
        beam_id = artist.selected_beam_id
        process.adjust_gripper_pos(beam_id, -30)
        process.dependency.compute(beam_id, process.compute_gripper_grasp_pose)
        artist.draw_gripper_all_positions(beam_id, delete_old=True)
        show_sequence_color(process, beam_id)

    def gripper_follow_ass_dir():
        """ Function invoked by user to change grasp face.
        """
        beam_id = artist.selected_beam_id
        process.search_grasp_face_from_joint_assembly_direction(beam_id)
        process.dependency.compute(beam_id, process.compute_gripper_grasp_pose)
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

            if go.Option().EnglishName == "GripperMovePos":
                run_cmd = gripper_move_pos

            if go.Option().EnglishName == "GripperMoveNeg":
                run_cmd = gripper_move_neg

            if go.Option().EnglishName == "GripperFollowAsemblyDirection":
                run_cmd = gripper_follow_ass_dir

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
                go.AddOption("GripperMovePos")
                go.AddOption("GripperMoveNeg")
                go.AddOption("GripperFollowAsemblyDirection")
                go.AddOption("GripperType")

            artist.selected_beam_id = beam_id

        print("Currently showing Beam %i of %i" % (assembly.get_beam_sequence(artist.selected_beam_id) + 1, len(assembly.sequence)))
        process.dependency.debug_print(artist.selected_beam_id)


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

    ###########################
    # Clamp related computation
    ###########################

    # Assign clamp to joints
    # process.assign_clamp_type_to_joints()
    # Search clamp attachment position

    for beam_id in assembly.sequence:
        # process.dependency.compute(beam_id, process.assign_clamp_type_to_joints)
        process.dependency.compute(beam_id, process.search_valid_clamp_orientation_with_guiding_vector)
        process.dependency.compute(beam_id, process.compute_clamp_attachapproach_attachretract_detachapproach)
        process.dependency.compute(beam_id, process.compute_clamp_detachretract)
        process.dependency.compute(beam_id, process.search_valid_jawapproach_vector_prioritizing_beam_side)

    #############################
    # Gripper related computation
    #############################

    for beam_id in assembly.sequence:
        # beam = assembly.beam(beam_id)
        # # Assign a gripper
        # process.dependency.compute(beam_id, process.assign_gripper_to_beam)
        # Compute gripper grasp pose
        process.dependency.compute(beam_id, process.compute_gripper_grasp_pose)


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
        print("Pickup station is not defined, cannot compute pickup poses.")


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
