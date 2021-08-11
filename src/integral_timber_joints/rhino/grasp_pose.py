import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext as sc  # type: ignore
from compas_rhino import artists
from compas_rhino.utilities.objects import get_object_name

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions


def show_sequence_color(process):
    # type: (RobotClampAssemblyProcess) -> None
    """Function to toggle (show/hide) visualization geometry and apply warning color
    Based on which beam is selected, we show beam visualization differently.

    # Beams
    - Beams (seq) before the selection = Interactive beam is shown (Warning colors applied)
    - The selected beam = Beam-in-Position is shown, position is based on current selected_key_position (Warning colors applied)
    - Beams (seq) after the selection = No visualization.

    # Clamps and Grippers
    - Gripper shown for the currently selected beam. (Warning colors applied)
    - Clamp(s) shown for the clamps clamping (assembling) the selected beam. (Warning colors applied)

    """
    rs.EnableRedraw(False)
    artist = get_process_artist()
    selected_beam_id = artist.selected_beam_id
    assembly = process.assembly

    # Loop through beams up to selected beam.
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

        # Show hide interactive beams
        if seq < selected_seq_num:
            artist.show_interactive_beam(beam_id)
        else:
            artist.hide_interactive_beam(beam_id)

        # Show hide beams in position
        if seq == selected_seq_num:
            artist.show_beam_at_one_position(beam_id)
        else:
            artist.hide_beam_all_positions(beam_id)

        # Show gripper and clamp
        if seq == selected_seq_num:
            artist.show_gripper_at_one_position(beam_id, color='normal')
            artist.show_asstool_at_one_position(beam_id, color='normal')
        else:
            artist.hide_gripper_all_positions(beam_id)
            artist.hide_asstool_all_positions(beam_id)

        # Print out for debug
        if problem:
            print(beam_id, problem)

    rs.EnableRedraw(True)


def compute_collision_show_color(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()
    selected_beam_id = artist.selected_beam_id
    assembly = process.assembly
    rs.EnableRedraw(False)

    # Get Mesh - Current Beam
    current_beam_meshes = []
    current_beam_meshes.extend(artist.interactive_beam_guid(selected_beam_id))

    # Get Meshes - Gripper
    gripper_meshes = artist.gripper_guids_at_position(selected_beam_id, artist.selected_key_position.current_gripper_pos)

    # Get Meshes - Clamps
    clamp_meshes = []
    for joint_id in assembly.get_joint_ids_with_tools_for_beam(selected_beam_id):
        clamp_meshes.extend(artist.asstool_guids_at_position(joint_id, artist.selected_key_position.current_clamp_pos))

    # Get Meshes - Already Assembled Beams
    other_beams_meshes = []
    for neighbour_id in assembly.get_already_built_beams(selected_beam_id):
        other_beams_meshes.extend(artist.interactive_beam_guid(neighbour_id))

    # Get Meshes - Already Assembled Beams but not neighbour
    other_non_neighbour_beams_meshes = []
    for neighbour_id in set(assembly.get_already_built_beams(selected_beam_id)) - set(assembly.get_already_built_neighbors(selected_beam_id)):
        other_non_neighbour_beams_meshes.extend(artist.interactive_beam_guid(neighbour_id))

    # Get Meshes - EnvModel
    env_meshes = []
    for env_id, env_model in process.environment_models.items():
        env_meshes.extend(artist.env_mesh_guids(env_id))
    rs.HideObjects(env_meshes)

    # Check Collisions between gripper vs other beams
    collisions = []
    for gripper_mesh in gripper_meshes:
        for other_beam in other_beams_meshes:
            lines = Rhino.Geometry.Intersect.Intersection.MeshMeshFast(rs.coercemesh(gripper_mesh), rs.coercemesh(other_beam))
            if len(lines) > 0 :
                collisions.append((gripper_mesh, other_beam))

    # Check Collisions between gripper vs clamps
    for gripper_mesh in gripper_meshes:
        for clamp_mesh in clamp_meshes:
            lines = Rhino.Geometry.Intersect.Intersection.MeshMeshFast(rs.coercemesh(gripper_mesh), rs.coercemesh(clamp_mesh))
            if len(lines) > 0 :
                collisions.append((gripper_mesh, clamp_mesh))

    # Check Collisions between gripper, clamp vs environment
    for tool_mesh in gripper_meshes + clamp_meshes:
        for env_mesh in env_meshes:
            lines = Rhino.Geometry.Intersect.Intersection.MeshMeshFast(rs.coercemesh(tool_mesh), rs.coercemesh(env_mesh))
            if len(lines) > 0 :
                collisions.append((tool_mesh, env_mesh))

    # Colour collision objects in collisions
    for rhino_object in set().union(*collisions):
        rs.ObjectColor(rhino_object, artist.color_meaning.get('warning', (200,100,0)))
        rs.ShowObject(rhino_object)

    print ("collisions count :%s" % len(collisions))

    # - between clams and other not clamping beams

    # Colour grpper and clamps if there are collisions

    rs.EnableRedraw(True)



def show_normal_color_and_unhide(process):
    artist = get_process_artist()
    assembly = process.assembly  # type: Assembly
    rs.EnableRedraw(False)
    for beam_id in assembly.sequence:
        artist.change_interactive_beam_colour(beam_id, 'normal')
        artist.hide_beam_all_positions(beam_id)
        artist.hide_gripper_all_positions(beam_id)
        artist.hide_asstool_all_positions(beam_id)

        artist.show_interactive_beam(beam_id)
    rs.EnableRedraw(True)


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    artist.selected_beam_id = None
    artist.selected_key_position.final_position()

    # Make sure beam, gripper and clamps are drawn if they are drawn yet
    # Initially hide them.
    rs.EnableRedraw(False)
    for beam_id in assembly.sequence:
        artist.draw_beam_all_positions(beam_id, redraw=False)
        artist.hide_beam_all_positions(beam_id)
        artist.draw_gripper_all_positions(beam_id, redraw=False)
        artist.hide_gripper_all_positions(beam_id)
        artist.draw_asstool_all_positions(beam_id, redraw=False)
        artist.hide_asstool_all_positions(beam_id)
    rs.EnableRedraw(True)
    rs.Redraw()

    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt("Select beam")
    go.EnablePreSelect(True, True)

    ###################################
    # Showing Different Beam
    ###################################

    def select_next_beam():
        """ Function invoked by user to change active element to the next one.
        """
        # Hide the current beam and grippers
        artist.hide_beam_all_positions(artist.selected_beam_id)
        artist.hide_gripper_all_positions(artist.selected_beam_id)
        artist.hide_asstool_all_positions(artist.selected_beam_id)
        # Increment the selected id
        artist.select_next_beam()
        artist.selected_key_position.final_position()
        show_sequence_color(process)

    def select_previous_beam():
        """ Function invoked by user to change active element to the previous one.
        """
        # Hide the current beam and grippers
        artist.hide_beam_all_positions(artist.selected_beam_id)
        artist.hide_gripper_all_positions(artist.selected_beam_id)
        artist.hide_asstool_all_positions(artist.selected_beam_id)
        # Decrement the selected id
        artist.select_previous_beam()
        artist.selected_key_position.final_position()
        show_sequence_color(process)

    ###################################
    # Showing Different Key Position
    ###################################

    def next_key_position():
        artist.selected_key_position.next_position()
        _show_key_position_beam_gripper_clamp()

    def prev_key_position():
        artist.selected_key_position.prev_position()
        _show_key_position_beam_gripper_clamp()

    def first_key_position():
        artist.selected_key_position.first_position()
        _show_key_position_beam_gripper_clamp()

    def _show_key_position_beam_gripper_clamp():
        artist.show_beam_at_one_position(artist.selected_beam_id)
        artist.show_gripper_at_one_position(artist.selected_beam_id)
        artist.show_asstool_at_one_position(artist.selected_beam_id)
        show_sequence_color(process)

    ###################################
    # Changing Gripper Definitions
    ###################################

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
        if selected_type_name == current_gripper_type:
            return
        if selected_type_name in type_names:
            print("Gripper type changed to %s. Recomputing frames and visualization..." % (selected_type_name))
            process.assembly.set_beam_attribute(beam_id, 'gripper_type', selected_type_name)

        process.dependency.invalidate(beam_id, process.assign_gripper_to_beam, downstream_only=True)
        process.dependency.compute_all(beam_id, attempt_all_parents_even_failure=True)

        # Redraw Visualization
        artist.draw_beam_all_positions(beam_id, delete_old=True)
        artist.draw_gripper_all_positions(beam_id, delete_old=True)
        show_sequence_color(process)

    def gripper_move_pos():
        """ Function invoked by user to move grasp pose.
        """
        beam_id = artist.selected_beam_id
        process.adjust_gripper_pos(beam_id, +30)
        process.dependency.compute_all(beam_id)
        artist.draw_gripper_all_positions(beam_id, delete_old=True)
        show_sequence_color(process)

    def gripper_move_neg():
        """ Function invoked by user to move grasp pose.
        """
        beam_id = artist.selected_beam_id
        process.adjust_gripper_pos(beam_id, -30)
        process.dependency.compute_all(beam_id)
        artist.draw_gripper_all_positions(beam_id, delete_old=True)
        show_sequence_color(process)

    def gripper_follow_ass_dir():
        """ Function invoked by user to change grasp face.
        This triggers search_grasp_face_from_joint_assembly_direction
        """
        beam_id = artist.selected_beam_id
        process.search_grasp_face_from_joint_assembly_direction(beam_id)
        process.dependency.compute_all(beam_id)

        # Redraw Visualization
        artist.draw_beam_all_positions(beam_id, delete_old=True)
        artist.draw_gripper_all_positions(beam_id, delete_old=True)
        show_sequence_color(process)

    def grasp_face():
        """ Function invoked by user to change grasp face.
        Uses get to select face 1 to 4.

        This function recursively ask user until cancel is pressed or
        """
        # Get current situation
        beam_id = artist.selected_beam_id

        for _ in range(10):
            current_grasp_face = process.assembly.get_beam_attribute(beam_id, 'gripper_grasp_face')

            # Ask user which face to grasp
            options = ['Back', 'Face1', 'Face2', 'Face3', 'Face4']
            options_number = ['Back', '1', '2', '3', '4']
            selected_grasp_face = rs.GetString("Which face to grasp for Beam(%s)? Current grasp face is: %s" %
                                               (beam_id, options[current_grasp_face]), options[current_grasp_face], options)
            if selected_grasp_face is None:
                # User press Escape
                return  # Exit loop
            elif selected_grasp_face in options:
                # User selected one of the Buttons
                selected_grasp_face = options.index(selected_grasp_face)
            elif selected_grasp_face in options_number:
                # User typed in just a number
                selected_grasp_face = options_number.index(selected_grasp_face)
            else:
                print("Input not recognized : %s" % selected_grasp_face)
                continue  # Continue loop and try again.
            if selected_grasp_face == 0:
                # User selected Back as option
                return  # Exit loop

            # Check if new selection is the same as before
            if selected_grasp_face != current_grasp_face:
                # Recompute dependency
                process.override_grasp_face(beam_id, selected_grasp_face)
                process.dependency.compute_all(beam_id)

                # Redraw Visualization
                artist.draw_beam_all_positions(beam_id, delete_old=True)
                artist.draw_gripper_all_positions(beam_id, delete_old=True)
                show_sequence_color(process)
                compute_collision_show_color(process)

            else:
                print("Selection is the same as before, grasp face unchanged.")

    ###################################
    # Changing Clamp Definitions
    ###################################

    def flip_clamp():
        beam_id = artist.selected_beam_id
        print("get_clamp_orientation_options:")
        print(process.get_clamp_orientation_options(beam_id))

        # Call fnction to change direction
        process.flip_clamp_guide_vector(beam_id)
        # Recompute Dependency
        process.dependency.invalidate(beam_id, process.search_valid_clamp_orientation_with_guiding_vector)
        process.dependency.compute_all(beam_id, attempt_all_parents_even_failure=True)
        # Refresh Display
        artist.draw_beam_all_positions(beam_id, delete_old=True, redraw=False)
        artist.draw_gripper_all_positions(beam_id, delete_old=True, redraw=False)
        artist.draw_asstool_all_positions(beam_id, delete_old=True)
        show_sequence_color(process)

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

            if go.Option().EnglishName == "ExitKeepGeo":
                return Rhino.Commands.Result.Cancel

            if go.Option().EnglishName == "NextBeam":
                run_cmd = select_next_beam

            if go.Option().EnglishName == "PreviousBeam":
                run_cmd = select_previous_beam

            if go.Option().EnglishName == "vShowNextKeyPosition":
                run_cmd = next_key_position

            if go.Option().EnglishName == "cShowFirstKeyPosition":
                run_cmd = first_key_position

            if go.Option().EnglishName == "xShowPrevKeyPosition":
                run_cmd = prev_key_position

            if go.Option().EnglishName == "GripperMovePos":
                run_cmd = gripper_move_pos

            if go.Option().EnglishName == "GripperMoveNeg":
                run_cmd = gripper_move_neg

            if go.Option().EnglishName == "GraspFace":
                run_cmd = grasp_face

            if go.Option().EnglishName == "GraspFaceFollowAsemblyDirection":
                run_cmd = gripper_follow_ass_dir

            if go.Option().EnglishName == "GripperType":
                run_cmd = gripper_changetype

            if go.Option().EnglishName == "FlipClampAttachPosition":
                run_cmd = flip_clamp

            run_cmd()

        elif result == Rhino.Input.GetResult.Cancel:
            # Cancels the color preview and exit function
            show_normal_color_and_unhide(process)
            return Rhino.Commands.Result.Cancel

        elif result != Rhino.Input.GetResult.Object:
            # Repeat last command
            if run_cmd is not None:
                run_cmd()
            else:
                # Cancels the color preview and exit function
                show_normal_color_and_unhide(process)
                return Rhino.Commands.Result.Cancel
        else:
            # User clicked on a beam.
            # Show color preview of the sequence at that point
            guid = go.Object(0).ObjectId
            beam_id = get_object_name(guid)

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
                go.AddOption("NextBeam")
                go.AddOption("PreviousBeam")
                go.AddOption("vShowNextKeyPosition")
                go.AddOption("cShowFirstKeyPosition")
                go.AddOption("xShowPrevKeyPosition")
                go.AddOption("GripperMovePos")
                go.AddOption("GripperMoveNeg")
                go.AddOption("GripperType")
                go.AddOption("GraspFace")
                go.AddOption("GraspFaceFollowAsemblyDirection")
                go.AddOption("FlipClampAttachPosition")
                go.AddOption("ExitKeepGeo")
                # Reminder that Enter key can repeat
                go.SetDefaultString("Press Enter to repeat.")
            artist.selected_beam_id = beam_id
            show_sequence_color(process)


        print("Showing Beam(%s) (%i of %i) at Position(%s) (%i of %i)" % (
            artist.selected_beam_id,
            assembly.get_beam_sequence(artist.selected_beam_id) + 1,
            len(assembly.sequence),
            artist.selected_key_position.current_pos_name,
            artist.selected_key_position.current_pos_num + 1,
            artist.selected_key_position.total_pos_number,
        ))

        # artist.show_beam_at_one_position(artist.selected_beam_id)
        # artist.show_gripper_at_one_position(artist.selected_beam_id)
        # artist.show_asstool_at_one_position(artist.selected_beam_id)
        show_sequence_color(process)
        compute_collision_show_color(process)

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
        process.dependency.compute_all(beam_id, attempt_all_parents_even_failure=True)


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
