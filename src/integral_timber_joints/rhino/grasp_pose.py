import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext as sc  # type: ignore
from compas_rhino import artists
from compas_rhino.utilities.objects import get_object_name

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.rhino.joints import draw_selectable_joint, delete_all_selectable_joints


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
            assembly_method = assembly.get_assembly_method(selected_beam_id)
            # Gripper and clamp only exist if it is not Manually Assembled or Undefined.
            if assembly_method not in [BeamAssemblyMethod.MANUAL_ASSEMBLY, BeamAssemblyMethod.UNDEFINED]:
                artist.show_asstool_at_one_position(beam_id, color='asstool_normal')
                artist.show_gripper_at_one_position(beam_id, color='gripper_normal')
                # Special color for SCREWED_WITHOUT_GRIPPER
                if assembly_method == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
                    grasping_joint_id = process.assembly.get_grasping_joint_id(beam_id)
                    tool_position = artist.selected_key_position.current_clamp_pos
                    # print (grasping_joint_id)
                    # print (tool_position)
                    rs.ObjectColor(artist.asstool_guids_at_position(grasping_joint_id, tool_position), artist.color_meaning.get('gripper_normal'))

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
    current_beam_meshes.extend(artist.beam_guids_at_position(selected_beam_id, 'assembly_wcf_final'))

    # Get Meshes - Gripper
    gripper_meshes = artist.gripper_guids_at_position(selected_beam_id, artist.selected_key_position.current_gripper_pos)

    # Get Meshes - Clamps
    clamp_meshes = []
    for joint_id in assembly.get_joint_ids_with_tools_for_beam(selected_beam_id):
        clamp_meshes.extend(artist.asstool_guids_at_position(joint_id, artist.selected_key_position.current_clamp_pos))

    # Get Meshes - Already Assembled Beams
    other_beams_meshes = []
    for neighbour_id in assembly.get_already_built_beams(selected_beam_id):
        other_beams_meshes.extend(artist.beam_guids_at_position(neighbour_id, 'assembly_wcf_final'))

    # Get Meshes - Already Assembled Beams but not neighbour
    other_non_neighbour_beams_meshes = []
    for neighbour_id in set(assembly.get_already_built_beams(selected_beam_id)) - set(assembly.get_already_built_neighbors(selected_beam_id)):
        other_non_neighbour_beams_meshes.extend(artist.beam_guids_at_position(neighbour_id, 'assembly_wcf_final'))

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
            if len(lines) > 0:
                collisions.append((gripper_mesh, other_beam))

    # Check Collisions between gripper vs clamps
    for gripper_mesh in gripper_meshes:
        for clamp_mesh in clamp_meshes:
            lines = Rhino.Geometry.Intersect.Intersection.MeshMeshFast(rs.coercemesh(gripper_mesh), rs.coercemesh(clamp_mesh))
            if len(lines) > 0:
                collisions.append((gripper_mesh, clamp_mesh))

    # Check Collisions between gripper, clamp vs environment
    for tool_mesh in gripper_meshes + clamp_meshes:
        for env_mesh in env_meshes:
            lines = Rhino.Geometry.Intersect.Intersection.MeshMeshFast(rs.coercemesh(tool_mesh), rs.coercemesh(env_mesh))
            if len(lines) > 0:
                collisions.append((tool_mesh, env_mesh))

    # Colour collision objects in collisions
    for rhino_object in set().union(*collisions):
        rs.ObjectColor(rhino_object, artist.color_meaning.get('warning', (200, 100, 0)))
        rs.ShowObject(rhino_object)

    print("collisions count :%s" % len(collisions))

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

###################################
# Showing Different Beam
###################################


def select_next_beam():
    """ Function invoked by user to change active element to the next one.
    """
    # Increment the selected id
    artist = get_process_artist()
    artist.select_next_beam()
    artist.selected_key_position.final_position()


def select_previous_beam():
    """ Function invoked by user to change active element to the previous one.
    """
    # Decrement the selected id
    artist = get_process_artist()
    artist.select_previous_beam()

###################################
# Showing Different Key Position
###################################


def next_key_position():
    artist = get_process_artist()
    artist.selected_key_position.next_position()


def prev_key_position():
    artist = get_process_artist()
    artist.selected_key_position.prev_position()


def first_key_position():
    artist = get_process_artist()
    artist.selected_key_position.first_position()

###################################
# Changing Gripper Definitions
###################################


def gripper_changetype():
    """ Function invoked by user to change gripper type.

    beam_attribute 'gripper_type' and 'gripper_id' will be changed.
    """
    process = get_process()
    artist = get_process_artist()
    if len(process.grippers) == 0:
        print("No gripper exist for you to choose. Very bad. Add some grippers first.")
        return

    # Switching to a sub function depending on SCREWED_WITHOUT_GRIPPER
    beam_id = artist.selected_beam_id
    if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
        _screwdriver_as_gripper_changetype(process, beam_id)
    else:
        _gripper_as_gripper_changetype(process, beam_id)

    # Invalidate downstream of assign_gripper_to_beam. Recompute all.
    process.dependency.invalidate(beam_id, process.assign_gripper_to_beam, downstream_only=False)
    process.dependency.compute_all(beam_id, attempt_all_parents_even_failure=True, verbose=True)

    # Redraw Visualization
    artist.draw_beam_all_positions(beam_id, delete_old=True, redraw=False)
    artist.draw_gripper_all_positions(beam_id, delete_old=True, redraw=False)
    if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
        artist.draw_asstool_all_positions(beam_id, delete_old=True, redraw=False)


def _gripper_as_gripper_changetype(process, beam_id):
    # type: (RobotClampAssemblyProcess, str) -> None
    """ Function invoked by user to change gripper type.
    Sub function that handles using regular gripper as gripper.

    beam_attribute 'gripper_type' and 'gripper_id' will be changed.
    """
    # List out current gripper for users to choose
    print("Available Grippers:")
    current_gripper_type = process.assembly.get_beam_attribute(beam_id, 'gripper_type')
    type_names = []
    gripper_ids = []
    for gripper in process.grippers:
        print("- %s : %s" % (gripper.name, gripper.type_name))
        if gripper.type_name not in type_names:
            type_names.append(gripper.type_name)
            gripper_ids.append(gripper.name)

    # Ask user which gripper to delete
    prompt = "Which gripper to use for Beam(%s)? Current gripper type is: %s" % (beam_id, current_gripper_type)
    selected_type_name = rs.GetString(prompt, current_gripper_type, type_names)
    # Dont change anything if the selection is the same
    if selected_type_name == current_gripper_type:
        print("Gripper type unchanged")
        return
    if selected_type_name in type_names:
        print("Gripper type changed to %s. Recomputing frames and visualization..." % (selected_type_name))
        process.assembly.set_beam_attribute(beam_id, 'gripper_id', gripper_ids[type_names.index(selected_type_name)])
        process.assembly.set_beam_attribute(beam_id, 'gripper_type', selected_type_name)


def _screwdriver_as_gripper_changetype(process, beam_id):
    # type: (RobotClampAssemblyProcess, str) -> None
    """ Function invoked by user to change gripper type.
    Sub function that handles using screwdriver as gripper.

    beam_attribute 'gripper_type' and 'gripper_id' will be changed.
    """

    # List out current gripper for users to choose
    print("Available Screwdrivers:")
    type_names = []
    gripper_ids = []
    for screwdriver in process.screwdrivers:
        print("- %s : %s" % (screwdriver.name, screwdriver.type_name))
        if screwdriver.type_name not in type_names:
            type_names.append(screwdriver.type_name)
            gripper_ids.append(screwdriver.name)

    # Ask user which gripper to delete
    current_gripper_id = process.assembly.get_beam_attribute(beam_id, 'gripper_id')
    current_gripper_type = process.assembly.get_beam_attribute(beam_id, 'gripper_type')
    prompt = "Which screwdriver to use for Beam(%s) as Gripper? Current gripper is: %s : %s" % (beam_id, current_gripper_type, current_gripper_id)
    selected_type_name = rs.GetString(prompt, current_gripper_type, type_names)
    # Dont change anything if the selection is the same
    if selected_type_name == current_gripper_type:
        print("Gripper type unchanged")
        return
    if selected_type_name in type_names:
        print("Gripper type changed to %s. Recomputing frames and visualization..." % (selected_type_name))
        process.assembly.set_beam_attribute(beam_id, 'gripper_id', gripper_ids[type_names.index(selected_type_name)])
        process.assembly.set_beam_attribute(beam_id, 'gripper_type', selected_type_name)

    process.dependency.invalidate(beam_id, process.assign_tool_type_to_joints)


def gripper_move_pos():
    """ Function invoked by user to move grasp pose.
    """
    process = get_process()
    artist = get_process_artist()
    beam_id = artist.selected_beam_id
    process.adjust_gripper_pos(beam_id, +30)
    process.dependency.compute_all(beam_id)
    artist.draw_gripper_all_positions(beam_id, delete_old=True, redraw=False)


def gripper_move_neg():
    """ Function invoked by user to move grasp pose.
    """
    process = get_process()
    artist = get_process_artist()
    beam_id = artist.selected_beam_id
    process.adjust_gripper_pos(beam_id, -30)
    process.dependency.compute_all(beam_id)
    artist.draw_gripper_all_positions(beam_id, delete_old=True, redraw=False)


def gripper_follow_ass_dir():
    """ Function invoked by user to change grasp face.
    This triggers set_grasp_face_following_assembly_direction
    """
    process = get_process()
    artist = get_process_artist()
    beam_id = artist.selected_beam_id
    process.set_grasp_face_following_assembly_direction(beam_id)
    process.dependency.compute_all(beam_id)

    # Redraw Visualization
    artist.draw_beam_all_positions(beam_id, delete_old=True, redraw=False)
    artist.draw_gripper_all_positions(beam_id, delete_old=True, redraw=False)


def grasp_face():
    """ Function invoked by user to change grasp face.
    Uses get to select face 1 to 4.

    This function recursively ask user until cancel is pressed or
    """
    # Get current situation
    process = get_process()
    artist = get_process_artist()
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
            artist.draw_beam_all_positions(beam_id, delete_old=True, redraw=False)
            artist.draw_gripper_all_positions(beam_id, delete_old=True, redraw=False)
            show_sequence_color(process)
            compute_collision_show_color(process)

        else:
            print("Selection is the same as before, grasp face unchanged.")

###################################
# Changing Clamp Definitions
###################################


def flip_clamp():
    process = get_process()
    artist = get_process_artist()
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


def change_grasping_joint():
    process = get_process()
    artist = get_process_artist()

    beam_id = artist.selected_beam_id
    joint_ids = process.assembly.get_joint_ids_with_tools_for_beam(beam_id)
    current_grasping_joint_id = process.assembly.get_grasping_joint_id(beam_id)
    print(current_grasping_joint_id)
    # * Draw selectable joints for user to select
    selectable_guids = []
    for joint_id in joint_ids:
        if joint_id[0] == current_grasping_joint_id[0] and joint_id[1] == current_grasping_joint_id[1]:
            color = (30, 85, 190)  # Blue
        else:
            color = (30, 190, 85)  # Green
        draw_selectable_joint(process, joint_id, redraw=False, color=color)
        selectable_guids.extend(artist._joint_features[joint_id])
    rs.EnableRedraw(True)

    # * Ask user to Pick in Rhino UI the grasping joint
    go = Rhino.Input.Custom.GetObject()

    def joint_feature_filter(rhino_object, geometry, component_index):
        return rhino_object.Attributes.ObjectId in selectable_guids
    go.SetCustomGeometryFilter(joint_feature_filter)
    go.SetCommandPrompt("Select joint as the Grasping Joint (current grasping joint is Blue):")
    go.EnablePreSelect(False, True)
    result = go.Get()
    print(result)
    if result is None or result == Rhino.Input.GetResult.Cancel:
        print("Cancel")
        delete_all_selectable_joints(process)
        return

    # * Parse user selection
    guid = go.Object(0).ObjectId
    name = get_object_name(guid)  # type: str
    ids = name.split('-')
    new_grasping_joint_id = (ids[0], ids[1])

    if new_grasping_joint_id == current_grasping_joint_id:
        print("Grasping Joint unchanged")
        delete_all_selectable_joints(process)
        return
    else:
        process.assembly.set_beam_attribute(beam_id, 'grasping_joint_id_preference', new_grasping_joint_id)
        print("Grasping changed from %s to %s" % (current_grasping_joint_id, new_grasping_joint_id))

    process.dependency.invalidate(beam_id, process.assign_tool_type_to_joints)
    process.dependency.compute_all(beam_id, attempt_all_parents_even_failure=False, verbose=True)
    artist.draw_asstool_all_positions(beam_id, delete_old=True, redraw=False)
    artist.redraw_interactive_beam(beam_id, force_update=True, redraw=False)
    delete_all_selectable_joints(process)


def _assembly_tools_guid_to_joint_id(beam_id, guid):
    process = get_process()
    artist = get_process_artist()
    joint_ids = process.assembly.get_joint_ids_with_tools_for_beam(beam_id)
    for joint_id in joint_ids:
        tool_position = artist.selected_key_position.current_clamp_pos
        guids = artist.asstool_guids_at_position(joint_id, tool_position)
        for _guid in guids:
            if _guid == guid:
                return joint_id
    return None


def _assembly_tools_selectable_guids(beam_id):
    process = get_process()
    artist = get_process_artist()
    joint_ids = process.assembly.get_joint_ids_with_tools_for_beam(beam_id)
    selectable_guids = []
    for joint_id in joint_ids:
        tool_position = artist.selected_key_position.current_clamp_pos
        guids = artist.asstool_guids_at_position(joint_id, tool_position)
        selectable_guids.extend(guids)
    return selectable_guids


def change_screwdriver_orientation():
    process = get_process()
    artist = get_process_artist()
    beam_id = artist.selected_beam_id

    # * Ask user to Pick in Rhino UI the screwdriver to flip
    go = Rhino.Input.Custom.GetObject()

    def joint_feature_filter(rhino_object, geometry, component_index):
        return rhino_object.Attributes.ObjectId in _assembly_tools_selectable_guids(beam_id)
    go.SetCustomGeometryFilter(joint_feature_filter)
    go.SetCommandPrompt("Select tool to flip orientation:")
    go.EnablePreSelect(False, True)
    result = go.Get()
    if result is None or result == Rhino.Input.GetResult.Cancel:
        print("Cancel")
        return
    guid = go.Object(0).ObjectId
    selected_joint_id = _assembly_tools_guid_to_joint_id(beam_id, guid)
    current_index = process.assembly.get_joint_attribute(selected_joint_id, 'tool_orientation_frame_index')
    if current_index == 0:
        process.assembly.set_joint_attribute(selected_joint_id, 'tool_orientation_frame_index', 1)
    else:
        process.assembly.set_joint_attribute(selected_joint_id, 'tool_orientation_frame_index', 0)

    process.dependency.invalidate(beam_id, process.compute_screwdriver_positions)
    process.dependency.invalidate(beam_id, process.compute_gripper_grasp_pose)
    process.dependency.compute_all(beam_id, attempt_all_parents_even_failure=False, verbose=True)
    artist.draw_asstool_all_positions(beam_id, delete_old=True, redraw=False)
    artist.redraw_interactive_beam(beam_id, force_update=True, redraw=False)


def change_screwdriver_type():
    process = get_process()
    artist = get_process_artist()
    beam_id = artist.selected_beam_id

    # * Ask user to Pick in Rhino UI the screwdriver to flip
    go = Rhino.Input.Custom.GetObject()

    def joint_feature_filter(rhino_object, geometry, component_index):
        return rhino_object.Attributes.ObjectId in _assembly_tools_selectable_guids(beam_id)
    go.SetCustomGeometryFilter(joint_feature_filter)
    go.SetCommandPrompt("Select tool to change type:")
    go.EnablePreSelect(False, True)
    result = go.Get()
    if result is None or result == Rhino.Input.GetResult.Cancel:
        print("Cancel")
        return
    guid = go.Object(0).ObjectId
    selected_joint_id = _assembly_tools_guid_to_joint_id(beam_id, guid)

    # Sort out available screwdriver types
    current_type = process.assembly.get_joint_attribute(selected_joint_id, 'tool_type')
    type_names = []
    for screwdriver in process.screwdrivers:
        print("- %s : %s" % (screwdriver.name, screwdriver.type_name))
        if screwdriver.type_name not in type_names:
            type_names.append(screwdriver.type_name)

    # Ask user which gripper to delete
    prompt = "Which screwdriver to use for Joint(%s)? Current gripper type is: %s" % (selected_joint_id, current_type)
    selected_type_name = rs.GetString(prompt, current_type, type_names)
    # Dont change anything if the selection is the same
    if selected_type_name == current_type:
        print("Screwdriver type unchanged")
        return
    if selected_type_name in type_names:
        print("Screwdriver type changed to %s. Recomputing frames and visualization..." % (selected_type_name))
        process.assembly.set_joint_attribute(selected_joint_id, 'tool_type_preference', selected_type_name)

    current_index = process.assembly.get_joint_attribute(selected_joint_id, 'tool_orientation_frame_index')
    if current_index == 0:
        process.assembly.set_joint_attribute(selected_joint_id, 'tool_orientation_frame_index', 1)
    else:
        process.assembly.set_joint_attribute(selected_joint_id, 'tool_orientation_frame_index', 0)

    process.dependency.invalidate(beam_id, process.assign_tool_type_to_joints)
    process.dependency.compute_all(beam_id, attempt_all_parents_even_failure=False, verbose=True)
    artist.draw_asstool_all_positions(beam_id, delete_old=True, redraw=False)
    artist.redraw_interactive_beam(beam_id, force_update=True, redraw=False)

###################################
# Show Rhino Menu
###################################


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
        artist.show_interactive_beam(beam_id)
    rs.EnableRedraw(True)
    rs.Redraw()

    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt("Select beam")
    go.EnablePreSelect(True, True)

    ###################################
    # Option menu
    ###################################

    def create_go_options():
        go.ClearCommandOptions()
        selected_beam_id = artist.selected_beam_id
        if selected_beam_id is not None:
            assembly_method = assembly.get_assembly_method(selected_beam_id)
            go.AddOption("Finish")
            go.AddOption("NextBeam")
            go.AddOption("PreviousBeam")
            go.AddOption("xFirstKeyPos")
            go.AddOption("vNextKeyPos")
            go.AddOption("cPrevKeyPos")

            if assembly_method == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
                go.AddOption("ChangeGraspingJoint")
            else:  # Methods with gripper
                go.AddOption("GripperMovePos")
                go.AddOption("GripperMoveNeg")
                go.AddOption("GripperType")
                go.AddOption("GraspFace")
                go.AddOption("GraspFaceFollowAsemblyDirection")

            if assembly_method in BeamAssemblyMethod.screw_methods:
                go.AddOption("ChangeToolOrientation")
                go.AddOption("ChangeScrewdriverType")

            if assembly_method == BeamAssemblyMethod.CLAMPED:
                go.AddOption("FlipClampAttachPosition")
            go.AddOption("ExitKeepGeo")

    name_to_function_dict = {
        'NextBeam': select_next_beam,
        'PreviousBeam': select_previous_beam,
        'vNextKeyPos': next_key_position,
        'xFirstKeyPos': first_key_position,
        'cPrevKeyPos': prev_key_position,
        'GripperMovePos': gripper_move_pos,
        'GripperMoveNeg': gripper_move_neg,
        'GraspFace': grasp_face,
        'GraspFaceFollowAsemblyDirection': gripper_follow_ass_dir,
        'GripperType': gripper_changetype,
        'FlipClampAttachPosition': flip_clamp,
        'ChangeGraspingJoint': change_grasping_joint,
        'ChangeToolOrientation': change_screwdriver_orientation,
        'ChangeScrewdriverType': change_screwdriver_type,

    }

    run_cmd = None  # Variable to remember the last command to allow easy rerun.

    while True:
        create_go_options()
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

            if go.Option().EnglishName in name_to_function_dict:
                run_cmd = name_to_function_dict[go.Option().EnglishName]

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
            # Change artist.selected_beam_id
            guid = go.Object(0).ObjectId
            beam_id = get_object_name(guid)
            artist.selected_beam_id = beam_id

            # Unselect obejcts, otherwwise the function will loop infiniitely
            rs.UnselectAllObjects()

            go.SetDefaultString("Press Enter to repeat.")

        print("Showing Beam(%s) (seq_id = %i) at Position(%s) (%i of %i)" % (
            artist.selected_beam_id,
            assembly.get_beam_sequence(artist.selected_beam_id),
            artist.selected_key_position.current_pos_name,
            artist.selected_key_position.current_pos_num + 1,
            artist.selected_key_position.total_pos_number,
        ))

        artist.hide_beam_all_positions(artist.selected_beam_id)
        artist.hide_gripper_all_positions(artist.selected_beam_id)
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
        process.dependency.compute_all(beam_id, attempt_all_parents_even_failure=False)


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
