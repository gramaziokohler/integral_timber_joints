import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext
import re
from collections import Counter

from compas.geometry import Frame, Vector, Point, bounding_box
from compas.geometry import Transformation
from compas_rhino.geometry import RhinoCurve, RhinoSurface
from compas_rhino.ui import CommandMenu
# from compas_rhino.utilities import delete_objects
from compas_rhino.utilities.objects import get_object_name, get_object_names

from integral_timber_joints.assembly import Assembly, BeamAssemblyMethod
from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.geometry import JointHalfLap, JointNonPlanarLap
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.process.compute_process_action_movement import recompute_initial_state
from integral_timber_joints.geometry import Screw_SL, Joint
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, purge_objects, recompute_dependent_solutions
try:
    from typing import Dict, Iterator, List, Optional, Tuple, Any
except:
    pass


def _create_joints_for_new_beam(process, new_beam):
    # type: (RobotClampAssemblyProcess, Beam) -> tuple[list[Joint], list[str]]
    assembly = process.assembly
    new_joints = []  # type: list[Joint]
    affected_neighbours = []  # type: list[str]
    beam_move = new_beam
    for existing_beam in assembly.beams():
        beam_stay = existing_beam
        if beam_stay == beam_move:
            continue
        # * Check for intersections. JointHalfLap first and JointNonPlanarLap next
        # print('Checking for Planar Joint : %s-%s' % (beam_id, existing_beam.name))
        j_s, j_m, screw_line = JointHalfLap.from_beam_beam_intersection(beam_stay, beam_move)
        if j_m is None or j_s is None:
            # print('Checking for Non-Planar Joint : %s-%s' % (beam_id, existing_beam.name))
            j_s, j_m, screw_line = JointNonPlanarLap.from_beam_beam_intersection(beam_stay, beam_move)

        if j_m is not None and j_s is not None:
            print('- New Joint (%s) : %s-%s added to assembly' % (j_m.__class__.__name__, new_beam.name, existing_beam.name))
            assembly.add_joint_pair(j_s, j_m, beam_stay.name, beam_move.name)
            new_joints.append(j_m)
            affected_neighbours.append(beam_stay.name)

    # * Check if the new joints all agreed to the same beam face direction
    moving_beam_face_ids = set([joint.face_id for joint in new_joints])
    if len(moving_beam_face_ids) > 1:
        # Recreate the joints using the majority face_id on the moving beam
        moving_beam_face_ids_majority = Counter([joint.face_id for joint in new_joints]).most_common(1)[0][0]
        if all([isinstance(joint, JointNonPlanarLap) for joint in new_joints]):
            new_joints = []
            for nbr_id in affected_neighbours:
                beam_stay = assembly.beam(nbr_id)
                j_s, j_m, screw_line = JointNonPlanarLap.from_beam_beam_intersection(beam_stay, beam_move, joint_face_id_move=moving_beam_face_ids_majority)
                assembly.add_joint_pair(j_s, j_m, beam_stay.name, beam_move.name)
                new_joints.append(j_m)

    return new_joints, affected_neighbours


def _add_beams_to_assembly(process, beams):
    # type: (RobotClampAssemblyProcess, list[Beam]) -> None
    """Shared function to add newly created Beams to Assembly

    - Auto assign Assembly Method
    - Compute Joint Screw
    - create Joints and redraw changes in Rhino

    This function can probably be refactered into Assembly class.
    """
    assembly = process.assembly
    new_beam_ids = []
    affected_neighbours = []

    # * Check for new Joints
    for beam in beams:
        beam_id = assembly.get_new_beam_id()
        beam.name = beam_id
        print('New Beam: %s' % beam_id)
        new_beam_ids.append(beam_id)

        # Add to assembly
        assembly.add_beam(beam)

        # * Check for joints (Joint_Halflap and JointNonPlanarLap)
        new_joints, affected_neighbours = _create_joints_for_new_beam(process, beam)
        affected_neighbours.extend(affected_neighbours)

        # * Automatically assign Assembly Method
        if len(new_joints) == 0:
            assembly.set_beam_attribute(beam_id, 'assembly_method', BeamAssemblyMethod.GROUND_CONTACT)
            print("- Automatically assigned Assembly method: GROUND_CONTACT")
        elif any([isinstance(joint, JointNonPlanarLap) for joint in new_joints]):
            assembly.set_beam_attribute(beam_id, 'assembly_method', BeamAssemblyMethod.SCREWED_WITH_GRIPPER)
            print("- Automatically assigned Assembly method: SCREWED_WITH_GRIPPER")
        else:
            assembly.set_beam_attribute(beam_id, 'assembly_method', BeamAssemblyMethod.CLAMPED)
            print("- Automatically assigned Assembly method: CLAMPED")

        # * Initial state changed since we add a new beam
        process.dependency.add_beam(beam_id)
        recompute_initial_state(process)

    # *Recompute dependent solutions for new beams and affected neighbours
    for beam_id in set(affected_neighbours + new_beam_ids):
        recompute_dependent_solutions(process, beam_id)

    # * Draw newly added beams and neighbours affected by new joint
    artist = get_process_artist()
    for beam_id in set(affected_neighbours + new_beam_ids):
        artist.redraw_interactive_beam(beam_id, force_update=True)
    show_assembly_color(process, set(affected_neighbours + new_beam_ids), redraw=True)

    print('%i Beams added to the assembly.' % len(new_beam_ids))


def _beam_order_comparator(x, y):
    # type: (Beam, Beam) -> int
    """Creating a order based on `rhino_name` and `rhino_select_order` attribute.
    Objects with names go first.
    Otherwise, objects are listed based on selection order.
    """

    x_rhino_name = x.rhino_name if hasattr(x, 'rhino_name') else None
    y_rhino_name = y.rhino_name if hasattr(y, 'rhino_name') else None
    x_rhino_select_order = x.rhino_select_order if hasattr(x, 'rhino_select_order') else None
    y_rhino_select_order = y.rhino_select_order if hasattr(y, 'rhino_select_order') else None

    def atoi(text):
        # type: (str) -> Any
        return int(text) if text.isdigit() else text

    def natural_keys(text):
        # type: (str) -> Any
        '''
        alist.sort(key=natural_keys) sorts in human order
        http://nedbatchelder.com/blog/200712/human_sorting.html
        (See Toothy's implementation in the comments)
        '''
        return [atoi(c) for c in re.split(r'(\d+)', text)]

    # Beams with both names will be compared.
    if x_rhino_name is not None and y_rhino_name is not None:
        return natural_keys(x_rhino_name) > natural_keys(y_rhino_name)
    # Beams with no names go to the back of the sort.
    if x_rhino_name is None and y_rhino_name is not None:
        return 1
    if y_rhino_name is None and x_rhino_name is not None:
        return -1

    if x_rhino_select_order is None and y_rhino_select_order is not None:
        return -1
    if y_rhino_select_order is None and x_rhino_select_order is not None:
        return 1
    return x_rhino_name > y_rhino_name


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
    assembly = process.assembly  # type: Assembly
    new_beams = []
    for rhino_select_order, guid in enumerate(guids):
        rhinoline = RhinoCurve.from_guid(guid)  # RhinoLine is not implemented sigh... RhinoCurve is used.
        centerline = rhinoline.to_compas()
        centerline_vector = Vector.from_start_end(centerline.start, centerline.end)
        # Compute guide vector: For vertical lines, guide vector points to world X
        # Otherwise, guide vector points to Z,

        if centerline_vector.angle(Vector(0, 0, 1)) < 0.001:
            guide_vector = Vector(1, 0, 0)
        else:
            guide_vector = Vector(0, 0, 1)

        # Create Beam object
        beam = Beam.from_centerline(centerline, guide_vector, width, height)
        # Find out the name in Rhino Properities for sorting
        beam.rhino_name = rs.ObjectName(guid)
        beam.rhino_select_order = rhino_select_order

        new_beams.append(beam)

    # Sorting by human sorting natural keys
    new_beams.sort(cmp=_beam_order_comparator)
    # Add to assembly
    _add_beams_to_assembly(process, new_beams)


def ui_add_beam_from_brep_box(process):
    # type: (RobotClampAssemblyProcess) -> None
    ''' Ask user for line(s) to create new beams.
    '''
    assembly = process.assembly  # type: Assembly
    # ask user to pick boxes
    print("Object names in Rhino can be used to srot beams.")
    guids = rs.GetObjects("Select Brep (uncut 6 sided box only)", filter=rs.filter.polysurface)
    # print(guids)

    def beam_frame_from_vectors(vectors):
        # type: (list[Vector]) -> Frame

        # Pick longest vector as X axis
        vectors.sort(key=lambda v: v.length, reverse=True)
        x_vector = vectors[0]
        # Pick shortest vector to be Z Axis
        vectors.sort(key=lambda v: v.length)
        for vector in vectors:
            if Vector.cross(vector, x_vector).length > 1e-4:
                z_vector = vector
                break
        frame = Frame([0, 0, 0], x_vector, Vector.cross(x_vector, z_vector).scaled(-1))
        return frame

    if guids is None:
        return

    new_beams = []
    for rhino_select_order, guid in enumerate(guids):
        brep = rs.coercebrep(guid)  # type: Rhino.Geometry.Brep
        edges = brep.DuplicateEdgeCurves(False)
        vectors = []  # type: list[Vector]
        corners = []
        for edge in edges:
            start = [edge.PointAtStart.X, edge.PointAtStart.Y, edge.PointAtStart.Z]
            end = [edge.PointAtEnd.X, edge.PointAtEnd.Y, edge.PointAtEnd.Z]
            vector = Vector.from_start_end(start, end)
            vectors.append(vector)
            corners.append(start)
            corners.append(end)
        # Find the frame of the beam (no origin yet)
        beam_frame = beam_frame_from_vectors(vectors)

        # Find the origin of the beam frame
        aligned_corners = [beam_frame.to_local_coordinates(point) for point in corners]
        bb = bounding_box(aligned_corners)
        bounds_in_wcf = [beam_frame.to_world_coordinates(point) for point in bb]
        min_corner = bounds_in_wcf[0]
        max_corner = bounds_in_wcf[6]
        beam_frame.point = min_corner

        # Find the size of the beam from the aligned bounding box.
        bb_size = Vector.from_start_end(bb[0], bb[6])
        length = bb_size.x
        height = bb_size.y
        width = bb_size.z

        # Construct beam object
        beam = Beam(frame=beam_frame, length=length, width=width, height=height)
        # Find out the name in Rhino Properities for sorting
        beam.rhino_name = rs.ObjectName(guid)
        beam.rhino_select_order = rhino_select_order

        new_beams.append(beam)

    # Sorting by human sorting natural keys
    new_beams.sort(cmp=_beam_order_comparator)
    # Add to assembly
    _add_beams_to_assembly(process, new_beams)


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
    print('Neighbour Beam Affected: %s' % neighbors)

    rs.EnableRedraw(False)

    # Delete Beams and their joints
    for beam_id in beam_ids:
        artist.delete_interactive_beam_visualization(beam_id)
        # Maybe need to delete grippers and other things
        assembly.remove_beam(beam_id)
        # Delete from Process Dependency
        process.dependency.delete_beam(beam_id)
        print('Beam Removed: %s' % beam_id)

    # Redraw neighbour beams since joints maybe gone.
    show_assembly_color(process, neighbors, delete_old=True, redraw=False)
    rs.EnableRedraw(True)


def ui_flip_beams(process):
    # type: (RobotClampAssemblyProcess) -> None
    '''Ask User to select a beam for flipping the joints' direction
    Only joints to earlier beams are flipped.

    Beams with non planar joints cannot be flipped.

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

        # * Check if any of its joints to previous beams are non planar.
        non_planar_exist = False
        for joint_id in process.assembly.get_joints_of_beam_connected_to_already_built(beam_id):
            if isinstance(process.assembly.joint(joint_id), JointNonPlanarLap):
                print("Sorry. Cannot flip beams with non planar joints. Offending joint: %s-%s" % joint_id)
                non_planar_exist = True
        if non_planar_exist:
            continue

        # * Loop though alread_built_neighbors and swap joint
        earlier_neighbors = assembly.get_already_built_neighbors(beam_id)
        for neighbour_id in earlier_neighbors:
            joint_id = (beam_id, neighbour_id)
            assembly.flip_lap_joint(joint_id)

        # * Update drownstream computation
        assembly.set_beam_attribute(beam_id, 'assembly_wcf_final', None)
        recompute_dependent_solutions(process, beam_id)

        # * Update visualization of flipped beam and neighbors
        for beam_id in earlier_neighbors + [beam_id]:
            artist.redraw_interactive_beam(beam_id, force_update=True)
        show_assembly_color(process, earlier_neighbors + [beam_id], redraw=True)

        print('Beam flipped: %s (Neighbours: %s)' % (beam_id, earlier_neighbors))


def show_assembly_method_color(process):
    # Color Visualization
    artist = get_process_artist()
    rs.EnableRedraw(False)
    print("Assembly Method Colour Legend:")
    print("- Red:       Undefined")
    print("- Black:     Ground")
    print("- Green:     Clamped")
    print("- LightBlue: ScrewedWithGripper")
    print("- DarkBlue:  ScrewedWithoutGripper")
    for beam_id in process.assembly.sequence:
        assembly_method = process.assembly.get_assembly_method(beam_id)
        if assembly_method == BeamAssemblyMethod.GROUND_CONTACT:
            artist.change_interactive_beam_colour(beam_id, 'assembly_method_ground')
        elif assembly_method == BeamAssemblyMethod.CLAMPED:
            artist.change_interactive_beam_colour(beam_id, 'assembly_method_clamped')
        elif assembly_method == BeamAssemblyMethod.SCREWED_WITH_GRIPPER:
            artist.change_interactive_beam_colour(beam_id, 'assembly_method_screwed_w_gripper')
        elif assembly_method == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
            artist.change_interactive_beam_colour(beam_id, 'assembly_method_screwed_wo_gripper')
        else:
            artist.change_interactive_beam_colour(beam_id, 'assembly_method_undefined')
    rs.EnableRedraw(True)


def ui_change_assembly_method(process, preselection=[]):
    # type: (RobotClampAssemblyProcess, Optional[list[str]]) -> None
    '''Visualize beams assembly method in different colour.
    Options for user to change assembly method.
    Ask User to select beams.

    Preselection (beam_ids) can be supplied and allow user to only select the Assembly Method.
    '''
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    show_assembly_method_color(process)

    # Ask user if they want to change anything
    while(True):
        new_assembly_method = rs.GetString("Change Assembly Method to:", "Back", list(BeamAssemblyMethod.names_to_value_dict.keys()) + ["Back"])
        if new_assembly_method is not None and not new_assembly_method.startswith("Back"):
            new_assembly_method = BeamAssemblyMethod.names_to_value_dict[new_assembly_method]

            # Ask user to select which to change
            beam_ids = preselection
            if len(beam_ids) == 0:
                guids = rs.GetObjects('Select beams to change to %s :' % new_assembly_method, custom_filter=get_existing_beams_filter(process))
                if guids is not None:
                    beam_ids = get_object_names(guids)

            # Make change to all selected beams
            beams_to_redraw = []
            if len(beam_ids) > 0:
                for beam_id in beam_ids:
                    old_assembly_method = assembly.get_assembly_method(beam_id)
                    if new_assembly_method != old_assembly_method:
                        # Changing assembly method
                        process.assembly.set_beam_attribute(beam_id, 'assembly_method', new_assembly_method)
                        beams_to_redraw.append(beam_id)
                        # print ('Beam(%s) change from %s to %s' % (beam_id, old_assembly_method, new_assembly_method))

                        # Change of Screw Hole situation will require a recomputation and mesh update
                        if (new_assembly_method in BeamAssemblyMethod.screw_methods) != (old_assembly_method in BeamAssemblyMethod.screw_methods):
                            for neighbour_beam_id in process.assembly.get_already_built_neighbors(beam_id):
                                # Redraw Neighbour Beams because joint screw changed
                                beams_to_redraw.append(neighbour_beam_id)

                        process.dependency.invalidate(beam_id, process.assign_tool_type_to_joints)
                        process.dependency.invalidate(beam_id, process.assign_gripper_to_beam)

            [artist.redraw_interactive_beam(beam_id, force_update=True, redraw=False) for beam_id in beams_to_redraw]
            show_assembly_method_color(process)

            # Exit function if there are preselection
            if len(preselection) > 0:
                return
        else:
            show_assembly_color(process)
            return


def ui_change_assembly_vector(process):
    # type: (RobotClampAssemblyProcess) -> None
    '''Ask User to select beams for redefining the beam's assembly vector.
    This only works for beams with no clamps'''
    artist = get_process_artist()
    # Ask user for input
    beams_to_exclude = [beam_id for beam_id in process.assembly.sequence if len(process.assembly.get_joint_ids_with_tools_for_beam(beam_id)) > 0]
    guids = rs.GetObjects('Select beams (with no clamps) for changing assembly vector:', custom_filter=get_existing_beams_filter(process, beams_to_exclude))
    if guids is None:
        # Quit when user press Enter without selection
        print("No Beam selected.")
        return
    beam_ids = get_object_names(guids)

    # Ask user for two points for the assembly vector.
    start = rs.GetPoint("Pick 2 points to define assembly vector. Start Point (vector.start)")
    end = rs.GetPoint("Pick 2 points to define assembly vector. End Point (vector.end)")
    if start is None or end is None:
        print("No Point selected.")
        return
    vector = Vector.from_start_end(start, end)

    for beam_id in beam_ids:
        process.assembly.compute_beam_assembly_direction_from_joints_and_sequence(beam_id, vector)
        process.dependency.invalidate(beam_id, process.compute_gripper_grasp_pose)
        process.dependency.compute_all(beam_id, attempt_all_parents_even_failure=True)
        artist.delete_beam_all_positions(beam_id)
        artist.delete_gripper_all_positions(beam_id)
        for joint_id in process.assembly.get_joint_ids_of_beam(beam_id):
            artist.delete_asstool_all_positions(joint_id, redraw=False)


def ui_orient_assembly(process):
    # type: (RobotClampAssemblyProcess) -> None
    '''Allow users to transform the assembly using UI similar to Rhino Orient3Pt
    '''
    # Draw text dots for user to see what they have picked easier.
    text_dots = []

    # Source frame
    # Ask for origin
    origin = rs.GetPoint("Sorce Frame - Pick point at frame origin")
    if origin is None:
        purge_objects(text_dots)
        return
    text_dots.append(rs.AddTextDot("So", origin))
    # Ask for X axis and Y Axis
    x_point = rs.GetPoint("Sorce Frame - Pick point on X direction.")
    if x_point is None:
        purge_objects(text_dots)
        return
    text_dots.append(rs.AddTextDot("Sx", x_point))
    y_point = rs.GetPoint("Sorce Frame - Pick point on Y direction.")
    if y_point is None:
        purge_objects(text_dots)
        return
    text_dots.append(rs.AddTextDot("Sy", y_point))

    source_frame = Frame(origin, x_point - origin, y_point - origin)

    # Target frame
    # Ask for origin
    origin = rs.GetPoint("Target Frame - Pick point at frame origin")
    if origin is None:
        purge_objects(text_dots)
        return
    text_dots.append(rs.AddTextDot("To", origin))
    # Ask for X axis and Y Axis
    x_point = rs.GetPoint("Target Frame - Pick point on X direction.")
    if x_point is None:
        purge_objects(text_dots)
        return
    text_dots.append(rs.AddTextDot("Tx", x_point))
    y_point = rs.GetPoint("Target Frame - Pick point on Y direction.")
    if y_point is None:
        purge_objects(text_dots)
        return
    text_dots.append(rs.AddTextDot("Ty", y_point))

    target_frame = Frame(origin, x_point - origin, y_point - origin)
    print("Please wait, transforming assembly from %s to %s" % (source_frame, target_frame))

    # Delete the helpful text dot
    rs.EnableRedraw(False)
    purge_objects(text_dots)

    T = Transformation.from_frame_to_frame(source_frame, target_frame)
    assembly = process.assembly
    artist = get_process_artist()

    # Artist clear everything
    for beam_id in assembly.sequence:
        artist.delete_beam_all_positions(beam_id, redraw=False)
        artist.delete_gripper_all_positions(beam_id, redraw=False)
        artist.delete_interactive_beam_visualization(beam_id, redraw=False)
        for joint_id in process.assembly.get_joint_ids_of_beam(beam_id):
            artist.delete_asstool_all_positions(joint_id, redraw=False)

    assembly.transform(T)
    # Clear Actions and Movements because they are no longer valid.
    # It would be nice if there exist a function to update their frames, but sadly, no.
    process.dependency.invalidate(beam_id, process.compute_pickup_frame)

    # Prompt artist to redraw almost everything
    for beam_id in assembly.sequence:
        artist.redraw_interactive_beam(beam_id, force_update=False, redraw=False)
    rs.EnableRedraw(True)

    print("Please wait, transforming assembly from %s to %s" % (source_frame, target_frame))


def something(process):
    #
    print('something')


def show_assembly_color(process, beam_ids=None, delete_old=False, redraw=True):
    """Activate assembly menu colour code.
    Problematic beams that cannot be assembled are highlighted
    """
    if beam_ids == None:
        beam_ids = process.assembly.sequence
    artist = get_process_artist()
    rs.EnableRedraw(False)
    for beam_id in beam_ids:
        if delete_old:
            artist.redraw_interactive_beam(beam_id, redraw=False)
        if process.assembly.beam_problems(beam_id):
            artist.change_interactive_beam_colour(beam_id, 'warning')
        else:
            artist.change_interactive_beam_colour(beam_id, 'active')
    if redraw:
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
                    {'name': 'FromBrepBox', 'action': ui_add_beam_from_brep_box},
                ]},
                {'name': 'DeleteBeam', 'action': ui_delete_beams},
                {'name': 'MoveAssembly', 'action': ui_orient_assembly},
                {'name': 'FlipBeamAssemblyDirection', 'action': ui_flip_beams},
                {'name': 'AssemblyMethod', 'action': ui_change_assembly_method},
                {'name': 'RedefineAssemblyVector', 'action': ui_change_assembly_vector},
            ]

        }

        result = CommandMenu(config).select_action()
        # User cancel command by Escape
        if result is None or 'action' not in result:
            print('Exit Function')
            hide_assembly_color(process)
            return Rhino.Commands.Result.Cancel

        action = result['action']
        # print(action)
        # User click Exit Button
        if action == 'Exit':
            print('Exit Function')
            hide_assembly_color(process)
            return Rhino.Commands.Result.Cancel

        # If user clicked "back" do nothing.
        if action == 'Back':
            continue
        # Run the action
        else:
            action(process)

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
