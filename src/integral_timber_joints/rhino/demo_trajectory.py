import scriptcontext
import rhinoscriptsyntax as rs
import Rhino
from compas_fab.robots.trajectory import JointTrajectory

from integral_timber_joints.process import Movement, RoboticMovement

from integral_timber_joints.rhino.load import get_process, get_process_artist
from integral_timber_joints.rhino.visualize_trajectory import (hide_interactive_beams,
    load_selected_external_movment_if_exist,
    redraw_state_and_trajectory,
    show_interactive_beams_delete_state_vis,
)


viewPort = rs.CurrentView()
process = get_process()
artist = get_process_artist()
starting_state_id = artist.selected_state_id
movements = process.movements

# Hide interactive beams and beams in positions
rs.EnableRedraw(False)
hide_interactive_beams(process)

# Show Environment Meshes
artist.draw_all_env_mesh(True, redraw=False)
rs.EnableRedraw(True)

# Draw trajectory
skipping_to_selected_movement = True

while True:
    # Goes to the begining (resets in second replay)
    artist.selected_state_id = starting_state_id

    for movement_index, movement in enumerate(movements):
        if artist.selected_state_id > movement_index + 1:
            continue

        artist.selected_state_id = movement_index + 1
        load_selected_external_movment_if_exist(process)
        redraw_state_and_trajectory(artist, process)

        # Skip non-robotic movements
        if not isinstance(movement, RoboticMovement):
            continue

        trajectory = movement.trajectory  # type: JointTrajectory

        if trajectory is None:
            print("Movement %s has no Trajectory" % (movement.movement_id))
            continue

        for point_index in range(len(trajectory.points)):
            # * Get Configuration at Trajectory Point
            config = trajectory.points[point_index]

            # * Get scene with the new trajectory point config
            scene = artist.get_current_selected_scene_state(robot_config_override=config)

            print("Showing Movement %s - %s Trajectory Point %i of %i" % (movement.movement_id, movement.tag, point_index + 1, len(trajectory.points)))
            rs.RotateView(viewPort, 0, 0.5)
            artist.draw_state(scene)

            # rs.EnableRedraw(True)
            Rhino.RhinoApp.Wait()
            if scriptcontext.escape_test(False):
                print("Thanks for spinning around with Victor")
                show_interactive_beams_delete_state_vis(process)
                exit()

print("Thanks for spinning around with Victor")
show_interactive_beams_delete_state_vis(process)