# Imports
import Rhino
import scriptcontext
import System
import Rhino.UI
import Eto.Drawing as drawing
import Eto.Forms as forms
from integral_timber_joints.rhino.load import get_process, get_process_artist
from integral_timber_joints.process import RoboticMovement
from compas_fab.robots.trajectory import JointTrajectory
import rhinoscriptsyntax as rs
import scriptcontext as sc  # type: ignore

# SampleEtoRoomNumber dialog class


class TrajectorySliderDialog(forms.Dialog[bool]):

    # Dialog box Class initializer
    def __init__(self):
        # Initialize drawing related parameters to local variable.
        self.process = get_process()
        self.artist = get_process_artist()
        self.movement = self.process.movements[self.artist.selected_state_id - 1]  # type: RoboticMovement
        self.trajectory = self.movement.trajectory  # type: JointTrajectory
        self.init_success = False
        if self.trajectory is None:
            print("Trajectory is None for %s. Dialog cannot continue" % (self.movement.movement_id))
            self.Close(True)
            return

        trajectory_point_count = len(self.trajectory.points)
        print("Trajectory for %s has %i points." % (self.movement.movement_id, trajectory_point_count))

        # Lock to avoid multi threaded update conflict
        self.update_lock = False

        # Initialize dialog box
        self.Title = 'Trajectory Slider (PgUp PgDn / Slider)'
        self.Padding = drawing.Padding(10)
        self.Resizable = True

        # Create controls for the dialog
        self.label_total = forms.Label(Text='Trajector Point Index (0 - %i):' % (trajectory_point_count - 1))

        # Create label to chow current index
        self.label_current = forms.Label(Text='Showing: %i' % (trajectory_point_count - 1))

        # Create a slider
        self.slider = forms.Slider()
        self.slider.MinValue = 0
        self.slider.MaxValue = trajectory_point_count - 1
        self.slider.Value = trajectory_point_count - 1
        self.slider.ValueChanged += self.OnSliderValueChanged
        self.slider.Size = drawing.Size(300, 25)

        # Create the abort button
        self.AbortButton = forms.Button(Text='Close')
        self.AbortButton.Click += self.OnCloseButtonClick

        # Create a table layout and add all the controls
        layout = forms.DynamicLayout()
        layout.Spacing = drawing.Size(5, 5)
        layout.AddRow(self.label_total, self.slider)
        layout.AddRow(None)  # spacer
        layout.AddRow(self.label_current, self.AbortButton)

        # Set the dialog content
        self.Content = layout
        self.init_success = True
    # Start of the class functions

    # Close button click handler
    def OnCloseButtonClick(self, sender, e):
        self.Close(True)

    # Slider value changed handler
    def OnSliderValueChanged(self, sender, e):
        # Multi threaded update lock
        if self.update_lock:
            # print ("Lock active for value %s" % self.m_slider.Value)
            return
        # print ("Lock okay for %s" % self.m_slider.Value)
        self.update_lock = True

        # * Get Configuration at Trajectory Point
        point_index = self.slider.Value
        self.label_current.Text = 'Showing: %i' % (point_index)
        config = self.trajectory.points[point_index]

        # * Get scene with the new trajectory point config
        scene = self.artist.get_current_selected_scene_state(robot_config_override=config)

        print("Showing Trajectory Point Index %i, Config = %s " % (point_index, config))
        self.artist.draw_state(scene)

        # Unlock the mutithread lock
        self.update_lock = False

    ## End of Dialog Class ##


# Test calling the dialog
def ShowTrajectorySliderDialog():
    dialog = TrajectorySliderDialog()
    if dialog.init_success:
        Rhino.UI.EtoExtensions.ShowSemiModal(dialog, Rhino.RhinoDoc.ActiveDoc, Rhino.UI.RhinoEtoApp.MainWindow)


##########################################################################
# During development, the following code simulates the steps before
# and after the dialog is called.
if __name__ == "__main__":
    from integral_timber_joints.rhino.visualize_trajectory import hide_interactive_beams
    from integral_timber_joints.rhino.visualize_trajectory import show_interactive_beams_delete_state_vis, redraw_state_and_trajectory

    # Situation before entering the dialog
    process = get_process()
    artist = get_process_artist()
    rs.EnableRedraw(False)
    hide_interactive_beams(process)
    redraw_state_and_trajectory(artist, process, True)

    # Running interactive aslider
    ShowTrajectorySliderDialog()

    # Situation after the dialog and after exiting main function
    show_interactive_beams_delete_state_vis(process)
