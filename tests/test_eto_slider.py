# Imports
import Rhino
import scriptcontext
import System
import Rhino.UI
import Eto.Drawing as drawing
import Eto.Forms as forms

# SampleEtoRoomNumber dialog class
class SampleEtoRoomNumberDialog(forms.Dialog[bool]):

    # Dialog box Class initializer
    def __init__(self):
        # Initialize dialog box
        self.Title = 'Sample Eto: Room Number'
        self.Padding = drawing.Padding(10)
        self.Resizable = False

        # Create controls for the dialog
        self.m_label = forms.Label(Text = 'Enter the Room Number:')
        # Create a slider
        self.m_slider = forms.Slider()
        self.m_slider.MaxValue = 100
        self.m_slider.MinValue = 0
        self.m_slider.Value = 50
        self.m_slider.ValueChanged += self.OnSliderValueChanged
        

        # Create the abort button
        self.AbortButton = forms.Button(Text = 'Close')
        self.AbortButton.Click += self.OnCloseButtonClick

        # Create a table layout and add all the controls
        layout = forms.DynamicLayout()
        layout.Spacing = drawing.Size(5, 5)
        layout.AddRow(self.m_label, self.m_slider)
        layout.AddRow(None) # spacer
        layout.AddRow(None, self.AbortButton)

        # Set the dialog content
        self.Content = layout

    # Start of the class functions

    # Close button click handler
    def OnCloseButtonClick(self, sender, e):
        self.Close(True)

    # Slider value changed handler
    def OnSliderValueChanged(self, sender, e):
         print (self.m_slider.Value)
         
    ## End of Dialog Class ##

# The script that will be using the dialog.
def DynamicSliderUpdate():
    dialog = SampleEtoRoomNumberDialog();
    rc = dialog.ShowModal(Rhino.UI.RhinoEtoApp.MainWindow)


##########################################################################
# Check to see if this file is being executed as the "main" python
# script instead of being used as a module by some other python script
# This allows us to use the module which ever way we want.
if __name__ == "__main__":
    DynamicSliderUpdate()