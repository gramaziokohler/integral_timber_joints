from compas_rhino.artists import RobotModelArtist

import compas_rhino

class ToolArtist(RobotModelArtist):
    def _exit_layer(self):
        if self.layer and self._previous_layer:
            compas_rhino.rs.CurrentLayer(self._previous_layer)
