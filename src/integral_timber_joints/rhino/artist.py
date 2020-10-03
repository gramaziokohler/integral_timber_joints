from compas.datastructures.mesh import Mesh
from integral_timber_joints.geometry import Beam
from compas_rhino.utilities import delete_objects, clear_layers
from compas_rhino.utilities import draw_mesh
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.assembly import Assembly


class ProcessArtist(object):
    """ Artist to draw Beams in Rhino
    Items are drawn in specific layers for quickly turning them on and off.
    Beam Brep : layer = itj::beams_brep         name = beam_id
    Beam Mesh : layer =  itj::beams_mesh        name = beam_id 
    Beam Sequence Tag - itj::beams_seqtag       
    self.guids is a dictionary that keeps the guids for objects drawn in Rhino 
    """

    def __init__(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        self.process = process # type: RobotClampAssemblyProcess

        # Store guid in dictionary
        self.guids = {} # type: dict[str, list[str]]
        for beam_id in process.assembly.beam_ids():
            self.guids[beam_id] = []
        self.settings = {
            'color.vertex': (255, 255, 255),
            'color.edge': (0, 0, 0),
            'color.face': (210, 210, 210),
            'color.normal:vertex': (0, 255, 0),
            'color.normal:face': (0, 255, 0),
            'scale.normal:vertex': 0.1,
            'scale.normal:face': 0.1,
            'show.vertices': True,
            'show.edges': True,
            'show.faces': True}

    def draw_beam_mesh(self, beam_id):
        assembly = self.process.assembly
        assembly.update_beam_mesh_with_joints(beam_id, True)
        beam_mesh = assembly.beam(beam_id).cached_mesh # type: Mesh
        v, f = beam_mesh.to_vertices_and_faces()
        guid = draw_mesh(v, f, name=beam_id, layer='itj::beams_mesh')
        self.guids[beam_id].append(guid)


    def draw_beam_brep(self, process):
        raise NotImplementedError

    def clear_all(self):
        for beam_id in self.guids:
            delete_objects(self.guids[beam_id])

    def clear_one_beam(self, beam_id):
        """ Clear all display geometry (brep, mesh, tag etc) related to a beam.
        """
        delete_objects(self.guids[beam_id])

    def clear_all_layers(self):
        """Clear the layer of the artist."""
        clear_layers(['itj::beams_brep', 'itj::beams_mesh', 'itj::beams_seqtag'])

if __name__ == "__main__":

    from integral_timber_joints.rhino.load import load_process, get_process, get_process_artist
    # Draw beams to canvas
    # load_process()

    artist = get_process_artist()
    process = get_process()
    for beam_id in process.assembly.beam_ids():
        artist.draw_beam_mesh(beam_id)