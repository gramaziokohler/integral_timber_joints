from compas.datastructures.mesh import Mesh
from integral_timber_joints.geometry import Beam
from compas_rhino.utilities import delete_objects, clear_layer
from compas_rhino.utilities import draw_mesh
from integral_timber_joints.geometry import beam
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.assembly import Assembly

import rhinoscriptsyntax as rs
import scriptcontext as sc
import Rhino

from compas_rhino.geometry import RhinoPlane
from compas.geometry import Frame

def AddAnnotationText(frame, text, height, layer):
    font = "Arial"  
    plane = RhinoPlane.from_geometry(frame).geometry
    justification = Rhino.Geometry.TextJustification.BottomLeft
    guid = sc.doc.Objects.AddText(text, plane, height, font, False, False, justification)
    import System.Guid
    if guid!=System.Guid.Empty:
        rs.ObjectLayer(guid, layer)
        # o = sc.doc.Objects.FindId(rs.coerceguid(guid))
        # o.Geometry.TextOrientation = Rhino.DocObjects.TextOrientation.InPlane
        # o.CommitChanges()
        sc.doc.Views.Redraw()
        return guid

class ProcessArtist(object):
    """ Artist to draw Beams in Rhino
    Items are drawn in specific layers for quickly turning them on and off.
    Beam Brep : layer = itj::beams_brep         name = beam_id
    Beam Mesh : layer =  itj::beams_mesh        name = beam_id 
    Beam Sequence Tag - itj::beams_seqtag       
    self.guids is a dictionary that keeps the guids for objects drawn in Rhino 
    """

    layers = ['itj::beams_brep', 'itj::beams_mesh', 'itj::beams_seqtag']

    def __init__(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        self.process = process # type: RobotClampAssemblyProcess

        # Store guid in dictionary
        self.guids = {} # type: dict[str, dict[str, list[str]]]
        for beam_id in process.assembly.beam_ids():
            self.guids[beam_id] = {}
            for layer in self.layers:
                self.guids[beam_id][layer] = []
        print (self.guids)

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

    def draw_beam_id(self, beam_id, faces = [1,3], padding_factor = 0.2, size_factor = 0.6):
        assembly = self.process.assembly
        seq_num = assembly.sequence.index(beam_id)
        for face_id in faces:
            # Get Face Frame
            beam = assembly.beam(beam_id)
            face_frame = beam.reference_side_wcf(face_id)
            
            # Move Frame orgin for padding
            beam_size_min = min(beam.height, beam.width)
            padding = beam_size_min * padding_factor
            padded_location_origin = face_frame.to_world_coordinates([padding, padding , beam_size_min * 0.01])
            face_frame.point = padded_location_origin
         
            # Test and Size
            tag_text = ".%s." % seq_num
            tag_height = beam_size_min * size_factor

            # Create Tag
            layer ='itj::beams_mesh'
            guid = AddAnnotationText(face_frame, tag_text, tag_height, layer)
            self.guids[beam_id][layer].append(guid)

    def draw_beam_mesh(self, beam_id, update_cache = False):
        # type:(str, bool) -> None
        assembly = self.process.assembly
        if beam_id not in assembly.beam_ids():
            raise KeyError("Beam %i not in Assembly" % beam_id)
        assembly.update_beam_mesh_with_joints(beam_id, not update_cache)
        beam_mesh = assembly.beam(beam_id).cached_mesh # type: Mesh
        v, f = beam_mesh.to_vertices_and_faces()
        layer ='itj::beams_mesh'
        guid = draw_mesh(v, f, name=beam_id, layer = layer)

        # print (self.guids)
        # print (self.guids[beam_id][layer])
        self.guids[beam_id][layer].append(guid)

    def redraw_beam(self, beam_id, force_update = True, draw_mesh=True, draw_tag = True):
        ''' Redraw a beam visualizations.
        '''
        self.clear_one_beam(beam_id)
        if draw_mesh: self.draw_beam_mesh(beam_id, force_update)
        if draw_tag: self.draw_beam_id(beam_id)

    def draw_beam_brep(self, process):
        raise NotImplementedError

    def clear_all(self):
        """ Clear all display geometry (brep, mesh, tag etc), all beams.
        Stored guid reference is also removed.
        """
        for beam_id in self.guids:
            self.clear_one_beam(beam_id)

    def clear_one_beam(self, beam_id):
        # type:(str) -> None
        """ Clear all display geometry (brep, mesh, tag etc) related to a beam.
        Stored guid reference is also removed.

        If beam_id is not yet tracked in self.guid, the new entry will be created.
        """
        if beam_id in self.guids:
            for layer in self.guids[beam_id]:
                delete_objects(self.guids[beam_id][layer])
                self.guids[beam_id][layer] = []
        else:
            self.guids[beam_id] = {}
            for layer in self.layers:
                self.guids[beam_id][layer] = []

    def empty_layers(self):
        # type:() -> None
        """Clear the default artist layers in Rhino.
        Create those layers if doesnt exist
        Stored guid reference is also removed.
        """
        for layer in self.layers:
            if not rs.IsLayer(layer):
                rs.AddLayer(layer)
            else:
                clear_layer(layer)

        # clear out saved guids. 
        for beam_id in self.process.assembly.beam_ids():
            for layer in self.layers:
                self.guids[beam_id][layer] = []

if __name__ == "__main__":

    from integral_timber_joints.rhino.load import load_process, get_process, get_process_artist
    # Draw beams to canvas
    # load_process()

    artist = get_process_artist()
    artist.clear_all_layers()
    process = get_process()
    for beam_id in process.assembly.beam_ids():
        artist.draw_beam_mesh(beam_id)
    artist.clear_one_beam('b15')