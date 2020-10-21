from compas.datastructures.mesh import Mesh
from integral_timber_joints.geometry import Beam, Frame
from compas_rhino.utilities import delete_objects, clear_layer
from compas_rhino.utilities import draw_mesh
from integral_timber_joints.geometry import beam
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.assembly import Assembly, assembly

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
    color_meaning = {
        'normal' : (0,0,0),
        'warning' : (255, 152, 0),
        'error' : (244, 67, 54),
        'active' : (76, 175, 80),
        'neighbors' : (0, 188, 212),
    }
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

    def draw_beam_seqtag(self, beam_id, faces = [1,3], padding_factor = 0.2, size_factor = 0.6):
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
            layer ='itj::beams_seqtag'
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
        self.delete_one_beam(beam_id)
        if draw_mesh: self.draw_beam_mesh(beam_id, force_update)
        if draw_tag: self.draw_beam_seqtag(beam_id)

    def draw_beam_brep(self, process):
        raise NotImplementedError

    def delete_one_beam(self, beam_id):
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

    def show_beam(self, beam_id):
        """ Show the beam of the beam_id.
        """
        guids = []
        for layer in self.layers:
            for guid in self.guids[beam_id][layer]:
                guids.append(guid) 
        rs.ShowObject(guids)
        # print ('Showing: %s' % guids)

    def hide_beam(self, beam_id):
        """ Show the beam of the beam_id.
        """
        guids = []
        for layer in self.layers:
            for guid in self.guids[beam_id][layer]:
                guids.append(guid) 
        rs.HideObject(guids)
        # print ('Hiding: %s' % guids)

    def show_until(self, beam_id):
        """ Show only the beams before and the given beam_id, others are hidden.
        If beam_id is None, all beams are hidden.
        If beam_id is not in the list of sequence, all beams are shown.
        """

        assembly = self.process.assembly
        show = True
        if beam_id is None: show = False
        for _beam_id in assembly.sequence:
            if show:
                self.show_beam(_beam_id)
            else:
                self.hide_beam(_beam_id)
            if _beam_id == beam_id:
                show = False

    def change_beam_colour(self, beam_id, meaning):
        """ Chagne the beam brep and mesh color to a given colour string
        Colour string refer to color_meaning dict
        """
        for layer in ['itj::beams_brep', 'itj::beams_mesh']:
            for guid in self.guids[beam_id][layer]:
                rs.ObjectColor(guid, self.color_meaning.get(meaning, (0,0,0)))
                
    # def draw_gripper_block(self, beam_id):
    #     """Draw the components of a gripper as a block in Rhino for fast display and 
    #     """


if __name__ == "__main__":

    from integral_timber_joints.rhino.load import load_process, get_process, get_process_artist
    # Draw beams to canvas
    # load_process()

    artist = get_process_artist()
    # artist.empty_layers()
    process = get_process()
    # for beam_id in process.assembly.beam_ids():
    #     artist.draw_beam_mesh(beam_id)
    #     artist.draw_beam_seqtag(beam_id)
    # artist.delete_one_beam('b15')
    # artist.hide_beam('b34')
    # artist.show_beam('b34')
    # artist.show_until('b25')
    """Draw the clamp as a block. Block name is the clamp_type"""
    for clamp_type in process.available_clamp_types:
        clamp = process.get_one_clamp_by_type(clamp_type).copy()
        clamp.open_gripper()
        clamp.open_clamp()
        clamp.current_frame = Frame.worldXY()
        clamp_meshes = clamp.draw_visuals()
        guids = []
        for clamp_mesh in clamp_meshes:
            v, f = clamp_mesh.to_vertices_and_faces()
            layer ='itj::clamps'
            guids.append(draw_mesh(v, f, name=clamp_type, layer = layer))
        block = rs.AddBlock(guids, (0,0,0), clamp_type, True)
        #rs.InsertBlock(block, (10,10,10))