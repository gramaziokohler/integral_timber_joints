import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

from compas.datastructures import Mesh
from compas.geometry import Frame
from compas_rhino.geometry import RhinoPlane
from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry import Beam
from integral_timber_joints.process import RobotClampAssemblyProcess
from compas_rhino.artists import RobotModelArtist
from integral_timber_joints.rhino.tool_artist import ToolArtist


def AddAnnotationText(frame, text, height, layer):
    font = "Arial"
    plane = RhinoPlane.from_geometry(frame).geometry
    justification = Rhino.Geometry.TextJustification.BottomLeft
    guid = sc.doc.Objects.AddText(text, plane, height, font, False, False, justification)
    import System.Guid
    if guid != System.Guid.Empty:
        rs.ObjectLayer(guid, layer)
        # o = sc.doc.Objects.FindId(rs.coerceguid(guid))
        # o.Geometry.TextOrientation = Rhino.DocObjects.TextOrientation.InPlane
        # o.CommitChanges()
        sc.doc.Views.Redraw()
        return guid


class ProcessKeyPosition(object):
    def __init__(self, pos_num = 0):
        self.pos_num = 0

    # pos_name, beam_pos, gripper_pos, clamp_pos
    pos_names = [
        ('beam_storage_approach',   'assembly_wcf_storage',         'assembly_wcf_storageapproach',     'clamp_wcf_final'),
        ('beam_storage_pick',       'assembly_wcf_storage',         'assembly_wcf_storage',             'clamp_wcf_final'),
        ('beam_storage_retract',    'assembly_wcf_storageretract',  'assembly_wcf_storageretract',      'clamp_wcf_final'),
        ('beam_inclampapproach',    'assembly_wcf_inclampapproach', 'assembly_wcf_inclampapproach',     'clamp_wcf_final'),
        ('beam_inclamp',            'assembly_wcf_inclamp',         'assembly_wcf_inclamp',             'clamp_wcf_final'),
        ('beam_final',              'assembly_wcf_final',           'assembly_wcf_final',               'clamp_wcf_final'),
        ('beam_finalretract',       'assembly_wcf_final',           'assembly_wcf_finalretract',        'clamp_wcf_final'),
    ]

    def next(self):
        self.pos_num += 1
        if self.pos_num >= len(self.pos_names):
            self.pos_num = 0

    def final(self):
        self.pos_num = 5

    @property
    def pos_name(self):
        return self.pos_names[self.pos_num][0]

    @property
    def beam_pos(self):
        return self.pos_names[self.pos_num][1]

    @property
    def gripper_pos(self):
        return self.pos_names[self.pos_num][2]

    @property
    def clamp_pos(self):
        return self.pos_names[self.pos_num][3]

    def to_data(self):
        data = {'pos_num' : self.pos_num}

    @classmethod
    def from_data(cls, data):
        return cls(data['pos_num'])


class ProcessArtist(object):
    """ Artist to draw Beams in Rhino
    Items are drawn in specific layers for quickly turning them on and off.
    Beam Brep : layer = itj::beams_brep         name = beam_id
    Beam Mesh : layer =  itj::beams_mesh        name = beam_id 
    Beam Sequence Tag - itj::beams_seqtag       
    self.guids is a dictionary that keeps the guids for objects drawn in Rhino 
    """

    # Rhino layers used to hold temporary visualization objects
    layers = [
        'itj::beams_brep',
        'itj::beams_mesh',
        'itj::beams_seqtag',
        'itj::clamps_at_final',
    ]

    color_meaning = {
        'normal': (0, 0, 0),
        'warning': (255, 152, 0),
        'error': (244, 67, 54),
        'active': (76, 175, 80),
        'unbuilt': (220, 220, 220),
        'unbuilt_warning': (255, 204, 204),
        'built': (61, 167, 219),
        'built_warning': (130, 20, 20),
        'neighbors': (0, 188, 212),
    }

    key_positions = [
        'storage_approach',

    ]

    def __init__(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        self.process = process  # type: RobotClampAssemblyProcess

        # # Create guid in dictionary to store geometries added to Rhino document
        self.guids = {}  # type: dict[str, dict[str, list[str]]]
        for beam_id in process.assembly.beam_ids():
            self.guids[beam_id] = {}

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

        # Empty existing layers and create them if not existing.
        # This also creates the guid dictionary
        self.empty_layers()

        self.selected_beam_id = None # type: str
        self.selected_key_position = ProcessKeyPosition(0)


    ######################
    # Beam
    ######################

    def draw_beam_seqtag(self, beam_id, faces=[1, 3], padding_factor=0.2, size_factor=0.6):
        assembly = self.process.assembly
        seq_num = assembly.sequence.index(beam_id)
        for face_id in faces:
            # Get Face Frame
            beam = assembly.beam(beam_id)
            face_frame = beam.reference_side_wcf(face_id)

            # Move Frame orgin for padding
            beam_size_min = min(beam.height, beam.width)
            padding = beam_size_min * padding_factor
            padded_location_origin = face_frame.to_world_coordinates([padding, padding, beam_size_min * 0.01])
            face_frame.point = padded_location_origin

            # Test and Size
            tag_text = ".%s." % seq_num
            tag_height = beam_size_min * size_factor

            # Create Tag
            layer = 'itj::beams_seqtag'
            guid = AddAnnotationText(face_frame, tag_text, tag_height, layer)
            self.guids[beam_id][layer].append(guid)

    def draw_beam_mesh(self, beam_id, update_cache=False):
        # type:(str, bool) -> None
        assembly = self.process.assembly
        if beam_id not in assembly.beam_ids():
            raise KeyError("Beam %i not in Assembly" % beam_id)
        assembly.update_beam_mesh_with_joints(beam_id, not update_cache)
        beam_mesh = assembly.beam(beam_id).cached_mesh  # type: Mesh
        v, f = beam_mesh.to_vertices_and_faces()
        layer = 'itj::beams_mesh'
        guid = draw_mesh(v, f, name=beam_id, layer=layer)
        self.guids[beam_id][layer].append(guid)

    def redraw_beam(self, beam_id, force_update=True, draw_mesh=True, draw_tag=True):
        ''' Redraw a beam visualizations.
        '''
        self.delete_one_beam(beam_id)
        if draw_mesh:
            self.draw_beam_mesh(beam_id, force_update)
        if draw_tag:
            self.draw_beam_seqtag(beam_id)

    def draw_beam_all_positions(self, beam_id, delete_old=False):
        """ Delete old beam geometry if delete_old is True
        Redraw them in Rhino in different layers.
        The resulting Rhino guids are kept in self.guids[beam_id][layer_name]

        This applies to all positions where the attribute is set in beam attributes.
        """
        for beam_position in self.beam_positions:
            layer_name = 'itj::beam::' + beam_position
            # If not delete_old, and there are already items drawn, we preserve them. 
            if len(self.guids[beam_id][layer_name]) > 0 and not delete_old:
                continue

            # Delete old geometry
            self.delete_beam_at_position(beam_id, beam_position)
            
            # Skip the rest of code if the position does not exist.
            if self.process.assembly.get_beam_attribute(beam_id, beam_position) is None:
                # print ("Skipping gripper position: %s" % (gripper_position))
                continue

            print("Drawing Beam(%s) in position: %s" % (beam_id, beam_position))

            # Transform the beam_mesh to location and 
            T = self.process.assembly.get_beam_transformaion_to(beam_id, beam_position)
            beam_mesh = self.process.assembly.beam(beam_id).cached_mesh.transformed(T) # type: Mesh
            v, f = beam_mesh.to_vertices_and_faces()
            guid = draw_mesh(v, f, name=beam_id, layer=layer_name)
            self.guids[beam_id][layer_name].append(guid)
    
    def delete_beam_at_position(self, beam_id, beam_position):
        """Delete all Rhino geometry associated to a beam at specified position
        """
        layer_name = 'itj::beam::' + beam_position
        guids = self.guids[beam_id][layer_name]
        if len(guids) > 0:
            delete_objects(guids)
            self.guids[beam_id][layer_name] = []

    def show_beam_at_one_position(self, beam_id, position = None):
        """ Show Beam only at the specified position.
        Position is the position attribute name, if left None, selected_key_position will be used. 
        """
        if position is None:
            position = self.selected_key_position.beam_pos

        for beam_position in self.beam_positions:
            layer_name = 'itj::beam::' + beam_position
            if beam_position == position:
                rs.ShowObject(self.guids[beam_id][layer_name])
            else:
                rs.HideObject(self.guids[beam_id][layer_name])

    def hide_beam_all_positions(self, beam_id):
        """ Hide all gripper instances in the specified positions.
        `positions` are defaulted to all position.
        # """

        self.show_beam_at_one_position(beam_id, '')

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

    @property
    def all_layer_names(self):
        for layer in self.layers:
            yield layer
        for gripper_position in self.gripper_positions:
            yield 'itj::gripper::' + gripper_position
        for clamp_position in self.clamp_positions:
            yield 'itj::clamp::' + clamp_position
        for beam_position in self.beam_positions:
            yield 'itj::beam::' + beam_position

    def empty_layers(self):
        # type:() -> None
        """Clear the default artist layers in Rhino.
        Create those layers if doesnt exist
        Gguid dictionary is reset to all empty lists.
        """
        for layer in self.layers:
            if not rs.IsLayer(layer):
                rs.AddLayer(layer)
            else:
                clear_layer(layer)

        # Clear Gripper and Clamp layers
        for layer_name in self.all_layer_names:
            if not rs.IsLayer(layer_name):
                rs.AddLayer(layer_name)
            else:
                clear_layer(layer_name)

        # clear out saved guids.
        for beam_id in self.process.assembly.beam_ids():
            for layer in self.all_layer_names:
                self.guids[beam_id][layer] = []

    ######################
    # Clamp and Gripper
    ######################

    beam_positions = [
        'assembly_wcf_storage',
        'assembly_wcf_storageretract',
        'assembly_wcf_inclampapproach',
        'assembly_wcf_inclamp',
        'assembly_wcf_final',
    ]
    gripper_positions = [
        'assembly_wcf_storageapproach',
        'assembly_wcf_storage',
        'assembly_wcf_storageretract',
        'assembly_wcf_inclampapproach',
        'assembly_wcf_inclamp',
        'assembly_wcf_final',
        'assembly_wcf_finalretract',
    ]
    clamp_positions = [
        'clamp_wcf_attachapproach1',
        'clamp_wcf_attachapproach2',
        'clamp_wcf_final',
        'clamp_wcf_detachretract1',
        'clamp_wcf_detachretract2',
    ]

    def draw_gripper_all_positions(self, beam_id, delete_old=False):
        """ Delete old gripper geometry if delete_old is True
        Redraw them in Rhino in different layers.
        The resulting Rhino guids are kept in self.guids[beam_id][layer_name]

        This applies to all positions where the attribute is set in beam attributes.
        """
        for gripper_position in self.gripper_positions:
            layer_name = 'itj::gripper::' + gripper_position
            # If not delete_old, and there are already items drawn, we preserve them. 
            if len(self.guids[beam_id][layer_name]) > 0 and not delete_old:
                continue

            # Delete old geometry
            self.delete_gripper_at_position(beam_id, gripper_position)
            
            # Skip the rest of code if the position does not exist.
            if self.process.assembly.get_beam_attribute(beam_id, gripper_position) is None:
                # print ("Skipping gripper position: %s" % (gripper_position))
                continue
            print("Drawing Gripper for Beam(%s) in position: %s" % (beam_id, gripper_position))
            gripper = self.process.get_gripper_of_beam(beam_id, gripper_position)
            gripper_artist = ToolArtist(gripper, layer_name)
            guid = gripper.draw_visual(gripper_artist)
            self.guids[beam_id][layer_name].extend(guid)

    def delete_gripper_all_positions(self, beam_id):
        """Delete all Rhino geometry associated to a a gripper.
        All positions are deleted
        """
        for gripper_position in self.gripper_positions:
            self.delete_gripper_at_position(beam_id, gripper_position)

    def delete_gripper_at_position(self, beam_id, gripper_position):
        """Delete all Rhino geometry associated to a gripper at specified position
        """
        layer_name = 'itj::gripper::' + gripper_position
        guids = self.guids[beam_id][layer_name]
        if len(guids) > 0:
            delete_objects(guids)
            self.guids[beam_id][layer_name] = []

    def show_gripper_at_one_position(self, beam_id, position = None):
        """ Show Gripper only at the specified position.
        Position is the position attribute name, if left None, selected_key_position will be used. 
        """
        if position is None:
            position = self.selected_key_position.gripper_pos

        for gripper_position in self.gripper_positions:
            layer_name = 'itj::gripper::' + gripper_position
            if gripper_position == position:
                rs.ShowObject(self.guids[beam_id][layer_name])
            else:
                rs.HideObject(self.guids[beam_id][layer_name])

    def hide_gripper_all_positions(self, beam_id):
        """ Hide all gripper instances in the specified positions.
        `positions` are defaulted to all position.
        # """

        self.show_gripper_at_one_position(beam_id, '')

    ######################
    # Robot
    ######################

    ######################
    # Show Hide Color
    ######################

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
        if beam_id is None:
            show = False
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
                rs.ObjectColor(guid, self.color_meaning.get(meaning, (0, 0, 0)))


if __name__ == "__main__":

    from integral_timber_joints.rhino.load import get_process, get_process_artist, load_process

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
            layer = 'itj::clamps'
            guids.append(draw_mesh(v, f, name=clamp_type, layer=layer))
        block = rs.AddBlock(guids, (0, 0, 0), clamp_type, True)
        #rs.InsertBlock(block, (10,10,10))
