import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas_rhino.artists import RobotModelArtist
from compas_rhino.geometry import RhinoPlane
from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry import Beam
from integral_timber_joints.process import RobotClampAssemblyProcess
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
    def __init__(self, current_pos_num=0):
        self.current_pos_num = 0
        self.current_beam_has_clamps = True

    # Constant names for beam, gripper and clamp positions.
    # beam_positions and gripper_positions relate directly to Beam Attributes in Assembly
    # clamp_positions  relate directly to Joint Attributes in Assembly
    beam_positions = [
        'assembly_wcf_pickup',
        'assembly_wcf_pickupretract',
        'assembly_wcf_inclampapproach',
        'assembly_wcf_inclamp',
        'assembly_wcf_final',
    ]
    gripper_positions = [
        'assembly_wcf_pickupapproach',
        'assembly_wcf_pickup',
        'assembly_wcf_pickupretract',
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

    # pos_name, beam_pos, gripper_pos, clamp_pos
    pos_names_for_beam_with_clamps = [
        ('clamp_attachapproach1',   'assembly_wcf_pickup',         'assembly_wcf_pickupapproach',     'clamp_wcf_attachapproach1'),
        ('clamp_attachapproach2',   'assembly_wcf_pickup',         'assembly_wcf_pickupapproach',     'clamp_wcf_attachapproach2'),
        ('beam_pickup_approach',   'assembly_wcf_pickup',         'assembly_wcf_pickupapproach',     'clamp_wcf_final'),
        ('beam_pickup_pick',       'assembly_wcf_pickup',         'assembly_wcf_pickup',             'clamp_wcf_final'),
        ('beam_pickup_retract',    'assembly_wcf_pickupretract',  'assembly_wcf_pickupretract',      'clamp_wcf_final'),
        ('beam_inclampapproach',    'assembly_wcf_inclampapproach', 'assembly_wcf_inclampapproach',     'clamp_wcf_final'),
        ('beam_inclamp',            'assembly_wcf_inclamp',         'assembly_wcf_inclamp',             'clamp_wcf_final'),
        ('beam_final',              'assembly_wcf_final',           'assembly_wcf_final',               'clamp_wcf_final'),
        ('beam_finalretract',       'assembly_wcf_final',           'assembly_wcf_finalretract',        'clamp_wcf_final'),
        ('clamp_detachretract1',    'assembly_wcf_final',           'assembly_wcf_finalretract',        'clamp_wcf_detachretract1'),
        ('clamp_detachretract2',    'assembly_wcf_final',           'assembly_wcf_finalretract',        'clamp_wcf_detachretract2'),
    ]

    pos_names_for_beam_without_clamps = [
        ('beam_pickup_approach',   'assembly_wcf_pickup',         'assembly_wcf_pickupapproach',     'clamp_wcf_final'),
        ('beam_pickup_pick',       'assembly_wcf_pickup',         'assembly_wcf_pickup',             'clamp_wcf_final'),
        ('beam_pickup_retract',    'assembly_wcf_pickupretract',  'assembly_wcf_pickupretract',      'clamp_wcf_final'),
        ('beam_inclamp',            'assembly_wcf_inclamp',         'assembly_wcf_inclamp',             'clamp_wcf_final'),
        ('beam_final',              'assembly_wcf_final',           'assembly_wcf_final',               'clamp_wcf_final'),
        ('beam_finalretract',       'assembly_wcf_final',           'assembly_wcf_finalretract',        'clamp_wcf_final'),
    ]

    def next_position(self):
        self.current_pos_num += 1
        if self.current_pos_num >= self.total_pos_number:
            self.current_pos_num = self.total_pos_number - 1

    def prev_position(self):
        self.current_pos_num += -1
        if self.current_pos_num < 0:
            self.current_pos_num = 0

    def final_position(self):
        """ Set `self.current_pos_num` to beam_final
        """
        if self.current_beam_has_clamps:
            self.current_pos_num = 7
        else:
            self.current_pos_num = 5

    @property
    def _get_current_pos_names(self):
        # just in case
        if self.current_pos_num >= self.total_pos_number:
            self.final_position()

        # Switching between two sets of key positions depending if it has clamps
        if self.current_beam_has_clamps:
            return self.pos_names_for_beam_with_clamps[self.current_pos_num]
        else:
            return self.pos_names_for_beam_without_clamps[self.current_pos_num]

    @property
    def current_pos_name(self):
        # type() -> str
        return self._get_current_pos_names[0]

    @property
    def current_beam_pos(self):
        return self._get_current_pos_names[1]

    @property
    def current_gripper_pos(self):
        return self._get_current_pos_names[2]

    @property
    def current_clamp_pos(self):
        return self._get_current_pos_names[3]

    @property
    def total_pos_number(self):
        if self.current_beam_has_clamps:
            return len(self.pos_names_for_beam_with_clamps)
        else:
            return len(self.pos_names_for_beam_without_clamps)

    def to_data(self):
        data = {'current_pos_num': self.current_pos_num}

    @classmethod
    def from_data(cls, data):
        return cls(data['current_pos_num'])


class ProcessArtist(object):
    """ Artist to draw Beams in Rhino
    Items are drawn in specific layers for quickly turning them on and off.
    Beam Brep : layer = itj::beams_brep         name = beam_id
    Beam Mesh : layer =  itj::beams_mesh        name = beam_id 
    Beam Sequence Tag - itj::beams_seqtag       
    self.gripper_guids, clamp_guids, beam_guids, interactive_guids
    are dictionary that keep track of the objects drawn in Rhino.
    """

    # Rhino layers used to hold temporary visualization objects
    interactive_layers = [
        'itj::interactive::beams_mesh',
        'itj::interactive::beams_seqtag',
        'itj::interactive::clamps_at_final',
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
        self.beam_guids = {}  # type: dict[str, dict[str, list[str]]]
        self.gripper_guids = {}  # type: dict[str, dict[str, list[str]]]
        self.interactive_guids = {}  # type: dict[str, dict[str, list[str]]]
        self.clamp_guids = {}  # type: dict[str, dict[str, list[str]]]

        for beam_id in process.assembly.beam_ids():
            self.beam_guids[beam_id] = {}
            self.gripper_guids[beam_id] = {}
            self.interactive_guids[beam_id] = {}
            for joint_id in self.process.assembly.get_joint_ids_of_beam(beam_id):
                self.clamp_guids[joint_id] = {}

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

        self._selected_beam_id = None  # type: str
        self.selected_key_position = ProcessKeyPosition(0)

    @property
    def selected_beam_id(self):
        return self._selected_beam_id

    @selected_beam_id.setter
    def selected_beam_id(self, beam_id):
        self._selected_beam_id = beam_id
        # update selected_key_position whether current_beam_has_clamps
        if beam_id is not None:
            self.selected_key_position.current_beam_has_clamps = len(self.process.assembly.get_joint_ids_of_beam_clamps(beam_id)) > 0

    #############################
    # Beam in Interactive Layers
    #############################

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
            layer = 'itj::interactive::beams_seqtag'
            guid = AddAnnotationText(face_frame, tag_text, tag_height, layer)
            self.interactive_guids[beam_id][layer].append(guid)

    def draw_beam_mesh(self, beam_id, update_cache=False):
        # type:(str, bool) -> None
        assembly = self.process.assembly
        if beam_id not in assembly.beam_ids():
            raise KeyError("Beam %i not in Assembly" % beam_id)
        assembly.update_beam_mesh_with_joints(beam_id, not update_cache)
        beam_mesh = assembly.beam(beam_id).cached_mesh  # type: Mesh
        v, f = beam_mesh.to_vertices_and_faces()
        layer = 'itj::interactive::beams_mesh'
        guid = draw_mesh(v, f, name=beam_id, layer=layer)
        self.interactive_guids[beam_id][layer].append(guid)

    def redraw_interactive_beam(self, beam_id, force_update=True, draw_mesh=True, draw_tag=True):
        ''' Redraw beam visualizations.
        Redraws interactive beam mesh and sequence tag
        '''
        self.delete_interactive_beam_visualization(beam_id)
        if draw_mesh:
            self.draw_beam_mesh(beam_id, force_update)
        if draw_tag:
            self.draw_beam_seqtag(beam_id)

    def interactive_beam_guid(self, beam_id, layer='itj::interactive::beams_mesh'):
        # type:(str, str) -> list(str)
        ''' Returns the interactive beam's guid(s) 
        Typically this is a list of one mesh that represent the beam. 
        '''
        return self.interactive_guids[beam_id][layer]

    ######################
    # Beam in Interactive Layers
    # Show Hide Color
    ######################

    def show_interactive_beam(self, beam_id):
        """ Show the beam of the beam_id.
        """
        for layer in self.interactive_layers:
            rs.ShowObject(self.interactive_guids[beam_id][layer])

    def hide_interactive_beam(self, beam_id):
        """ Show the beam of the beam_id.
        """
        for layer in self.interactive_layers:
            rs.HideObject(self.interactive_guids[beam_id][layer])

    def show_interactive_beam_until(self, beam_id):
        """ Show only the beams before and the given beam_id, others are hidden.
        If beam_id is None, all beams are hidden.
        If beam_id is not in the list of sequence, all beams are shown.
        """

        assembly = self.process.assembly
        show = True
        if beam_id is None:
            show = False
        # Looping through beams in sequence, flip the show switch after it reaches the beam_id
        for _beam_id in assembly.sequence:
            if show:
                self.show_interactive_beam(_beam_id)
            else:
                self.hide_interactive_beam(_beam_id)
            if _beam_id == beam_id:
                show = False

    def change_interactive_beam_colour(self, beam_id, meaning, layer='itj::interactive::beams_mesh'):
        # type(str, str) -> None
        """ Chagne the beam brep and mesh color to a given colour string
        Colour string refer to color_meaning dict
        """
        for guid in self.interactive_guids[beam_id][layer]:
            rs.ObjectColor(guid, self.color_meaning.get(meaning, (0, 0, 0)))

    #############################
    # Beam in different positions
    #############################

    def draw_beam_all_positions(self, beam_id, delete_old=False):
        """ Delete old beam geometry if delete_old is True
        Redraw them in Rhino in different layers.
        The resulting Rhino guids are kept in self.beam_guids[beam_id][position]

        This applies to all positions where the attribute is set in beam attributes.
        """
        for beam_position in ProcessKeyPosition.beam_positions:
            layer_name = 'itj::beam::' + beam_position
            # If not delete_old, and there are already items drawn, we preserve them.
            if len(self.beam_guids[beam_id][beam_position]) > 0 and not delete_old:
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
            beam_mesh = self.process.assembly.beam(beam_id).cached_mesh.transformed(T)  # type: Mesh
            v, f = beam_mesh.to_vertices_and_faces()
            guid = draw_mesh(v, f, name=beam_id, layer=layer_name)
            self.beam_guids[beam_id][beam_position].append(guid)

    def delete_beam_at_position(self, beam_id, beam_position):
        """Delete all Rhino geometry associated to a beam at specified position
        """
        guids = self.beam_guids[beam_id][beam_position]
        if len(guids) > 0:
            delete_objects(guids)
            self.beam_guids[beam_id][beam_position] = []

    def show_beam_at_one_position(self, beam_id, position=None):
        """ Show Beam only at the specified position.
        Position is the position attribute name, if left None, selected_key_position will be used. 
        """
        if position is None:
            position = self.selected_key_position.current_beam_pos

        for beam_position in ProcessKeyPosition.beam_positions:
            if beam_position == position:
                rs.ShowObject(self.beam_guids[beam_id][beam_position])
            else:
                rs.HideObject(self.beam_guids[beam_id][beam_position])

    def hide_beam_all_positions(self, beam_id):
        """ Hide all gripper instances in the specified positions.
        `positions` are defaulted to all position.
        # """

        self.show_beam_at_one_position(beam_id, '')

    def draw_beam_brep(self, process):
        raise NotImplementedError

    def delete_interactive_beam_visualization(self, beam_id):
        # type:(str) -> None
        """ Delete visualization geometry geometry (brep, mesh, tag etc) related to a beam.
        Tools are not affected.
        Stored guid reference is also removed.

        If beam_id is not yet tracked in self.guid, the new entry will be created.
        """
        if beam_id in self.interactive_guids:
            for layer in self.interactive_guids[beam_id]:
                delete_objects(self.interactive_guids[beam_id][layer])
                self.interactive_guids[beam_id][layer] = []
        else:
            self.interactive_guids[beam_id] = {}
            for layer in self.interactive_layers:
                self.interactive_guids[beam_id][layer] = []

    @property
    def all_layer_names(self):
        for layer in self.interactive_layers:
            yield layer
        for gripper_position in ProcessKeyPosition.gripper_positions:
            yield 'itj::gripper::' + gripper_position
        for clamp_position in ProcessKeyPosition.clamp_positions:
            yield 'itj::clamp::' + clamp_position
        for beam_position in ProcessKeyPosition.beam_positions:
            yield 'itj::beam::' + beam_position

    def empty_layers(self):
        # type:() -> None
        """Clear the default artist layers in Rhino.
        Create those layers if doesnt exist
        Gguid dictionary is reset to all empty lists.
        """
        for layer in self.interactive_layers:
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

        # Create new guid dictionary / clear out previously-saved guids.
        for beam_id in self.process.assembly.beam_ids():
            for layer in self.interactive_layers:
                self.interactive_guids[beam_id][layer] = []
            for position in ProcessKeyPosition.beam_positions:
                self.beam_guids[beam_id][position] = []
            for position in ProcessKeyPosition.gripper_positions:
                self.gripper_guids[beam_id][position] = []
            for joint_id in self.process.assembly.get_joint_ids_of_beam(beam_id):
                for position in ProcessKeyPosition.clamp_positions:
                    self.clamp_guids[joint_id][position] = []

    ######################
    # Drawing Gripper
    ######################

    def draw_gripper_all_positions(self, beam_id, delete_old=False):
        """ Delete old gripper geometry if delete_old is True
        Redraw them in Rhino in different layers.
        The resulting Rhino guids are kept in self.gripper_guids[beam_id][gripper_position]

        This applies to all positions where the attribute is set in beam attributes.
        """
        for gripper_position in ProcessKeyPosition.gripper_positions:
            layer_name = 'itj::gripper::' + gripper_position
            # If not delete_old, and there are already items drawn, we preserve them.
            if len(self.gripper_guids[beam_id][gripper_position]) > 0 and not delete_old:
                continue

            # Delete old geometry
            self.delete_gripper_at_position(beam_id, gripper_position)

            # Skip the rest of code if the position does not exist.
            if self.process.assembly.get_beam_attribute(beam_id, gripper_position) is None:
                print("Skipping Gripper on Beam(%s) at position: %s" % (beam_id, gripper_position))
                continue
            print("Drawing Gripper for Beam(%s) in position: %s" % (beam_id, gripper_position))
            gripper = self.process.get_gripper_of_beam(beam_id, gripper_position)
            artist = ToolArtist(gripper, layer_name)
            guid = gripper.draw_visual(artist)
            self.gripper_guids[beam_id][gripper_position].extend(guid)

    def delete_gripper_all_positions(self, beam_id):
        """Delete all Rhino geometry associated to a a gripper.
        All positions are deleted
        """
        for gripper_position in ProcessKeyPosition.gripper_positions:
            self.delete_gripper_at_position(beam_id, gripper_position)

    def delete_gripper_at_position(self, beam_id, gripper_position):
        """Delete all Rhino geometry associated to a gripper at specified position
        """
        guids = self.gripper_guids[beam_id][gripper_position]
        if len(guids) > 0:
            delete_objects(guids)
            self.gripper_guids[beam_id][gripper_position] = []

    def show_gripper_at_one_position(self, beam_id, position=None):
        """ Show Gripper only at the specified position.

        `position` is the position attribute name, if left `None`, 
        selected_key_position saved in artist will be used.
        """
        if position is None:
            position = self.selected_key_position.current_gripper_pos

        for gripper_position in ProcessKeyPosition.gripper_positions:
            if gripper_position == position:
                rs.ShowObject(self.gripper_guids[beam_id][gripper_position])
            else:
                rs.HideObject(self.gripper_guids[beam_id][gripper_position])

    def hide_gripper_all_positions(self, beam_id):
        """ Hide all gripper instances in the specified positions.

        `positions` are defaulted to all position.
        """

        self.show_gripper_at_one_position(beam_id, '')

    ######################
    # Drawing Clamp
    ######################

    def draw_clamp_all_positions(self, beam_id, delete_old=False):
        """ Delete old clamp geometry if delete_old is True
        Redraw them in Rhino in different layers.
        The resulting Rhino guids are kept in self.clamp_guids[joint_id][clamp_position]

        This applies to all positions where the attribute is set in joint attributes.
        """
        # clamp_id == joint_id
        # Loop through all clamps that are clamping this beam
        for joint_id in self.process.assembly.get_joint_ids_of_beam_clamps(beam_id, clamping_this_beam=True):
            # Loop through each position
            for clamp_position in ProcessKeyPosition.clamp_positions:
                layer_name = 'itj::clamp::' + clamp_position
                # If not delete_old, and there are already items drawn, we preserve them.
                if len(self.clamp_guids[joint_id][clamp_position]) > 0 and not delete_old:
                    continue

                # Delete old geometry
                self.delete_clamp_at_position(joint_id, clamp_position)

                # Skip the rest of code if the position does not exist.
                if self.process.assembly.get_joint_attribute(joint_id, clamp_position) is None:
                    print("Skipping clamp(%s) at position: %s" % (joint_id, clamp_position))
                    continue
                print("Drawing Clamp(%s) for Beam(%s) in position: %s" % (joint_id, beam_id, clamp_position))
                clamp = self.process.get_clamp_of_joint(joint_id, clamp_position)
                artist = ToolArtist(clamp, layer_name)
                guid = clamp.draw_visual(artist)
                # print ('new clamp_guids: ' , guid)
                self.clamp_guids[joint_id][clamp_position].extend(guid)
                # print ('saved clamp_guids' , self.clamp_guids[joint_id][clamp_position])

    def delete_clamp_all_positions(self, joint_id):
        """Delete all Rhino geometry associated to a a gripper.
        All positions are deleted
        """
        for clamp_position in ProcessKeyPosition.clamp_positions:
            self.delete_gripper_at_position(joint_id, clamp_position)

    def delete_clamp_at_position(self, joint_id, clamp_position):
        """Delete all Rhino geometry associated to a gripper at specified position
        """
        guids = self.clamp_guids[joint_id][clamp_position]
        if len(guids) > 0:
            delete_objects(guids)
            self.clamp_guids[joint_id][clamp_position] = []

    def show_clamp_at_one_position(self, beam_id, position=None, clamping_this_beam=True):
        """ Show Gripper only at the specified position.

        `position` is the position attribute name, if left `None`, 
        selected_key_position saved in artist will be used.
        """
        if position is None:
            position = self.selected_key_position.current_clamp_pos

        for joint_id in self.process.assembly.get_joint_ids_of_beam_clamps(beam_id, clamping_this_beam=clamping_this_beam):
            for clamp_position in ProcessKeyPosition.clamp_positions:
                if clamp_position == position:
                    rs.ShowObject(self.clamp_guids[joint_id][clamp_position])
                else:
                    rs.HideObject(self.clamp_guids[joint_id][clamp_position])

    def hide_clamp_all_positions(self, beam_id, clamping_this_beam=True):
        """ Hide all gripper instances in the specified positions.

        `positions` are defaulted to all position.
        """
        self.show_clamp_at_one_position(beam_id, '', clamping_this_beam=clamping_this_beam)

    ######################
    # Robot
    ######################


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
