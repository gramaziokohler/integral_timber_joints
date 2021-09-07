import uuid

import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext as sc  # type: ignore
from compas.datastructures import Mesh
from compas.geometry import Frame, Transformation
from compas.robots import Configuration
from compas_rhino.artists import MeshArtist, RobotModelArtist
from compas_rhino.geometry import RhinoPlane
from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh
from Rhino.DocObjects.ObjectColorSource import ColorFromObject  # type: ignore
from System.Drawing import Color  # type: ignore

from integral_timber_joints.assembly import Assembly, BeamAssemblyMethod
from integral_timber_joints.geometry import Beam
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.process.state import ObjectState, SceneState
from integral_timber_joints.rhino.tool_artist import ToolArtist
from integral_timber_joints.rhino.utility import purge_objects
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist
from integral_timber_joints.tools import Clamp, Gripper, Tool

try:
    from typing import Any, Dict, List, Optional, Tuple, Type
except:
    pass


TOL = sc.doc.ModelAbsoluteTolerance

guid = uuid.UUID


def AddAnnotationText(frame, text, height, layer, redraw=True):
    rs.EnableRedraw(False)
    font = "Arial"
    plane = RhinoPlane.from_geometry(frame).geometry
    justification = Rhino.Geometry.TextJustification.BottomLeft
    guid = sc.doc.Objects.AddText(text, plane, height, font, False, False, justification)
    import System.Guid  # type: ignore
    if guid != System.Guid.Empty:
        rs.ObjectLayer(guid, layer)
        # o = sc.doc.Objects.FindId(rs.coerceguid(guid))
        # o.Geometry.TextOrientation = Rhino.DocObjects.TextOrientation.InPlane
        # o.CommitChanges()
        if redraw:
            rs.EnableRedraw(True)
            sc.doc.Views.Redraw()
        return guid


class ProcessKeyPosition(object):
    def __init__(self, process=None, beam_id=None, current_pos_num=0):
        # type: (RobotClampAssemblyProcess, str, int) -> None
        """Initializing the Key Positions accoring to the assembly method of beam."""
        if process is not None and beam_id is not None:
            self.beam_tool_count = len(list(process.assembly.get_joint_ids_with_tools_for_beam(beam_id)))
            self.beam_assembly_method = process.assembly.get_assembly_method(beam_id)
        else:
            self.beam_tool_count = 0
            self.beam_assembly_method = BeamAssemblyMethod.UNDEFINED
        self.current_pos_num = current_pos_num

    # pos_name, beam_pos, gripper_pos, clamp_pos
    pos_names_for_beam_with_clamps = [
        ('clamp_attachapproach1',   'assembly_wcf_storage',         None,
         'clamp_wcf_attachapproach1.open_clamp.open_gripper'),
        ('clamp_attachapproach2',   'assembly_wcf_storage',         None,
         'clamp_wcf_attachapproach2.open_clamp.open_gripper'),
        ('beam_pickup_approach',    'assembly_wcf_pickup',
         'assembly_wcf_pickupapproach.open_gripper',         'clamp_wcf_final.open_clamp.close_gripper'),
        ('beam_pickup_pick',        'assembly_wcf_pickup',
         'assembly_wcf_pickup.close_gripper',                'clamp_wcf_final.open_clamp.close_gripper'),
        ('beam_pickup_retract',     'assembly_wcf_pickupretract',
         'assembly_wcf_pickupretract.close_gripper',         'clamp_wcf_final.open_clamp.close_gripper'),
        ('beam_inclampapproach',    'assembly_wcf_inclampapproach',
         'assembly_wcf_inclampapproach.close_gripper',       'clamp_wcf_final.open_clamp.close_gripper'),
        ('beam_inclamp',            'assembly_wcf_inclamp',
         'assembly_wcf_inclamp.close_gripper',               'clamp_wcf_final.open_clamp.close_gripper'),
        ('beam_final',              'assembly_wcf_final',
         'assembly_wcf_final.close_gripper',                 'clamp_wcf_final.close_clamp.close_gripper'),
        ('beam_finalretract',       'assembly_wcf_final',
         'assembly_wcf_finalretract.open_gripper',           'clamp_wcf_final.close_clamp.close_gripper'),
        ('clamp_detachretract1',    'assembly_wcf_final',           None,
         'clamp_wcf_detachretract1.open_clamp.open_gripper'),
        ('clamp_detachretract2',    'assembly_wcf_final',           None,
         'clamp_wcf_detachretract2.open_clamp.open_gripper'),
    ]

    pos_names_for_beam_without_clamps = [
        ('beam_pickup_approach',
         'assembly_wcf_pickup',  # Beam Position
         'assembly_wcf_pickupapproach.open_gripper',  # Gripper Position
         None),

        ('beam_pickup_pick',
         'assembly_wcf_pickup',  # Beam Position
         'assembly_wcf_pickup.close_gripper',  # Gripper Position
         None),

        ('beam_pickup_retract',
         'assembly_wcf_pickupretract',  # Beam Position
         'assembly_wcf_pickupretract.close_gripper',  # Gripper Position
         None),

        ('beam_inclamp',
         'assembly_wcf_inclamp',  # Beam Position
         'assembly_wcf_inclamp.close_gripper',  # Gripper Position
         None),

        ('beam_final',
         'assembly_wcf_final',  # Beam Position
         'assembly_wcf_final.close_gripper',  # Gripper Position
         None),

        ('beam_finalretract',
         'assembly_wcf_final',  # Beam Position
         'assembly_wcf_finalretract.open_gripper',  # Gripper Position
         None),
    ]

    # pos_name, beam_pos, gripper_pos, screwdriver_pos
    pos_names_for_beam_with_screwdriver_with_gripper = [
        ('beam_pickup',
         'assembly_wcf_pickup',  # Beam Position
         'assembly_wcf_pickup.close_gripper',  # Gripper Position
         'screwdriver_pickup_attached.close_gripper'),  # Screwdriver Position

        ('screwdriver_assembleapproach',
         'assembly_wcf_assembleapproach',  # Beam Position
         'assembly_wcf_assembleapproach.close_gripper',  # Gripper Position
         'screwdriver_assembleapproach_attached.close_gripper'),  # Screwdriver Position

        ('screwdriver_assembled',
         'assembly_wcf_final',  # Beam Position
         'assembly_wcf_final.close_gripper',  # Gripper Position
         'screwdriver_assembled_attached.close_gripper'),  # Screwdriver Position

        ('screwdriver_assembled',
         'assembly_wcf_final',  # Beam Position
         'assembly_wcf_finalretract.open_gripper',  # Gripper Position
         'screwdriver_assembled_attached.close_gripper'),  # Screwdriver Position

        ('screwdriver_retracted',
         'assembly_wcf_final',  # Beam Position
         None,  # Gripper Position
         'screwdriver_assembled_retractedfurther.open_gripper'),  # Screwdriver Position
    ]

    # pos_name, beam_pos, gripper_pos, screwdriver_pos
    pos_names_for_beam_with_screwdriver_without_gripper = [
        ('beam_pickup',
         'assembly_wcf_pickup',  # Beam Position
         None,
         'screwdriver_pickup_attached.close_gripper'),  # Screwdriver Position

        ('screwdriver_assembleapproach',
         'assembly_wcf_assembleapproach',  # Beam Position
         None,
         'screwdriver_assembleapproach_attached.close_gripper'),  # Screwdriver Position

        ('screwdriver_assembled',
         'assembly_wcf_final',  # Beam Position
         None,
         'screwdriver_assembled_attached.close_gripper'),  # Screwdriver Position

        ('screwdriver_retracted',
         'assembly_wcf_final',  # Beam Position
         None,
         'screwdriver_assembled_retracted.open_gripper'),  # Screwdriver Position
    ]

    @property
    def _all_pos_names(self):
        return self.pos_names_for_beam_with_clamps +\
            self.pos_names_for_beam_without_clamps +\
            self.pos_names_for_beam_with_screwdriver_with_gripper +\
            self.pos_names_for_beam_with_screwdriver_without_gripper

    @property
    def possible_beam_positions(self):
        # type: () -> set[str]
        """All possible beam positions. Used for creating layers in Rhino"""
        positions = [names[1] for names in self._all_pos_names if names[1] is not None]
        return set(positions)

    @property
    def possible_gripper_positions(self):
        # type: () -> set[str]
        """All possible gripper positions. Used for creating layers in Rhino"""
        positions = [names[2] for names in self._all_pos_names if names[2] is not None]
        return set(positions)

    @property
    def possible_tool_positions(self):
        # type: () -> set[str]
        """All possible tool positions. Used for creating layers in Rhino"""
        positions = [names[3] for names in self._all_pos_names if names[3] is not None]
        return set(positions)

    def next_position(self):
        self.current_pos_num += 1
        if self.current_pos_num >= self.total_pos_number:
            self.current_pos_num = self.total_pos_number - 1

    def prev_position(self):
        self.current_pos_num += -1
        if self.current_pos_num < 0:
            self.current_pos_num = 0

    def first_position(self):
        """ Set `self.current_pos_num` to beam_final
        """
        self.current_pos_num = 0

    def final_position(self):
        """ Set `self.current_pos_num` to beam_final
        """
        if self.beam_tool_count == 0:
            self.current_pos_num = 5
        elif self.beam_assembly_method == BeamAssemblyMethod.CLAMPED:
            self.current_pos_num = 7
        elif self.beam_assembly_method == BeamAssemblyMethod.SCREWED_WITH_GRIPPER:
            self.current_pos_num = 2
        elif self.beam_assembly_method == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
            self.current_pos_num = 2
        else:
            self.current_pos_num = 0

    def _get_pos_names(self):
        # type: () -> List[Tuple[str,str,str,str]]
        """Getting the right pos_name array depending on the assembly method"""

        # Switching between two sets of key positions depending if it has clamps
        if self.beam_tool_count == 0:
            return self.pos_names_for_beam_without_clamps
        elif self.beam_assembly_method == BeamAssemblyMethod.CLAMPED:
            return self.pos_names_for_beam_with_clamps
        elif self.beam_assembly_method == BeamAssemblyMethod.SCREWED_WITH_GRIPPER:
            return self.pos_names_for_beam_with_screwdriver_with_gripper
        elif self.beam_assembly_method == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
            return self.pos_names_for_beam_with_screwdriver_without_gripper
        else:
            return []

    @property
    def _get_current_pos_names(self):
        # type: () -> Tuple[str,str,str,str]
        """Getting the 4 part Tuple of where things are.
        Order is: pos_name, beam_pos, gripper_pos, assembly_tool_pos
        """
        # just in case
        if self.current_pos_num >= self.total_pos_number:
            self.final_position()

        # Switching between two sets of key positions depending if it has clamps
        pos_names = self._get_pos_names()
        return pos_names[self.current_pos_num]

    @property
    def current_pos_name(self):
        # type: () -> str
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
        return len(self._get_pos_names())

    @property
    def get_all_asstool_positions(self):
        """Returning all possible Assembly Tool Positions according to the current beam"""
        return set([x[3] for x in self._get_pos_names()])

    def to_data(self):
        # type: () -> dict[str, Any]
        data = {'current_pos_num': self.current_pos_num,
                'beam_tool_count': self.beam_tool_count,
                'beam_assembly_method': self.beam_assembly_method,
                }
        return data

    @classmethod
    def from_data(cls, data):
        # type: (dict[str, Any]) -> ProcessKeyPosition
        p = cls()
        p.current_beam_pos = data.get('current_pos_num', 0)
        p.beam_tool_count = data.get('beam_tool_count', 0)
        p.beam_assembly_method = data.get('beam_assembly_method', BeamAssemblyMethod.UNDEFINED)
        return p


class ProcessArtist(object):
    """ Artist to draw Beams in Rhino
    Items are drawn in specific layers for quickly turning them on and off.
    Beam Brep : layer = itj::beams_brep         name = beam_id
    Beam Mesh : layer =  itj::beams_mesh        name = beam_id
    Beam Sequence Tag - itj::beams_seqtag
    self.gripper_guids, asstool_guids, beam_guids, interactive_guids
    are dictionary that keep track of the objects drawn in Rhino.
    """

    # Rhino layers used to hold temporary visualization objects
    interactive_layers = [
        'itj::interactive::beams_mesh',
        'itj::interactive::beams_brep',
        'itj::interactive::beams_seqtag',
        'itj::interactive::clamps_at_final',
    ]

    state_visualization_layer = 'itj::state_visualization'
    tools_in_storage_layer = 'itj::tools::in_storage'
    env_mesh_layer = 'itj::envmesh'
    robot_layer = 'itj::robot'

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
        'env_model': (230, 153, 0),
        'assembly_method_undefined': (176, 65, 62),  # red
        'assembly_method_ground': (71, 51, 53),  # black
        'assembly_method_clamped': (84, 155, 135),  # green
        'assembly_method_screwed_w_gripper': (126, 178, 221),  # lightblue
        'assembly_method_screwed_wo_gripper': (68, 94, 147),  # deepblue
    }

    key_positions = [
        'storage_approach',

    ]

    def __init__(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        self.process = process
        self.assembly_artist = AssemblyNurbsArtist(process.assembly, 'itj::interactive::beams_brep')

        # # Create guid in dictionary to store geometries added to Rhino document
        self._beam_guids = {}  # type: dict[str, dict[str, list[str]]]
        self._gripper_guids = {}  # type: dict[str, dict[str, list[str]]]
        self._asstool_guids = {}  # type: dict[str, dict[str, list[str]]]
        self._interactive_guids = {}  # type: dict[str, dict[str, list[str]]]
        self._state_visualization_guids = {}  # type: dict[str, list[str]]
        self._tools_in_storage_guids = {}  # type: dict[str, list[str]]
        self._env_mesh_guids = {}  # type: dict[str, list[str]]
        self._robot_guids = {'visual': [], 'collision': []}  # type: dict[str, list[str]]

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
        self.selected_state_id = 0  # type: str # State Id = Start State of Movement with the same ID"""
        self.selected_key_position = ProcessKeyPosition(process, self.selected_beam_id, 0)

        # Robot
        if self.process.robot_model is not None:
            print("Creating new RobotModelArtist")
            self.robot_artist = RobotModelArtist(self.process.robot_model, self.robot_layer)
            self.robot_artist.scale(1000)
        else:
            self.robot_artist = None

    #######################################
    # Functions to handle the guid records
    #######################################
    def beam_guids(self, beam_id):
        # type: (str) -> dict[str, list[guid]]
        if beam_id not in self._beam_guids:
            self._beam_guids[beam_id] = {}
        return self._beam_guids[beam_id]

    def beam_guids_at_position(self, beam_id, position_id):
        # type: (str, str) -> list[guid]
        if position_id not in self.beam_guids(beam_id):
            self.beam_guids(beam_id)[position_id] = []
        return self.beam_guids(beam_id)[position_id]

    def gripper_guids(self, beam_id):
        # type: (str) -> dict[str, list[guid]]
        if beam_id not in self._gripper_guids:
            self._gripper_guids[beam_id] = {}
        return self._gripper_guids[beam_id]

    def gripper_guids_at_position(self, beam_id, position_id):
        # type: (str, str) -> list[guid]
        if position_id not in self.gripper_guids(beam_id):
            self.gripper_guids(beam_id)[position_id] = []
        return self.gripper_guids(beam_id)[position_id]

    def asstool_guids(self, joint_id):
        # type: (tuple(str, str)) -> dict[str, list[guid]]
        if joint_id not in self._asstool_guids:
            self._asstool_guids[joint_id] = {}
        return self._asstool_guids[joint_id]

    def asstool_guids_at_position(self, joint_id, position_id):
        # type: (str, str) -> list[guid]
        if position_id not in self.asstool_guids(joint_id):
            self.asstool_guids(joint_id)[position_id] = []
        return self.asstool_guids(joint_id)[position_id]

    def interactive_guids(self, beam_id):
        # type: (tuple(str, str)) -> dict[str, list[guid]]
        if beam_id not in self._interactive_guids:
            self._interactive_guids[beam_id] = {}
        return self._interactive_guids[beam_id]

    def interactive_guids_at_layer(self, beam_id, layer_name):
        # type: (str, str) -> list[guid]
        if layer_name not in self.interactive_guids(beam_id):
            self.interactive_guids(beam_id)[layer_name] = []
        return self.interactive_guids(beam_id)[layer_name]

    def state_visualization_guids(self, object_id):
        # type: (str) -> list[guid]
        if object_id not in self._state_visualization_guids:
            self._state_visualization_guids[object_id] = []
        return self._state_visualization_guids[object_id]

    def tools_in_storage_guids(self, tool_id):
        # type: (str) -> list[guid]
        if tool_id not in self._tools_in_storage_guids:
            self._tools_in_storage_guids[tool_id] = []
        return self._tools_in_storage_guids[tool_id]

    def env_mesh_guids(self, env_id):
        # type: (str) -> list[guid]
        if env_id not in self._env_mesh_guids:
            self._env_mesh_guids[env_id] = []
        return self._env_mesh_guids[env_id]

    ###########################################################
    # Functions to keep track of user selected interactive beam
    ###########################################################

    @property
    def selected_beam_id(self):
        return self._selected_beam_id

    @selected_beam_id.setter
    def selected_beam_id(self, beam_id):
        # Do not change anything if the id is the same
        if beam_id == self._selected_beam_id:
            return
        self._selected_beam_id = beam_id

        # update selected_key_position object
        if beam_id is not None:
            self.selected_key_position = ProcessKeyPosition(self.process, self.selected_beam_id, 0)
            self.selected_key_position.final_position()

    def select_next_beam(self):
        # type: () -> str
        """ Increment self.selected_beam_id based on its seq_num """
        assembly = self.process.assembly
        seq_num = assembly.get_beam_sequence(self.selected_beam_id) + 1
        seq_num = min(seq_num,  len(assembly.sequence) - 1)  # seq_num not more than len(assembly.sequence) - 1
        self.selected_beam_id = assembly.sequence[seq_num]

    def select_previous_beam(self):
        # type: () -> str
        """ Increment self.selected_beam_id based on its seq_num """
        assembly = self.process.assembly
        seq_num = assembly.get_beam_sequence(self.selected_beam_id) - 1
        seq_num = max(seq_num,  0)  # seq_num not less than 0
        self.selected_beam_id = assembly.sequence[seq_num]

    #############################
    # Beam in Interactive Layers
    #############################

    def draw_beam_seqtag(self, beam_id, faces=[1, 3], padding_factor=0.2, size_factor=0.6, redraw=True):
        assembly = self.process.assembly
        seq_num = assembly.sequence.index(beam_id)
        rs.EnableRedraw(False)
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
            guid = AddAnnotationText(face_frame, tag_text, tag_height, layer, redraw=redraw)
            self.interactive_guids_at_layer(beam_id, layer).append(guid)
        if redraw:
            rs.EnableRedraw(True)

    def draw_beam_mesh(self, beam_id, update_cache=False, redraw=True):
        # type:(str, bool, bool) -> None
        assembly = self.process.assembly
        if beam_id not in assembly.beam_ids():
            raise KeyError("Beam %i not in Assembly" % beam_id)
        beam_mesh = assembly.get_beam_mesh_in_wcf(beam_id, not update_cache)

        # Layer
        layer = 'itj::interactive::beams_mesh'
        rs.CurrentLayer(layer)

        # Draw Mesh
        guids = self.draw_meshes_get_guids([beam_mesh], beam_id)
        self.interactive_guids_at_layer(beam_id, layer).extend(guids)

        # Redraw
        if redraw:
            rs.EnableRedraw(True)

    def draw_beam_brep(self, beam_id, delete_old_brep=True, update_mesh_cache=False, redraw=True):
        # type:(str, bool, bool, bool) -> None
        assembly = self.process.assembly
        if beam_id not in assembly.beam_ids():
            raise KeyError("Beam %i not in Assembly" % beam_id)

        if update_mesh_cache:
            assembly.beam(beam_id).remove_cached_mesh()
            assembly.get_beam_mesh_in_wcf(beam_id, False)

        # Layer
        layer = 'itj::interactive::beams_brep'
        rs.CurrentLayer(layer)

        # Draw Nurbs using Nurbs Assembly Artist
        other_feature_shapes = []
        # for joint_id in self.process.assembly.get_joint_ids_with_tools_for_beam(beam_id):
        #     try:
        #         tool = self.process.get_tool_of_joint(joint_id)
        #         tool.set_state
        #         other_feature_shapes.
        #     except:
        #         print ("Warninig: cannot get tool of joint %s for Boolean" % joint_id)

        guids = self.assembly_artist.draw_beam(beam_id=beam_id, delete_old=delete_old_brep, redraw=False, other_feature_shapes=other_feature_shapes)
        self.interactive_guids_at_layer(beam_id, layer).extend(guids)

        # Redraw
        if redraw:
            rs.EnableRedraw(True)

    def redraw_interactive_beam(self, beam_id, force_update=True, draw_mesh=False, draw_nurbs=True, draw_tag=True, redraw=True):
        ''' Redraw beam visualizations.
        Redraws interactive beam mesh and sequence tag
        '''
        rs.EnableRedraw(False)
        self.delete_interactive_beam_visualization(beam_id, redraw=False)
        if draw_mesh:
            self.draw_beam_mesh(beam_id, update_cache=force_update, redraw=False)
        if draw_nurbs:
            self.draw_beam_brep(beam_id, delete_old_brep=force_update, update_mesh_cache=False, redraw=False)
        if draw_tag:
            self.draw_beam_seqtag(beam_id, redraw=False)
        if redraw:
            rs.EnableRedraw(True)

    def interactive_beam_guid(self, beam_id, layer='itj::interactive::beams_brep'):
        # type:(str, str) -> list[guid]
        ''' Returns the interactive beam's guid(s)
        Typically this is a list of one mesh that represent the beam.
        '''
        return self.interactive_guids_at_layer(beam_id, layer)

    ######################
    # Beam in Interactive Layers
    # Show Hide Color
    ######################

    def show_interactive_beam(self, beam_id):
        """ Show the beam of the beam_id.
        """
        for layer in self.interactive_layers:
            rs.ShowObject(self.interactive_guids_at_layer(beam_id, layer))

    def hide_interactive_beam(self, beam_id):
        """ Show the beam of the beam_id.
        """
        for layer in self.interactive_layers:
            rs.HideObject(self.interactive_guids_at_layer(beam_id, layer))

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

    def change_interactive_beam_colour(self, beam_id, meaning, layer='itj::interactive::beams_brep'):
        # type(str, str) -> None
        """ Chagne the beam brep and mesh color to a given colour string
        Colour string refer to color_meaning dict
        """
        for guid in self.interactive_guids_at_layer(beam_id, layer):
            rs.ObjectColor(guid, self.color_meaning.get(meaning, (0, 0, 0)))

    #############################
    # Beam in different positions
    #############################

    def draw_beam_all_positions(self, beam_id, delete_old=False, verbose=False, redraw=True):
        """ Delete old beam geometry if delete_old is True
        Redraw them in Rhino in different layers.
        The resulting Rhino guids are kept in self.beam_guids_at_position(beam_id, position)

        This applies to all positions where the attribute is set in beam attributes.
        """
        rs.EnableRedraw(False)

        for beam_position in ProcessKeyPosition().possible_beam_positions:
            layer_name = 'itj::beam::' + beam_position
            # If not delete_old, and there are already items drawn, we preserve them.
            if len(self.beam_guids_at_position(beam_id, beam_position)) > 0 and not delete_old:
                continue

            # Delete old geometry
            self.delete_beam_at_position(beam_id, beam_position, redraw=False)

            # Skip the rest of code if the position does not exist.
            if self.process.assembly.get_beam_attribute(beam_id, beam_position) is None:
                if verbose:
                    print("Skipping Beam (%s) position: %s" % (beam_id, beam_position))
                continue

            if verbose:
                print("Drawing Beam(%s) in position: %s" % (beam_id, beam_position))

            # Transform the beam_mesh to location and
            T = self.process.assembly.get_beam_transformaion_to(beam_id, beam_position)

            beam_mesh = self.process.assembly.get_beam_mesh_in_wcf(beam_id).transformed(T)  # type: Mesh
            guids = self.draw_meshes_get_guids([beam_mesh], beam_id, redraw=False)
            self.beam_guids_at_position(beam_id, beam_position).extend(guids)

        if redraw:
            rs.EnableRedraw(True)

    def delete_beam_all_positions(self, beam_id, redraw=True):
        """Delete all Rhino geometry associated to a beam at all position.
        """
        rs.EnableRedraw(False)
        for beam_position in ProcessKeyPosition().possible_beam_positions:
            # The redraw is supressed in each individual call to save time.
            self.delete_beam_at_position(beam_id, beam_position, redraw=False)
        if redraw:
            rs.EnableRedraw(True)

    def delete_beam_at_position(self, beam_id, beam_position, redraw=True):
        # type:(str, str, bool) -> None
        """Delete all Rhino geometry associated to a beam at specified position

        No change will be made if the beam_id or beam_position do not exist in the guid dictionary.
        """
        if len(self.beam_guids_at_position(beam_id, beam_position)) == 0:
            return
        guids = self.beam_guids_at_position(beam_id, beam_position)
        if len(guids) > 0:
            purge_objects(guids, redraw)
            del self.beam_guids_at_position(beam_id, beam_position)[:]

    def show_beam_at_one_position(self, beam_id, position=None):
        """ Show Beam only at the specified position.
        Position is the position attribute name, if left None, selected_key_position will be used.
        """
        if position is None:
            position = self.selected_key_position.current_beam_pos

        for beam_position in ProcessKeyPosition().possible_beam_positions:
            if beam_position == position:
                rs.ShowObject(self.beam_guids_at_position(beam_id, beam_position))
            else:
                rs.HideObject(self.beam_guids_at_position(beam_id, beam_position))

    def hide_beam_all_positions(self, beam_id):
        """ Hide all gripper instances in the specified positions.
        `positions` are defaulted to all position.
        # """

        self.show_beam_at_one_position(beam_id, '')

    def delete_interactive_beam_visualization(self, beam_id, redraw=True):
        # type:(str, bool) -> None
        """ Delete visualization geometry geometry (brep, mesh, tag etc) related to a beam.
        Tools are not affected.
        Stored guid reference is also removed.

        If beam_id is not yet tracked in self.guid, the new entry will be created.
        """
        rs.EnableRedraw(False)
        if beam_id in self.process.assembly.beam_ids():
            for layer in self.interactive_layers:
                if len(self.interactive_guids_at_layer(beam_id, layer)) > 0:
                    purge_objects(self.interactive_guids_at_layer(beam_id, layer), redraw=False)
                    del self.interactive_guids_at_layer(beam_id, layer)[:]
        if redraw:
            rs.EnableRedraw(True)

    @property
    def all_layer_names(self):
        for layer in self.interactive_layers:
            yield layer
        for gripper_position in ProcessKeyPosition().possible_gripper_positions:
            yield 'itj::gripper::' + gripper_position
        for tool_position in ProcessKeyPosition().possible_tool_positions:
            yield 'itj::tool::' + tool_position
        for beam_position in ProcessKeyPosition().possible_beam_positions:
            yield 'itj::beam::' + beam_position
        yield self.state_visualization_layer
        yield self.tools_in_storage_layer
        yield self.env_mesh_layer
        yield self.robot_layer

    def empty_layers(self):
        # type:() -> None
        """Clear the default artist layers in Rhino.
        Create those layers if doesnt exist
        Gguid dictionary is reset to all empty lists.
        """
        # Clear Interactive, Gripper, Clamp, BeamPos, State Visualization layers
        for layer_name in self.all_layer_names:
            if not rs.IsLayer(layer_name):
                rs.AddLayer(layer_name)
            else:
                clear_layer(layer_name)

        # Create new guid dictionary / clear out previously-saved guids.
        for beam_id in self.process.assembly.beam_ids():
            self.delete_interactive_beam_visualization(beam_id, redraw=False)
            self.delete_beam_all_positions(beam_id=beam_id, redraw=False)
            self.delete_gripper_all_positions(beam_id, redraw=False)
            for joint_id in self.process.assembly.get_joint_ids_of_beam(beam_id):
                self.delete_asstool_all_positions(joint_id=joint_id, redraw=False)

        self._state_visualization_guids = {}
        self._tools_in_storage_guids = {}

    ######################
    # Drawing Gripper
    ######################

    def draw_gripper_all_positions(self, beam_id, delete_old=False, verbose=False, redraw=True):
        """ Delete old gripper geometry if delete_old is True
        Redraw them in Rhino in different layers.
        The resulting Rhino guids are kept in self.gripper_guids[beam_id][gripper_position]

        This applies to all positions where the attribute is set in beam attributes.
        """
        rs.EnableRedraw(False)

        for gripper_position in ProcessKeyPosition().possible_gripper_positions:
            layer_name = 'itj::gripper::' + gripper_position
            # If not delete_old, and there are already items drawn, we preserve them.
            if len(self.gripper_guids_at_position(beam_id, gripper_position)) > 0 and not delete_old:
                continue

            # Check if the position string contains a dot notation for states , such as open gripper
            tool_states = []  # tool_states are function names that chagen state of the tool
            attribute_name = gripper_position
            if '.' in gripper_position:
                attribute_name = gripper_position.split('.')[0]
                tool_states = gripper_position.split('.')[1:]
            layer_name = 'itj::gripper::' + gripper_position

            # Delete old geometry
            self.delete_gripper_at_position(beam_id, gripper_position, redraw=False)

            # Skip the rest of code if the position does not exist.
            if self.process.assembly.get_beam_attribute(beam_id, attribute_name) is None:
                if verbose:
                    print("Skipping Gripper on Beam(%s) at position: %s" % (beam_id, attribute_name))
                continue

            # Draw Gripper
            if verbose:
                print("Drawing Gripper for Beam(%s) in position: %s" % (beam_id, attribute_name))
            gripper = self.process.get_gripper_of_beam(beam_id, attribute_name)

            # Set Tool State (better visualization)
            for state in tool_states:
                getattr(gripper, state)()

            artist = ToolArtist(gripper, layer_name)
            new_guids = gripper.draw_visual(artist)
            self.gripper_guids_at_position(beam_id, gripper_position).extend(new_guids)

            # Draw ToolChanger and Robot Wrist
            new_guids = self.draw_toolchanger_and_robot_wrist(beam_id, gripper.current_frame, layer_name, )
            self.gripper_guids_at_position(beam_id, gripper_position).extend(new_guids)

        if redraw:
            rs.EnableRedraw(True)

    def draw_toolchanger_and_robot_wrist(self, beam_id, tcp_frame, layer_name):
        # type: (str, Frame, str) -> list[guid]
        new_guids = []

        # Draw Tool Changer with TCP at tcp_frame
        tool_changer = self.process.robot_toolchanger
        tool_changer.set_current_frame_from_tcp(tcp_frame)
        new_guids.extend(tool_changer.draw_visual(ToolArtist(tool_changer, layer_name)))
        # Draw rob_wrist at tool_changer.current_frame
        robot_wrist = self.process.robot_wrist
        robot_wrist.current_frame = tool_changer.current_frame
        new_guids.extend(robot_wrist.draw_visual(ToolArtist(robot_wrist, layer_name)))

        return new_guids

    def delete_gripper_all_positions(self, beam_id, redraw=True):
        """Delete all Rhino geometry associated to a a gripper.
        All positions are deleted
        """
        rs.EnableRedraw(False)
        for gripper_position in ProcessKeyPosition().possible_gripper_positions:
            # The redraw is supressed in each individual call to save time.
            self.delete_gripper_at_position(beam_id, gripper_position, redraw=False)
        if redraw:
            rs.EnableRedraw(True)

    def delete_gripper_at_position(self, beam_id, gripper_position, redraw=True):
        """Delete all Rhino geometry associated to a gripper at specified position

        No change will be made if the beam_id or gripper_position do not exist in the guid dictionary.
        """

        guids = self.gripper_guids_at_position(beam_id, gripper_position)
        if len(guids) > 0:
            purge_objects(guids, redraw)
            del self.gripper_guids_at_position(beam_id, gripper_position)[:]

    def show_gripper_at_one_position(self, beam_id, position=None, color=None):
        """ Show Gripper only at the specified position.

        `position` is the position attribute name, if left `None`,
        selected_key_position saved in artist will be used.

        `color` is a string as used in artist.color_meaning.get(color)
        """
        if position is None:
            position = self.selected_key_position.current_gripper_pos

        for gripper_position in ProcessKeyPosition().possible_gripper_positions:
            if gripper_position == position:
                rs.ShowObject(self.gripper_guids_at_position(beam_id, gripper_position))
                # Change color for shown object
                if color is not None:
                    rs.ObjectColor(self.gripper_guids_at_position(beam_id, gripper_position), self.color_meaning.get(color))
            else:
                rs.HideObject(self.gripper_guids_at_position(beam_id, gripper_position))

    def hide_gripper_all_positions(self, beam_id):
        """ Hide all gripper instances in the specified positions.

        `positions` are defaulted to all position.
        """

        self.show_gripper_at_one_position(beam_id, '')

    ######################
    # Drawing Clamp
    ######################

    def draw_asstool_all_positions(self, beam_id, delete_old=False, verbose=False, redraw=True):
        """ Delete old Assembly Tool (Clamps/Screwdrivers) geometry if delete_old is True
        Redraw them in Rhino in different layers.
        The resulting Rhino guids are kept in self.asstool_guids(joint_id, clamp_position)

        This applies to all positions where the attribute is set in joint attributes.
        """
        rs.EnableRedraw(False)
        # Loop through all clamps that are clamping this beam
        for joint_id in self.process.assembly.get_joint_ids_with_tools_for_beam(beam_id):
            # Smart about the tool positions of this beam
            tool_positions = ProcessKeyPosition(self.process, beam_id).get_all_asstool_positions
            # Loop through each position
            for tool_position in tool_positions:
                # If not delete_old, and there are already items drawn, we preserve them.
                if len(self.asstool_guids_at_position(joint_id, tool_position)) > 0 and not delete_old:
                    continue

                # Check if the position string contains a dot notation for states , such as open gripper
                tool_states = []  # tool_states are function names that chagen state of the tool
                attribute_name = tool_position
                if '.' in tool_position:
                    attribute_name = tool_position.split('.')[0]
                    tool_states = tool_position.split('.')[1:]
                layer_name = 'itj::tool::' + tool_position

                # Delete old geometry
                self.delete_asstool_at_position(joint_id, tool_position, redraw=False)

                # Skip the rest of code if the position does not exist.
                if self.process.assembly.get_joint_attribute(joint_id, attribute_name) is None:
                    if verbose:
                        print("Skipping tool(%s) at position: %s" % (joint_id, attribute_name))
                    continue

                # Draw Tool
                if verbose:
                    print("Drawing Tool(%s) for Beam(%s) in position: %s" % (joint_id, beam_id, tool_position))
                tool = self.process.get_tool_of_joint(joint_id, attribute_name)

                # Set Tool State (better visualization)
                for state in tool_states:
                    getattr(tool, state)()

                artist = ToolArtist(tool, layer_name)
                new_guids = tool.draw_visual(artist)
                self.asstool_guids_at_position(joint_id, tool_position).extend(new_guids)

                # Draw ToolChanger and Robot Wrist
                new_guids = self.draw_toolchanger_and_robot_wrist(beam_id, tool.current_frame, layer_name)
                self.asstool_guids_at_position(joint_id, tool_position).extend(new_guids)
        if redraw:
            rs.EnableRedraw(True)

    def delete_asstool_all_positions(self, joint_id, redraw=True):
        """Delete all Rhino geometry associated to Assembly Tool
        (Clamp/Screwdriver). All positions are deleted
        """
        rs.EnableRedraw(False)
        beam_id = joint_id[1]
        for tool_position in ProcessKeyPosition(self.process, beam_id).get_all_asstool_positions:
            # The redraw is supressed in each individual call to save time.
            self.delete_asstool_at_position(joint_id, tool_position, redraw=False)
        if redraw:
            rs.EnableRedraw(True)

    def delete_asstool_at_position(self, joint_id, tool_position, redraw=True):
        """Delete all Rhino geometry associated to an Assembly Tool
        (Clamp/Screwdriver) at specified position

        No change will be made if the joint_id or tool_position
        do not exist in the guid dictionary.
        """

        guids = self.asstool_guids_at_position(joint_id, tool_position)
        if len(guids) > 0:
            purge_objects(guids, redraw=False)
            del self.asstool_guids_at_position(joint_id, tool_position)[:]
        if redraw:
            rs.EnableRedraw(True)

    def show_asstool_at_one_position(self, beam_id, position=None, color=None):
        """ Show Assembly Tool (Clamp/Screwdriver) only at the specified position.

        `position` is the position attribute name, if left `None`,
        selected_key_position saved in artist will be used.

        `color` is a string as used in artist.color_meaning.get(color)
        """
        if position is None:
            position = self.selected_key_position.current_clamp_pos

        for joint_id in self.process.assembly.get_joint_ids_with_tools_for_beam(beam_id):
            for tool_position in ProcessKeyPosition(self.process, beam_id).get_all_asstool_positions:
                if tool_position == position:
                    rs.ShowObject(self.asstool_guids_at_position(joint_id, tool_position))
                    # Change color for shown object
                    if color is not None:
                        rs.ObjectColor(self.asstool_guids_at_position(joint_id, tool_position), self.color_meaning.get(color))
                else:
                    rs.HideObject(self.asstool_guids_at_position(joint_id, tool_position))

    def hide_asstool_all_positions(self, beam_id):
        """ Hide all gripper instances in the specified positions.

        `positions` are defaulted to all position.
        """
        self.show_asstool_at_one_position(beam_id, '')

    #################################
    # Tools in storage Visualization
    #################################

    def draw_tool_in_storage(self, tool_id, delete_old=False):
        tool = self.process.tool(tool_id)
        if len(self.tools_in_storage_guids(tool_id)) > 0 and not delete_old:
            self.show_tool_in_storage(tool_id)
            return
        layer_name = self.tools_in_storage_layer

        # Set default state
        if isinstance(tool, Clamp):
            tool.open_clamp()
        if isinstance(tool, Gripper):
            tool.close_gripper()

        # Delete old geometry
            self.delete_tool_in_storage(tool_id)

        # Set Tool to storage frame
        tool.current_frame = tool.tool_storage_frame.copy()

        # Artist Draw Tool add Guids to dictionary
        artist = ToolArtist(tool, layer_name)
        new_guids = tool.draw_visual(artist)
        del self.tools_in_storage_guids(tool_id)[:]
        self.tools_in_storage_guids(tool_id).extend(new_guids)

    def show_tool_in_storage(self, tool_id):
        rs.ShowObject(self.tools_in_storage_guids(tool_id))

    def hide_tool_in_storage(self, tool_id):
        rs.HideObject(self.tools_in_storage_guids(tool_id))

    def hide_all_tools_in_storage(self):
        for tool_id in self._tools_in_storage_guids.keys():
            self.hide_tool_in_storage(tool_id)

    def delete_tool_in_storage(self, tool_id):
        purge_objects(self.tools_in_storage_guids(tool_id), redraw=False)
        del self.tools_in_storage_guids(tool_id)[:]

    def delete_all_tools_in_storage(self):
        for tool_id in self._tools_in_storage_guids.keys():
            self.delete_tool_in_storage(tool_id)

    #############
    # Env Mesh
    #############

    def draw_all_env_mesh(self, delete_old=False, redraw=True):
        rs.EnableRedraw(False)
        if delete_old:
            self.draw_all_env_mesh(redraw=False)
        rs.CurrentLayer(self.env_mesh_layer)
        for env_id, mesh in self.process.environment_models.items():
            # Draw the geometry only if it hasen't been drawn yet
            if len(self.env_mesh_guids(env_id)) == 0:
                guids = self.draw_meshes_get_guids([mesh], env_id, redraw=False, color=self.color_meaning['env_model'])
                self.env_mesh_guids(env_id).extend(guids)
        if redraw:
            rs.EnableRedraw(True)

    def delete_all_env_mesh(self, redraw=True):
        rs.EnableRedraw(False)
        for env_id, guids in self._env_mesh_guids.items():
            purge_objects(guids, redraw=False)
            del self.env_mesh_guids(env_id)[:]
        if redraw:
            rs.EnableRedraw(True)

    def hide_all_env_mesh(self, redraw=True):
        rs.EnableRedraw(False)
        for env_id in self._env_mesh_guids.keys():
            rs.HideObject(self.env_mesh_guids(env_id))
        if redraw:
            rs.EnableRedraw(True)

    def show_all_env_mesh(self, redraw=True):
        rs.EnableRedraw(False)
        for env_id in self._env_mesh_guids.keys():
            rs.ShowObject(self.env_mesh_guids(env_id))
        if redraw:
            rs.EnableRedraw(True)

    ######################
    # Robot
    ######################

    def draw_robot(self, configuration, draw_collision=True, auto_hide=True, redraw=False):
        # type: (Configuration, bool, bool, bool) -> None
        """Draws process.robot_model in Rhino viewport in mm scale.

        `configuration` can be full or partial Configuration of the RobotModel.
        If a partial config is given, the other values will be filled in from robot_initial_config.
        It must be in meter scale as consistent with process.robot_initial_config

        """
        # Skip if robot model is not set
        if self.process.robot_model is None:
            return

        # Update config
        merged_config = self.process.robot_initial_config.scaled(1000).merged(configuration.scaled(1000))
        self.robot_artist.update(merged_config)

        # Clear old geometry
        guid_key = 'collision' if draw_collision else 'visual'
        self.delete_robot(guid_key)

        # Redraw Robot
        rs.EnableRedraw(False)
        if auto_hide:
            self.hide_robot()
        if draw_collision:
            self._robot_guids[guid_key] = self.robot_artist.draw_collision()
        else:
            self._robot_guids[guid_key] = self.robot_artist.draw_visual()

        # Redraw
        if redraw:
            rs.EnableRedraw(True)

    def delete_robot(self, guid_key=None, redraw=False):
        # type: (str, bool) -> None
        if guid_key is None:
            guid_keys = ['collision', 'visual']
        else:
            guid_keys = [guid_key]

        for guid_key in guid_keys:
            purge_objects(self._robot_guids[guid_key], redraw=False)
            self._robot_guids[guid_key] = []

    def show_robot_collision(self, redraw=False):
        # type: (bool) -> None
        rs.EnableRedraw(False)
        for guid in self._robot_guids['collision']:
            rs.ShowObject(guid)
        print("Unhiding Robot collision")
        # Redraw
        if redraw:
            rs.EnableRedraw(True)

    def show_robot_visual(self, redraw=False):
        # type: (bool) -> None
        rs.EnableRedraw(False)
        for guid in self._robot_guids['visual']:
            rs.ShowObject(guid)
        print("Unhiding Robot visual")
        # Redraw
        if redraw:
            rs.EnableRedraw(True)

    def hide_robot(self, redraw=False):
        # type: (bool) -> None
        """Hide the RobotModel in Rhino. Not deleting it"""
        rs.EnableRedraw(False)

        for guid in self._robot_guids['collision']:
            rs.HideObject(guid)
        for guid in self._robot_guids['visual']:
            rs.HideObject(guid)

        if redraw:
            rs.EnableRedraw(True)

    ######################
    # State
    ######################

    def draw_meshes_get_guids(self, meshes, name, disjoint=True, redraw=False, color=None):
        """
        Draws a mesh to Rhino in a specific color and return the guids.

        Layer is not handeled here to avoid unnecessary repeated calls and save time.
        Call rs.CurrentLayer(layer) to change layer.

        To save time, redraw defaults to False, which will not trigger Rhino redrawing.
        You should call rs.EnableRedraw(True) after multiple calls to this function.
        """
        guids = []
        rs.EnableRedraw(False)
        for mesh in meshes:
            v, f = mesh.to_vertices_and_faces()
            # Redraw for individual call supressed here.
            guid = draw_mesh(v, f, name=name, redraw=False, disjoint=disjoint, color=color)
            guids.append(guid)
        if redraw:
            rs.EnableRedraw(True)
        return guids

    def draw_state(self, scene=None, redraw=True):
        # type: (SceneState, bool) -> None
        """Draw objects that relates to a specific object state dictionary.

        Please call delete_state() to erase previous geometry before calling this.

        """
        if scene is None:
            # Limit range
            if self.selected_state_id < 0:
                self.selected_state_id = 0
            if self.selected_state_id > len(self.process.movements):
                self.selected_state_id = len(self.process.movements)

            if self.selected_state_id == 0:
                scene = self.process.initial_state
            else:
                # Note state_id = 1 is referring to end of the first (0) movement.
                scene = self.process.get_movement_end_scene(self.process.movements[self.selected_state_id - 1])

        # Layer:
        rs.CurrentLayer(self.state_visualization_layer)
        rs.EnableRedraw(False)

        # * Temp holder for object_id and their list of Compas Meshes
        meshes = {}

        # * Beams
        for beam_id in self.process.assembly.sequence:
            beam = self.process.assembly.beam(beam_id)
            beam_mesh = self.process.assembly.get_beam_mesh_in_wcf(beam_id)
            T = Transformation.from_frame_to_frame(beam.frame, scene[(beam_id, 'f')])
            meshes[beam_id] = [beam_mesh.transformed(T)]

        # * Tools
        for tool_id in self.process.tool_ids:
            tool = self.process.tool(tool_id)
            object_state = ObjectState(scene[(tool_id, 'f')], scene[(tool_id, 'a')], scene[(tool_id, 'c')])
            meshes[tool_id] = tool.draw_state(object_state)

        # * Tool Changer
        tool_changer_key = ('tool_changer', 'f')
        object_state = ObjectState(scene[tool_changer_key], True, None)
        meshes['tool_changer'] = self.process.robot_toolchanger.draw_state(object_state)


        # * Draw Robot if state has robot config otherwise draw robot wrist
        # Drawing Robot Geometry in Rhino and GUIDs is not managed by the draw method above.
        # It is directly managed by the draw_robot() function
        if ('robot', 'c') in scene and scene[('robot', 'c')] is not None:
            configuration = scene[('robot', 'c')]
            self.draw_robot(configuration)
        else:
            # Draw rob_wrist at tool_changer.current_frame
            robot_wrist = self.process.robot_wrist
            robot_wrist.current_frame = scene[('robot', 'f')]
            meshes['rob_wrist'] = robot_wrist.draw_visual()

        # * Draw meshes to Rhinoand add guids to tracking dict
        for object_id in meshes:
            if meshes[object_id] is not None:
                guids = self.draw_meshes_get_guids(meshes[object_id], object_id, redraw=False)
                self.state_visualization_guids(object_id).extend(guids)
                # Add a color to the objects that are attached-to-robot
                attachment_key = (object_id, 'a')
                if attachment_key in scene and scene[attachment_key]:
                    meshes_apply_color(guids, (0.7, 1, 1, 1))
                if object_id == 'rob_wrist':
                    meshes_apply_color(guids, (0.6, 1, 1, 1))
        # Enable Redraw
        if redraw:
            rs.EnableRedraw(True)
            sc.doc.Views.Redraw()

    def delete_state(self, redraw=True):
        # type: (bool) -> None
        """Delete all geometry that is related to showing object state"""
        for object_id, guids in self._state_visualization_guids.items():
            purge_objects(guids, redraw=False)
        self._state_visualization_guids = {}
        # Delete old robot visualization
        self.delete_robot()
        # Enable Redraw
        if redraw:
            rs.EnableRedraw(True)
            sc.doc.Views.Redraw()


def meshes_apply_color(guids, color):
    for guid in guids:
        obj = sc.doc.Objects.Find(guid)
        r, g, b, a = [i * 255 for i in color]
        attr = obj.Attributes
        attr.ObjectColor = Color.FromArgb(a, r, g, b)
        attr.ColorSource = ColorFromObject
        obj.CommitChanges()


if __name__ == "__main__":

    from integral_timber_joints.rhino.load import get_process, get_process_artist, load_process

    # Draw beams to canvas
    # load_process()

    artist = get_process_artist()
    # artist.empty_layers()
    process = get_process()
    # for beam_id in process.assembly.beam_ids():
    #     artist.draw_beam_seqtag(beam_id)
    """Draw the clamp as a block. Block name is the clamp_type"""
    for tool_type in process.available_assembly_tool_types:
        tool = process.get_one_tool_by_type(tool_type).copy()
        tool.open_gripper()
        tool.current_frame = Frame.worldXY()
        meshes = tool.draw_visuals()
        guids = []
        for mesh in meshes:
            v, f = mesh.to_vertices_and_faces()
            layer = 'itj::tool'
            guids.append(draw_mesh(v, f, name=tool_type, layer=layer))
        block = rs.AddBlock(guids, (0, 0, 0), tool_type, True)
        #rs.InsertBlock(block, (10,10,10))
