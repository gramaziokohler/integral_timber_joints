from __future__ import absolute_import, division, print_function

try:
    from typing import Dict, Iterator, List, Optional, Tuple, Generator, Any
except:
    pass

from compas.datastructures import Network, Mesh
from compas.geometry import Point, Transformation, Translation, Vector, Shape
from compas.geometry.primitives.plane import Plane
from compas.geometry.primitives.frame import Frame
from compas.geometry.transformations.transformation import Transformation
from compas.geometry.transformations.rotation import Rotation
from compas import is_rhino

from compas.rpc import Proxy

from integral_timber_joints.assembly.beam_assembly_method import BeamAssemblyMethod
from integral_timber_joints.geometry import Beamcut, Joint, JointNonPlanarLap
from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.geometry.beamcut_plane import Beamcut_plane

try:
    from integral_timber_joints.geometry.screw import Screw_SL
except:
    pass


class Assembly(Network):
    """A data structure for discrete element assemblies.

    An assembly is essentially a network of assembly elements.
    Each element is represented by a vertex of the network.
    Each interface or connection between elements is represented by an edge of the network.

    Attributes
    ----------
    network : :class:`compas.Network`, optional
    attributes : dict, optional
        User-defined attributes of the assembly.
        Built-in attributes are:
        * name (str) : ``'Assembly'``
    default_element_attribute : dict, optional
        User-defined default attributes of the elements of the assembly.
        The built-in attributes are:
        * is_planned (bool) : ``False``
        * is_placed (bool) : ``False``
    default_connection_attributes : dict, optional
        User-defined default attributes of the connections of the assembly.

    Examples
    --------
    >>> # Beam
    >>> b1 = Beam(Frame.worldXY(), 1000, 100, 100, 'b1')
    >>> b2 = Beam(Frame([100,200,0],[0,1,0],[0,0,1]), 1000, 100, 100, 'b2')

    >>> # Joint
    >>> j1 = Joint_90lap(100,4,100,50,25)

    >>> # Assembly
    >>> a = Assembly()
    >>> a.add_beam(b1)
    >>> a.add_beam(b2)
    >>> a.add_one_joint(j1, 'b1', 'b2')

    """

    def __init__(self):

        # Call super class init
        Network.__init__(self)

        # Create default attributes
        self.attributes.update({
            'name': 'Unnamed_Assembly',
            'assembly_vector_if_no_joint': Vector(0, 0, -50),    # Determines the direction of assembly when the element has no neighbouring joint
            'beam_in_jaw_position_gap_offset': 10,  # Determines the distance from assembly_wcf_inclamp before first engagement.
            'sequence': [],                         # List of (beam_id / beam.name / node key)
        })
        # Default attributes for beams (node)
        self.update_default_node_attributes({
            'beam': None,                           # This should always contain a beam object, assigned by add_beam()
            'is_planned': False,                    # Parameter used in visualization
            'is_placed': False,                     # Parameter used in visualization
            'is_visible': True,                     # Parameter used in visualization
            'is_attached': True,                    # Parameter used in visualization
            'design_guide_vector_jawapproach': Vector(1, 1, 1),
            'assembly_vector_final': None,
            'assembly_vector_jawapproach': None,
            # * Clamps related Positions
            'assembly_wcf_storage': None,           # Beam storage position (modeled as beam frame) before getting to the pickup point
            'assembly_wcf_pickupapproach': None,    # Beam gripper position (modeled as beam frame) before approaching pickup point
            'assembly_wcf_pickup': None,            # Beam position at pick-up point
            'assembly_wcf_pickupretract': None,     # Beam position after lifting off from pick-up point
            'assembly_wcf_inclampapproach': None,   # Beam position before being placed inside clamp jaw
            'assembly_wcf_inclamp': None,           # Beam position inside the clamp, ready for final clamping move
            'assembly_wcf_final': None,             # Beam position in final modeled position, same as beam.frame
            'assembly_wcf_finalretract': None,      # Beam gripper position (modeled as beam frame) after releasing and retracting from final position.
            # * Screwdriver related Positions
            'assembly_wcf_assembleapproach': None,   # Beam position after free motion transfer
            'assembly_wcf_assemblebegin': None,      # Beam position after short linear move, ready for screwing action
            # * Gripper Information
            'gripper_type': None,
            'gripper_id': None,
            'gripper_grasp_face': None,             # Grasp pose expressed in relationship to Beam Face
            'gripper_grasp_dist_from_start': None,  # Grasp pose expressed in relationship to Beam Length Parameter
            'gripper_tcp_in_ocf': None,             # Gripper grasp pose expressed in TCP location relative to the OCF
            'design_guide_vector_grasp': Vector(1, 1, 1),      # Gripper grasp pose guide Vector (align with Z of TCP in WFC)
            'beam_cuts': None,                         # Beamcut objects at the start or end or anywhere on the beam. Can be empty
            'assembly_method': BeamAssemblyMethod.UNDEFINED,  # AssemblyMethod determine how individual beam is assembled.
            'actions': [],                         # List of high-level Assembly Actions
            'tool_as_gripper_joint_id': None,
        })
        # Default attributes for joints (edge)
        self.update_default_edge_attributes({
            'sequence_earlier': False,
            'clamp_wcf_attachapproach1': None,      # Clamp position beforing approaching attachment point (1 happens before 2)
            'clamp_wcf_attachapproach2': None,      # Clamp position beforing approaching attachment point
            'clamp_wcf_final': None,                # Clamp position at attachment point "clamp_frame_wcf"
            'clamp_wcf_attachretract': None,        # Robot Position (modeled as clamp frame) after releasing the clamp at final position and retracting.
            'clamp_wcf_detachapproach': None,       # Robot Position (modeled as clamp frame) before picking up the clamp from final position.
            'clamp_wcf_detachretract1': None,       # Clamp position after assembly, retracting from attachment point (1 happens before 2)
            'clamp_wcf_detachretract2': None,       # Clamp position after assembly, retracting from attachment point

            'screwdriver_assembleapproach_attached': None,   # Screwdriver position when beam = assemble approach, tool = attached.
            'screwdriver_assemblebegin_attached': None,   # Screwdriver position when beam = assemble approach, tool = attached.
            'screwdriver_assembled_attached': None,   # Screwdriver position when beam = assembled, tool = attached.
            'screwdriver_assembled_retracted': None,   # Screwdriver position when beam = assembled, tool = retracted from beam.
            'screwdriver_assembled_retractedfurther': None,   # Screwdriver position when beam = assembled, tool = retractedfurther from beam.

            'tool_orientation_frame_index' : 0,
            'tool_type': None,
            'tool_id': None,

            'has_screw':False,
            'screw':None,
        })

    # ----------------------------
    # Properity Setter and Getter
    # ----------------------------

    @property
    def name(self):
        """str : The name of the assembly."""
        return self.attributes['name']

    @name.setter
    def name(self, value):
        self.attributes['name'] = value

    def __str__(self):
        return self.name

    def copy(self):
        """Returns a copy of this assembly.

        Elements and their _source are copied
        Connecions and their dictionary type of data are copied
        """
        from copy import deepcopy
        return deepcopy(self)

    @property
    def sequence(self):
        # type: () -> List[str]
        return self.attributes['sequence']

    @sequence.setter
    def sequence(self, beam_id_list):
        # type: (List[str]) -> None
        self.attributes['sequence'] = beam_id_list

    @property
    def number_of_beams(self):
        return self.number_of_nodes()

    @property
    def number_of_joints(self):
        return self.number_of_edges()

    # ----------------------------
    # Adding Beams and Joints
    # ----------------------------

    def add_beam(self, beam):
        # type: (Beam) -> None
        """Add a beam to the assembly
        Beam also added to the end of the sequence.
        """
        assert self.has_node(beam.name) == False
        self.add_node(beam.name, {'beam': beam})
        self.sequence.append(beam.name)

    def add_one_joint(self, joint, beam_id, neighbor_beam_id):
        # type: (Joint, str, str) -> None
        """Add a joint.
        If the joint already exist, the new joint object will orverwrite the old one.
        """
        self.add_edge(beam_id, neighbor_beam_id, {'joint': joint})
        # Check assembly sequence to see if this side of the joint is the earlier
        sequence_earlier = self.sequence.index(beam_id) < self.sequence.index(neighbor_beam_id)
        self.set_joint_attribute((beam_id, neighbor_beam_id), 'sequence_earlier', sequence_earlier)

    def add_joint_pair(self, joint1, joint2, beam1_id, beam2_id):
        # type: (Joint, Joint, str, str) -> None
        """Add a pair of joint

        If the joint already exist, the new joint object will orverwrite the old one.
        """
        self.add_one_joint(joint1, beam1_id, beam2_id)
        self.add_one_joint(joint2, beam2_id, beam1_id)
        self.beam(beam1_id).remove_cached_mesh()
        self.beam(beam2_id).remove_cached_mesh()

    def remove_beam(self, beam_id):
        assert self.has_node(beam_id)
        assert beam_id in self.sequence
        for nbr in self.neighbors(beam_id):  # This is a hot fix for compas Network.delete_node fail to delete edges.
            del self.adjacency[nbr][beam_id]
            del self.adjacency[beam_id][nbr]
            del self.edge[nbr][beam_id]
            del self.edge[beam_id][nbr]
        self.delete_node(beam_id)
        self.sequence.remove(beam_id)

    # ----------------------------------------------
    # Getting / Iterating Beams and Joint attributes
    # ----------------------------------------------

    def set_beam_attribute(self, beam_id, attribute_key, value):
        """ Setting an attribute of one beam

        Parameters
        ----------
        beam_id : str
            Identifier of the Beam (Beam.name)
        attribute_key : str
        value : any

        Note
        ----
        Value will be deepcopied if value.copy() is callable,
        otherwise copy.deepcopy(value) will be used.

        This deep copying resolves a problem with jsonpickle not de/serializing properly,
        between IPy and cpython. Throwing error `IDproxy not subscriptable`

        """
        if value is None:
            self.unset_node_attribute(beam_id, attribute_key)
        else:
            # Make a copy of the attribute before setting it.
            if callable(getattr(value, "copy", None)):
                copied_value = value.copy()
            else:
                from copy import deepcopy
                copied_value = deepcopy(value)
            self.node_attribute(beam_id, attribute_key, copied_value)

    def get_beam_attribute(self, beam_id, attribute_key):
        """ Getting an attribute of one beam

        Parameters
        ----------
        beam_id : str
            Identifier of the Beam (Beam.name)
        attribute_key : str

        Returns
        -------
        [obj]
            Returns the attribute
        """
        return self.node_attribute(beam_id, attribute_key)

    def set_joint_attribute(self, joint_id, attribute_key, value):
        """ Setting an attribute of one joint

        Parameters
        ----------
        joint_id : (str, str)
            Identifier of the Joint (beam_id, neighbor_beam_id)
        attribute_key : [str]
        value : any

        Note
        ----
        Value will be deepcopied if value.copy() is callable,
        otherwise copy.deepcopy(value) will be used.

        This deep copying resolves a problem with jsonpickle not de/serializing properly,
        between IPy and cpython. Throwing error `IDproxy not subscriptable`

        """
        if value is None:
            self.unset_edge_attribute(joint_id, attribute_key)
        else:
            # Make a copy of the attribute before setting it.
            if callable(getattr(value, "copy", None)):
                copied_value = value.copy()
            else:
                from copy import deepcopy
                copied_value = deepcopy(value)
            self.edge_attribute(joint_id, attribute_key, copied_value)

    def get_joint_attribute(self, joint_id, attribute_key):
        """ Getting an attribute of one joint

        Parameters
        ----------
        joint_id : (str, str)
            Identifier of the Joint (beam_id, neighbor_beam_id)
        attribute_key : [str]
        """
        return self.edge_attribute(joint_id, attribute_key)

    def set_joint_shared_attribute(self, joint_id, attribute_key, value):
        """ Setting a shared attribute acrosss a pair of joint.
        Accessing an attribute via joint_id ('b1','b2') will be the same as joint_id ('b2','b1')

        Note
        ----
        The implementation stores as with typical `set_joint_attribute()`, however,
        the joint_id used is based on string lexicographical ordering, NOT sequence.
        We will always store and retrive values from joint(lesser_string, greater_string)

        """
        if (joint_id[0] > joint_id[1]):
            joint_id = (joint_id[1], joint_id[0])
        self.set_joint_attribute(joint_id=joint_id, attribute_key=attribute_key, value=value)

    def get_joint_shared_attribute(self, joint_id, attribute_key):
        """ Setting a shared attribute acrosss a pair of joint.
        Accessing an attribute via joint_id ('b1','b2') will be the same as joint_id ('b2','b1')

        Note
        ----
        The implementation stores as with typical `set_joint_attribute()`, however,
        the joint_id used is based on string lexicographical ordering, NOT sequence.
        We will always store and retrive values from joint(lesser_string, greater_string)

        """
        if (joint_id[0] > joint_id[1]):
            joint_id = (joint_id[1], joint_id[0])
        return self.edge_attribute(joint_id, attribute_key)

    def beam(self, key):
        # type: (str) -> Beam
        """Get a beam by its key."""
        return self.node_attribute(key, 'beam')

    def joint(self, joint_id):
        # type: (tuple[str,str]) -> Joint
        """Get a joint by its id.
        joint_id is a Tuple """
        return self.edge_attribute(joint_id, 'joint')

    def get_joints_of_beam(self, beam_id):
        # type: (str) -> list[Joint]
        """Get all the joints attached to a beam"""
        joints = []
        for neighbor_beam_id in self.neighbors_out(beam_id):
            joints.append(self.joint((beam_id, neighbor_beam_id)))
        return joints

    def get_joint_ids_of_beam(self, beam_id):
        # type: (str) -> list[tuple[str,str]]
        """Get all the ids of joints (beam_id, neighbor_beam_id) that are attached to a beam
        """
        joint_ids = []
        for neighbor_beam_id in self.neighbors_out(beam_id):
            joint_ids.append((beam_id, neighbor_beam_id))
        return joint_ids

    def get_reverse_joint_ids_of_beam(self, beam_id):
        # type: (str) -> list[tuple[str,str]]
        """Get all the ids of joints (neighbor_beam_id, beam_id) that are attached to a beam
        """
        joint_ids = []
        for neighbor_beam_id in self.neighbors_out(beam_id):
            joint_ids.append((neighbor_beam_id, beam_id))
        return joint_ids

    def get_assembly_method(self, beam_id):
        # type: (str) -> BeamAssemblyMethod
        return self.get_beam_attribute(beam_id, 'assembly_method')

    # --------------------------------------------
    # Assembly Sequence Functions
    # --------------------------------------------

    def get_beam_sequence(self, beam_id):
        # type: (str) -> Optional[int]
        """ Returns the (0-counting) position of the beam in self.sequence
        If beam is not in self.sequence, returns None
        """
        if beam_id not in self.sequence:
            return None
        return self.sequence.index(beam_id)

    def _shift_sequence_by_one(self, beam_id, shift_earlier=True):
        # type: (str, bool) -> Tuple[str, str]
        """Swap the sequence of a beam with the previous or next item.
        Invalidly swapping [first item earlier, or last item later] will have no effect.

        Return the pair of affected beam_ids
        """
        sequence = self.sequence
        i = sequence.index(beam_id)
        # Swap item with previous / next item
        if shift_earlier and i > 0:
            sequence[i-1], sequence[i] = sequence[i], sequence[i-1]
            self.align_screw_direction_by_assembly_sequence(beam_id)
            self.align_screw_direction_by_assembly_sequence(sequence[i-1])
            return [sequence[i], sequence[i-1]]
        if not shift_earlier and i < len(sequence) - 1:
            sequence[i+1], sequence[i] = sequence[i], sequence[i+1]
            self.align_screw_direction_by_assembly_sequence(beam_id)
            self.align_screw_direction_by_assembly_sequence(sequence[i+1])
            return [sequence[i], sequence[i+1]]

    def shift_beam_sequence(self, beam_id, shift_amount, update_assembly_direction=True, allow_change_joint_direction=True):
        # type: (str, int, bool, bool) -> List[str]
        """Move the sequence of a beam earlier or later by the shift_amount.
        Negative shift_amount means moving earlier.

        Assembly direction is recomputed.
        However, if no assembly direction is possibble and allow_change_joint_direction,
        new joint direction will be computed.

        Return all beam_ids in which their sequence number is changed
        """
        if shift_amount == 0:
            return []

        affected_ids = set()
        org_earlier_neighbors = self.get_already_built_neighbors(beam_id)

        # Perform as many swaps as needed, before or after
        for _ in range(abs(shift_amount)):
            affected_id_pair = self._shift_sequence_by_one(beam_id, shift_earlier=(shift_amount < 0))
            affected_ids = affected_ids.union(set(affected_id_pair))

        if update_assembly_direction:
            # Compute which neighbour changed before-after relationship with the shifted beam.
            now_earlier_neighbors = self.get_already_built_neighbors(beam_id)
            neighbour_with_joint_to_flip = set(org_earlier_neighbors).symmetric_difference(set(now_earlier_neighbors))
            all_affected_earlier_neighbours = set(org_earlier_neighbors).union(set(now_earlier_neighbors))
            print("neighbour_with_joint_to_flip %s" % neighbour_with_joint_to_flip)

            # Flip those joints if they are flippable
            for nbr_beam_id in neighbour_with_joint_to_flip:
                # Non Planar joints are not flipped but are recreated.
                joint_id = (beam_id, nbr_beam_id)
                original_joint = self.joint(joint_id)
                if isinstance(original_joint, JointNonPlanarLap):
                    original_thickness = original_joint.thickness
                    if self.sequence.index(beam_id) < self.sequence.index(nbr_beam_id):
                        j1, j2, screw_line = JointNonPlanarLap.from_beam_beam_intersection(self.beam(beam_id), self.beam(nbr_beam_id), thickness=original_thickness)
                        joint_thickness = j2.thickness
                    else:
                        j2, j1, screw_line = JointNonPlanarLap.from_beam_beam_intersection(self.beam(nbr_beam_id), self.beam(beam_id), thickness=original_thickness)
                        joint_thickness = j1.thickness
                    screw = Screw_SL.AutoLength_Factory(center_line=screw_line, head_side_thickness=joint_thickness)
                    self.add_joint_pair(j1, j2, beam_id, nbr_beam_id)
                    self.set_screw_of_joint(joint_id, screw)
                else:
                    self.flip_lap_joint(joint_id)
                    screw = self.get_screw_of_joint(joint_id)
                    screw.flip()

            # Recompute all affected beams screw hole, assembly directions and realign joints if needed.
            for _beam_id in list(neighbour_with_joint_to_flip) + [beam_id]:
                self.compute_beam_assembly_direction_from_joints_and_sequence(_beam_id)
                # If assembly direction is not valid, we try aligning its joints again to see.
                if self.beam_problems(_beam_id):
                    if allow_change_joint_direction:
                        print('Search for another joint config for Beam %s' % _beam_id)
                        self.search_for_halflap_joints_with_previous_beams(_beam_id, self.get_beam_attribute(beam_id, 'assembly_vector_final'))
                        self.compute_beam_assembly_direction_from_joints_and_sequence(_beam_id)
                #
                self.align_screw_direction_by_assembly_sequence(beam_id)

            # The affected beams include all_affected_earlier_neighbours
            affected_ids = affected_ids.union(all_affected_earlier_neighbours)

        return list(affected_ids)

    # --------------------------------------------
    # Beam cuts
    # --------------------------------------------

    def beam_cuts(self, beam_id):
        # type: (str) -> list[Beamcut]
        # Creating the beam attribute if the attribute is None as default
        if self.get_beam_attribute(beam_id, 'beam_cuts') is None:
            self.set_beam_attribute(beam_id, 'beam_cuts', [])
        return self.get_beam_attribute(beam_id, 'beam_cuts')

    def add_ocf_beam_cut_from_wcf_plane(self, beam_id, wcf_plane):
        # type: (str, Plane) -> None
        """Add a OCF type Beamcut object to a beam.
        The Beam cut is defined from a plane that is in OCF.

        The plane input to this function is in WCF,
        the function will convert it to OCF.

        Note: beam(beam_id).cached_mesh will be reset to None
        """

        beam = self.beam(beam_id)
        beam_from_world = Transformation.from_frame(beam.frame).inverse()

        ocf_plane = wcf_plane.transformed(beam_from_world)
        beam_cut = Beamcut_plane(ocf_plane)
        self.beam_cuts(beam_id).append(beam_cut)
        self.beam(beam_id).remove_cached_mesh()

    def remove_all_beam_cuts(self, beam_id):
        # type: (str, Beamcut) -> None
        """Set the Beamcut object at the end of beam. None if un-cut."""
        del self.beam_cuts(beam_id)[:]

    # --------------------------------------------
    # Iterating through all Beams and Joints
    # --------------------------------------------

    def beams(self):
        # type: () -> Iterator[Beam]
        for key, data in self.nodes(data=True):
            yield data['beam']

    def beam_ids(self):
        # type: () -> Iterator[str]
        for key in self.nodes(data=False):
            yield key

    def joint_ids(self):
        # type: () -> Iterator[Tuple[str, str]]
        for key in self.edges(data=False):
            yield key

    def joints(self):
        # type: () -> Iterator[Joint]
        """ Iterate through a list of joints.
        Typically each connection has two joint objects, belonging to each beam.
        """
        for key, data in self.edges(data=True):
            yield data['joint']

    def earlier_joint_ids(self):
        # type: () -> Iterator[Tuple[Tuple[str, str], Tuple[str, str]]]
        """ Iterate through a list of joint pairs.
        There is an assumption that all joints are in pairs.
        """
        for joint_id, data in self.edges(data=True):
            if data['sequence_earlier']:
                yield joint_id

    def later_joint_ids(self):
        # type: () -> Iterator[Tuple[Tuple[str, str], Tuple[str, str]]]
        """ Iterate through a list of joint pairs.
        There is an assumption that all joints are in pairs.
        """
        for joint_id, data in self.edges(data=True):
            if not data['sequence_earlier']:
                yield joint_id

    def get_new_beam_id(self):
        # type: () -> str
        """ Returns the next available beam_id in the format of 'b%i'.
        Starting from 'b0' to maximum 'b999', it returns the next unused beam_id
        If there are holes between the used_id, the hole will be returned.
        """
        for i in range(1000):
            beam_id = 'b%i' % i
            if beam_id not in self.nodes(data=False):
                return beam_id

    # -------------------------
    # Transformation
    # -------------------------

    def transform(self, transformation):
        """Transform the assembly.
        Tansformation should contain translation and rotation only.
        """
        # Change the frame in beam
        [beam.transform(transformation) for beam in self.beams()]

        # Helper function to transform attributes
        def transform_beam_attribute_if_not_none(beam_id, attribute_name, _transformation):
            if self.get_beam_attribute(beam_id, attribute_name) is not None:
                self.get_beam_attribute(beam_id, attribute_name).transform(_transformation)

        def transform_joint_attribute_if_not_none(joint_id, attribute_name, _transformation):
            if self.get_joint_attribute(joint_id, attribute_name) is not None:
                self.get_joint_attribute(joint_id, attribute_name).transform(_transformation)

        # Transform key position frames and vectors in assembly.
        # Storage and Pickup frames are not affected.
        for beam_id in self.sequence:
            transform_beam_attribute_if_not_none(beam_id, 'assembly_wcf_inclampapproach', transformation)
            transform_beam_attribute_if_not_none(beam_id, 'assembly_wcf_inclamp', transformation)
            transform_beam_attribute_if_not_none(beam_id, 'assembly_wcf_final', transformation)
            transform_beam_attribute_if_not_none(beam_id, 'assembly_wcf_finalretract', transformation)
            transform_beam_attribute_if_not_none(beam_id, 'assembly_vector_final', transformation)
            transform_beam_attribute_if_not_none(beam_id, 'assembly_vector_jawapproach', transformation)
            transform_beam_attribute_if_not_none(beam_id, 'design_guide_vector_jawapproach', transformation)

            for tool_id in self.get_joint_ids_with_tools_for_beam(beam_id):
                transform_joint_attribute_if_not_none(tool_id, 'clamp_wcf_attachapproach1', transformation)
                transform_joint_attribute_if_not_none(tool_id, 'clamp_wcf_attachapproach2', transformation)
                transform_joint_attribute_if_not_none(tool_id, 'clamp_wcf_final', transformation)
                transform_joint_attribute_if_not_none(tool_id, 'clamp_wcf_attachretract', transformation)
                transform_joint_attribute_if_not_none(tool_id, 'clamp_wcf_detachapproach', transformation)
                transform_joint_attribute_if_not_none(tool_id, 'clamp_wcf_detachretract1', transformation)
                transform_joint_attribute_if_not_none(tool_id, 'clamp_wcf_detachretract2', transformation)

    def rotate_beam_to_align_Y_axis(self, beam_id, Y_axis_guide):
        # type: (str, Vector) -> list(str)
        """ Rotates the beam such that the Y axis is aligned to a guide vector.
        If the guide vector is parallel to the beam's X axis, error will occur.
        The beam object will be transformed, all joints related to the beam is deleted.

        All existing beams will be checked for new joints.
        Returns a list of joint_id that are affected."""
        beam = self.beam(beam_id)
        beam_centerline = beam.get_center_line()
        if beam.frame.xaxis.angle(Y_axis_guide) < 0.01:
            raise ValueError("The guide vector cannot be parallel to the beam's X axis")

        original_frame = Frame(beam_centerline.start, beam.frame.xaxis, beam.frame.yaxis)
        new_zaxis = beam.frame.xaxis.cross(Y_axis_guide)
        new_yaxis = new_zaxis.cross(beam.frame.xaxis)
        new_frame = Frame(beam_centerline.start, beam.frame.xaxis, new_yaxis)

        T = Transformation.from_frame_to_frame(original_frame, new_frame)
        beam.transform(T)

        # Delete old joints
        affected_neighbours = []
        for nbr in self.neighbors(beam_id):  # This is a hot fix for compas Network.delete_node fail to delete edges.
            del self.adjacency[nbr][beam_id]
            del self.adjacency[beam_id][nbr]
            del self.edge[nbr][beam_id]
            del self.edge[beam_id][nbr]
            affected_neighbours.append(nbr)

        # Test to see if there are new joints

    # --------------------------------
    # Beam and Joints Features Boolean
    # --------------------------------
    def get_beam_negative_shapes(self, beam_id):
        beam = self.beam(beam_id)
        # Collect all the features
        negative_shapes = []
        for joints in self.get_joints_of_beam(beam_id):
            negative_shapes += joints.get_feature_shapes(beam)
        for beam_cut in self.beam_cuts(beam_id):
            negative_shapes += beam_cut.get_feature_shapes(beam)
        for nbr_id in self.get_unbuilt_neighbors(beam_id):
            screw = self.get_screw_of_joint((beam_id, nbr_id))
            if screw is not None:
                negative_shapes += screw.get_thread_side_feature_shapes()
        for nbr_id in self.get_already_built_neighbors(beam_id):
            screw = self.get_screw_of_joint((beam_id, nbr_id))
            if screw is not None:
                negative_shapes += screw.get_head_side_feature_shapes()
        return negative_shapes

    def get_beam_mesh_in_wcf(self, beam_id, use_cache_if_available=True):
        # type: (str, bool) -> Mesh
        """Get the mesh of a beam at its designed location in wcf.
        Update the cached_mesh of a beam if necessary, taking into account all the joints and features attached.

        Parameters
        ----------
        beam_id : int
            index of the beam
        skip_if_cached : bool, optional
            Skip recomputeing cache mesh if it already exist, by default False.
        """
        # Return cached mesh if available
        beam = self.beam(beam_id)
        cached_mesh = beam.cached_mesh
        if use_cache_if_available and (cached_mesh is not None):
            return cached_mesh

        negative_shapes = self.get_beam_negative_shapes(beam_id)
        self._boolean_beam_mesh(beam, negative_shapes)

        return beam.cached_mesh

    def get_beam_mesh_in_ocf(self, beam_id, use_cache_if_available=True):
        # type: (str, bool) -> Mesh
        """Beam mesh in the ObjectCoordinateFrame (OCF).
        This mesh can be transformed to self.frame to be shown at final location.
        This can also be transformed to other frames for key position visualization.
        """
        beam = self.beam(beam_id)
        mesh = self.get_beam_mesh_in_wcf(beam_id, use_cache_if_available)
        T = Transformation.from_frame_to_frame(beam.frame, Frame.worldXY())
        return mesh.transformed(T)

    def get_beam_mesh_at(self, beam_id, position_name, use_cache_if_available=True):
        # type: (str, str, bool) -> Mesh
        """Beam mesh in the ObjectCoordinateFrame (OCF).
        This mesh can be transformed to self.frame to be shown at final location.
        This can also be transformed to other frames for key position visualization.
        """
        mesh = self.get_beam_mesh_in_wcf(beam_id, use_cache_if_available)
        T = self.get_beam_transformaion_to(beam_id, position_name)
        return mesh.transformed(T)

    def _boolean_beam_mesh(self, beam, negative_shapes=[]):
        # type:(Beam, list[Shape]) -> Mesh
        """Computes the beam geometry with boolean difference of all joints and features.
        This is manually triggered.
        Parameters
        ----------
        beam_features: list(BeamFeature)
            Objects that implements the BeamFeature ABC

        Returns
        -------
        compas.datastructures.Mesh
            The beam mesh with joint geoemtry removed

        Note
        ----------
        self.cached_mesh is updated.
        features object need to implement the get_feature_meshes(beam)-> list[Mesh] function.
        """

        negative_meshes = []
        # First mesh in the list is the uncut beam mesh
        beam.cached_mesh = beam.draw_uncut_mesh()

        ####################################
        # compas_trimesh implementation
        ####################################

        if len(negative_shapes) > 0:
            beam.cached_mesh.quads_to_triangles()
            negative_meshes.append(beam.cached_mesh)
            # Convert negative shapes to meshes
            for shape in negative_shapes:
                negative_mesh = Mesh.from_shape(shape)
                negative_mesh.quads_to_triangles()
                negative_meshes.append(negative_mesh)

            # Calls trimesh to perform boolean
            if is_rhino():
                from compas.rpc import Proxy
                trimesh_proxy = Proxy(package='compas_trimesh')
                result = trimesh_proxy.trimesh_subtract_multiple(negative_meshes)
                beam.cached_mesh = result
            else:
                from compas_trimesh import trimesh_subtract_multiple
                result = trimesh_subtract_multiple(negative_meshes)
                beam.cached_mesh = result

        ####################################
        # compas_cgal implementation
        ####################################

        # if len(beam_features) > 0:

        #     # Calls trimesh to perform boolean
        #     from compas.rpc import Proxy
        #     proxy = Proxy()
        #     proxy.package = 'compas_cgal.booleans'

        #     # Convert uncut beam to triangle
        #     self.cached_mesh.quads_to_triangles()

        #     # compas_cgal uses tuple(vertices and faces) as mesh representation
        #     result_v_f = self.cached_mesh.to_vertices_and_faces()

        #     # Compute the negative meshes from the features
        #     for feature in beam_features:
        #         for bool_negative in feature.get_feature_meshes(self):
        #             bool_negative.quads_to_triangles()
        #             bool_negative_v_f = bool_negative.to_vertices_and_faces()
        #             result_v_f = proxy.boolean_difference(result_v_f, bool_negative_v_f)

        #     # Reassemble vertices and faces back to Mesh
        #     result = Mesh.from_vertices_and_faces(* result_v_f)
        #     self.cached_mesh = result

        ####################################
        # End of implementation
        ####################################

        beam.cached_mesh.name = beam.name + "_mesh"

        return beam.cached_mesh

    # -------------------------------
    # Beam and Gripper Transforamtion
    # -------------------------------

    def get_t_gripper_tcf_from_beam(self, beam_id):
        # type: (str) -> Transformation
        """ Returns the transformation from `gripper_tcp` to `beam_ocf`

        The grasp is retrived from beam attribute `gripper_tcp_in_ocf`

        ### Credits:
        YiJiang contributed this rather clear way of expressing everytihing in Transformation.
        If one day we have time, we should all switch to this notation to be consistent with
        robotics code community.

        x_from_y, means Y expressed in the X's coordinate system
        """
        # Grasp
        beam_ocf_from_gripper_tcp = Transformation.from_frame(self.get_beam_attribute(beam_id, "gripper_tcp_in_ocf"))

        # gripper_inverse
        gripper_tcp_from_beam = beam_ocf_from_gripper_tcp.inverse()
        return gripper_tcp_from_beam

    def get_beam_transformaion_to(self, beam_id, attribute_name):
        # type: (str, str) -> Optional[Transformation]
        """ Get the transformation from the beam's assembly_wcf_final to a frame specified in attributes"""
        from compas.geometry import Transformation
        source_frame = self.get_beam_attribute(beam_id, 'assembly_wcf_final')
        target_frame = self.get_beam_attribute(beam_id, attribute_name)
        assert source_frame is not None
        if target_frame is None:
            return None
        return Transformation.from_frame_to_frame(source_frame, target_frame)

    # --------------------------------------------
    # Joints and Neighbours
    # --------------------------------------------

    def get_joints_of_beam_connected_to_already_built(self, beam_id):
        # type: (str, bool) -> list[tuple[str,str]]
        """Return the joints ids (beam_id, neighbor_id) that are connected to already-assembled beams
        """
        sequence_i_of_beam = self.get_beam_sequence(beam_id)
        return [(beam_id, neighbor_id) for neighbor_id in self.get_already_built_neighbors(beam_id)]

    def get_already_built_beams(self, beam_id):
        # type: (str) -> list[str]
        "Return the beam ids of already-built beams relative to the given beam_id"
        this_beam_sequence = self.get_beam_sequence(beam_id)
        return self.sequence[:this_beam_sequence]

    def get_already_built_neighbors(self, beam_id):
        # type: (str) -> list[str]
        "Return the beam ids of neighbours that are connected to already-assembled beams"
        # The if statement checkes if the joint attribute 'sequence_earlier' == False
        this_beam_sequence = self.get_beam_sequence(beam_id)
        return [neighbor_id for neighbor_id in self.neighbors_out(beam_id) if self.get_beam_sequence(neighbor_id) < this_beam_sequence]

    def get_unbuilt_neighbors(self, beam_id):
        # type: (str) -> list[str]
        "Return the beam ids of neighbours that are connected to the selected beams, but not yet built."
        # The if statement checkes if the joint attribute 'sequence_earlier' == False
        this_beam_sequence = self.get_beam_sequence(beam_id)
        return [neighbor_id for neighbor_id in self.neighbors_out(beam_id) if self.get_beam_sequence(neighbor_id) > this_beam_sequence]

    # --------------------------------------------
    # Assembly Tools / Screws and Neighbours
    # --------------------------------------------

    def get_joint_ids_with_tools_for_beam(self, beam_id):
        # type: (str) -> List[tuple[str, str]]
        """Returning a list of joint_ids on the given beam that require assembly tool.
        Format is: (neighbour_id, beam_id)
        """
        if self.get_assembly_method(beam_id) <= BeamAssemblyMethod.GROUND_CONTACT:
            return []

        return [(neighbour_id, beam_id) for neighbour_id in self.get_already_built_neighbors(beam_id)]

    def get_screw_of_joint(self, joint_id):
        #type: (Tuple[str,str]) -> Screw_SL
        if self.get_joint_shared_attribute(joint_id, 'has_screw'):
            return self.get_joint_shared_attribute(joint_id, 'screw')
        else:
            return None

    def set_screw_of_joint(self, joint_id, screw):
        #type: (Tuple[str,str], Screw_SL) -> None
        return self.set_joint_shared_attribute(joint_id, 'screw', screw)


    def align_screw_direction_by_assembly_sequence(self, beam_id):
        # type: (Assembly, str) -> None
        """Flip the joint screws for beams that are screwed.
        If the beam's ssembly method require screws, joint.has_screw=True

        Only the joints with previously built beams will be modifued.
        If the joint is with an earlier beam, the joint will have a screw head.

        """
        for joint_id in self.get_joints_of_beam_connected_to_already_built(beam_id):
            inverse_joint_id = (joint_id[1], joint_id[0])
            # print(self.get_assembly_method(beam_id))
            # print(BeamAssemblyMethod.screw_methods)
            if self.get_assembly_method(beam_id) in BeamAssemblyMethod.screw_methods:
                # print("Screwing Joint %s, %s" % (joint_id, self.joint(joint_id)))
                self.joint(joint_id).has_screw = True
                self.joint(joint_id).screw_head_side = True
                self.joint(inverse_joint_id).has_screw = True
                self.joint(inverse_joint_id).screw_head_side = False

            else:
                # print("Notscrewing Joint %s, %s" % (joint_id, self.joint(joint_id)))
                self.joint(joint_id).has_screw = False
                self.joint(inverse_joint_id).has_screw = False

    def get_joint_id_where_screwdriver_is_gripper(self, beam_id):
        # type: (str) -> Optional[Tuple[str, str]]
        """Returns the joint id where a screwdriver is used as gripper.

        Returns None if beam is not SCREWED_WITHOUT_GRIPPER type
        or if it has no joints that require tool.
        """
        if self.get_assembly_method(beam_id) != BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
            return None

        value_from_attributes = self.get_beam_attribute(beam_id, 'tool_as_gripper_joint_id')
        if value_from_attributes is not None:
            return value_from_attributes
        else:
            joint_ids = self.get_joint_ids_with_tools_for_beam(beam_id)
            if len(joint_ids) > 0:
                return joint_ids[0]
            else:
                return None

    # -------------------------------------
    # Computing joints and joint directions
    # -------------------------------------

    def search_for_halflap_joints_with_previous_beams(self,
                                                      beam_id,
                                                      guide_assembly_vector=None,
                                                      parallel_tolerance=0.001,
                                                      verbose=False):
        ''' Computes the joints between the given beam, and all other (earlier) beams.
        All previous beams wil be chcked fo potential joints.

        When the joints are found, they are added to the assembly as a new edge.
        This will then enable neighbour queries.

        Side Effect
        -----------
        beam.cached_mesh set to None for both beams.
        '''
        from integral_timber_joints.geometry import JointHalfLap

        for bid_neighbor in self.get_already_built_beams(beam_id):
            beam_m = self.beam(beam_id)
            beam_s = self.beam(bid_neighbor)
            if verbose:
                print("Checking Between %s - %s" % (beam_m.name, beam_s.name))

            option1 = JointHalfLap.from_beam_beam_intersection(beam_s, beam_m,  face_choice=0)  # joint1, joint2
            option2 = JointHalfLap.from_beam_beam_intersection(beam_s, beam_m,  face_choice=1)

            if option1[0] is not None and option1[1] is not None:
                if verbose:
                    print("Face Pair Option 1: %s , %s" % (option1[0], option1[1]))
                # Pick a guide vector (here just pick option 1 ffs)
                if guide_assembly_vector is None:
                    guide_assembly_vector = option1[0].get_assembly_direction(beam_m)

                # Pick between option 1 or 2.
                # Of course only if option 2 is possible

                if guide_assembly_vector.angle(option1[0].get_assembly_direction(beam_m)) < parallel_tolerance or option2[0] is None or option2[1] is None:
                    joint_s, joint_m, screw_line = option1
                else:
                    joint_s, joint_m, screw_line = option2

                self.add_joint_pair(joint_m, joint_s, beam_id, bid_neighbor)
                screw = Screw_SL.AutoLength_Factory(center_line=screw_line, head_side_thickness=joint_m.thickness)
                self.set_screw_of_joint((beam_id, bid_neighbor), screw)
                beam_m.remove_cached_mesh()
                beam_s.remove_cached_mesh()
            else:
                if verbose:
                    print("Face Pair Intersection Not Found")

    def flip_lap_joint(self, joint_id):
        """Flip the opening direction of a pair of lap joints
        This only work for joints that support swap_faceid_to_opposite_face()

        Side Effect
        -----------
        beam_attribute 'assembly_wcf_inclamp' is unset
        beam.cached_mesh set to None for both beams.
        """
        beam_id, neighbour_id = joint_id
        self.joint((beam_id, neighbour_id)).swap_faceid_to_opposite_face()
        self.joint((neighbour_id, beam_id)).swap_faceid_to_opposite_face()
        self.set_beam_attribute(beam_id, 'assembly_wcf_inclamp', None)
        self.beam(beam_id).remove_cached_mesh()
        self.beam(neighbour_id).remove_cached_mesh()

    def flip_screw(self, joint_id):
        screw = self.get_joint_shared_attribute(joint_id, 'screw') #type: Screw_SL
        if screw is not None:
            screw.flip()
    # -----------------------
    # Assembly Directions
    # -----------------------

    def compute_all_assembly_direction_from_joints_and_sequence(self, assembly_vector_if_no_joint=None, beam_in_jaw_position_gap_offset=None):
        """Computes the assembly of all beams based on the assembly sequence and the joints the beam

        Based on the assembly sequence of the beams. We can deduce the assembly direction
        from the joints that are to be clamped to the already-assembled beams.

        All joint pairs are checked and if they contradict,
        'assembly_vector_final' and 'assembly_wcf_inclamp' will be set to None

        Side Effect
        -----------

        beam_attribute 'assembly_wcf_final' is set
        beam_attribute 'assembly_vector_final' is set (if no contradict)
        beam_attribute 'assembly_wcf_inclamp' is set (if no contradict)
        """

        for beam_id in self.sequence:
            self.compute_beam_assembly_direction_from_joints_and_sequence(beam_id, assembly_vector_if_no_joint, beam_in_jaw_position_gap_offset)

    compute_assembly_direction_from_joints_and_sequence = compute_all_assembly_direction_from_joints_and_sequence

    def compute_beam_assembly_direction_from_joints_and_sequence(self, beam_id, assembly_vector_if_no_joint=None, beam_in_jaw_position_gap_offset=None):

        # Take the default value from assembly attribute of setting values are not given
        if assembly_vector_if_no_joint is None:
            assembly_vector_if_no_joint = self.attributes.get('assembly_vector_if_no_joint')
            if assembly_vector_if_no_joint is None:
                print('Default assembly_vector_if_no_joint is not set! using 0,0,-40')
                assembly_vector_if_no_joint = Vector(0, 0, -40)
        if beam_in_jaw_position_gap_offset is None:
            beam_in_jaw_position_gap_offset = self.attributes.get('beam_in_jaw_position_gap_offset')
            if beam_in_jaw_position_gap_offset is None:
                print('Default beam_in_jaw_position_gap_offset is not set! using 10')
                beam_in_jaw_position_gap_offset = 10

        # Helper function to quickly determine contradicting direction
        def assembly_direction_contradict(vectors):
            for i, vector_i in enumerate(vectors):
                for j, vector_j in enumerate(list(vectors)[i+1:]):
                    if vector_i.dot(vector_j) < 1e-5:
                        return True
            return False

        beam = self.beam(beam_id)
        print("Computing assembly direction for Beam %s:" % beam)
        # Compute assembly_wcf_final.
        # It is equal to beam.frame, because the beam is modelled in its final position
        self.set_beam_attribute(beam_id, 'assembly_wcf_final', beam.frame.copy())

        # Compute assembly_wcf_inclamp
        # We are going to check all the joint pairs.
        # Vector is computed from averaging the assembly direction of the joint
        # Vector length is computed from the combined height of pairs of joint.
        # The maximum length value is taken if there are multiple joints.
        assembly_direction_wcf = []
        assembly_vector_length = 0.0
        for _, neighbout_beam_id in self.get_joints_of_beam_connected_to_already_built(beam_id):
            print("neighbout_beam_id: %s" % neighbout_beam_id)
            joint_on_this_beam = self.joint((beam_id, neighbout_beam_id))
            joint_on_this_neighbor = self.joint((neighbout_beam_id, beam_id))

            assembly_direction_wcf.append(joint_on_this_beam.get_assembly_direction(beam).unitized())
            assembly_vector_length = max(joint_on_this_neighbor.height + joint_on_this_beam.height, assembly_vector_length)

        if len(assembly_direction_wcf) == 0:
            # No Neighbouring joint case
            # Handles no_clamp (perhaps) grounded beams v.s. typical clamp installed beams
            assembly_vector_wcf = assembly_vector_if_no_joint
            print("No Neighbour: Assembly Direction of Beam (%s) using downward movement %s (ref to wcf)." % (beam.name, assembly_vector_wcf))
        else:
            # Test if the list of assembly_direction_wcf contradicts eachother
            contradict = assembly_direction_contradict(assembly_direction_wcf)
            if contradict:
                # No Solution Case
                print("WARNING: Beam (%s) Assembly direction is blocked by joints: %s" % (beam_id, assembly_direction_wcf))
                assembly_vector_wcf = None
            else:
                # Solution exist Case
                assembly_vector_wcf = Vector.sum_vectors(assembly_direction_wcf).unitized().scaled(assembly_vector_length + beam_in_jaw_position_gap_offset)
                print("Assembly Direction of Beam (%s) computed (from %s): assembly_vector_wcf = %s (ref to wcf)." % (beam_id, assembly_direction_wcf, assembly_vector_wcf))

        self.set_beam_attribute(beam_id, 'assembly_vector_final', assembly_vector_wcf)

        # Compute the assembly_wcf_inclamp by moving back along the assembly_direction_wcf.
        if assembly_vector_wcf is None:
            self.set_beam_attribute(beam_id, 'assembly_wcf_inclamp', None)
        else:
            wcf_clamp_transform = Translation.from_vector(assembly_vector_wcf.scaled(-1.0))
            self.set_beam_attribute(beam_id, 'assembly_wcf_inclamp', beam.frame.transformed(wcf_clamp_transform))

    def beam_problems(self, beam_id):
        # type: (str) -> tuple[bool, list[str]]
        """ Returns a tuple describing if the beam is valid in the assembly
        """
        beam_problems = []
        if self.get_beam_attribute(beam_id, 'assembly_wcf_final') is None:
            beam_problems.append('assembly_wcf_final is None, this shouldn not happen.')
        if self.get_beam_attribute(beam_id, 'assembly_vector_final') is None:
            beam_problems.append('assembly_vector_final is None, possibly conflicting joints.')

        return beam_problems
