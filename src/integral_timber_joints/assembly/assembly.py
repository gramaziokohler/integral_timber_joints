from __future__ import absolute_import, division, print_function

from compas.datastructures import Network
from compas.rpc import Proxy


class Assembly(Network):
    """A data structure for discrete element assemblies.

    An assembly is essentially a network of assembly elements.
    Each element is represented by a vertex of the network.
    Each interface or connection between elements is represented by an edge of the network.

    Attributes
    ----------
    network : :class:`compas.Network`, optional
    elements : list of :class:`Element`, optional
        A list of assembly elements.
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
    >>> assembly = Assembly()
    >>> for i in range(2):
    >>>     element = Element.from_box(Box(Frame.worldXY(), 10, 5, 2))
    >>>     assembly.add_element(element)
    """

    def __init__(self):

        # Call super class init
        Network.__init__(self)

        # Create default attributes
        self.attributes.update({
            'name': 'Unnamed_Assembly',
            'sequence': [],                     # List of (beam_id / beam.name / node key)
        })
        # Default attributes for beams (node)
        self.update_default_node_attributes({
            'is_planned': False,
            'is_placed': False,
            'is_visible': True,
            'is_attached': True,
            'design_guide_vector_jawapproach': None,
            'assembly_vector_final': None,
            'assembly_vector_jawapproach': None,
            'assembly_vector_pickup': None,
            'assembly_wcf_storageapproach': None,   # Beam gripper position (modeled as beam frame) before approaching pickup point
            'assembly_wcf_storage': None,           # Beam position at pick-up point
            'assembly_wcf_storageretract': None,    # Beam position after lifting off from pick-up point
            'assembly_wcf_inclampapproach': None,   # Beam position before being placed inside clamp jaw
            'assembly_wcf_inclamp': None,           # Beam position inside the clamp, ready for final clamping move
            'assembly_wcf_final': None,             # Beam position in final modeled position, same as beam.frame
            'assembly_wcf_finalretract': None,      # Beam gripper position (modeled as beam frame) after releasing and retracting from final position.
            'gripper_type': None,
            'gripper_id': None,
            'gripper_grasp_face': None,             # Grasp pose expressed in relationship to Beam Face
            'gripper_grasp_dist_from_start': None,  # Grasp pose expressed in relationship to Beam Length Parameter
            'gripper_tcp_in_ocf': None,             # Gripper grasp pose expressed in TCP location relative to the OCF
            'design_guide_vector_grasp': None,      # Gripper grasp pose guide Vector (align with Z of TCP in WFC)
            'design_guide_vector_storage_pickup': None,  # Gripper grasp pose guide Vector (align with Z of TCP in WFC)
        })
        # Default attributes for joints (edge)
        self.update_default_edge_attributes({
            'sequence_earlier': False,
            'is_clamp_attached_side': False,
            'clamp_wcf_attachapproach1': None,      # Clamp position beforing approaching attachment point (1 happens before 2)
            'clamp_wcf_attachapproach2': None,      # Clamp position beforing approaching attachment point
            'clamp_wcf_final': None,                # Clamp position at attachment point "clamp_frame_wcf"
            'clamp_wcf_attachretract': None,        # Robot Position (modeled as clamp frame) after releasing the clamp at final position and retracting.
            'clamp_wcf_detachapproach': None,       # Robot Position (modeled as clamp frame) before picking up the clamp from final position.
            'clamp_wcf_detachretract1': None,       # Clamp position after assembly, retracting from attachment point (1 happens before 2)
            'clamp_wcf_detachretract2': None,       # Clamp position after assembly, retracting from attachment point
            'clamp_type': None,
            'clamp_id': None,
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
        return self.attributes['sequence']

    @sequence.setter
    def sequence(self, beam_id_list):
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
        """Add a beam"""
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
        """Add a joint"""
        self.add_one_joint(joint1, beam1_id, beam2_id)
        self.add_one_joint(joint2, beam2_id, beam1_id)
        self.beam(beam1_id).cached_mesh = None
        self.beam(beam2_id).cached_mesh = None

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

    def beam(self, key):
        """Get a beam by its key."""
        return self.node_attribute(key, 'beam')

    def joint(self, joint_id):
        """Get a joint by its id.
        joint_id is a Tuple """
        return self.edge_attribute(joint_id, 'joint')

    def get_joints_of_beam(self, beam_id):
        """Get all the joints attached to a beam"""
        joints = []
        for neighbor_beam_id in self.neighbors_out(beam_id):
            joints.append(self.joint((beam_id, neighbor_beam_id)))
        return joints

    def get_joint_ids_of_beam(self, beam_id):
        """Get all the ids of joints (beam_id, neighbor_beam_id) that are attached to a beam
        """
        joint_ids = []
        for neighbor_beam_id in self.neighbors_out(beam_id):
            joint_ids.append((beam_id, neighbor_beam_id))
        return joint_ids

    def get_reverse_joint_ids_of_beam(self, beam_id):
        """Get all the ids of joints (neighbor_beam_id, beam_id) that are attached to a beam
        """
        joint_ids = []
        for neighbor_beam_id in self.neighbors_out(beam_id):
            joint_ids.append((neighbor_beam_id, beam_id))
        return joint_ids

    def get_beam_sequence(self, beam_id):
        # type: (str) -> optional(int)
        """ Returns the (0-counting) position of the beam in self.sequence
        If beam is not in self.sequence, returns None
        """
        if beam_id not in self.sequence: return None
        return self.sequence.index(beam_id)
    # --------------------------------------------
    # Iterating through all Beams and Joints
    # --------------------------------------------

    def beams(self):
        # type: () -> Iterator[Beams]
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

    # --------------------------------------------
    # Beam Joints Geometrical Functions
    # --------------------------------------------

    def update_beam_mesh_with_joints(self, beam_id, skip_if_cached=False):
        """Update the cached_mesh of a beam, taking into account all the joints attached.

        Parameters
        ----------
        beam_id : int
            index of the beam
        skip_if_cached : bool, optional
            Skip recomputeing cache mesh if it already exist, by default False.
        """
        if skip_if_cached and (self.beam(beam_id).cached_mesh is not None):
            return
        joints = self.get_joints_of_beam(beam_id)
        self.beam(beam_id).update_cached_mesh(joints)

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

    def get_joints_of_beam_connected_to_already_built(self, beam_id):
        # type: (self)
        """Return the joints ids (beam_id, neighbor_id) that are connected to already-assembled beams
        """
        sequence_i_of_beam = self.get_beam_sequence(beam_id)
        return [(beam_id, neighbor_id) for neighbor_id in self.get_already_built_neighbors(beam_id)]

    def get_joint_ids_of_beam_clamps(self, beam_id):
        # type: (self)
        """Return the list [joint_id] where is_clamp_attached_side = True
        where
        are attached.
        """
        # The if statement checkes if the joint attribute 'is_clamp_attached_side' == True
        return [joint_id for joint_id in self.get_reverse_joint_ids_of_beam(beam_id) if self.get_joint_attribute(joint_id, 'is_clamp_attached_side') == True]

    def get_already_built_beams(self, beam_id):
         # type: (self) -> list[str]
        "Return the beam ids of already-built beams relative to the given beam_id"
        this_beam_sequence = self.get_beam_sequence(beam_id)
        return self.sequence[:this_beam_sequence]

    def get_already_built_neighbors(self, beam_id):
        # type: (self) -> list[str]
        "Return the beam ids of neighbours that are connected to already-assembled beams"
        # The if statement checkes if the joint attribute 'sequence_earlier' == False
        this_beam_sequence = self.get_beam_sequence(beam_id)
        return [neighbor_id for neighbor_id in self.neighbors_out(beam_id) if self.get_beam_sequence(neighbor_id) < this_beam_sequence]

    def compute_assembly_direction_from_joints_and_sequence(self, assembly_vector_if_no_joint, beam_in_jaw_position_gap_offset):
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
        from compas.geometry import Vector
        from compas.geometry import Translation

        def assembly_direction_contradict(vectors):
            for i, vector_i in enumerate(vectors):
                for j, vector_j in enumerate(list(vectors)[i+1:]):
                    if vector_i.dot(vector_j) < 0: return True
            return False

        for beam_id in self.sequence:

            beam = self.beam(beam_id)
            print ("> %s" % beam)
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
                print ("neighbout_beam_id: %s" % neighbout_beam_id)
                joint_on_this_beam = self.joint((beam_id, neighbout_beam_id))
                joint_on_this_neighbor = self.joint((neighbout_beam_id, beam_id))

                assembly_direction_wcf.append(joint_on_this_beam.get_assembly_direction(beam).unitized())
                assembly_vector_length = max(joint_on_this_neighbor.height + joint_on_this_beam.height, assembly_vector_length)

            if len(assembly_direction_wcf) == 0:
                # No Neighbouring joint case
                # Handles no_clamp (perhaps) grounded beams v.s. typical clamp installed beams
                assembly_vector_wcf = assembly_vector_if_no_joint
                print("Assembly Direction of Beam (%s) using downward movement %s (ref to wcf)." % (beam.name, assembly_direction_wcf))
            else:
                # Test if the list of assembly_direction_wcf contradicts eachother
                contradict = assembly_direction_contradict(assembly_direction_wcf)
                if contradict :
                    # No Solution Case
                    print("WARNING: Beam (%s) Assembly direction is blocked by joints." % (beam_id))
                    assembly_vector_wcf = None
                else:
                    # Solution exist Case
                    assembly_vector_wcf = Vector.sum_vectors(assembly_direction_wcf).unitized().scaled(assembly_vector_length + beam_in_jaw_position_gap_offset)
                    print("Assembly Direction of Beam (%s) computed from Joints to be %s (ref to wcf)." % (beam_id, assembly_direction_wcf))

            self.set_beam_attribute(beam_id, 'assembly_vector_final', assembly_vector_wcf)

            # Compute the assembly_wcf_inclamp by moving back along the assembly_direction_wcf.
            if assembly_vector_wcf is None:
                self.set_beam_attribute(beam_id, 'assembly_wcf_inclamp', None)
                print ('assembly_wcf_inclamp set to None')

            else:
                wcf_clamp_transform = Translation.from_vector(assembly_vector_wcf.scaled(-1.0))
                self.set_beam_attribute(beam_id, 'assembly_wcf_inclamp', beam.frame.transformed(wcf_clamp_transform))
