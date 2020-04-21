from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.datastructures import Network

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
            'sequence': [],
            })
        # Default attributes for beams (node)
        self.update_default_node_attributes({
            'is_planned': False,
            'is_placed': False,
            'is_visible': True,
            'is_attached': True,
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
        self.add_node(beam.name, {'beam' : beam})

    def add_one_joint(self, joint, beam_id, neighbour_beam_id):
        # type: (Joint, str, str) -> None
        """Add a joint"""
        self.add_edge(beam_id, neighbour_beam_id, {'joint' : joint})

    def add_joint_pair(self, joint1, joint2, beam1_id, beam2_id):
        # type: (Joint, Joint, str, str) -> None
        """Add a joint"""
        self.add_one_joint(joint1, beam1_id, beam2_id)
        self.add_one_joint(joint2, beam2_id, beam1_id)

    # ----------------------------
    # Getting / Iterating Beams and Joints
    # ----------------------------

    def get_beam(self, key):
        """Get a beam by its key."""
        return self.node_attribute(key, 'beam')

    def get_joint(self, beam_id, neighbour_beam_id):
        """Get a joint by its id."""
        return self.edge_attribute((beam_id, neighbour_beam_id), 'joint')

    def beams(self):
        for key, data in self.nodes(data = True):
            yield data['beam']

    def beam_ids(self):
        for key in self.nodes(data = False):
            yield key
    def set_beam_attribute(self, beam_id, attribute_key, value):
        """ Setting an attribute of one beam

        Parameters
        ----------
        beam_id : str
            Identifier of the Beam (Beam.name)
        attribute_key : str
        value : str

        Returns
        -------
        [obj]
            Returns the attribute
        """
        self.node_attribute(beam_id, attribute_key, value)

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
            Identifier of the Joint (beam_id, neighbour_beam_id)
        attribute_key : [str]
        value : [obj]
        """
        self.edge_attribute(joint_id, attribute_key, value)

    def get_joint_attribute(self, joint_id, attribute_key):
        """ Getting an attribute of one joint

        Parameters
        ----------
        joint_id : (str, str)
            Identifier of the Joint (beam_id, neighbour_beam_id)
        attribute_key : [str]
        """
        return self.edge_attribute(joint_id, attribute_key)
