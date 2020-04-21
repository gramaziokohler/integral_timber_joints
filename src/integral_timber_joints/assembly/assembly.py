from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json

from compas.datastructures import Network

from .element import Element
from .utilities import _deserialize_from_data
from .utilities import _serialize_to_data

__all__ = ['Assembly']


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
            'assembly_sequence': [],
            })
        self.update_default_node_attributes({
            'is_planned': False,
            'is_placed': False
        })


    @property
    def name(self):
        """str : The name of the assembly."""
        return self.attributes.get('name', None)

    @name.setter
    def name(self, value):
        self.attributes['name'] = value

    def sequence(self):
        return self.attributes['assembly_sequence']

    def number_of_elements(self):
        """Compute the number of elements of the assembly.

        Returns
        -------
        int
            The number of elements.

        """
        return self.number_of_nodes()

    def number_of_connections(self):
        """Compute the number of connections of the assembly.

        Returns
        -------
        int
            the number of connections.

        """
        return self.number_of_edges()

    @property
    def data(self):
        """dict : A data dict representing all internal persistant data for serialisation.
        The dict has the following structure:
        * 'type'            => string (name of the class)
        * 'beams'           => list of dict of Beam.to_data()
        """
        from copy import deepcopy
        # Call super class to_data
        data_dict = super(Assembly, self.__class__).data.fget(self)

        # Overridding part of the to_data method to serialize the beams
        for key in self.node:
            data_dict['node'][repr(key)] = deepcopy(self.node[key])
            data_dict['node'][repr(key)]['beam'] = data_dict['node'][repr(key)]['beam'].to_data()

        # Niceity reminder of the serialized data type
        data_dict['type'] = self.__class__.__name__

        return data_dict

    # @data.setter
    # def data(self, data):
    #     # Deserialize elements from vertex dictionary
    #     for _vkey, vdata in data['vertex'].items():
    #         vdata['element'] = Element.from_data(vdata['element'])

    #     self.network = Network.from_data(data)

    @data.setter
    def data(self, data):
        # Call super class data.setter
        super(Assembly, self.__class__).data.fset(self, data)

        # Reconstruct the Beam objects
        for key, attr in iter(self.node.items()):
            self.node[key]['beam'] = Beam.from_data(self.node[key]['beam'])

    def clear(self):
        """Clear all the assembly data."""
        self.clear()

    def add_element(self, element, key=None, attr_dict={}, **kwattr):
        """Add an element to the assembly.

        Parameters
        ----------
        element : Element
            The element to add.
        attr_dict : dict, optional
            A dictionary of element attributes. Default is ``None``.

        Returns
        -------
        hashable
            The identifier of the element.
        """
        attr_dict.update(kwattr)
        x, y, z = element.frame.point
        key = self.add_node(
            key=key,
            attr_dict=attr_dict,
            x=x, y=y, z=z, element=element)
        return key

    def add_connection(self, u, v, attr_dict=None, **kwattr):
        """Add a connection between two elements and specify its attributes.

        Parameters
        ----------
        u : hashable
            The identifier of the first element of the connection.
        v : hashable
            The identifier of the second element of the connection.
        attr_dict : dict, optional
            A dictionary of connection attributes.
        kwattr
            Other connection attributes as additional keyword arguments.

        Returns
        -------
        tuple
            The identifiers of the elements.
        """
        return self.network.add_edge(u, v, attr_dict, **kwattr)

    def transform(self, transformation):
        """Transforms this assembly.

        Parameters
        ----------
        transformation : :class:`Transformation`

        Returns
        -------
        None
        """
        for _k, element in self.elements(data=False):
            element.transform(transformation)

    def transformed(self, transformation):
        """Returns a transformed copy of this assembly.

        Parameters
        ----------
        transformation : :class:`Transformation`

        Returns
        -------
        Assembly
        """
        assembly = self.copy()
        assembly.transform(transformation)
        return assembly

    def copy(self):
        """Returns a copy of this assembly.

        Elements and their _source are copied
        Connecions and their dictionary type of data are copied
        """

        return Assembly.from_data(self.to_data())

    def element(self, key):
        """Get an element by its key."""
        return self.network.vertex[key]['element']

    def elements(self, data=False):
        """Iterate over the elements of the assembly.

        Parameters
        ----------
        data : bool, optional
            If ``True``, yield both the identifier and the attributes.

        Yields
        ------
        2-tuple
            The next element as a (key, element) tuple, if ``data`` is ``False``.
        3-tuple
            The next element as a (key, element, attr) tuple, if ``data`` is ``True``.

        """
        if data:
            for vkey, vattr in self.network.vertices(True):
                yield vkey, vattr['element'], vattr
        else:
            for vkey in self.network.vertices(data):
                yield vkey, self.network.vertex[vkey]['element']

    def beams(self):
        for key, element in self.elements():
            yield element._source

    def beam(self, key):
        """Get an element by its key."""
        return self.network.vertex[key]['element']._source

    def connections(self, data=False):
        """Iterate over the connections of the network.

        Parameters
        ----------
        data : bool, optional
            If ``True``, yield both the identifier and the attributes.

        Yields
        ------
        2-tuple
            The next connection identifier (u, v), if ``data`` is ``False``.
        3-tuple
            The next connection as a (u, v, attr) tuple, if ``data`` is ``True``.

        """
        return self.network.edges(data)

    def set_beam_attribute(self, beam_id, key, value):
        return self.network.set_vertex_attribute(beam_id, key, value)

    def get_beam_attribute(self, beam_id, key, value=None):
        return self.network.get_vertex_attribute(beam_id, key, value)

    def set_joint_attribute(self, joint_id, key, value):
        return self.network.set_edge_attribute(joint_id, key, value)

    def get_joint_attribute(self, joint_id, key, value=None):
        return self.network.get_edge_attribute(joint_id, key, value)
