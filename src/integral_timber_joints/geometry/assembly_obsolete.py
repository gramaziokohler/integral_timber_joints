# This file originates from Keerthana Udaykumar's <kudaykum@student.ethz.ch> master thesis on `Timber Grammar` in Fall 2019
# Pulled from https://github.com/KEERTHANAUDAY/timber_grammar on 2019-10-09 commit: d389364304f3740e652dbe4e85291402a2df0636
# This file is there on modified by Victor Leung <leung@arch.eth.ch> for use in Integral Timber Joint project

# Assembly Object
#   Assembly object are used to model a system that consist of multiple Beam objects.
#   It holds all the Beam objects
# It hold reference
#

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json

from compas.datastructures import Network
from compas.datastructures.network.core import Graph

from integral_timber_joints.datastructures.beam import Beam
from integral_timber_joints.datastructures.utils import create_id

from copy import deepcopy

__all__ = ['Assembly']

class Assembly(Network):
    """Assembly class keeps all the beams and their connections
    This is inhereting the compas.datastructures Network class.
    Note the inhertance from : Assembly - Network - BaseNetwork -  Graph - Datastructure - object
    becuase some functions are located in different class methods and some are not documented

    In general network_attribute, node_attribute and edge_attribute will be used to store related objects and data.
    Graph Class performs serialization and deserialization them.

    Network.nodes are used to store Beams
    Network.edges are used to store Joints
    This implementation will not allow 3-way joints, but that is deemed outside scope.


    """
    def __init__(self):
        Network.__init__(self)
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
        return self.network.attributes.get('name', None)

    @name.setter
    def name(self, value):
        self.network.attributes['name'] = value

    # @property
    # def beams(self):
    #     return list(self.nodes())

    def add_beam(self, beam, key=None):
        """
        Parameters
        ----------
        beam : integral_timber_joint.datastructure.Beam
            A beam object
        key : hashable, optional
            An identifier for the beam. Defaults to None

        Returns
        -------
        vkey : hashable
            The new beam key.
        """
        return self.add_node(key, {'beam' : beam})

    def number_of_elements(self):
        return self.number_of_vertices()

    def number_of_connections(self):
        return self.network.number_of_edges()

    @property
    def data(self):
        """dict : A data dict representing all internal persistant data for serialisation.
        The dict has the following structure:
        * 'type'            => string (name of the class)
        * 'beams'           => list of dict of Beam.to_data()
        """
        # data = super(Assembly, self).data
        data = super(Assembly, self.__class__).data.fget(self)

        # Niceity reminder of the serialized data type
        data['type'] = self.__class__.__name__

        # Overridding part of the to_data method to serialize the beams
        for key in self.node:
            data['node'][repr(key)] = deepcopy(self.node[key])
            data['node'][repr(key)]['beam'] = data['node'][repr(key)]['beam'].to_data()

        return data

    @data.setter
    def data(self, data):
        super(Assembly, self.__class__).data.fset(self, data)

        # Reconstruct the Beam objects
        for key, attr in iter(self.node.items()):
            self.node[key]['beam'] = Beam.from_data(self.node[key]['beam'])

if __name__ == '__main__':
    import compas
    import tempfile
    import os

    #Test 1 : Test serialization

    #Create Beam object
    beam = Beam.debug_get_dummy_beam(include_joint=True)

    #Add Beam to Assembly
    model = Assembly()
    model.add_beam(beam)

    print("Test 1: Print out test model's data dictionary:")
    print(model.data)
    print("-- -- -- -- -- -- -- --")

    print("Test 2: Model Data Save and Load to JSON, comparing two Model's data dictionary:")

    #Save Assembly
    model.to_json(os.path.join(tempfile.gettempdir(), "model.json"),pretty=True)

    #Load the saved Assembly
    loaded_model = Assembly.from_json(os.path.join(tempfile.gettempdir(), "model.json"))
    assert (id(model) != id(loaded_model))
    assert (model.data == loaded_model.data)
    if (model.data == loaded_model.data):
        print("Correct")
    else:
        print("Incorrect")

    print("-- -- -- -- -- -- -- --")

    print("Test 3: Print out loaded data")
    print (loaded_model.data)

    print("-- -- -- -- -- -- -- --")
