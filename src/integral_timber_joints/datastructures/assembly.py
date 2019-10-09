# This file originates from Keerthana Udaykumar's <kudaykum@student.ethz.ch> master thesis on `Timber Grammar` in Fall 2019
# Pulled from https://github.com/KEERTHANAUDAY/timber_grammar on 2019-10-09 commit: d389364304f3740e652dbe4e85291402a2df0636
# This file is there on modified by Victor Leung <leung@arch.eth.ch> for use in Integral Timber Joint project

# Assembly Object
#   Assembly object are used to model a system that consist of multiple Beam objects.
#   It holds all the Beam objects
# It hold reference
#


import compas
import json
from .beam import Beam

from .utils import create_id

__all__ = ['Assembly']

class Assembly(object):

    def __init__(self):

        self.beams=[]

    @property
    def data(self):
        """dict : A data dict representing all internal persistant data for serialisation.
        The dict has the following structure:
        * 'type'            => string (name of the class)
        * 'beams'           => list of dict of Beam.to_data()
        """
        data = {
            'type'      : self.__class__.__name__,
            'beams'     : [beam.to_data() for beam in self.beams]
            }
        return data


    @data.setter
    def data(self, data):
        for beam_data in data.get('beams'):
            self.beams.append(Beam.from_data(beam_data))

    @classmethod
    def from_data(cls,data):
        """Construct an assembled Beam object from structured data.
        Parameters
        ----------
        data : dict
            The data dictionary.

        Returns
        -------
        object
            An object of the type Model

        Note
        ____
        This constructor method is meant to be used in conjuction with
        the corresponding *to_data* method.
        """
        new_object = cls()
        new_object.data = data

        return new_object


    def to_data(self):
        """Returns a dict of structured data representing the data structure.
        Actual implementation is in @property def data(self)

        Returns
        _______
        dict
            The structued data.

        Note
        ____
        This method produces the data that can be used in conjuction with the
        corresponding *from_data* class method.

        """
        return self.data

    @classmethod
    def from_json(cls, filepath):
        """Construct a datastructure from structured data contained in a json file.

            Parameters
            ---------
            filepath : str
            The path to the json file.

        Returns
        -------
        object
            An object of the type of ``cls``.

        Note
        ----
        This constructor method is meant to be used in conjuction with the
        corresponding *to_json* method.

        """

        # PATH = os.path.abspath(os.path.join(filepath, '..', 'data'))
        with open(filepath, 'r') as fp:
            data = json.load(fp)
        print('Json Loaded: Type=', data['type'])

        assert data['type'] ==  cls.__name__ , "Deserialized object type: %s is not equal to %s." % (data['type'] , cls.__name__)
        return cls.from_data(data)

    def to_json(self, filepath, pretty=False):
        """Serialise the structured data representing the data structure to json.

        Parameters
        ----------
        filepath : str
            The path to the json file.

        """
        # PATH = os.path.abspath(os.path.join(filepath, '..', 'data'))
        with open(filepath, 'w+') as fp:
            if pretty:
                json.dump(self.data, fp, sort_keys=True, indent=4)
            else:
                json.dump(self.data, fp)

if __name__ == '__main__':
    import compas
    import tempfile
    import os

    #Test 1 : Save a Assembly and load it back and compare their data
    print("Test 1: Model Data Save and Load to JSON")

    #Create Beam object
    beam = Beam.debug_get_dummy_beam(include_joint=True)

    #Add beam to Assembly
    model = Assembly()
    model.beams.append(beam)

    #Save Assembly
    model.to_json(os.path.join(tempfile.gettempdir(), "model.json"),pretty=True)

    #Load the saved Assembly
    loaded_model = Assembly.from_json(os.path.join(tempfile.gettempdir(), "model.json"))

    print("Test 1: Comparing two Model's data dictionary:")
    assert (model.data == loaded_model.data)
    if (model.data == loaded_model.data):
        print("Correct")
    else:
        print("Incorrect")

    print("-- -- -- -- -- -- -- --")

    print("Test 3: Print out dummy beam (with Joint) data")
    print (model.data)

    print("-- -- -- -- -- -- -- --")
