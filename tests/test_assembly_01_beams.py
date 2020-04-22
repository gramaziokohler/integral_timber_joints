import os  # Save file location
import tempfile  # Save file location

from compas.geometry import Frame
from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.assembly.assembly import Assembly

def test_beam_in_assembly():
    length = 1000
    width = 100
    height = 150
    beam1 = Beam(Frame((0, 0, 0), (1, 0, 0), (0, 1, 0)), length, width, height,'b1')
    beam2 = Beam(Frame((0, 200, 0), (1, 0, 0), (0, 1, 0)), length, width, height,'b2')

    a = Assembly()
    a.name = 'TestAssembly'
    a.add_beam(beam1)
    a.add_beam(beam2)

    # Check object retrival
    assert id(beam1) == id (a.beam('b1'))
    assert id(beam2) == id (a.beam('b2'))

    # Check Iteration
    assert beam1 in a.beams()
    assert beam2 in a.beams()

    # Check sequence
    assert a.sequence == ['b1', 'b2']

    # Check display functions
    assert str(a) is not None

    # Check default attributes
    assert a.get_beam_attribute('b1','is_visible') == True

    # Check assigning attributes
    assert a.get_beam_attribute('b1','my_attribute') is None
    a.set_beam_attribute('b1','my_attribute',42)
    assert a.get_beam_attribute('b1','my_attribute') == 42

    # Check serialization
    import jsonpickle
    a2 = jsonpickle.decode(jsonpickle.encode(a))
    print (jsonpickle.encode(a))
    assert id(a) != id(a2)
    assert len(a2.sequence) == 2
    assert a2.get_beam_attribute('b1','my_attribute') == 42

if __name__ == "__main__":
    test_beam_in_assembly()
    pass
