import os  # Save file location
import tempfile  # Save file location

from compas.geometry import Frame
from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.geometry.joint_90lap import Joint_90lap
from integral_timber_joints.assembly.assembly import Assembly

def test_beam_and_joint_in_assembly():
    length = 1000
    width = 100
    height = 150
    beam1 = Beam(Frame((0, 0, 0), (1, 0, 0), (0, 1, 0)), length, width, height,'b1')
    beam2 = Beam(Frame((0, 200, 0), (1, 0, 0), (0, 1, 0)), length, width, height,'b2')
    beam3 = Beam(Frame((0, 400, 0), (1, 0, 0), (0, 1, 0)), length, width, height,'b3')

    j1_2 = Joint_90lap(100,1,100,100,50)
    j2_1 = Joint_90lap(100,1,100,100,50)
    j2_3 = Joint_90lap(100,1,100,100,50)
    j3_2 = Joint_90lap(100,1,100,100,50)

    a = Assembly()
    a.name = 'TestAssembly'
    a.add_beam(beam1)
    a.add_beam(beam2)
    a.add_beam(beam3)

    # Check object add and retrival
    a.add_joint_pair(j1_2, j2_1, 'b1', 'b2')
    assert a.joint('b1', 'b2') is j1_2
    assert a.joint('b2', 'b1') is j2_1

    # Check muliple call to add joint will overwite old data.
    a.add_joint_pair(None, None, 'b2', 'b3')
    a.add_joint_pair(j2_3, j3_2, 'b2', 'b3')
    assert a.joint('b2', 'b3') is j2_3
    assert a.joint('b3', 'b2') is j3_2

    # Check connectivity and valance
    assert len(list(a.joints())) == 4
    assert len(list(a.joint_id_pairs())) == 2

    # Check get_joints_of_beam() function
    assert a.get_joints_of_beam('b1')[0] == j1_2
    assert a.get_joints_of_beam('b3')[0] == j3_2
    assert len(a.get_joints_of_beam('b2')) == 2
    assert j2_1 in a.get_joints_of_beam('b2')
    assert j2_3 in a.get_joints_of_beam('b2')


if __name__ == "__main__":
    test_beam_and_joint_in_assembly()
    print ("Test file pass (%s)" % __file__)
    pass
