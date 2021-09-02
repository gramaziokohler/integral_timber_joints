import os  # Save file location
import tempfile  # Save file location

from compas.geometry import Frame
from integral_timber_joints.geometry.beam import Beam
from integral_timber_joints.geometry.joint_90lap import Joint_90lap
from integral_timber_joints.assembly import Assembly

def test_beam_joint_boolean():
    length = 1000
    width = 100
    height = 150
    beam1 = Beam(Frame((0, 0, 0), (1, 0, 0), (0, 1, 0)), length, width, height, "b1")
    beam2 = Beam(Frame((0, 0, 0), (1, 0, 0), (0, 1, 0)), length, width, height, "b2")
    joint = Joint_90lap(300,3,height,100,20,"dummy_joint")

    a = Assembly()

    a.add_beam(beam1)
    a.add_beam(beam2)
    a.add_one_joint(joint,'b1', 'b2')

    # Assert mesh before boolean
    mesh = beam1.draw_uncut_mesh()
    assert mesh.number_of_vertices() == 8
    assert mesh.number_of_faces() == 12

    # Update beam mesh (requires boolean)
    mesh = a.get_beam_mesh('b1')
    assert mesh is not None
    assert beam1.cached_mesh is mesh
    assert mesh.number_of_vertices() == 16
    assert mesh.number_of_faces() == 28

    # Check if the cached mesh survives the mesh serialization
    import jsonpickle
    loaded_beam = jsonpickle.decode(jsonpickle.encode(beam1))

    assert loaded_beam.cached_mesh.number_of_vertices() == 16
    assert loaded_beam.cached_mesh.number_of_faces() == 28


def test_beam_multiple_joint_boolean():
    length = 2000
    width = 100
    height = 100
    beam = Beam(Frame((0, 0, 0), (1, 0, 0), (0, 1, 0)), length, width, height,"dummy_beam_1")
    joints = []
    joints.append(Joint_90lap(300,1,100,100,20,"dummy_joint"))
    joints.append(Joint_90lap(500,2,100,100,20,"dummy_joint"))
    joints.append(Joint_90lap(800,3,100,100,20,"dummy_joint"))
    joints.append(Joint_90lap(1200,4,100,100,20,"dummy_joint"))

    # Update beam mesh (requires boolean)
    beam.update_cached_mesh(joints)

    assert beam.cached_mesh is not None
    assert beam.cached_mesh.number_of_vertices() == 40


if __name__ == "__main__":
    test_beam_joint_boolean()
    test_beam_multiple_joint_boolean()
    pass
