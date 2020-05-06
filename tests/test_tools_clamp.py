from integral_timber_joints.tools import Clamp

from compas.geometry import Box, Frame
from compas.datastructures import Mesh
from compas.robots import Link, Axis, Joint
import jsonpickle

def test_clamp_jsonpickle():
    clamp_1 = Clamp('c1')

    mesh_1 =  Mesh.from_vertices_and_faces(* Box(Frame.worldXY(), 1.0, 2.0, 3.0).to_vertices_and_faces())
    mesh_2 =  Mesh.from_vertices_and_faces(* Box(Frame.worldXY(), 6.0, 5.0, 4.0).to_vertices_and_faces())

    clamp_base = clamp_1.add_link('clamp_base', mesh_1)
    clamp_jaw = clamp_1.add_link('clamp_jaw',mesh_2)
    clamp_1.add_joint('joint_1', Joint.PRISMATIC, clamp_base, clamp_jaw, axis = [1,0,0], limit = (-100, 100))



    clamp_2 = jsonpickle.decode(jsonpickle.encode(clamp_1, keys=True), keys=True)

    # Checks to see if the draw_visuals() runs properly after pickling
    visuals_1 = clamp_1.draw_visuals()
    visuals_2 = clamp_2.draw_visuals()

    # Checks to see if the draw_visuals() returns same results after pickling
    assert visuals_1[0].to_data() == visuals_2[0].to_data()
    assert visuals_1[1].to_data() == visuals_2[1].to_data()

if __name__ == "__main__":
    test_clamp_jsonpickle()
