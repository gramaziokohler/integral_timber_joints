from compas.datastructures import Mesh
from compas.geometry import Box, Frame

import jsonpickle

def test_compas_mesh_jsonpickle():
    # This test only pass when jsonpickle.encode keys = True

    mesh_1 =  Mesh.from_vertices_and_faces(* Box(Frame.worldXY(), 1.0, 2.0, 3.0).to_vertices_and_faces())
    mesh_2 = jsonpickle.decode(jsonpickle.encode(mesh_1, keys=True), keys=True)

    # Assert de-pickled data is same from the perspective of compas native to_data()
    assert mesh_1.to_data() == mesh_2.to_data()

    # Assert that second pickling is the same as first pickling
    assert jsonpickle.encode(mesh_1, keys=True) == jsonpickle.encode(mesh_2, keys=True)

if __name__ == "__main__":
    test_compas_mesh_jsonpickle()
    pass
