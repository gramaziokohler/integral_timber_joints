import cProfile

import compas
from compas.datastructures import Mesh, mesh_transform
from compas.geometry import Frame, Point, Transformation, Vector

print('compas.__version__ : ', compas.__version__)

f1 = Frame([2, 2, 2], [0.12, 0.58, 0.81], [-0.80, 0.53, -0.26])
f2 = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
T = Transformation.from_frame_to_frame(f1, f2)
mesh = Mesh.from_ply(compas.get_bunny())

print('mesh.number_of_vertices() : ', mesh.number_of_vertices())


def transforamtion_test():
    for _ in range(1):
        mesh_transform(mesh, T)


cProfile.run('transforamtion_test()', sort=2)
