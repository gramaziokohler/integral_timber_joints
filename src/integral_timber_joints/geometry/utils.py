import uuid

from compas.datastructures.mesh import Mesh
from compas.geometry.primitives.frame import Frame
from compas.geometry.primitives.point import Point
from compas.geometry.primitives.vector import Vector
from compas.geometry.shapes.box import Box


def create_id():
    """Generates a UUID
    Return:
    ------
    str: UUID
    """
    g = str(uuid.uuid1())
    return g


def mesh_box_from_vertices(vertices):
    # type: (list[Point]) -> Mesh
    """ Creates an box with 8 vertices and 6 quad faces.
    Note that the ordering of the points are clockwise from top view
    for both bottom [0-3] and top [4-7] corners.
    This is different from compas.geometry.Box corners order
    """

    faces = [[0, 1, 2, 3], [0, 3, 7, 4], [3, 2, 6, 7], [2, 1, 5, 6], [1, 0, 4, 5], [4, 7, 6, 5]]
    mesh_box = Mesh.from_vertices_and_faces(vertices, faces)
    return mesh_box


def mesh_move_vertex_from_neighbor(mesh, vertices_ids, nbr_vertices_id, distance):
    # type: (Mesh, list[int], list[int], float) -> None
    """ Moves the specified vertices (`vertices_ids`) away from or towards
    its neighbour specified in `nbr_vertices_id`.

    Positive distance is moving away.
    """
    for id, nbr_id in zip(vertices_ids, nbr_vertices_id):
        vertex_coordinates = mesh.vertex_coordinates(id)
        v = Vector.from_start_end(mesh.vertex_coordinates(nbr_id), vertex_coordinates)
        v.unitize()
        v.scale(distance)
        new_values = [a+b for a, b in zip(vertex_coordinates, v)]
        mesh.vertex_attributes(id, 'xyz', new_values)


if __name__ == "__main__":
    a = create_id()
    print(type(a))
    print(a)
