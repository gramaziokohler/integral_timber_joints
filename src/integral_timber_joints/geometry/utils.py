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
    Note that the ordering of the given vertices are clockwise from top view
    for both bottom [0-3] and top [4-7] corners.
    This is different from compas.geometry.Box corners order
    """
    assert len(vertices) == 8
    faces = [[0, 1, 2, 3], [0, 3, 7, 4], [3, 2, 6, 7], [2, 1, 5, 6], [1, 0, 4, 5], [4, 7, 6, 5]]
    mesh_box = Mesh.from_vertices_and_faces(vertices, faces)
    return mesh_box


def triangle_box_from_vertices(vertices):
    # type: (list[Point]) -> Mesh
    """ Creates an triangle-box with 6 vertices and 2 triangle + 3 quad faces.
    Note that the ordering of the given vertices are clockwise from top view
    for both bottom [0-2] and top [3-5] corners.
    This is different from compas.geometry.Box corners order
    """
    assert len(vertices) == 6
    faces = [[0, 1, 2], [3, 5, 4], [0, 2, 5, 3], [2, 1, 4, 5], [1, 0, 3, 4]]
    mesh_box = Mesh.from_vertices_and_faces(vertices, faces)
    return mesh_box


def polygon_box_from_vertices(vertices):
    # type: (list[Point]) -> Mesh
    """ Creates an polygon-extrude-like-box with n vertices and (n-4) triangle + n quad faces.
    Note that the ordering of the given vertices are clockwise from top view
    for both bottom and top corners. Top and botton cap will be triangulated.

    Number of vertices must be even number.
    """
    assert len(vertices) % 2 == 0
    n = len(vertices) / 2
    faces = []
    # Top and bottom caps - triangles
    for i in range(n - 2):
        faces.append([0, i+1, i+2])
        faces.append([i+2+n, i+1+n, n])

    # Side walls - quads
    for i in range(n):
        x = i % n
        y = (i+1) % n
        faces.append([y, x, x+n, y+n])
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
