import uuid

from compas.datastructures.mesh import Mesh
from compas.geometry import Frame, Point, Vector
from compas.geometry import Box, Polyhedron


def create_id():
    """Generates a UUID
    Return:
    ------
    str: UUID
    """
    g = str(uuid.uuid1())
    return g


def polyhedron_box_from_vertices(vertices):
    # type: (list[Point]) -> Mesh
    """ Creates an polygon-extrude-like-box with n vertices and (n-4) triangle + n quad faces.
    Note that the ordering of the given vertices are clockwise from top view
    for both bottom and top corners. Top and botton cap will be triangulated.

    Number of vertices must be even number.
    """
    assert len(vertices) % 2 == 0
    n = int(len(vertices) / 2)
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
    polyhedron = Polyhedron(vertices, faces)
    return polyhedron


def move_vertex_from_neighbor(vertices, vertices_ids, nbr_vertices_id, distance):
    # type: (Mesh, list[int], list[int], float) -> None
    """ Moves the specified vertices (`vertices_ids`) away from or towards
    its neighbour specified in `nbr_vertices_id`.

    `vertices` : list of list of 3 numbers

    Positive distance is moving away.
    """
    for id, nbr_id in zip(vertices_ids, nbr_vertices_id):
        vertex_coordinates = vertices[id]
        v = Vector.from_start_end(vertices[nbr_id], vertex_coordinates)
        if v.length < 1e-6:
            continue
        v.unitize()
        v.scale(distance)
        new_values = [a+b for a, b in zip(vertex_coordinates, v)]
        vertices[id] = new_values


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
