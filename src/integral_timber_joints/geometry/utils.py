import uuid

from compas.datastructures.mesh import Mesh, mesh_weld
from compas.geometry import Frame, Point, Vector
from compas.geometry import Box, Polyhedron, Line, Transformation
from compas.geometry import is_polygon_convex, transform_points


def create_id():
    """Generates a UUID
    Return:
    ------
    str: UUID
    """
    g = str(uuid.uuid1())
    return g


def eval_quad(square, u, v):
    # type: (list[Point], float, float) -> Point
    """Evaluate a point in a quad defined by four points.
    u value is along 0-1 and 3-2
    v value is along the connected line in direction of 1-2 or 0-3
    """
    line_01 = Line(square[0], square[1])
    line_32 = Line(square[3], square[2])
    point_on_01 = line_01.point(u)
    point_on_32 = line_32.point(u)
    line_01_32 = Line(point_on_01, point_on_32)
    return line_01_32.point(v)


def simplify_polygon(pts, tol=1e-5):
    new_pts = []
    n = len(pts)
    for i in range(n):
        v1 = Vector.from_start_end(pts[(i-1) % n], pts[i % n])
        v2 = Vector.from_start_end(pts[i % n], pts[(i+1) % n])
        if v1.angle(v2) > tol:
            new_pts.append(pts[i])
    return new_pts


def conforming_delaunay_triangulation(pts, normal):
    # type: (list[Point], Vector) -> Mesh

    v_x = Vector.from_start_end(pts[0], pts[1])
    v_y = normal.cross(v_x)

    # Transform the points to XY plane.
    T = Transformation.from_frame(Frame(pts[0], v_x, v_y))
    pts = transform_points(pts, T.inverse())
    pts = [[round(x, 2) for x in point] for point in pts]
    from compas.rpc import Proxy
    triangle = Proxy('compas.geometry')
    # print(pts)
    v, f = triangle.conforming_delaunay_triangulation(pts)
    v = transform_points(v, T)
    print("Original len(v) = %i, Resulting len(v) = %i, len(f) = %s" % (len(pts), len(v), len(f)))

    # Unify mesh normal to be equal to intended normal
    # print(f)
    # n = len(v) - 1
    # f = [[i % n for i in face] for face in f]
    # print ("f = %s" % f)

    # mesh = Mesh.from_vertices_and_faces(v[:-1], f)
    mesh = Mesh.from_vertices_and_faces(v, f)
    mesh_weld(mesh, 1e-6)
    v, f = mesh.to_vertices_and_faces()
    print("After cenverting to mesh, len(v) = %s, len(f) = %s" % (len(v), len(f)))

    # mesh.unify_cycles()
    if normal.dot(mesh.face_normal(0)) < 0:
        mesh.flip_cycles()
    v, f = mesh.to_vertices_and_faces()
    print("After flipping cycles, len(v) = %s, len(f) = %s" % (len(v), len(f)))
    return v, f


def polyhedron_box_from_vertices(vertices):
    # type: (list[Point]) -> Mesh
    """ Creates an polygon-extrude-like-box with n vertices and (n-4) triangle + n quad faces.
    Note that the ordering of the given vertices are clockwise from top view
    for both bottom and top corners. Top and botton cap will be triangulated.

    Number of vertices must be even number. Bottom vertices first, top vertices last.
    """
    assert len(vertices) % 2 == 0
    n = int(len(vertices) / 2)
    faces = []

    # Top and bottom caps - triangles
    if is_polygon_convex(vertices[:n]):
        # Btm and top cap
        for i in range(n - 2):
            faces.append([0, i+1, i+2])
            faces.append([i+2+n, i+1+n, n])

        # Side walls - quads
        for i in range(n):
            x = i % n
            y = (i+1) % n
            faces.append([y, x, x+n, y+n])

    else:
        btm_vertices = vertices[:n]
        vec_top_btm = Vector.from_start_end(vertices[n], vertices[0])
        # btm_vertices.append(btm_vertices[0])
        top_vertices = vertices[n:]
        vec_btm_top = Vector.from_start_end(vertices[0], vertices[n])
        # top_vertices.append(top_vertices[0])

        v, f = conforming_delaunay_triangulation(btm_vertices, vec_top_btm)
        mesh_btm = Mesh.from_vertices_and_faces(v, f)
        print("edges_on_boundary", mesh_btm.edges_on_boundary())
        vertices = v
        faces = f

        v, f = conforming_delaunay_triangulation(top_vertices, vec_btm_top)
        assert len(v) == len(vertices)
        n = len(v)  # Remove overlapping point
        for face in f:
            faces.append([i + n for i in face])
        vertices.extend(v)

        for x, y in mesh_btm.edges_on_boundary():
            faces.append([x, y, y + n, x + n])

        # Welding vertices and faces after this delaunay capping
        # mesh = mesh_weld(Mesh.from_vertices_and_faces(vertices, faces), 1e-8)
        # vertices, faces = mesh.to_vertices_and_faces()

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
