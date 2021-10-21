from compas.datastructures import Mesh
from compas.geometry import Point, Shape, Cylinder, Polyhedron
from compas_rhino.utilities.drawing import draw_breps, draw_cylinders


def mesh_to_brep(mesh):
    # type: (Mesh) -> dict
    """Converts a mesh to a brep schema for use in draw_breps()

    >>> breps = mesh_to_brep(mesh)
    >>> guids = draw_breps(breps, join=True)
    """
    vertices = [mesh.vertex_attributes(vertex, 'xyz') for vertex in mesh.vertices()]
    breps = []
    for face in mesh.faces():
        face = {'points': [vertices[vertex] for vertex in mesh.face_vertices(face)]}
        face['points'].append(face['points'][0])
        breps.append(face)
    return breps


def vertices_and_faces_to_brep_struct(vertices_and_faces):
    # type: (Mesh) -> dict
    """Converts a (vertices_and_faces) to a brep schema for use in draw_breps()

    >>> breps = mesh_to_brep(mesh)
    >>> guids = draw_breps(breps, join=True)
    """
    vertices, faces = vertices_and_faces
    if isinstance(vertices[0], Point):
        vertices = [list(v) for v in vertices]
    breps = []
    for face in faces:
        f = {'points': [vertices[vertex_index] for vertex_index in face]}
        f['points'].append(f['points'][0])
        breps.append(f)
    return breps

def draw_shapes_as_brep_get_guids(shapes, verbose=False):
    # type: (list[Shape], bool) -> list[guid]
    """Drawing list of shapes in Rhino as Breps and returning the resulting guids"""
    def vprint(str):
        if verbose:
            print(str)

    brep_guids = []
    for shape in shapes:
        if isinstance(shape, Polyhedron):
            polyhedron = shape
            vertices_and_faces = polyhedron.to_vertices_and_faces()
            struct = vertices_and_faces_to_brep_struct(vertices_and_faces)
            vprint("Polyhedron : %s" % struct)
            guids = draw_breps(struct, join=True, redraw=False)
            brep_guids.extend(guids)

        elif isinstance(shape, Cylinder):
            cylinder = shape
            start = cylinder.center + cylinder.normal.scaled(cylinder.height / 2)
            end = cylinder.center - cylinder.normal.scaled(cylinder.height / 2)
            struct = {'start': list(start), 'end': list(end), 'radius': cylinder.circle.radius}
            vprint("Cylinder : %s" % struct)
            guids = draw_cylinders([struct], cap=True, redraw=False)
            brep_guids.extend(guids)

    return brep_guids