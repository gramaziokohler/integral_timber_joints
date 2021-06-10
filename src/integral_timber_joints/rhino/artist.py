from compas.datastructures import Mesh
from compas.geometry import Point
from compas_rhino.utilities.drawing import draw_breps


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
