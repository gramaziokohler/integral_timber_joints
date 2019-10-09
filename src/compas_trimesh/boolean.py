# This file originates from Keerthana Udaykumar's <kudaykum@student.ethz.ch> master thesis on `Timber Grammar` in Fall 2019
# Pulled from https://github.com/KEERTHANAUDAY/timber_grammar on 2019-10-09 commit: d389364304f3740e652dbe4e85291402a2df0636
# This file is there on modified by Victor Leung <leung@arch.eth.ch> for use in Integral Timber Joint project

# This package is a helper package for performing trimesh boolean function via Proxy call.
# An external file can use the following syntax to perform the boolean:
#
#       from compas.rpc import Proxy
#       trimesh_proxy = Proxy(package='compas_trimesh')
#       result = trimesh_proxy.trimesh_subtract(mesh_a, mesh_b)
#       result_mesh = Mesh.from_data(result['value'])
#       return result_mesh


__all__ = ['trimesh_subtract','trimesh_subtract_multiple']

def trimesh_subtract(c_mesh1,c_mesh2):
    import compas
    from compas.datastructures import Mesh
    import trimesh
    assert isinstance(c_mesh1,Mesh)
    assert isinstance(c_mesh2,Mesh)

    mesh1_v = c_mesh1.to_vertices_and_faces()[0]
    mesh1_f = c_mesh1.to_vertices_and_faces()[1]

    mesh2_v = c_mesh2.to_vertices_and_faces()[0]
    mesh2_f = c_mesh2.to_vertices_and_faces()[1]

    mesh_1 = trimesh.Trimesh(vertices=mesh1_v, faces=mesh1_f, process=False)
    mesh_2 = trimesh.Trimesh(vertices=mesh2_v, faces=mesh2_f, process=False)

    #print(mesh_1.vertices)
    #print(mesh_2.vertices)
    boolean_sub = mesh_1.difference(mesh_2,engine='scad')
    #print(boolean_sub.vertices)


    result_compas_mesh = Mesh.from_vertices_and_faces(boolean_sub.vertices, boolean_sub.faces)
    return result_compas_mesh

def trimesh_subtract_multiple(meshes):
    import compas
    from compas.datastructures import Mesh
    import trimesh
    #import trimesh.boolean.difference as difference

    #Create a list of Trimesh objects from compas mesh objects
    tri_meshes = []
    for mesh in meshes:
        assert isinstance(mesh,Mesh)
        vertices = mesh.to_vertices_and_faces()[0]
        faces = mesh.to_vertices_and_faces()[1]
        new_trimesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
        tri_meshes.append(new_trimesh)

    #Calls Trimesh function to perform boolean.
    boolean_sub =  trimesh.boolean.difference(tri_meshes,engine='scad')

    #Recreate a compas mesh from the result and returns
    result_compas_mesh = Mesh.from_vertices_and_faces(boolean_sub.vertices, boolean_sub.faces)
    return result_compas_mesh


if __name__ == '__main__':

    import compas
    from compas.geometry import Frame
    from compas.geometry import Box
    from compas.datastructures import Mesh

    #construct the mesh
    box = Box(Frame.worldXY(),500,100,100)
    box_mesh = Mesh.from_vertices_and_faces(box.vertices, box.faces)

    box_2 = Box(([250,20,20], [300,0,100], [0,100,0]), 100, 50, 80)
    box_mesh_2 = Mesh.from_vertices_and_faces(box_2.vertices, box_2.faces)

    result_1 = trimesh_subtract(box_mesh, box_mesh_2)
    result_2 = trimesh_subtract_multiple([box_mesh, box_mesh_2])

    print("Comparing two boolean result")

    if (result_1.data == result_2.data):
        print("Correct")
    else:
        print("Incorrect")

