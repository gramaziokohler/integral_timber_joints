import os, time
import numpy as np
from collections import defaultdict
from itertools import combinations

import pybullet_planning as pp
from pybullet_planning import wait_if_gui
from pybullet_planning import elapsed_time, LockRenderer

from compas.robots import Joint
from compas.datastructures import Mesh
from compas.geometry import Frame, Transformation
from compas_fab.robots import Configuration, AttachedCollisionMesh, CollisionMesh
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.conversions import transformation_from_pose
from compas_fab_pychoreo.utils import is_configurations_close, verify_trajectory, LOGGER
from compas.datastructures.mesh.triangulation import mesh_quads_to_triangles

from CGAL import CGAL_Polygon_mesh_processing

from CGAL.CGAL_Polygon_mesh_processing import Point_3_Vector
from CGAL.CGAL_Polygon_mesh_processing import Polygon_Vector
from CGAL.CGAL_Polygon_mesh_processing import Polylines
from CGAL.CGAL_Polygon_mesh_processing import Int_Vector

from CGAL.CGAL_Polyhedron_3 import Polyhedron_3
from CGAL.CGAL_Kernel import Point_3

# import igl
# https://anaconda.org/conda-forge/cgal
# from compas_cgal.meshing import remesh, remesh_constrained
# from CGAL.CGAL_Polygon_mesh_processing import split_long_edges

from pybullet_planning.interfaces.env_manager.pose_transformation import unit_pose
from integral_timber_joints.planning.parsing import parse_process

# https://github.com/CGAL/cgal-swig-bindings/blob/main/examples/python/test_pmp.py
def cgal_mesh_from_V_F(V, F):
    P = Polyhedron_3()
    points = Point_3_Vector()
    points.reserve(len(V))
    for pt in V:
        points.append(Point_3(pt[0], pt[1], pt[2]))

    polygons = Polygon_Vector()
    polygons.reserve(len(F))
    for face in F:
        polygon = Int_Vector()
        polygon.reserve(3)
        polygon.append(int(face[0]))
        polygon.append(int(face[1]))
        polygon.append(int(face[2]))
        polygons.append(polygon)

    CGAL_Polygon_mesh_processing.polygon_soup_to_polygon_mesh(
        points, polygons, P)
    assert P.size_of_vertices() == len(V)
    assert P.size_of_facets() == len(F)
    return P

def V_F_from_cgal_mesh(P):
    # set vertex id, this will be written in memory so we can access it in halfedges
    V = []
    vertices = list(P.vertices())
    for i, v in enumerate(vertices):
        v.set_id(i)
        pt = v.point()
        V.append([pt.x(), pt.y(), pt.z()])

    F = []
    for face in P.facets():
        e1 = face.halfedge()
        e2 = e1.next()
        e3 = e2.next()
        assert e3.next() == e1
        F.append([e1.vertex().id(), e2.vertex().id(), e3.vertex().id()])
    return V, F

def cgal_split_long_edges(V, F, max_length=0.3, verbose=False):
    P = cgal_mesh_from_V_F(V, F)
    if verbose:
        print('Before splitting long edges, #V: {}, #F: {}'.format(P.size_of_vertices(), P.size_of_facets()))
    # the range of edges to be split if they are longer than given threshold
    hlist = []
    for hh in P.halfedges():
        hlist.append(hh)
    CGAL_Polygon_mesh_processing.split_long_edges(hlist, max_length, P)
    if verbose:
        print('After splitting long edges, #V: {}, #F: {}'.format(P.size_of_vertices(), P.size_of_facets()))
    return V_F_from_cgal_mesh(P)

def draw_mesh_at_pose(mesh: Mesh, pose=unit_pose(), sharp_angle=np.pi * 0.4):
    V, F = mesh.to_vertices_and_faces()
    np_V, np_F = np.array(V), np.array(F, int)
    new_V = np.array(pp.apply_affine(pose, np_V))
    with LockRenderer():
        for fid in range(np_F.shape[0]):
            for v0, v1 in combinations(np_F[fid, :], 2):
                pp.add_line(new_V[v0,:], new_V[v1,:], pp.apply_alpha(pp.BLACK, 0.2))

        # # https://libigl.github.io/libigl-python-bindings/igl_docs/#sharp_edges
        # SE = igl.sharp_edges(np_V, np_F, sharp_angle)[0]
        # for i in range(SE.shape[0]):
        #     v0, v1 = SE[i,:]
        #     pp.add_line(new_V[v0,:], new_V[v1,:], color=pp.RED, width=3)

###################################

HERE = os.path.dirname(__file__)
design_dir = '220407_CantiBoxLeft'
problem = 'CantiBoxLeft_process.json'
problem_subdir = '.'
process = parse_process(design_dir, problem, subdir=problem_subdir)

viewer = True
sharp_angle = np.pi * 0.4

beam_id = list(process.assembly.beam_ids())[0]
mesh = process.assembly.get_beam_mesh_in_ocf(beam_id).copy()
mesh_quads_to_triangles(mesh)
# add mesh to environment at origin

with PyChoreoClient(viewer=viewer) as client:
    pp.draw_pose(pp.unit_pose())

    # * original geometry
    orig_cm = CollisionMesh(mesh, beam_id)
    orig_cm.scale(1e-3)
    client.add_collision_mesh(orig_cm)

    body = client._get_collision_object_bodies('^{}$'.format(beam_id))[0]
    for link in pp.get_all_links(body):
        pp.set_color(body, link=link, color=pp.apply_alpha(pp.GREY, 0.6))
    old_link_from_vertices = pp.get_body_collision_vertices(body)
    with LockRenderer():
        for body_link, vertices in old_link_from_vertices.items():
            old_link_from_vertices[body_link] = vertices
            for vert in vertices:
                pp.draw_point(vert, size=0.01, color=pp.RED, width=3)
    draw_mesh_at_pose(mesh, sharp_angle=sharp_angle)

    # * remeshed geometry
    # https://compas.dev/compas_cgal/latest/api/generated/compas_cgal.subdivision.catmull_clark.html
    V, F = mesh.to_vertices_and_faces()
    # np_V, np_F = np.array(V), np.array(F)
    # new_mesh_V_F = igl.upsample(np_V, np_F, number_of_subdivs=3)
    # new_mesh_V_F = remesh((V,F), 0.5)
    new_mesh_V_F = cgal_split_long_edges(V, F, max_length=0.1, verbose=True)

    # # https://libigl.github.io/libigl-python-bindings/igl_docs/#sharp_edges
    # sharp_edges = igl.sharp_edges(np_V, np_F, sharp_angle)[0]
    # new_mesh_V_F = remesh_constrained((V,F), 0.1, sharp_edges)

    remeshed_mesh = Mesh.from_vertices_and_faces(*new_mesh_V_F)
    remeshed_cm = CollisionMesh(remeshed_mesh, beam_id + '_remeshed')
    client.add_collision_mesh(remeshed_cm)
    body = client._get_collision_object_bodies('^{}$'.format(beam_id + '_remeshed'))[0]

    pp.set_pose(body, pp.Pose(point=(0,2,0)))
    for link in pp.get_all_links(body):
        pp.set_color(body, link=link, color=pp.apply_alpha(pp.WHITE, 0.6))
    draw_mesh_at_pose(remeshed_mesh, pose=pp.Pose(point=(0,2,0)), sharp_angle=sharp_angle)

    old_link_from_vertices = pp.get_body_collision_vertices(body)
    with LockRenderer():
        for body_link, vertices in old_link_from_vertices.items():
            old_link_from_vertices[body_link] = vertices
            for vert in vertices:
                pp.draw_point(vert, size=0.01, color=pp.BLUE, width=3)

    wait_if_gui('first conf')

    # pp.set_pose(body, pp.Pose(point=(0,2,1.0)))
    # new_link_from_vertices = pp.get_body_collision_vertices(body)
    # with LockRenderer():
    #     for body_link, vertices in new_link_from_vertices.items():
    #         new_link_from_vertices[body_link] = vertices
    #         for old_vert, vert in zip(old_link_from_vertices[body_link], vertices):
    #             pp.draw_point(vert, size=0.01, color=pp.RED)
    #             pp.add_line(old_vert, vert)
    # draw_mesh_at_pose(remeshed_mesh, pose=pp.Pose(point=(0,2,1.0)), sharp_angle=sharp_angle)
    # wait_if_gui('Finish')
