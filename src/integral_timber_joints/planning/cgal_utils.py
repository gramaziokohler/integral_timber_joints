from .utils import LOGGER

try:
    from CGAL import CGAL_Polygon_mesh_processing

    from CGAL.CGAL_Polygon_mesh_processing import Point_3_Vector
    from CGAL.CGAL_Polygon_mesh_processing import Polygon_Vector
    from CGAL.CGAL_Polygon_mesh_processing import Polylines
    from CGAL.CGAL_Polygon_mesh_processing import Int_Vector

    from CGAL.CGAL_Polyhedron_3 import Polyhedron_3
    from CGAL.CGAL_Kernel import Point_3

    HAS_CGAL = True
except ImportError:
    LOGGER.warning('CGAL not installed so remeshing functionality (densifying vertices in mesh for sweep collision checking) not enabled. Please install CGAL bindings by `conda install -c conda-forge cgal`')
    HAS_CGAL = False

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
    # https://doc.cgal.org/latest/Polygon_mesh_processing/group__PMP__meshing__grp.html#gaafd017f4424c3942bfdcc93874c8f596
    # https://github.com/CGAL/cgal-swig-bindings/blob/422c3e2a3230e478dd609e1dc44d6529e2bc963d/examples/python/test_pmp.py#L84-L88
    P = cgal_mesh_from_V_F(V, F)
    if verbose:
        LOGGER.debug('Before splitting long edges, #V: {}, #F: {}'.format(P.size_of_vertices(), P.size_of_facets()))
    # the range of edges to be split if they are longer than given threshold
    hlist = []
    for hh in P.halfedges():
        hlist.append(hh)
    CGAL_Polygon_mesh_processing.split_long_edges(hlist, max_length, P)
    if verbose:
        LOGGER.debug('After splitting long edges, #V: {}, #F: {}'.format(P.size_of_vertices(), P.size_of_facets()))
    return V_F_from_cgal_mesh(P)



