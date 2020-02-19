import errno
import os
import sys

import compas.geometry as cg
import ghpythonlib.components as ghcomp
import Rhino
import Rhino.Geometry as rg
import rhinoscriptsyntax as rs
import scriptcontext as sc
from compas.datastructures.mesh import Mesh
from compas.geometry import Frame
from Grasshopper import DataTree as Tree
from Grasshopper.Kernel.Data import GH_Path as Path
from System import Array

# --------------------------------------------------

# Rhino.Geometry <> list


def list2rPt(a):
    return rg.Point3d(a[0], a[1], a[2])


def list2rVec(a):
    return rg.Vector3d(a[0], a[1], a[2])


def rVec2list(v):
    return [v.X, v.Y, v.Z]


def rPt2list(p):
    return [p.X, p.Y, p.Z]


def list2rLine(a):
    # tuple or list of two tuples/lists with 3 coordinates each
    return rg.Line(list2rPt(a[0]), list2rPt(a[1]))


def cVec2list(v):
    return [v.x, v.y, v.z]


def cPt2list(p):
    return [p.x, p.y, p.z]

# Rhino.Geometry <> compas.geometry


def cLine2rLine(L):
    return rg.Line(list2rPt(L[0]), list2rPt(L[1]))


def cFrame2rPlane(frame):
    return rg.Plane(list2rPt(frame.point), list2rVec(frame.xaxis), list2rVec(frame.yaxis))


def cPln2rPln(pln):
    return rg.Plane(list2rPt(pln.point), list2rVec(pln.normal))


def cPt2rPt(cPt):
    # return rg.Point3d(cPt['x'], cPt['y'], cPt['z']) #old version?
    return rg.Point3d(cPt.x, cPt.y, cPt.z)


def rPt2cPt(rPt):
    return cg.Point(rPt.X, rPt.Y, rPt.Z)


def cVec2rVec(v):
    return rg.Vector3d(v.x, v.y, v.z)


def rVec2cVec(v):
    return cg.Vector(v.X, v.Y, v.Z)


def rPlane2cFrame(plane):
    return Frame(plane.Origin, plane.XAxis, plane.YAxis)


# other
def rMesh_from8points(pts):
    M = rg.Mesh()
    for pt in pts:
        M.Vertices.Add(pt)

    M.Faces.AddFace(0, 4, 7, 3)
    M.Faces.AddFace(1, 5, 6, 2)
    M.Faces.AddFace(2, 6, 7, 3)
    M.Faces.AddFace(3, 7, 4, 0)
    M.Faces.AddFace(0, 1, 2, 3)
    M.Faces.AddFace(4, 5, 6, 7)
    return M


def rMesh_fromBeam(beam):
    pts = [list2rPt(p) for p in beam.corners]
    return rMesh_from8points(pts)


def rBrep_fromBeam(beam):
    pts = [list2rPt(p) for p in beam.corners]
    B = rg.Brep.CreateFromBox(pts)
    # B.Flip()
    return B


def cMesh2rMesh(mesh):
    # convert compas mesh to rhino mesh
    # TODO: not handling quad meshes yet!

    vertices, faces = mesh.to_vertices_and_faces()
    rhino_mesh = rg.Mesh()
    for v in vertices:
        rhino_mesh.Vertices.Add(list2rPt(v))
    for f in faces:
        rhino_mesh.Faces.AddFace(f[0], f[1], f[2])
    return rhino_mesh


def rMesh2cMesh(rg_mesh):
    if isinstance(rg_mesh, rg.Mesh):
        # single mesh
        compas_mesh_vertices = rg_mesh.Vertices.ToPoint3dArray()
        compas_mesh_faces = rg_mesh.Faces.ToIntArray(True)
        compas_mesh_faces = [compas_mesh_faces[i:i + 3] for i in range(0, len(compas_mesh_faces), 3)]
        return Mesh.from_vertices_and_faces(compas_mesh_vertices, compas_mesh_faces)
    else:
        # array of meshes
        meshes = []
        for m in rg_mesh:
            compas_mesh_vertices = m.Vertices.ToPoint3dArray()
            compas_mesh_faces = m.Faces.ToIntArray(True)
            compas_mesh_faces = [compas_mesh_faces[i:i + 3] for i in range(0, len(compas_mesh_faces), 3)]
            meshes.append(Mesh.from_vertices_and_faces(compas_mesh_vertices, compas_mesh_faces))
        return meshes

# network functions


def get_network_rhino_axes(net):
    new_axes = []
    for edge in net.edges():
        v1, v2 = net.edge_coordinates(edge[0], edge[1])
        v1 = rs.coerce3dpoint(v1)
        v2 = rs.coerce3dpoint(v2)
        new_axes.append(rg.Line(v1, v2))
    return new_axes


# GH tree <-> lists

def tree_to_list(input, retrieve_base=lambda x: x[0]):
    """Returns a list representation of a Grasshopper DataTree"""
    def extend_at(path, index, simple_input, rest_list):
        target = path[index]
        if len(rest_list) <= target:
            rest_list.extend([None] * (target - len(rest_list) + 1))
        if index == path.Length - 1:
            rest_list[target] = list(simple_input)
        else:
            if rest_list[target] is None:
                rest_list[target] = []
            extend_at(path, index + 1, simple_input, rest_list[target])
    all = []
    for i in range(input.BranchCount):
        path = input.Path(i)
        extend_at(path, 0, input.Branch(path), all)
    return retrieve_base(all)


def simple_tree_to_list(tree_input):
    lst = []
    for i in range(tree_input.BranchCount):
        branchList = tree_input.Branch(i)
        # branchPath = tree_input.Path(i)
        b = []
        for j in range(branchList.Count):
            b.append(branchList[j])
        lst.append(b)
    return lst


def list_to_tree(alist, none_and_holes=False, base_path=[0]):
    """
    Transforms nestings of lists or tuples to a Grasshopper DataTree
    Usage:
    mytree = [ [1,2], 3, [],[ 4,[5]] ]
    a = list_to_tree(mytree)
    b = list_to_tree(mytree, none_and_holes=True, base_path=[7,1])
    """
    def process_one_item(alist, tree, track):
        path = Path(Array[int](track))
        if len(alist) == 0 and none_and_holes:
            tree.EnsurePath(path)
            return
        for i, item in enumerate(alist):
            if hasattr(item, '__iter__'):  # if list or tuple
                track.append(i)
                process_one_item(item, tree, track)
                track.pop()
            else:
                if none_and_holes:
                    tree.Insert(item, path, i)
                elif item is not None:
                    tree.Add(item, path)

    tree = Tree[object]()
    if alist is not None:
        process_one_item(alist, tree, base_path[:])
    return tree


def two_points_vector(p1, p2, unit):
    vect = rg.Vector3d(p2.X - p1.X, p2.Y - p1.Y, p2.Z - p1.Z)
    if unit:
        vect.Unitize()
    return vect


def pt_xyz_to_zyx(p):
    return rg.Point3d(p.Z, p.Y, p.X)


def pt_zyx_to_xyz(p):
    return rg.Point3d(p.Z, p.Y, p.X)


def sorted_zyx(pts_list):
    temp_reversed_pts = [pt_xyz_to_zyx(p) for p in pts_list]
    sorted_pts = ghcomp.SortPoints(temp_reversed_pts)[0]
    return [pt_zyx_to_xyz(p) for p in sorted_pts]


def unify_beam_axis_orientation(a):
    if isinstance(a, rg.Line):
        start_pt = a.From
        end_pt = a.To
    else:
        start_pt = a.PointAtStart
        end_pt = a.PointAtEnd

    pts_list = [start_pt, end_pt]
    temp_reversed_pts = [pt_xyz_to_zyx(p) for p in pts_list]
    sorted_pts = ghcomp.SortPoints(temp_reversed_pts)[0]
    re_reversed_sorted_pts = [pt_zyx_to_xyz(p) for p in sorted_pts]

    """
    if re_reversed_sorted_pts[0].Z < re_reversed_sorted_pts[1].Z:
        pass
    else:
        print("SORTING ERROR")
    """

    return rg.Line(re_reversed_sorted_pts[0], re_reversed_sorted_pts[1])


# Rhino export

def is_path_creatable(pathname=None):
    """
    `True` if the current user has sufficient permissions to create the passed
    pathname; `False` otherwise.
    """
    # Parent directory of the passed path. If empty, we substitute the current
    # working directory (CWD) instead.
    dirname = os.path.dirname(pathname) or os.getcwd()
    return os.access(dirname, os.W_OK)


def is_pathname_valid(pathname=None):
    """
    `True` if the passed pathname is a valid pathname for the current OS;
    `False` otherwise.
    """
    # If this pathname is either not a string or is but is empty, this pathname
    # is invalid.

    ERROR_INVALID_NAME = 123

    try:
        if not isinstance(pathname, str) or not pathname:
            return False

        # Strip this pathname's Windows-specific drive specifier (e.g., `C:\`)
        # if any. Since Windows prohibits path components from containing `:`
        # characters, failing to strip this `:`-suffixed prefix would
        # erroneously invalidate all valid absolute Windows pathnames.
        _, pathname = os.path.splitdrive(pathname)

        # Directory guaranteed to exist. If the current OS is Windows, this is
        # the drive to which Windows was installed (e.g., the "%HOMEDRIVE%"
        # environment variable); else, the typical root directory.
        root_dirname = os.environ.get('HOMEDRIVE', 'C:') \
            if sys.platform == 'win32' else os.path.sep
        assert os.path.isdir(root_dirname)   # ...Murphy and her ironclad Law

        # Append a path separator to this directory if needed.
        root_dirname = root_dirname.rstrip(os.path.sep) + os.path.sep

        # Test whether each path component split from this pathname is valid or
        # not, ignoring non-existent and non-readable path components.
        for pathname_part in pathname.split(os.path.sep):
            try:
                os.lstat(root_dirname + pathname_part)
            # If an OS-specific exception is raised, its error code
            # indicates whether this pathname is valid or not. Unless this
            # is the case, this exception implies an ignorable kernel or
            # filesystem complaint (e.g., path not found or inaccessible).
            #
            # Only the following exceptions indicate invalid pathnames:
            #
            # * Instances of the Windows-specific "WindowsError" class
            #   defining the "winerror" attribute whose value is
            #   "ERROR_INVALID_NAME". Under Windows, "winerror" is more
            #   fine-grained and hence useful than the generic "errno"
            #   attribute. When a too-long pathname is passed, for example,
            #   "errno" is "ENOENT" (i.e., no such file or directory) rather
            #   than "ENAMETOOLONG" (i.e., file name too long).
            # * Instances of the cross-platform "OSError" class defining the
            #   generic "errno" attribute whose value is either:
            #   * Under most POSIX-compatible OSes, "ENAMETOOLONG".
            #   * Under some edge-case OSes (e.g., SunOS, *BSD), "ERANGE".
            except OSError as exc:
                if hasattr(exc, 'winerror'):
                    if exc.winerror == ERROR_INVALID_NAME:
                        return False
                elif exc.errno in {errno.ENAMETOOLONG, errno.ERANGE}:
                    return False
    # If a "TypeError" exception was raised, it almost certainly has the
    # error message "embedded NUL character" indicating an invalid pathname.
    except TypeError as exc:
        return False, exc
    # If no exception was raised, all path components and hence this
    # pathname itself are valid. (Praise be to the curmudgeonly python.)
    else:
        return True
    # If any other exception was raised, this is an unrelated fatal issue
    # (e.g., a bug). Permit this exception to unwind the call stack.
    #
    # Did we mention this should be shipped with Python already?


def is_path_exists_or_creatable(pathname=None):
    """
    `True` if the passed pathname is a valid pathname for the current OS _and_
    either currently exists or is hypothetically creatable; `False` otherwise.

    This function is guaranteed to _never_ raise exceptions.
    """
    try:
        # To prevent "os" module calls from raising undesirable exceptions on
        # invalid pathnames, is_pathname_valid() is explicitly called first.
        return is_pathname_valid(pathname) and (os.path.exists(pathname) or is_path_creatable(pathname))
    # Report failure on non-fatal filesystem complaints (e.g., connection
    # timeouts, permissions issues) implying this path to be inaccessible. All
    # other exceptions are unrelated fatal issues and should not be caught here.
    except OSError:
        return False


def exportByLayer(fileName, fileFolderPath, layerNames, ghdoc):

    if not os.path.isdir(fileFolderPath):
        os.mkdir(fileFolderPath)
    fileNameFolderPath = os.path.join(fileFolderPath, fileName)

    sc.doc = Rhino.RhinoDoc.ActiveDoc
    selectTheObjects = True
    for layerName in layerNames:
        # 'objIds_perLayer' is assigned to but never used
        objIds_perLayer = rs.ObjectsByLayer(layerName, selectTheObjects)  # select the geometires for export

    fileNameFolderPath2 = chr(34) + fileNameFolderPath + chr(34)
    commandString = "_-Export " + fileNameFolderPath2 + " _Enter _Enter _Enter"

    echo = False
    done = rs.Command(commandString, echo)
    if done:
        print("Geometry successfully exported from the following layers %s\nto: %s" % (layerNames, fileNameFolderPath))
    else:
        print("Something went wrong. Export terminated")

    rs.UnselectAllObjects()

    sc.doc = ghdoc
