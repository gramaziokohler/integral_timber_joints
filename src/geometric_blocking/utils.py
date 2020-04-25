import numpy as np
from numpy.linalg import norm
import cdd

__all__ = [
    'compute_feasible_region_from_block_dir',
]

def compute_feasible_region_from_block_dir(block_dirs, verbose=False):
    """ Compute extreme ray representation of feasible assembly region,
    given blocking direction vectors.
    The feasible assembly region is constrained by some hyperplanes, which use
    block_dirs as normals. cdd package allows us to convert the inequality
    representation to a generator (vertices and rays) of a polyhedron.
    More info on cddlib:
    https://pycddlib.readthedocs.io/en/latest/index.html
    Other packages on vertex enumeration:
    https://mathoverflow.net/questions/203966/computionally-efficient-vertex-enumeration-for-convex-polytopes
    Parameters
    ----------
    block_dirs : list of 3-tuples
        a list blocking directions.
    Returns
    -------
    f_rays: list of 3-tuples
        extreme rays of the feasible assembly region
    lin_set: list of int
        indices of rays that is linear (both directions)
    """
    mat_hrep = [] # "half-space" representation
    for vec in block_dirs:
        # For a polyhedron described as P = {x | A x <= b}
        # the H-representation is the matrix [b -A]
        mat_hrep.append([0, -vec[0], -vec[1], -vec[2]])
    mat = cdd.Matrix(mat_hrep, number_type='fraction')
    mat.rep_type = cdd.RepType.INEQUALITY
    poly = cdd.Polyhedron(mat)
    ext = poly.get_generators()
    lin_set = list(ext.lin_set) # linear set both directions

    nt = cdd.NumberTypeable('float')
    f_verts = []
    f_rays = []
    # linear_rays = []
    for i in range(ext.row_size):
        if ext[i][0] == 1:
            f_verts.append(tuple([nt.make_number(num) for num in ext[i][1:4]]))
        elif ext[i][0] == 0:
            # TODO: numerical instability?
            ray_vec = [nt.make_number(num) for num in ext[i][1:4]]
            ray_vec /= norm(ray_vec)
            f_rays.append(tuple(ray_vec))
            # if i in lin_set:
            #     lin_vec_set.append(tuple([- nt.make_number(num) for num in ext[i][1:4]]))

    # if f_verts:
    #     assert len(f_verts) == 1
        # np.testing.assert_almost_equal f_verts[0] == [0,0,0]

    # TODO: QR decomposition to make orthogonal
    if verbose:
        print('##############')
        print('ext:\n {}'.format(ext))
        print('ext linset:\n {}'.format(ext.lin_set))
        print('verts:\n {}'.format(f_verts))
        print('rays:\n {}'.format(f_rays))

    return f_rays, lin_set
