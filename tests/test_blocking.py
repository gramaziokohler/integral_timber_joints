# from geometric_blocking import compute_feasible_region_from_block_dir


from compas.geometry import Vector
from compas.rpc import Proxy

def compute_feasible_region_from_block_dir_proxy(block_dirs):
    geometric_blocking = Proxy(package='geometric_blocking')
    return geometric_blocking.compute_feasible_region_from_block_dir(block_dirs)


def test_proxycall_geometric_blocking():
    block_dirs = [[0,0,1], [0,0,-1.0]]
    f_rays, lin_set = compute_feasible_region_from_block_dir_proxy(block_dirs)

    assert [1.0, 0.0, 0.0] in f_rays
    assert [0.0, 1.0, 0.0] in f_rays
    assert 0 in lin_set
    assert 1 in lin_set

def test_proxycall_geometric_blocking_compas_vector():
    block_dirs = [Vector(0,0,1), Vector(0,0,-1.0)]
    f_rays, lin_set = compute_feasible_region_from_block_dir_proxy(block_dirs)

    assert [1.0, 0.0, 0.0] in f_rays
    assert [0.0, 1.0, 0.0] in f_rays
    assert 0 in lin_set
    assert 1 in lin_set


def test_blocking_negative_zero():
    block_dirs = [Vector(0.000, 0.000, 1.000), Vector(0.000, 0.000, -1.000), Vector(-0.000, 1.000, 0.000)]
    f_rays, lin_set = compute_feasible_region_from_block_dir_proxy(block_dirs)

    assert [1.0, 0.0, 0.0] in f_rays
    assert [0.0, -1.0, 0.0] in f_rays



if __name__ == "__main__":
    test_proxycall_geometric_blocking()
    test_proxycall_geometric_blocking_compas_vector()
    test_blocking_negative_zero()
    pass
