import numpy as np

from workbench.research.analysis.compare_voxel_mesh import component_count


def test_voxel_connectivity_requires_a_shared_face():
    assert component_count(np.empty((0,3),dtype=int)) == 0
    assert component_count(np.array([[0,0,0],[1,0,0],[1,1,0]])) == 1
    assert component_count(np.array([[0,0,0],[1,1,0],[2,2,1]])) == 3
