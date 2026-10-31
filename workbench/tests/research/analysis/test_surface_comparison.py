import numpy as np

from workbench.research.analysis.compare_surface_mesh import sample_surface


def test_surface_sampling_weights_area_instead_of_triangle_count():
    vertices = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.],
                         [10., 0., 0.], [20., 0., 0.], [10., 10., 0.]])
    samples = sample_surface(vertices, np.array([[0, 1, 2], [3, 4, 5]]), 10_000)
    assert .98 < np.mean(samples[:, 0] >= 10.) < .999
    assert np.all(samples[:, 2] == 0.)
    small = samples[samples[:, 0] < 10.]
    large = samples[samples[:, 0] >= 10.]
    assert np.all(small[:, :2] >= 0.) and np.all(small[:, 0]+small[:, 1] <= 1.)
    assert np.all(large[:, 0] >= 10.) and np.all(large[:, 0]+large[:, 1] <= 20.)
