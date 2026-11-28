from dataclasses import replace

import numpy as np
import pytest

from salus_perception.native_ground import classify
from salus_perception.radial_ground import RadialGroundConfig, non_ground_points


@pytest.mark.parametrize("virtual_origin", [True, False])
def test_native_classifier_matches_reference(virtual_origin):
    config = replace(RadialGroundConfig(), use_virtual_ground_point=virtual_origin)
    rng = np.random.default_rng(104)
    for _ in range(8):
        points = rng.uniform((-25., -25., -.4), (25., 25., 1.), size=(12000, 3))
        points[::1000, 1] = np.nan
        np.testing.assert_array_equal(
            non_ground_points(points, config, classifier=classify),
            non_ground_points(points, config),
        )
