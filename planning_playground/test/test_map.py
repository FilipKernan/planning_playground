import pytest

import numpy as np

import planning_playground.map.import_map as import_map
from planning_playground.planners.types import Node

"""
test to make sure that the map is imported correctly
test to make sure that the grid size is correct
test to make sure that the map is the correct size

test to make sure collision checking works correctly
    - test that a point is in collision
    - test that a point is not in collision
    - test that a point is on the edge of the map
    - test that a point is outside the map
    - test pixel to grid conversion
    - test grid to pixel conversion
    - test that the non-discrete collision checking works correctly - compare against the discrete version
"""


class MapFixture:
    def __init__(self):
        # setup the planner and all required objects here
        self.map = import_map.Map2d(
            "planning_playground/map/map_empty.png", grid_size=10
        )


@pytest.fixture
def test_a_star():
    return MapFixture()
