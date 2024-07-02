import pytest

import numpy as np

import planning_playground.map.import_map as import_map
import planning_playground.motion_models.holonomic_model as holonomic_model
import planning_playground.viz.viz_plan as viz_plan
from planning_playground.planners.rrt_star_planner import RRTStarPlanner
from planning_playground.planners.rrt_planner import RRTPlanner
from planning_playground.planners.a_star_planner import AStarPlanner
from planning_playground.planners.types import PathPlanningResult
from planning_playground.planners.types import Node


class RRTStarPlannerFixture:
    def __init__(self):
        # setup the planner and all required objects here
        self.map = import_map.Map2d(
            "planning_playground/map/map_empty.png", grid_size=60
        )
        self.motion_model = holonomic_model.HolonomicModel(
            [1, 1], 1, self.map, is_discrete=False
        )
        self.planner = RRTStarPlanner(self.map, self.motion_model)
        self.planner.radius = 50
        self.start = (100, 200, 0)
        self.goal = (800, 750, 0)
        self.planner.start_node = Node(self.motion_model, self.start, None)
        self.planner.nodes[self.start] = self.planner.start_node
        self.planner.goal_node = Node(self.motion_model, self.goal, None)
        self.result = PathPlanningResult()


@pytest.fixture
def test_rrt_star():
    return RRTStarPlannerFixture()


def create_test_nodes(motion_model, planner):
    state_1 = (150, 200, 0)
    state_2 = (100, 250, 0)
    state_3 = (150, 250, 0)
    node_1 = Node(motion_model, state_1, planner.start_node)
    planner.nodes[state_1] = node_1
    node_2 = Node(motion_model, state_2, planner.start_node)
    planner.nodes[state_2] = node_2
    node_3 = Node(motion_model, state_3, node_1)
    planner.nodes[state_3] = node_3


def test_rewire_where_new_node_is_better_path(test_rrt_star):
    # access the test_rrt_star object
    planner = test_rrt_star.planner
    motion_model = test_rrt_star.motion_model
    # create some nodes
    create_test_nodes(motion_model, planner)
    # create a new node
    new_state = (140, 225, 0)
    new_node = Node(motion_model, new_state, None)
    planner.process_new_node(new_node, test_rrt_star.result, 0)
    assert (
        new_node in planner.nodes.values()
    ), f"{str(new_node)} not in {str(planner.nodes.values())}"
    assert (
        new_node.parent == planner.nodes[(100, 200, 0)]
    ), f"{str(new_node.parent)} != {str(planner.nodes[(100, 200, 0)])}"
    assert (
        planner.nodes[(150, 250, 0)].parent == new_node
    ), f"{str(planner.nodes[(150, 250, 0)].parent)} != {str(new_node)}"
    assert (
        planner.nodes[(150, 200, 0)].parent == planner.nodes[(100, 200, 0)]
    ), f"{str(planner.nodes[(150, 200, 0)].parent)} != {str(planner.nodes[(100, 200, 0)])}"
    assert (
        planner.nodes[(100, 250, 0)].parent == planner.nodes[(100, 200, 0)]
    ), f"{str(planner.nodes[(100, 250, 0)].parent)} != {str(planner.nodes[(100, 200, 0)])}"


def test_rewire_where_new_node_is_not_better_path(test_rrt_star):
    # access the test_rrt_star object
    planner = test_rrt_star.planner
    motion_model = test_rrt_star.motion_model
    # create some nodes
    create_test_nodes(motion_model, planner)
    # create a new node
    planner.nodes.pop((150, 200, 0))
    assert (150, 200, 0) not in planner.nodes.keys()

    new_state = (140, 225, 0)
    new_node = Node(motion_model, new_state, planner.nodes[(100, 200, 0)])
    planner.nodes[(150, 250, 0)].parent = new_node
    planner.nodes[new_state] = new_node
    planner.nodes[(100, 250, 0)].calculate_cost(test_rrt_star.result.timing_data)

    sampled_state = (140, 200, 0)
    sampled_node = Node(motion_model, sampled_state, None)
    planner.process_new_node(sampled_node, test_rrt_star.result, 0)
    assert (
        sampled_node in planner.nodes.values()
    ), f"{str(sampled_node)} not in {[str(n) for n in planner.nodes.values()]}"
    assert (
        sampled_node.parent == planner.nodes[(100, 200, 0)]
    ), f"{str(sampled_node.parent)} != {str(planner.nodes[(100, 200, 0)])}"
    assert (
        planner.nodes[(150, 250, 0)].parent == new_node
    ), f"{str(planner.nodes[(150, 250, 0)].parent)} != {str(new_node)}"
    assert (
        new_node.parent == planner.nodes[(100, 200, 0)]
    ), f"{str(new_node.parent)} != {str(planner.nodes[(100, 200, 0)])}"


def test_rewire_where_new_node_is_on_best_path(test_rrt_star):
    # access the test_rrt_star object
    planner = test_rrt_star.planner
    motion_model = test_rrt_star.motion_model
    # create some nodes
    state_1 = (160, 240, 0)
    state_2 = (120, 230, 0)
    node_1 = Node(motion_model, state_1, planner.start_node)
    planner.nodes[state_1] = node_1
    node_2 = Node(motion_model, state_2, planner.start_node)
    planner.nodes[state_2] = node_2
    # create a new node
    new_state = (145, 220, 0)
    new_node = Node(motion_model, new_state, None)
    planner.process_new_node(new_node, test_rrt_star.result, 0)
    assert (
        new_node in planner.nodes.values()
    ), f"{str(new_node)} not in {str(planner.nodes.values())}"
    assert (
        new_node.parent == planner.nodes[(100, 200, 0)]
    ), f"{str(new_node.parent)} != {str(planner.nodes[(100, 200, 0)])}"
    assert (
        planner.nodes[(160, 240, 0)].parent == new_node
    ), f"{str(planner.nodes[(160, 240, 0)].parent)} != {str(new_node)}"
    assert (
        planner.nodes[(120, 230, 0)].parent == planner.nodes[(100, 200, 0)]
    ), f"{str(planner.nodes[(130, 230, 0)].parent)} != {str(planner.nodes[(100, 200, 0)])}"
    pass


def test_rewire_where_new_node_close_to_high_cost_node(test_rrt_star):
    # access the test_rrt_star object
    planner = test_rrt_star.planner
    motion_model = test_rrt_star.motion_model
    # create some nodes
    state_1 = (150, 225, 0)
    state_2 = (120, 270, 0)
    state_3 = (175, 240, 0)
    node_1 = Node(motion_model, state_1, planner.start_node)
    planner.nodes[state_1] = node_1
    node_2 = Node(motion_model, state_2, planner.start_node)
    planner.nodes[state_2] = node_2
    node_3 = Node(motion_model, state_3, node_2)
    planner.nodes[state_3] = node_3
    # create a new node
    new_state = (170, 240, 0)
    new_node = Node(motion_model, new_state, None)
    nearest, neighborhood = planner.get_neighborhood(
        new_node, test_rrt_star.result.timing_data
    )
    assert len(neighborhood) == 2, f"neighborhood is {[str(n) for n in neighborhood]}"
    assert nearest == planner.nodes[(175, 240, 0)], f"nearest is {str(nearest)}"
    assert (
        node_3 in neighborhood
    ), f"node_3 is not in neighborhood, {[str(n) for n in neighborhood]} and node_3 is {str(node_3)}"
    assert node_1 in neighborhood, f"node_1 is not in neighborhood"

    planner.process_new_node(new_node, test_rrt_star.result, 0)
    assert (
        new_node in planner.nodes.values()
    ), f"{str(new_node)} not in {[str(n) for n in planner.nodes.values()]}"
    assert new_node.parent == node_1, f"{str(new_node.parent)} != {str(node_1)}"
    assert node_3.parent == new_node, f"{str(node_3.parent)} != {str(new_node)}"
    assert (
        node_2.parent == planner.nodes[(100, 200, 0)]
    ), f"{str(node_2.parent)} != {str(planner.nodes[(100, 200, 0)])}"


def test_new_node_on_existing_path(test_rrt_star):
    # access the test_rrt_star object
    planner = test_rrt_star.planner
    motion_model = test_rrt_star.motion_model
    # create some nodes
    state_1 = (150, 200, 0)
    node_1 = Node(motion_model, state_1, planner.start_node)
    planner.nodes[state_1] = node_1

    new_state = (125, 200, 0)
    new_node = Node(motion_model, new_state, None)
    planner.process_new_node(new_node, test_rrt_star.result, 0)
    assert (
        new_node in planner.nodes.values()
    ), f"{str(new_node)} not in {[str(n) for n in planner.nodes.values()]}"
    assert (
        new_node.parent == planner.start_node
    ), f"{str(new_node.parent)} != {str(planner.start_node)}"
    assert (
        node_1.parent == planner.start_node
    ), f"{str(node_1.parent)} != {str(planner.start_node)}"  # failing on this line


def test_node_on_existing_path_but_requires_rewire(test_rrt_star):
    # access the test_rrt_star object
    planner = test_rrt_star.planner
    motion_model = test_rrt_star.motion_model
    # create some nodes
    state_1 = (170, 200, 0)
    node_1 = Node(motion_model, state_1, planner.start_node)
    planner.nodes[state_1] = node_1

    new_state = (125, 200, 0)
    new_node = Node(motion_model, new_state, None)
    planner.process_new_node(new_node, test_rrt_star.result, 0)
    assert (
        new_node in planner.nodes.values()
    ), f"{str(new_node)} not in {[str(n) for n in planner.nodes.values()]}"
    assert (
        new_node.parent == planner.start_node
    ), f"{str(new_node.parent)} != {str(planner.start_node)}"
    assert (
        node_1.parent == new_node
    ), f"{str(node_1.parent)} != {str(new_node)}"  # failing on this line


def test_rewire_of_other_neighbor(test_rrt_star):
    # access the test_rrt_star object
    planner = test_rrt_star.planner
    motion_model = test_rrt_star.motion_model
    # create some nodes
    state_1 = (125, 225, 0)
    state_2 = (100, 275, 0)
    state_3 = (150, 300, 0)
    state_4 = (150, 275, 0)
    node_1 = Node(motion_model, state_1, planner.start_node)
    planner.nodes[state_1] = node_1
    node_2 = Node(motion_model, state_2, planner.start_node)
    planner.nodes[state_2] = node_2
    node_3 = Node(motion_model, state_3, node_2)
    planner.nodes[state_3] = node_3
    node_4 = Node(motion_model, state_4, node_3)
    planner.nodes[state_4] = node_4
    # create a new node
    new_state = (125, 250, 0)
    new_node = Node(motion_model, new_state, None)
    planner.process_new_node(new_node, test_rrt_star.result, 0)
    assert (
        new_node in planner.nodes.values()
    ), f"{str(new_node)} not in {[str(n) for n in planner.nodes.values()]}"
    assert new_node.parent == node_1, f"{str(new_node.parent)} != {str(node_1)}"
    assert (
        node_4.parent == new_node
    ), f"{str(node_4.parent)} != {str(new_node)}"  # this is failing
    assert node_3.parent == node_2, f"{str(node_3.parent)} != {str(node_2)}"


# add test case to prevent cycles
# def test_plan(test_rrt_star):
#     pass
