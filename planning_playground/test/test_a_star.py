import pytest

import numpy as np

import planning_playground.map.import_map as import_map
import planning_playground.motion_models.holonomic_model as holonomic_model
from planning_playground.search.a_star_planner import AStarPlanner
from planning_playground.search.types import PathPlanningResult
from planning_playground.search.types import Node

"""
expand lowest cost node
    - test that the lowest cost node is expanded
    - test that the lowest cost node is removed from the open list
    - test that the lowest cost node is added to the closed list
    - test that the lowest cost node neighbors are added to the open list
    - test that the neighbors are added to the open list with the correct cost

test that open list is sorted by total cost

get_neighboring_nodes test
    - make sure that the cost is correctly calculated
    - make sure that everything is calculated correctly after a rewire

rewire where new node is better path
    - new node is in the closed list, but has a lower cost
    - new node replaces old node in the closed list
    - all nodes are updated and the open list is updated with the new costs

rewire where new node is not a better path
    - new node is in the closed list, but has a higher cost  - not added to the open list
    - new node does not replace old node in the closed list


write tests for is in closed list
    - test that a node is in the closed list
    - test that a node is not in the closed list
    both should use a helper function from the motion model

write tests for termination
    - test that the planner terminates when the goal is reached - this should use a helper method
    - test that the planner terminates when the open list is empty
"""
GRID_SIZE = 10
MAP_SIZE = 1000
POSITION_DISCRETIZATION = MAP_SIZE // GRID_SIZE


class AstarPlannerFixture:
    def __init__(self):
        # setup the planner and all required objects here
        self.map = import_map.Map2d(
            "planning_playground/map/map_empty.png", grid_size=GRID_SIZE
        )
        self.motion_model = holonomic_model.HolonomicModel(
            (1.0, 1.0), 1, self.map, is_discrete=True
        )
        self.planner = AStarPlanner(self.map, self.motion_model)
        self.start = (POSITION_DISCRETIZATION, 250, 0)
        self.goal = (600, 400, 0)
        self.planner.start_node = Node(self.motion_model, self.start, None)
        self.goal_node = Node(self.motion_model, self.goal, None)
        self.planner.goal_node = Node(self.motion_model, self.goal, None)
        self.planner.start_up(self.start, self.goal)
        self.result = PathPlanningResult()

    def get_closed_node(self, state):
        return self.planner.closed_dict[self.motion_model.get_discretized_state(state)]

    def get_open_node(self, state):
        print("state", state)
        print("open list", list(node.get_state() for node in self.planner.open_list))
        for node in self.planner.open_list:
            if node.get_state() == state:
                return node


@pytest.fixture
def test_a_star():
    return AstarPlannerFixture()


def create_graph(motion_model, planner):
    state_1 = tuple(
        sum(x)
        for x in zip(planner.start_node.get_state(), (POSITION_DISCRETIZATION, 0, 0))
    )
    state_2 = tuple(
        sum(x)
        for x in zip(planner.start_node.get_state(), (0, POSITION_DISCRETIZATION, 0))
    )
    state_3 = tuple(
        sum(x)
        for x in zip(planner.start_node.get_state(), (0, -POSITION_DISCRETIZATION, 0))
    )
    state_1_1 = tuple(sum(x) for x in zip(state_1, (POSITION_DISCRETIZATION, 0, 0)))
    state_1_2 = tuple(sum(x) for x in zip(state_1, (0, POSITION_DISCRETIZATION, 0)))
    # this should be the lowest total cost
    state_1_3 = tuple(sum(x) for x in zip(state_1, (0, -POSITION_DISCRETIZATION, 0)))

    state_2_1 = tuple(sum(x) for x in zip(state_2, (-POSITION_DISCRETIZATION, 0, 0)))
    state_2_2 = tuple(sum(x) for x in zip(state_2_1, (0, POSITION_DISCRETIZATION, 0)))
    state_2_3 = tuple(sum(x) for x in zip(state_2_2, (0, POSITION_DISCRETIZATION, 0)))
    state_2_4 = tuple(sum(x) for x in zip(state_2_3, (POSITION_DISCRETIZATION, 0, 0)))
    state_2_5 = tuple(sum(x) for x in zip(state_2_4, (POSITION_DISCRETIZATION, 0, 0)))
    state_2_6 = tuple(sum(x) for x in zip(state_2_5, (0, -POSITION_DISCRETIZATION, 0)))
    state_2_7 = tuple(sum(x) for x in zip(state_2_6, (POSITION_DISCRETIZATION, 0, 0)))

    node_1 = Node(motion_model, state_1, planner.start_node)
    planner.add_node_to_closed_dict(node_1)
    node_2 = Node(motion_model, state_2, planner.start_node)
    planner.add_node_to_closed_dict(node_2)

    node_2_1 = Node(motion_model, state_2_1, node_2)
    planner.add_node_to_closed_dict(node_2_1)
    node_2_2 = Node(motion_model, state_2_2, node_2_1)
    planner.add_node_to_closed_dict(node_2_2)
    node_2_3 = Node(motion_model, state_2_3, node_2_2)
    planner.add_node_to_closed_dict(node_2_3)
    node_2_4 = Node(motion_model, state_2_4, node_2_3)
    planner.add_node_to_closed_dict(node_2_4)
    node_2_5 = Node(motion_model, state_2_5, node_2_4)
    planner.add_node_to_closed_dict(node_2_5)
    node_2_6 = Node(motion_model, state_2_6, node_2_5)
    planner.add_node_to_closed_dict(node_2_6)
    print("node_2_6", state_2_6, node_2_6.get_total_cost())

    node_3 = Node(motion_model, state_3, planner.start_node)
    planner.add_node_to_open_list(node_3)
    node_1_1 = Node(motion_model, state_1_1, node_1)
    planner.add_node_to_closed_dict(node_1_1)
    node_1_2 = Node(motion_model, state_1_2, node_1)
    print("node_1_2", state_1_2, node_1_2.get_total_cost())
    planner.add_node_to_open_list(node_1_2)
    node_1_3 = Node(motion_model, state_1_3, node_1)
    planner.add_node_to_open_list(node_1_3)
    node_2_7 = Node(motion_model, state_2_7, node_2_6)
    planner.add_node_to_open_list(node_2_7)
    print("node_2_7", state_2_7, node_2_7.get_total_cost())
    print("planer dict keys", planner.closed_dict.keys())
    assert len(planner.open_list) == 4
    assert node_2_6.get_cost() > node_1_2.get_cost()
    assert node_2_6.get_heuristic() == node_1_2.get_heuristic()
    assert node_2_6.get_total_cost() > node_1_2.get_total_cost()
    assert (node_2_7.get_total_cost() - node_2_6.get_total_cost()) < 50
    assert node_2_7.parent == node_2_6


def test_expand_lowest_cost(test_a_star):
    create_graph(test_a_star.motion_model, test_a_star.planner)

    expanded_node = test_a_star.planner.expand(test_a_star.goal_node)
    assert expanded_node.get_state() == (
        2 * POSITION_DISCRETIZATION,
        3.5 * POSITION_DISCRETIZATION,
        0,
    )
    assert test_a_star.motion_model.are_states_equal(
        expanded_node.get_state(),
        (2 * POSITION_DISCRETIZATION, 3.5 * POSITION_DISCRETIZATION, 0),
    )
    assert expanded_node in test_a_star.planner.closed_dict.values()
    assert (
        test_a_star.motion_model.get_discretized_state(expanded_node.get_state())
        in test_a_star.planner.closed_dict.keys()
    )


def test_open_list_sorted(test_a_star):
    create_graph(test_a_star.motion_model, test_a_star.planner)
    assert test_a_star.planner.open_list[0].get_state() == (
        2 * POSITION_DISCRETIZATION,
        3.5 * POSITION_DISCRETIZATION,
        0,
    )
    assert sorted(test_a_star.planner.open_list) == test_a_star.planner.open_list


def test_rewire_new_node_better_path(test_a_star):
    # make sure that both the parent and child nodes are updated
    create_graph(test_a_star.motion_model, test_a_star.planner)
    lowest_cost_state = tuple(
        sum(x)
        for x in zip(
            test_a_star.start,
            (
                POSITION_DISCRETIZATION,
                POSITION_DISCRETIZATION,
                0,
            ),
        )
    )
    lowest_cost_node = test_a_star.planner.open_list[0]
    assert lowest_cost_node.get_state() == lowest_cost_state

    state_2_6 = tuple(
        sum(x)
        for x in zip(
            test_a_star.start,
            (
                POSITION_DISCRETIZATION,
                2 * POSITION_DISCRETIZATION,
                0,
            ),
        )
    )
    print("state_2_6", state_2_6)
    print("disc state", test_a_star.motion_model.get_discretized_state(state_2_6))
    node_2_6 = test_a_star.get_closed_node(state_2_6)

    state_2_7 = tuple(
        sum(x)
        for x in zip(
            state_2_6,
            (
                POSITION_DISCRETIZATION,
                0,
                0,
            ),
        )
    )
    node_2_7 = test_a_star.get_open_node(state_2_7)

    assert node_2_6.get_total_cost() > lowest_cost_node.get_total_cost()
    assert node_2_6.parent != lowest_cost_node
    assert node_2_6 not in lowest_cost_node.children
    assert node_2_7.parent == node_2_6
    assert node_2_7 in node_2_6.children
    rewired = test_a_star.planner.rewire(lowest_cost_node, node_2_6)
    assert rewired

    assert node_2_6.parent == lowest_cost_node
    assert node_2_6 in lowest_cost_node.children
    assert node_2_7.parent == node_2_6
    assert node_2_7 in node_2_6.children
    assert node_2_6.get_cost() == lowest_cost_node.get_cost() + POSITION_DISCRETIZATION


def test_rewire_node_not_in_closed(test_a_star):
    # make sure that both the parent and child nodes are updated
    create_graph(test_a_star.motion_model, test_a_star.planner)
    lowest_cost_state = tuple(
        sum(x)
        for x in zip(
            test_a_star.start,
            (
                POSITION_DISCRETIZATION,
                POSITION_DISCRETIZATION,
                0,
            ),
        )
    )
    lowest_cost_node = test_a_star.planner.open_list[0]
    assert lowest_cost_node.get_state() == lowest_cost_state

    state_2_6 = tuple(
        sum(x)
        for x in zip(
            test_a_star.start,
            (
                POSITION_DISCRETIZATION,
                2 * POSITION_DISCRETIZATION,
                0,
            ),
        )
    )
    print("state_2_6", state_2_6)
    node_2_6 = test_a_star.get_closed_node(state_2_6)
    test_a_star.planner.closed_dict.pop(
        test_a_star.motion_model.get_discretized_state(state_2_6)
    )

    state_2_7 = tuple(
        sum(x)
        for x in zip(
            state_2_6,
            (
                POSITION_DISCRETIZATION,
                0,
                0,
            ),
        )
    )
    node_2_7 = test_a_star.get_open_node(state_2_7)
    assert node_2_6.get_total_cost() > lowest_cost_node.get_total_cost()
    assert node_2_6.parent != lowest_cost_node
    assert node_2_6 not in lowest_cost_node.children
    assert node_2_7.parent == node_2_6
    assert node_2_7 in node_2_6.children
    assert not test_a_star.planner.rewire(lowest_cost_node, node_2_6)

    assert node_2_6.get_total_cost() > lowest_cost_node.get_total_cost()
    assert node_2_6.parent != lowest_cost_node
    assert node_2_6 not in lowest_cost_node.children
    assert node_2_7.parent == node_2_6
    assert node_2_7 in node_2_6.children


def test_rewire_new_node_not_better_path(test_a_star):
    # make sure that both the parent and child nodes are updated
    create_graph(test_a_star.motion_model, test_a_star.planner)
    lowest_cost_state = tuple(
        sum(x)
        for x in zip(
            test_a_star.start,
            (
                POSITION_DISCRETIZATION,
                POSITION_DISCRETIZATION,
                0,
            ),
        )
    )
    lowest_cost_node = test_a_star.planner.open_list[0]
    assert lowest_cost_node.get_state() == lowest_cost_state

    state_2_6 = tuple(
        sum(x)
        for x in zip(
            test_a_star.start,
            (
                POSITION_DISCRETIZATION,
                2 * POSITION_DISCRETIZATION,
                0,
            ),
        )
    )
    print("state_2_6", state_2_6)
    node_2_6 = test_a_star.get_closed_node(state_2_6)

    state_2_7 = tuple(
        sum(x)
        for x in zip(
            state_2_6,
            (
                POSITION_DISCRETIZATION,
                0,
                0,
            ),
        )
    )
    node_2_7 = test_a_star.get_open_node(state_2_7)
    node_2_6.cost = 0

    assert node_2_6.get_total_cost() < lowest_cost_node.get_total_cost()
    assert node_2_6.parent != lowest_cost_node
    assert node_2_6 not in lowest_cost_node.children
    assert node_2_7.parent == node_2_6
    assert node_2_7 in node_2_6.children
    assert not test_a_star.planner.rewire(node_2_6, lowest_cost_node)

    assert node_2_6.get_total_cost() < lowest_cost_node.get_total_cost()
    assert node_2_6.parent != lowest_cost_node
    assert node_2_6 not in lowest_cost_node.children
    assert node_2_7.parent == node_2_6
    assert node_2_7 in node_2_6.children


def test_is_in_closed_dict(test_a_star):
    create_graph(test_a_star.motion_model, test_a_star.planner)
    state_2_6 = tuple(
        sum(x)
        for x in zip(
            test_a_star.start,
            (
                POSITION_DISCRETIZATION,
                3 * POSITION_DISCRETIZATION,
                0,
            ),
        )
    )
    assert test_a_star.planner.is_in_closed_dict(state_2_6)
    state_2_7 = tuple(
        sum(x)
        for x in zip(
            state_2_6,
            (
                POSITION_DISCRETIZATION,
                0,
                0,
            ),
        )
    )
    assert not test_a_star.planner.is_in_closed_dict(state_2_7)


# this will only work for the holonomic model
def test_termination(test_a_star):
    goal_state = test_a_star.goal
    disc = test_a_star.motion_model.get_state_discretization()

    for i in range(-disc[0], 2 * disc[0]):
        state = (goal_state[0] + i, goal_state[1], 0)
        if i >= 0 and i < disc[0]:
            assert test_a_star.planner.within_termination_bounds(
                state, goal_state
            ), f"state: {state}, goal: {goal_state}, disc_state: {test_a_star.motion_model.get_discretized_state(state)}, disc_goal: {test_a_star.motion_model.get_discretized_state(goal_state)}"
        else:
            assert not test_a_star.planner.within_termination_bounds(
                state, goal_state
            ), f"state: {state}, goal: {goal_state}, disc_state: {test_a_star.motion_model.get_discretized_state(state)}, disc_goal: {test_a_star.motion_model.get_discretized_state(goal_state)}"

    for j in range(-disc[1], 2 * disc[1]):
        state = (goal_state[0], goal_state[1] + j, 0)
        if j >= 0 and j < disc[0]:
            assert test_a_star.planner.within_termination_bounds(
                state, goal_state
            ), f"state: {state}, goal: {goal_state}, disc_state: {test_a_star.motion_model.get_discretized_state(state)}, disc_goal: {test_a_star.motion_model.get_discretized_state(goal_state)}"
        else:
            assert not test_a_star.planner.within_termination_bounds(
                state, goal_state
            ), f"state: {state}, goal: {goal_state}, disc_state: {test_a_star.motion_model.get_discretized_state(state)}, disc_goal: {test_a_star.motion_model.get_discretized_state(goal_state)}"

    for theta in range(360):
        print("disc", disc)
        state = (goal_state[0], goal_state[1], theta)
        if theta <= disc[2] // 2 or theta > 360 + (-disc[2] // 2):
            assert test_a_star.planner.within_termination_bounds(
                state, goal_state
            ), f"state: {state}, goal: {goal_state}, disc_state: {test_a_star.motion_model.get_discretized_state(state)}, disc_goal: {test_a_star.motion_model.get_discretized_state(goal_state)}"
        else:
            assert not test_a_star.planner.within_termination_bounds(
                state, goal_state
            ), f"state: {state}, goal: {goal_state}, disc_state: {test_a_star.motion_model.get_discretized_state(state)}, disc_goal: {test_a_star.motion_model.get_discretized_state(goal_state)}"
    pass
