"""Regression checks for frontier selection, recovery, and conservative planning."""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
from scipy.interpolate import splrep

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_lib.navigation import TrajectoryPlan
from asl_tb3_msgs.msg import TurtleBotState
from frontier_exploration import FrontierExploration, explore
from navigator import AStar, KnownFreeOccupancy, Navigator
from std_msgs.msg import Bool


def grid(probs, resolution=0.1, window=1):
    return StochOccupancyGrid2D(
        resolution, np.array(probs.shape[::-1]), np.zeros(2), window, probs)


def partial_map():
    probs = np.zeros((40, 40), dtype=int)
    probs[:, 25:] = -1
    return grid(probs)


def explorer(occupancy=None):
    # Exercise callbacks without DDS discovery or a running robot.
    node = SimpleNamespace(
        occupancy=partial_map() if occupancy is None else occupancy,
        state=TurtleBotState(x=1.0, y=2.0, theta=0.0),
        active_goal=None, failed_goals=[],
        goal_pub=Mock(), get_logger=Mock(), now_seconds=lambda: 100.0,
        count_publishers=lambda topic: 1,
    )
    parameters = {
        "frontier_window_size": 11, "min_unknown": 0.25, "min_free": 0.35,
        "occupied_thresh": 50, "min_goal_distance": 0.4,
        "failed_goal_radius": 0.4, "failed_goal_cooldown": 60.0,
    }
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node.goal_pub.get_subscription_count.return_value = 1
    node.send_goal = lambda goal: FrontierExploration.send_goal(node, goal)
    return node


def test_frontiers_preserve_ros_values_and_coordinates():
    occupancy = partial_map()
    states = explore(occupancy)
    assert len(states) > 0
    assert np.any(states[:, 0] < 2.5)
    assert np.any(states[:, 0] >= 2.5)  # P1 allows unknown candidate centers.
    assert set(np.unique(occupancy.probs)) == {-1, 0}


def test_nearest_acceptable_goal_is_chosen_with_stable_ties():
    node = explorer()
    candidates = np.array([[1.9, 2.0], [1.6, 2.0], [1.0, 2.6]])
    with patch("frontier_exploration.explore", return_value=candidates):
        FrontierExploration.select_goal(node)
    np.testing.assert_allclose(node.active_goal, [1.6, 2.0])


def test_too_close_unknown_and_occupied_candidates_are_skipped():
    node = explorer()
    node.occupancy.probs[20, 15] = -1
    node.occupancy.probs[20, 16] = 100
    candidates = np.array([[1.1, 2.0], [1.5, 2.0], [1.6, 2.0], [1.8, 2.0]])
    with patch("frontier_exploration.explore", return_value=candidates):
        FrontierExploration.select_goal(node)
    np.testing.assert_allclose(node.active_goal, [1.8, 2.0])


def test_goal_near_obstacle_is_skipped():
    probs = np.zeros((40, 40), dtype=int)
    probs[20, 16] = 100
    node = explorer(grid(probs, window=3))
    candidates = np.array([[1.5, 2.0], [1.9, 2.0]])
    with patch("frontier_exploration.explore", return_value=candidates):
        FrontierExploration.select_goal(node)
    np.testing.assert_allclose(node.active_goal, [1.9, 2.0])


def test_only_one_goal_is_in_flight():
    node = explorer()
    FrontierExploration.select_goal(node)
    assert node.active_goal is not None
    assert node.goal_pub.publish.call_count == 1
    assert node.occupancy.is_free(node.active_goal)
    FrontierExploration.select_goal(node)
    assert node.goal_pub.publish.call_count == 1


@pytest.mark.parametrize("missing", ["state", "occupancy", "navigator", "feedback"])
def test_waits_for_inputs_and_navigator(missing):
    node = explorer()
    if missing == "navigator":
        node.goal_pub.get_subscription_count.return_value = 0
    elif missing == "feedback":
        node.count_publishers = lambda topic: 0
    else:
        setattr(node, missing, None)
    FrontierExploration.select_goal(node)
    node.goal_pub.publish.assert_not_called()


def test_failure_chooses_another_region_and_cooldown_expires():
    node = explorer()
    FrontierExploration.select_goal(node)
    first = node.active_goal.copy()
    FrontierExploration.result_callback(node, Bool(data=False))
    assert node.active_goal is None and len(node.failed_goals) == 1
    FrontierExploration.select_goal(node)
    assert np.linalg.norm(node.active_goal - first) > 0.4
    node.active_goal = None
    node.now_seconds = lambda: 200.0
    FrontierExploration.select_goal(node)
    assert not node.failed_goals
    np.testing.assert_array_equal(node.active_goal, first)


def test_success_advances_using_updated_pose():
    node = explorer()
    FrontierExploration.select_goal(node)
    first = node.active_goal.copy()
    FrontierExploration.result_callback(node, Bool(data=True))
    assert not node.failed_goals
    node.state = TurtleBotState(x=float(first[0]), y=float(first[1]), theta=0.0)
    FrontierExploration.select_goal(node)
    assert np.linalg.norm(node.active_goal - first) >= 0.4


def test_waits_without_frontiers_and_resumes_when_the_map_changes():
    node = explorer(grid(np.zeros((40, 40), dtype=int)))
    for _ in range(5):
        FrontierExploration.select_goal(node)
    node.goal_pub.publish.assert_not_called()
    node.occupancy = partial_map()
    FrontierExploration.select_goal(node)
    assert node.active_goal is not None


def test_waits_when_all_goals_are_on_cooldown_and_retries_after_expiry():
    node = explorer()
    goal = np.array([1.8, 2.0])
    node.failed_goals = [(goal, 160.0)]
    with patch("frontier_exploration.explore", return_value=np.array([goal])):
        FrontierExploration.select_goal(node)
        node.goal_pub.publish.assert_not_called()
        node.now_seconds = lambda: 161.0
        FrontierExploration.select_goal(node)
    np.testing.assert_array_equal(node.active_goal, goal)


def test_disconnected_goal_is_left_to_navigator_and_failure_selects_another():
    node = explorer()
    node.occupancy.probs[:, 15] = 100  # A wall separates robot and nearest goal.
    candidates = np.array([[1.8, 2.0], [1.0, 3.0]])
    with patch("frontier_exploration.explore", return_value=candidates):
        FrontierExploration.select_goal(node)
        np.testing.assert_array_equal(node.active_goal, candidates[0])
        FrontierExploration.result_callback(node, Bool(data=False))
        FrontierExploration.select_goal(node)
    np.testing.assert_array_equal(node.active_goal, candidates[1])


def test_conservative_planner_rejects_unknown_shortcuts():
    probs = np.zeros((20, 20), dtype=int)
    probs[:, 10] = -1
    occupancy = KnownFreeOccupancy(grid(probs))
    planner = AStar([0, 0], [1.9, 1.9], [0.5, 1.0], [1.5, 1.0], occupancy, 0.1)
    assert not planner.solve()
    assert not occupancy.is_free(np.array([-0.1, 1.0]))


def test_conservative_planner_cannot_cut_diagonal_corners():
    probs = np.full((4, 4), 100)
    probs[1, 1] = probs[2, 2] = 0
    planner = AStar([0, 0], [0.3, 0.3], [0.1, 0.1], [0.2, 0.2],
                    KnownFreeOccupancy(grid(probs)), 0.1)
    assert not planner.solve()


def test_spline_collision_check_includes_between_path_vertices():
    times = np.arange(4.0)
    path = np.column_stack((np.array([0.2, 0.7, 1.2, 1.7]), np.ones(4)))
    plan = TrajectoryPlan(path, splrep(times, path[:, 0], s=0),
                          splrep(times, path[:, 1], s=0), 3.0)
    probs = np.zeros((20, 20), dtype=int)
    assert Navigator.path_is_safe(plan, KnownFreeOccupancy(grid(probs)))
    probs[10, 10] = 100  # No original path vertex is on the obstacle.
    assert not Navigator.path_is_safe(plan, KnownFreeOccupancy(grid(probs)))
