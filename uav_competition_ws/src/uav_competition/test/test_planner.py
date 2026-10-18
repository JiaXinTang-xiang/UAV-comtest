"""Regression: a direct area 3 -> 4 leg crosses the map's middle wall."""
from pathlib import Path
import pytest
from uav_competition.path_planner import CoursePlanner

WORLD = Path(__file__).parents[1] / 'worlds/competition_course.world'


def test_wall_detour_and_clearance():
    planner = CoursePlanner(WORLD, .7)
    assert not planner.visible((5, 5), (2, 3))
    path = planner.plan((5, 5), (2, 3))
    assert len(path) > 2
    assert all(planner.visible(a, b) for a, b in zip(path, path[1:]))
    assert min(p[0] for p in path) < 2


def test_wall_waypoint_is_rejected():
    with pytest.raises(ValueError):
        CoursePlanner(WORLD).plan((5, 9), (8, 7))


def test_closed_corridor_has_no_path():
    planner = CoursePlanner(WORLD)
    planner.boxes.append((0, 10, 4.8, 5.2))
    with pytest.raises(ValueError):
        planner.plan((5, 9), (4, 1))


def test_unknown_obstacle_forces_a_new_route():
    planner = CoursePlanner(WORLD, .7)
    assert planner.visible((5, 5), (2, 5))
    # A box face seen only through lidar endpoints, absent from the world map.
    planner.dynamic = [(3.7, y) for y in (4.8, 4.9, 5, 5.1, 5.2)]
    assert not planner.visible((5, 5), (2, 5))
    path = planner.plan((5, 5), (2, 5))
    assert len(path) > 2
    assert all(planner.visible(a, b) for a, b in zip(path, path[1:]))


def test_left_outer_passage_rejected_with_current_vehicle_margin():
    planner = CoursePlanner(WORLD, .7)
    with pytest.raises(ValueError):
        planner.plan((7.5, 5), (9.4, 3))
