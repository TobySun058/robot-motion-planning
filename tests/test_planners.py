import math

from planners.geometric_rrt import point_to_rotated_square_distance, steer
from planners.grid_search import label_correcting_path
from planners.kinodynamic_rrt import propagate


def test_grid_search_finds_shortest_open_grid_path():
    result = label_correcting_path(
        rows=3,
        cols=3,
        obstacles={(1, 1)},
        start=(0, 0),
        goal=(2, 2),
    )
    assert result.path[0] == (0, 0)
    assert result.path[-1] == (2, 2)
    assert result.cost == 4.0


def test_steer_respects_step_size():
    point = steer((0.0, 0.0), (3.0, 4.0), 1.0)
    assert math.isclose(math.dist((0.0, 0.0), point), 1.0)


def test_square_distance_is_zero_inside_obstacle():
    assert point_to_rotated_square_distance((0.0, 0.0), (0.0, 0.0), 0.0) == 0.0


def test_differential_drive_straight_line():
    final_state, trajectory = propagate(
        state=(0.0, 0.0, 0.0),
        control=(0.25, 0.0),
        duration=2.0,
    )
    assert math.isclose(final_state[0], 0.5, abs_tol=1e-9)
    assert math.isclose(final_state[1], 0.0, abs_tol=1e-9)
    assert len(trajectory) == 20
