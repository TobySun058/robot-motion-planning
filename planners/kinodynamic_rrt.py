"""Kinodynamic RRT for differential-drive motion constraints."""

from dataclasses import dataclass
import math
import random

from planners.geometric_rrt import Obstacle, edge_is_free


State = tuple[float, float, float]
Control = tuple[float, float]


@dataclass
class KinodynamicResult:
    states: list[State]
    controls: list[Control]
    iterations: int


def sinc(value: float) -> float:
    return 1.0 if abs(value) < 1e-12 else math.sin(value) / value


def propagate(
    state: State,
    control: Control,
    duration: float,
    samples: int = 20,
) -> tuple[State, list[State]]:
    """Propagate exact constant-control differential-drive motion."""
    x, y, theta = state
    v, omega = control

    def state_at(t: float) -> State:
        dx = v * t * sinc(0.5 * omega * t) * math.cos(theta + 0.5 * omega * t)
        dy = v * t * sinc(0.5 * omega * t) * math.sin(theta + 0.5 * omega * t)
        heading = (theta + omega * t + math.pi) % (2 * math.pi) - math.pi
        return (x + dx, y + dy, heading)

    trajectory = [state_at(duration * i / samples) for i in range(1, samples + 1)]
    return trajectory[-1], trajectory


def default_controls() -> list[Control]:
    angular = [-1.82, -1.46, -1.09, -0.73, -0.36, 0.0, 0.36, 0.73, 1.09, 1.46, 1.82]
    return [(v, w) for v in (-0.25, 0.25) for w in angular] + [(0.0, 0.0)]


def trajectory_is_free(
    start: State,
    trajectory: list[State],
    obstacles: list[Obstacle],
    robot_radius: float,
    bounds: tuple[float, float, float, float],
) -> bool:
    previous = (start[0], start[1])
    for state in trajectory:
        point = (state[0], state[1])
        if not edge_is_free(previous, point, obstacles, robot_radius, bounds, samples=5):
            return False
        previous = point
    return True


def plan_kinodynamic_rrt(
    start: State,
    goal_region: tuple[tuple[float, float], tuple[float, float]],
    obstacles: list[Obstacle],
    *,
    controls: list[Control] | None = None,
    duration: float = 0.3,
    robot_radius: float = 0.5,
    bounds: tuple[float, float, float, float] = (0.0, 10.0, 0.0, 10.0),
    goal_bias: float = 0.3,
    max_iterations: int = 10_000,
    seed: int | None = None,
) -> KinodynamicResult:
    rng = random.Random(seed)
    controls = controls or default_controls()
    goal_min, goal_max = goal_region

    states = [start]
    parents = [-1]
    parent_controls: list[Control | None] = [None]

    def in_goal(state: State) -> bool:
        return (
            goal_min[0] <= state[0] <= goal_max[0]
            and goal_min[1] <= state[1] <= goal_max[1]
        )

    for iteration in range(1, max_iterations + 1):
        sample: State
        if rng.random() < goal_bias:
            sample = (
                rng.uniform(goal_min[0], goal_max[0]),
                rng.uniform(goal_min[1], goal_max[1]),
                rng.uniform(-math.pi, math.pi),
            )
        else:
            sample = (
                rng.uniform(bounds[0], bounds[1]),
                rng.uniform(bounds[2], bounds[3]),
                rng.uniform(-math.pi, math.pi),
            )

        nearest_idx = min(
            range(len(states)),
            key=lambda i: math.hypot(states[i][0] - sample[0], states[i][1] - sample[1]),
        )
        nearest = states[nearest_idx]

        best_state = None
        best_trajectory = None
        best_control = None
        best_cost = float("inf")

        for control in controls:
            candidate, trajectory = propagate(nearest, control, duration)
            cost = math.hypot(candidate[0] - sample[0], candidate[1] - sample[1])
            if cost < best_cost:
                best_cost = cost
                best_state = candidate
                best_trajectory = trajectory
                best_control = control

        assert best_state is not None and best_trajectory is not None and best_control is not None

        if not trajectory_is_free(nearest, best_trajectory, obstacles, robot_radius, bounds):
            continue

        states.append(best_state)
        parents.append(nearest_idx)
        parent_controls.append(best_control)

        if in_goal(best_state):
            path_states: list[State] = []
            path_controls: list[Control] = []
            node = len(states) - 1
            while node != -1:
                path_states.append(states[node])
                control = parent_controls[node]
                if control is not None:
                    path_controls.append(control)
                node = parents[node]
            path_states.reverse()
            path_controls.reverse()
            return KinodynamicResult(path_states, path_controls, iteration)

    return KinodynamicResult([], [], max_iterations)


if __name__ == "__main__":
    from planners.geometric_rrt import default_obstacles

    result = plan_kinodynamic_rrt(
        start=(0.5, 0.5, 0.0),
        goal_region=((9.0, 9.0), (10.0, 10.0)),
        obstacles=default_obstacles(),
        seed=7,
    )
    print(f"Found path: {bool(result.states)}")
    print(f"Iterations: {result.iterations}")
    print(f"Controls: {len(result.controls)}")
