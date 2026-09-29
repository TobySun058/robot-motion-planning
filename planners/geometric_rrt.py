"""Geometric RRT for a disk robot in a 2-D workspace."""

from dataclasses import dataclass
import math
import random


Point = tuple[float, float]
Obstacle = tuple[Point, float]  # center, rotation angle in radians


@dataclass
class RRTResult:
    path: list[Point]
    iterations: int


def point_to_rotated_square_distance(
    point: Point,
    center: Point,
    angle: float,
    half_width: float = 0.5,
) -> float:
    """Euclidean distance from a point to a rotated axis-aligned square."""
    px, py = point
    ox, oy = center
    c, s = math.cos(angle), math.sin(angle)
    dx, dy = px - ox, py - oy

    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    outside_x = max(abs(local_x) - half_width, 0.0)
    outside_y = max(abs(local_y) - half_width, 0.0)
    return math.hypot(outside_x, outside_y)


def edge_is_free(
    a: Point,
    b: Point,
    obstacles: list[Obstacle],
    robot_radius: float,
    bounds: tuple[float, float, float, float],
    samples: int = 40,
) -> bool:
    min_x, max_x, min_y, max_y = bounds
    for i in range(samples + 1):
        t = i / samples
        p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))

        if not (
            min_x + robot_radius <= p[0] <= max_x - robot_radius
            and min_y + robot_radius <= p[1] <= max_y - robot_radius
        ):
            return False

        for center, angle in obstacles:
            if point_to_rotated_square_distance(p, center, angle) < robot_radius:
                return False
    return True


def steer(a: Point, b: Point, step_size: float) -> Point:
    distance = math.dist(a, b)
    if distance <= step_size:
        return b
    scale = step_size / distance
    return (a[0] + scale * (b[0] - a[0]), a[1] + scale * (b[1] - a[1]))


def plan_rrt(
    start: Point,
    goal_region: tuple[Point, Point],
    obstacles: list[Obstacle],
    *,
    bounds: tuple[float, float, float, float] = (0.0, 10.0, 0.0, 10.0),
    robot_radius: float = 0.5,
    step_size: float = 0.5,
    goal_bias: float = 0.3,
    max_iterations: int = 10_000,
    seed: int | None = None,
) -> RRTResult:
    """Plan a geometric RRT path to an axis-aligned rectangular goal region."""
    rng = random.Random(seed)
    goal_min, goal_max = goal_region
    vertices: list[Point] = [start]
    parents = [-1]

    def in_goal(p: Point) -> bool:
        return goal_min[0] <= p[0] <= goal_max[0] and goal_min[1] <= p[1] <= goal_max[1]

    for iteration in range(1, max_iterations + 1):
        if rng.random() < goal_bias:
            sample = (
                rng.uniform(goal_min[0], goal_max[0]),
                rng.uniform(goal_min[1], goal_max[1]),
            )
        else:
            sample = (
                rng.uniform(bounds[0], bounds[1]),
                rng.uniform(bounds[2], bounds[3]),
            )

        nearest_idx = min(
            range(len(vertices)),
            key=lambda i: math.dist(vertices[i], sample),
        )
        nearest = vertices[nearest_idx]
        new_point = steer(nearest, sample, step_size)

        if not edge_is_free(nearest, new_point, obstacles, robot_radius, bounds):
            continue

        vertices.append(new_point)
        parents.append(nearest_idx)

        if in_goal(new_point):
            path: list[Point] = []
            node = len(vertices) - 1
            while node != -1:
                path.append(vertices[node])
                node = parents[node]
            path.reverse()
            return RRTResult(path=path, iterations=iteration)

    return RRTResult(path=[], iterations=max_iterations)


def default_obstacles() -> list[Obstacle]:
    return [
        ((2.0, 2.0), math.radians(15)),
        ((3.6, 6.8), math.radians(-25)),
        ((6.1, 4.2), math.radians(10)),
        ((1.6, 7.4), math.radians(-35)),
        ((7.9, 2.3), math.radians(30)),
    ]


def demo() -> None:
    result = plan_rrt(
        start=(0.5, 0.5),
        goal_region=((9.0, 9.0), (10.0, 10.0)),
        obstacles=default_obstacles(),
        seed=7,
    )
    print(f"Found path: {bool(result.path)}")
    print(f"Iterations: {result.iterations}")
    print(f"Waypoints: {len(result.path)}")


if __name__ == "__main__":
    demo()
