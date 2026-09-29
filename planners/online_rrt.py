"""Optimistic occupancy-grid planning with RRT and replanning support."""

from dataclasses import dataclass
import math
import random
from typing import Iterable

import numpy as np


Point = tuple[float, float]


@dataclass
class OccupancyMap:
    width: int
    height: int
    resolution: float

    def __post_init__(self) -> None:
        self.cells = -np.ones((self.height, self.width), dtype=np.int8)

    def world_to_grid(self, point: Point) -> tuple[int, int]:
        x, y = point
        gx = min(self.width - 1, max(0, int(x / self.resolution)))
        gy = min(self.height - 1, max(0, int(y / self.resolution)))
        return gx, gy

    def mark_free(self, point: Point) -> None:
        gx, gy = self.world_to_grid(point)
        if self.cells[gy, gx] == -1:
            self.cells[gy, gx] = 0

    def mark_occupied(self, point: Point) -> None:
        gx, gy = self.world_to_grid(point)
        self.cells[gy, gx] = 1

    def is_traversable(self, point: Point) -> bool:
        gx, gy = self.world_to_grid(point)
        return self.cells[gy, gx] != 1


def reveal_sensor_measurements(
    occupancy: OccupancyMap,
    robot: Point,
    occupied_points: Iterable[Point],
    sensor_range: float,
) -> None:
    """Reveal nearby occupied cells; unknown space remains optimistic/free for planning."""
    for point in occupied_points:
        if math.dist(robot, point) <= sensor_range:
            occupancy.mark_occupied(point)


def edge_is_traversable(
    occupancy: OccupancyMap,
    a: Point,
    b: Point,
    samples: int = 30,
) -> bool:
    for i in range(samples + 1):
        t = i / samples
        p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        if not occupancy.is_traversable(p):
            return False
    return True


def optimistic_rrt(
    occupancy: OccupancyMap,
    start: Point,
    goal: Point,
    *,
    step_size: float = 0.25,
    goal_tolerance: float = 0.35,
    max_iterations: int = 10_000,
    goal_bias: float = 0.3,
    seed: int | None = None,
) -> list[Point]:
    rng = random.Random(seed)
    bounds = (
        0.0,
        occupancy.width * occupancy.resolution,
        0.0,
        occupancy.height * occupancy.resolution,
    )
    vertices = [start]
    parents = [-1]

    for _ in range(max_iterations):
        sample = goal if rng.random() < goal_bias else (
            rng.uniform(bounds[0], bounds[1]),
            rng.uniform(bounds[2], bounds[3]),
        )

        nearest_idx = min(
            range(len(vertices)),
            key=lambda i: math.dist(vertices[i], sample),
        )
        nearest = vertices[nearest_idx]
        distance = math.dist(nearest, sample)
        if distance == 0.0:
            continue
        scale = min(step_size / distance, 1.0)
        new_point = (
            nearest[0] + scale * (sample[0] - nearest[0]),
            nearest[1] + scale * (sample[1] - nearest[1]),
        )

        if not edge_is_traversable(occupancy, nearest, new_point):
            continue

        vertices.append(new_point)
        parents.append(nearest_idx)

        if math.dist(new_point, goal) <= goal_tolerance:
            path = [goal, new_point]
            node = parents[-1]
            while node != -1:
                path.append(vertices[node])
                node = parents[node]
            return list(reversed(path))

    return []


def path_is_valid(occupancy: OccupancyMap, path: list[Point]) -> bool:
    return all(
        edge_is_traversable(occupancy, a, b)
        for a, b in zip(path, path[1:])
    )


def demo() -> None:
    occupancy = OccupancyMap(width=40, height=40, resolution=0.25)
    path = optimistic_rrt(
        occupancy,
        start=(0.5, 0.5),
        goal=(9.25, 9.25),
        seed=3,
    )
    print(f"Initial optimistic path has {len(path)} waypoints.")


if __name__ == "__main__":
    demo()
