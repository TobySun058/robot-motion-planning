"""Grid-based shortest-path planning with a label-correcting algorithm."""

from collections import deque
from dataclasses import dataclass
from typing import Iterable

GridNode = tuple[int, int]


@dataclass(frozen=True)
class GridPlan:
    path: list[GridNode]
    cost: float


def label_correcting_path(
    rows: int,
    cols: int,
    obstacles: Iterable[GridNode],
    start: GridNode,
    goal: GridNode,
) -> GridPlan:
    """Compute a shortest 4-connected path on a binary occupancy grid."""
    blocked = set(obstacles)
    if start in blocked or goal in blocked:
        return GridPlan([], float("inf"))

    costs = {(r, c): float("inf") for r in range(rows) for c in range(cols)}
    costs[start] = 0.0
    parent: dict[GridNode, GridNode] = {}
    queue = deque([start])
    in_queue = {start}
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while queue:
        node = queue.popleft()
        in_queue.discard(node)

        for dr, dc in moves:
            nxt = (node[0] + dr, node[1] + dc)
            if not (0 <= nxt[0] < rows and 0 <= nxt[1] < cols):
                continue
            if nxt in blocked:
                continue

            new_cost = costs[node] + 1.0
            if new_cost < costs[nxt]:
                costs[nxt] = new_cost
                parent[nxt] = node
                if nxt not in in_queue:
                    queue.append(nxt)
                    in_queue.add(nxt)

    if costs[goal] == float("inf"):
        return GridPlan([], float("inf"))

    path = [goal]
    current = goal
    while current != start:
        current = parent[current]
        path.append(current)
    path.reverse()
    return GridPlan(path, costs[goal])


def demo() -> None:
    obstacle_cells = {
        (1, 1), (1, 2), (2, 1), (2, 2),
        (1, 7), (1, 8), (2, 7), (2, 8),
        (3, 5), (4, 5), (3, 6), (4, 6),
        (6, 2), (6, 3), (6, 4), (7, 3), (7, 4),
        (7, 1), (7, 2), (8, 1),
    }
    result = label_correcting_path(
        rows=10,
        cols=10,
        obstacles=obstacle_cells,
        start=(0, 0),
        goal=(9, 9),
    )
    print(f"Path cost: {result.cost}")
    print(f"Nodes in path: {len(result.path)}")
    print(result.path)


if __name__ == "__main__":
    demo()
