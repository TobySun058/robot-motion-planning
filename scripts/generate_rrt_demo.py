from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import math
import random

from planners.geometric_rrt import (
    default_obstacles,
    edge_is_free,
    steer,
)

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "assets" / "rrt-replanning-demo.gif"
OUT.parent.mkdir(parents=True, exist_ok=True)

W, H = 900, 620
MARGIN = 52
BOUNDS = (0.0, 10.0, 0.0, 10.0)
START = (0.7, 0.7)
GOAL_REGION = ((9.0, 9.0), (9.5, 9.5))
ROBOT_RADIUS = 0.5
STEP_SIZE = 0.5

BG = (248, 250, 253)
NAVY = (25, 35, 54)
MUTED = (101, 111, 128)
GRID = (226, 231, 238)
TREE = (166, 190, 225)
BLUE = (36, 99, 201)
GREEN = (41, 145, 95)
RED = (206, 67, 75)
OBSTACLE = (88, 101, 121)
OLD_PATH = (176, 181, 190)
WHITE = (255, 255, 255)

REGULAR = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def font(size, bold=False):
    try:
        return ImageFont.truetype(BOLD if bold else REGULAR, size)
    except OSError:
        return ImageFont.load_default()


def trace_rrt(start, goal_region, obstacles, seed, max_iterations=4000):
    rng = random.Random(seed)
    goal_min, goal_max = goal_region
    vertices = [start]
    parents = [-1]
    edges = []

    def in_goal(point):
        return (
            goal_min[0] <= point[0] <= goal_max[0]
            and goal_min[1] <= point[1] <= goal_max[1]
        )

    for iteration in range(1, max_iterations + 1):
        if rng.random() < 0.3:
            sample = (
                rng.uniform(goal_min[0], goal_max[0]),
                rng.uniform(goal_min[1], goal_max[1]),
            )
        else:
            sample = (
                rng.uniform(BOUNDS[0], BOUNDS[1]),
                rng.uniform(BOUNDS[2], BOUNDS[3]),
            )

        nearest_idx = min(
            range(len(vertices)),
            key=lambda i: math.dist(vertices[i], sample),
        )
        nearest = vertices[nearest_idx]
        new_point = steer(nearest, sample, STEP_SIZE)

        if not edge_is_free(
            nearest,
            new_point,
            obstacles,
            ROBOT_RADIUS,
            BOUNDS,
        ):
            continue

        vertices.append(new_point)
        parents.append(nearest_idx)
        edges.append((nearest, new_point))

        if in_goal(new_point):
            path = []
            node = len(vertices) - 1
            while node != -1:
                path.append(vertices[node])
                node = parents[node]
            path.reverse()
            return edges, path, iteration

    return edges, [], max_iterations


def world_to_px(point):
    x, y = point
    px = MARGIN + int((x / 10.0) * (W - 2 * MARGIN))
    py = H - MARGIN - int((y / 10.0) * (H - 2 * MARGIN - 58))
    return px, py


def rotated_square(center, angle, half_width=0.5):
    cx, cy = center
    points = []
    c, s = math.cos(angle), math.sin(angle)
    for dx, dy in [
        (-half_width, -half_width),
        (half_width, -half_width),
        (half_width, half_width),
        (-half_width, half_width),
    ]:
        x = cx + c * dx - s * dy
        y = cy + s * dx + c * dy
        points.append(world_to_px((x, y)))
    return points


def draw_base(title, subtitle, obstacles, hidden=None):
    image = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(image)
    draw.text((MARGIN, 20), title, fill=NAVY, font=font(26, True))
    draw.text((MARGIN, 53), subtitle, fill=MUTED, font=font(14))

    for value in range(11):
        x = world_to_px((value, 0))[0]
        y0 = world_to_px((0, 0))[1]
        y1 = world_to_px((0, 10))[1]
        draw.line((x, y0, x, y1), fill=GRID, width=1)
        y = world_to_px((0, value))[1]
        x0 = world_to_px((0, 0))[0]
        x1 = world_to_px((10, 0))[0]
        draw.line((x0, y, x1, y), fill=GRID, width=1)

    for center, angle in obstacles:
        draw.polygon(rotated_square(center, angle), fill=OBSTACLE)

    if hidden is not None:
        center, angle = hidden
        draw.polygon(rotated_square(center, angle), fill=RED)
        hx, hy = world_to_px(center)
        draw.text((hx + 18, hy - 12), "new obstacle", fill=RED, font=font(13, True))

    gx0, gy0 = world_to_px(GOAL_REGION[0])
    gx1, gy1 = world_to_px(GOAL_REGION[1])
    draw.rounded_rectangle(
        (min(gx0, gx1), min(gy0, gy1), max(gx0, gx1), max(gy0, gy1)),
        radius=7,
        fill=(223, 245, 231),
        outline=GREEN,
        width=2,
    )
    sx, sy = world_to_px(START)
    draw.ellipse((sx - 7, sy - 7, sx + 7, sy + 7), fill=BLUE)
    draw.text((sx + 12, sy - 9), "start", fill=BLUE, font=font(13, True))
    return image


def draw_edges(draw, edges, count=None, color=TREE):
    visible = edges if count is None else edges[:count]
    for a, b in visible:
        draw.line((*world_to_px(a), *world_to_px(b)), fill=color, width=2)


def draw_path(draw, path, color, width=6):
    for a, b in zip(path, path[1:]):
        draw.line((*world_to_px(a), *world_to_px(b)), fill=color, width=width)


obstacles = default_obstacles()
first_edges, first_path, first_iterations = trace_rrt(
    START,
    GOAL_REGION,
    obstacles,
    seed=7,
)

hidden_obstacle = ((5.45, 5.35), math.radians(5))
second_edges, second_path, second_iterations = trace_rrt(
    START,
    GOAL_REGION,
    obstacles + [hidden_obstacle],
    seed=11,
)

if not first_path or not second_path:
    raise RuntimeError("Deterministic demo seeds failed to produce paths.")

frames = []

# Grow the first RRT tree.
for fraction in (0.10, 0.22, 0.38, 0.58, 0.78, 1.0):
    image = draw_base(
        "Geometric RRT",
        "Goal-biased sampling grows a collision-checked tree in continuous space.",
        obstacles,
    )
    draw = ImageDraw.Draw(image)
    count = max(1, int(len(first_edges) * fraction))
    draw_edges(draw, first_edges, count=count)
    if fraction == 1.0:
        draw_path(draw, first_path, BLUE)
        draw.text(
            (W - 260, 25),
            f"path found · {first_iterations} iterations",
            fill=GREEN,
            font=font(14, True),
        )
    frames.append(image)

# Reveal a new obstacle intersecting the old path.
for _ in range(2):
    image = draw_base(
        "Online replanning",
        "A newly observed obstacle invalidates the original route.",
        obstacles,
        hidden=hidden_obstacle,
    )
    draw = ImageDraw.Draw(image)
    draw_path(draw, first_path, OLD_PATH, width=5)
    frames.append(image)

# Grow a new tree and show the replanned path.
for fraction in (0.20, 0.42, 0.68, 1.0):
    image = draw_base(
        "Online replanning",
        "RRT searches again with the updated obstacle set.",
        obstacles,
        hidden=hidden_obstacle,
    )
    draw = ImageDraw.Draw(image)
    draw_path(draw, first_path, OLD_PATH, width=4)
    count = max(1, int(len(second_edges) * fraction))
    draw_edges(draw, second_edges, count=count, color=(137, 176, 223))
    if fraction == 1.0:
        draw_path(draw, second_path, GREEN)
        draw.text(
            (W - 295, 25),
            f"replanned · {second_iterations} iterations",
            fill=GREEN,
            font=font(14, True),
        )
    frames.append(image)

frames = [
    frame.resize((720, 496), Image.Resampling.LANCZOS).convert(
        "P",
        palette=Image.Palette.ADAPTIVE,
        colors=128,
    )
    for frame in frames
]
frames[0].save(
    OUT,
    save_all=True,
    append_images=frames[1:],
    duration=[300, 300, 300, 300, 350, 1500, 900, 900, 350, 350, 450, 1800],
    loop=0,
    optimize=True,
    disposal=2,
)
print(OUT)
