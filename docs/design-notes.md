# Design Notes

## From graph search to sampling-based planning

The first implementation used a label-correcting shortest-path algorithm on a discretized occupancy grid. This is useful as a deterministic baseline but becomes increasingly expensive as grid resolution increases.

Geometric RRT removes the fixed grid and samples directly in continuous configuration space. For the disk-robot experiments, collision checking inflates rotated square obstacles by the robot radius.

## Planning with unknown space

The online-planning experiment used an optimistic map convention:

- unknown: traversable for planning;
- sensed free: traversable;
- sensed occupied: blocked.

This creates a simple receding loop in which the robot plans using current knowledge and replans after sensing reveals a collision with the planned route.

The approach is intentionally lightweight. A more principled treatment would model uncertainty explicitly or incorporate risk into planning.

## Kinodynamic extension

For a differential-drive state `(x, y, theta)`, direct Euclidean steering may produce dynamically impossible edges.

The kinodynamic variant therefore evaluates a finite control set. Each control `(v, omega)` is propagated for a fixed horizon, and the feasible rollout whose terminal position is closest to the sampled target is selected.

The original control set used linear velocities of ±0.25 m/s and angular velocities spanning approximately ±1.82 rad/s.

## ROS 2 integration

The original real-robot script combined planner logic with machine-specific process launching. The portfolio version separates those concerns.

The planner node now assumes standard ROS 2 components are launched externally and focuses on:

- consuming odometry and occupancy maps;
- planning RRT paths;
- checking the remaining path after map updates;
- publishing waypoint goals.

This makes the planning code easier to read, reuse, and test.
