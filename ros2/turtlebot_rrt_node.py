"""ROS 2 RRT waypoint planner for TurtleBot-style navigation stacks.

This file intentionally contains only the planner node. Robot bring-up, SLAM, and Nav2
should be launched separately using standard ROS 2 launch files.
"""

import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from transforms3d.euler import euler2quat, quat2euler


Point = tuple[float, float]


class TurtleBotRRTNode(Node):
    def __init__(self) -> None:
        super().__init__("turtlebot_rrt_planner")

        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 2.0)
        self.declare_parameter("step_size", 0.5)
        self.declare_parameter("robot_radius", 0.4)
        self.declare_parameter("goal_tolerance", 0.2)
        self.declare_parameter("waypoint_tolerance", 0.3)
        self.declare_parameter("goal_bias", 0.3)
        self.declare_parameter("max_iterations", 10_000)

        self.goal = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
        )
        self.step_size = float(self.get_parameter("step_size").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.waypoint_tolerance = float(self.get_parameter("waypoint_tolerance").value)
        self.goal_bias = float(self.get_parameter("goal_bias").value)
        self.max_iterations = int(self.get_parameter("max_iterations").value)

        self.current_pos: Point | None = None
        self.current_theta = 0.0
        self.map_data: np.ndarray | None = None
        self.map_resolution = 0.05
        self.map_origin: Point | None = None
        self.map_width = 0
        self.map_height = 0
        self.occupied_points: list[Point] = []

        self.path: list[Point] = []
        self.waypoint_index = 0

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.create_timer(0.1, self._tick)

        self.get_logger().info("TurtleBot RRT planner initialized.")

    def _on_odom(self, msg: Odometry) -> None:
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        _, _, self.current_theta = quat2euler([q.x, q.y, q.z, q.w])

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_data = np.asarray(msg.data, dtype=np.int16).reshape(
            (self.map_height, self.map_width)
        )

        occupied = np.argwhere(self.map_data > 50)
        self.occupied_points = [
            (
                self.map_origin[0] + col * self.map_resolution,
                self.map_origin[1] + row * self.map_resolution,
            )
            for row, col in occupied
        ]

    def _bounds(self) -> tuple[float, float, float, float] | None:
        if self.map_origin is None:
            return None
        min_x, min_y = self.map_origin
        return (
            min_x,
            min_x + self.map_width * self.map_resolution,
            min_y,
            min_y + self.map_height * self.map_resolution,
        )

    def _point_free(self, point: Point) -> bool:
        bounds = self._bounds()
        if bounds is None:
            return False
        if not (bounds[0] <= point[0] <= bounds[1] and bounds[2] <= point[1] <= bounds[3]):
            return False

        clearance = self.robot_radius + self.map_resolution
        return all(math.dist(point, obstacle) >= clearance for obstacle in self.occupied_points)

    def _edge_free(self, a: Point, b: Point, samples: int = 20) -> bool:
        for i in range(samples + 1):
            t = i / samples
            point = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            if not self._point_free(point):
                return False
        return True

    @staticmethod
    def _steer(a: Point, b: Point, step_size: float) -> Point:
        distance = math.dist(a, b)
        if distance <= step_size:
            return b
        scale = step_size / distance
        return (a[0] + scale * (b[0] - a[0]), a[1] + scale * (b[1] - a[1]))

    def _rrt(self, start: Point, goal: Point) -> list[Point]:
        bounds = self._bounds()
        if bounds is None:
            return []

        vertices = [start]
        parents = [-1]

        for _ in range(self.max_iterations):
            sample = goal if random.random() < self.goal_bias else (
                random.uniform(bounds[0], bounds[1]),
                random.uniform(bounds[2], bounds[3]),
            )
            nearest_idx = min(
                range(len(vertices)),
                key=lambda i: math.dist(vertices[i], sample),
            )
            nearest = vertices[nearest_idx]
            new_point = self._steer(nearest, sample, self.step_size)

            if not self._edge_free(nearest, new_point):
                continue

            vertices.append(new_point)
            parents.append(nearest_idx)

            if math.dist(new_point, goal) <= self.goal_tolerance and self._edge_free(new_point, goal):
                vertices.append(goal)
                parents.append(len(vertices) - 2)

                path: list[Point] = []
                node = len(vertices) - 1
                while node != -1:
                    path.append(vertices[node])
                    node = parents[node]
                return list(reversed(path))

        return []

    def _remaining_path_valid(self) -> bool:
        if not self.path or self.waypoint_index >= len(self.path):
            return False

        start = self.current_pos
        if start is None:
            return False

        points = [start, *self.path[self.waypoint_index:]]
        return all(self._edge_free(a, b) for a, b in zip(points, points[1:]))

    def _publish_waypoint(self, waypoint: Point) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = waypoint[0]
        msg.pose.position.y = waypoint[1]

        next_index = self.waypoint_index + 1
        if next_index < len(self.path):
            nxt = self.path[next_index]
            yaw = math.atan2(nxt[1] - waypoint[1], nxt[0] - waypoint[0])
        else:
            yaw = self.current_theta

        q = euler2quat(0.0, 0.0, yaw)
        msg.pose.orientation.w = q[0]
        msg.pose.orientation.x = q[1]
        msg.pose.orientation.y = q[2]
        msg.pose.orientation.z = q[3]
        self.goal_pub.publish(msg)

    def _tick(self) -> None:
        if self.current_pos is None or self.map_data is None:
            return

        if math.dist(self.current_pos, self.goal) <= self.goal_tolerance:
            return

        if not self._remaining_path_valid():
            self.get_logger().info("Planning / replanning RRT path.")
            self.path = self._rrt(self.current_pos, self.goal)
            self.waypoint_index = 0
            if not self.path:
                self.get_logger().warning("RRT could not find a path.")
                return

        if self.waypoint_index >= len(self.path):
            return

        waypoint = self.path[self.waypoint_index]
        self._publish_waypoint(waypoint)

        if math.dist(self.current_pos, waypoint) <= self.waypoint_tolerance:
            self.waypoint_index += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TurtleBotRRTNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
