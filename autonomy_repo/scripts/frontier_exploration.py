#!/usr/bin/env python3
"""Choose frontier goals for the existing TurtleBot navigator."""

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from scipy.signal import convolve2d
from std_msgs.msg import Bool

from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl


def window_counts(mask, window_size):
    return convolve2d(
        mask.astype(int), np.ones((window_size, window_size), dtype=int),
        mode="same", boundary="fill", fillvalue=0,
    )


def explore(occupancy, window_size=11, min_unknown=0.25, min_free=0.35,
            occupied_thresh=50):
    """Problem 1 frontier detector, using ROS occupancy values unchanged."""
    unknown = occupancy.probs == -1
    occupied = occupancy.probs >= occupied_thresh
    free = ~unknown & ~occupied
    area = window_size ** 2
    frontier = (
        (window_counts(unknown, window_size) >= min_unknown * area)
        & (window_counts(occupied, window_size) == 0)
        & (window_counts(free, window_size) >= min_free * area)
    )
    ky, kx = np.nonzero(frontier)
    return occupancy.grid2state(np.column_stack((kx, ky)))


class FrontierExploration(Node):
    def __init__(self):
        super().__init__("frontier_exploration")
        defaults = {
            "frontier_window_size": 11,
            "occupancy_window_size": 9,
            "min_unknown": 0.25,
            "min_free": 0.35,
            "occupied_thresh": 50,
            "min_goal_distance": 0.4,
            "failed_goal_radius": 0.4,
            "failed_goal_cooldown": 60.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in ("frontier_window_size", "occupancy_window_size"):
            value = self.get_parameter(name).value
            if value < 1 or value % 2 != 1:
                raise ValueError(f"{name} must be a positive odd integer")

        self.state = None
        self.occupancy = None
        self.active_goal = None
        self.failed_goals = []  # (world-coordinate position, ROS-clock expiry)

        self.goal_pub = self.create_publisher(TurtleBotState, "/cmd_nav", 10)
        self.state_sub = self.create_subscription(
            TurtleBotState, "/state", self.state_callback, 10)
        # Match BaseNavigator's volatile QoS: both nodes must receive live maps.
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10)
        self.result_sub = self.create_subscription(
            Bool, "/nav_success", self.result_callback, 10)
        self.timer = self.create_timer(1.0, self.select_goal)
        self.get_logger().info("Waiting for /map, /state, and navigator connections")

        self.motor_sub = self.create_subscription(Bool, "/detector_bool", self.detector_callback, 10)
        self.image_detected = False
        self.pause = False

    def detector_callback(self, msg: Bool) -> None:
                # This handles incoming messages from the topic
            self.image_detected = msg.data
            if self.image_detected:
                self.pause = True
            

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def state_callback(self, msg):
        self.state = msg

    def map_callback(self, msg):
        if (msg.info.resolution <= 0 or msg.info.width == 0 or msg.info.height == 0
                or len(msg.data) != msg.info.width * msg.info.height):
            self.get_logger().warn("Ignoring invalid occupancy grid")
            return
        # Same axis-aligned map convention used by BaseNavigator and Problem 1.
        orientation = msg.info.origin.orientation
        if abs(orientation.x) + abs(orientation.y) + abs(orientation.z) > 1e-6:
            self.get_logger().error("Rotated map origins are unsupported by the navigator")
            self.occupancy = None
            return
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=self.get_parameter("occupancy_window_size").value,
            probs=msg.data,
        )

    def result_callback(self, msg):
        if self.active_goal is None:
            return
        if msg.data:
            self.get_logger().info("Frontier reached; selecting the next goal")
        else:
            expiry = self.now_seconds() + self.get_parameter("failed_goal_cooldown").value
            self.failed_goals.append((self.active_goal.copy(), expiry))
            self.get_logger().info("Navigation failed; temporarily excluding this frontier region")
        self.active_goal = None

    def select_goal(self):
        """Try frontiers from nearest to farthest; let the navigator find a path."""
        if self.state is None or self.occupancy is None or self.active_goal is not None :
            return
        if (self.goal_pub.get_subscription_count() == 0
                or self.count_publishers("/nav_success") == 0):
            return
        now = self.now_seconds()
        self.failed_goals = [(goal, expiry) for goal, expiry in self.failed_goals
                             if expiry > now]
        occupancy = self.occupancy
        frontiers = explore(
            occupancy,
            window_size=self.get_parameter("frontier_window_size").value,
            min_unknown=self.get_parameter("min_unknown").value,
            min_free=self.get_parameter("min_free").value,
            occupied_thresh=self.get_parameter("occupied_thresh").value,
        )
        current = np.array([self.state.x, self.state.y])
        distances = np.linalg.norm(frontiers - current, axis=1)
        radius = self.get_parameter("failed_goal_radius").value
        for index in np.argsort(distances, kind="stable"):
            goal = frontiers[index]
            if distances[index] < self.get_parameter("min_goal_distance").value:
                continue
            if any(np.linalg.norm(goal - failed) <= radius for failed, _ in self.failed_goals):
                continue
            # P1 permits unknown frontier centers; navigation goals must be known free.
            kx, ky = occupancy.state2grid(goal)
            if not 0 <= occupancy.probs[ky, kx] < self.get_parameter("occupied_thresh").value:
                continue
            if not occupancy.is_free(goal):
                continue
            if self.image_detected:
                print("image detected")
                goal = current
            self.send_goal(goal)
            return
        # No usable goal: keep listening and try again on the next timer tick.

    def send_goal(self, goal):
        current = np.array([self.state.x, self.state.y])
        delta = goal - current
        print("goal")
        if self.image_detected:
            print("image detected")
    
        command = TurtleBotState(
            x=float(goal[0]), y=float(goal[1]),
            theta=float(np.arctan2(delta[1], delta[0])),
        )

        self.active_goal = goal.copy()
        self.goal_pub.publish(command)
        self.get_logger().info(f"Exploring ({goal[0]:.2f}, {goal[1]:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
