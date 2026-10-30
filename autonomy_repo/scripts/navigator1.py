#!/usr/bin/env python3

import numpy as np
import rclpy
from scipy.interpolate import splev, splrep

from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl


class AStar:
    """Eight-connected A* using the supplied occupancy map."""

    def __init__(
        self,
        statespace_lo,
        statespace_hi,
        x_init,
        x_goal,
        occupancy,
        resolution,
    ):
        self.statespace_lo = np.asarray(statespace_lo)
        self.statespace_hi = np.asarray(statespace_hi)
        self.occupancy = occupancy
        self.resolution = resolution
        self.x_offset = tuple(x_init)

        self.x_init = self.snap_to_grid(x_init)
        self.x_goal = self.snap_to_grid(x_goal)

        self.open_set = {self.x_init}
        self.closed_set = set()
        self.cost_to_arrive = {self.x_init: 0.0}
        self.est_cost_through = {
            self.x_init: self.distance(self.x_init, self.x_goal)
        }
        self.came_from = {}
        self.path = None

    def snap_to_grid(self, x):
        return tuple(
            self.resolution
            * round((x[i] - self.x_offset[i]) / self.resolution)
            + self.x_offset[i]
            for i in range(2)
        )

    def is_free(self, x):
        x_array = np.asarray(x)
        if np.any(x_array < self.statespace_lo):
            return False
        if np.any(x_array > self.statespace_hi):
            return False
        return self.occupancy.is_free(x)

    def distance(self, x1, x2):
        return float(
            np.linalg.norm(np.asarray(x1) - np.asarray(x2))
        )

    def get_neighbors(self, x):
        neighbors = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue

                neighbor = self.snap_to_grid(
                    (
                        x[0] + dx * self.resolution,
                        x[1] + dy * self.resolution,
                    )
                )

                if self.is_free(neighbor):
                    neighbors.append(neighbor)

        return neighbors

    def reconstruct_path(self):
        path = [self.x_goal]
        while path[-1] != self.x_init:
            path.append(self.came_from[path[-1]])
        return list(reversed(path))

    def solve(self):
        if not self.is_free(self.x_goal):
            return False

        while self.open_set:
            current = min(
                self.open_set,
                key=lambda x: self.est_cost_through[x],
            )

            if current == self.x_goal:
                self.path = self.reconstruct_path()
                return True

            self.open_set.remove(current)
            self.closed_set.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closed_set:
                    continue

                new_cost = (
                    self.cost_to_arrive[current]
                    + self.distance(current, neighbor)
                )

                if new_cost >= self.cost_to_arrive.get(
                    neighbor, float("inf")
                ):
                    continue

                self.came_from[neighbor] = current
                self.cost_to_arrive[neighbor] = new_cost
                self.est_cost_through[neighbor] = (
                    new_cost + self.distance(neighbor, self.x_goal)
                )
                self.open_set.add(neighbor)

        return False


class Navigator(BaseNavigator):
    def __init__(self):
        super().__init__("navigator")

        # Starting gains. Use your HW1 heading gain if different.
        self.kp_heading = 2.0

        self.kpx = 1.0
        self.kpy = 1.0
        self.kdx = 2.0
        self.kdy = 2.0

        self.V_PREV_THRES = 0.0001

        self.v_desired = 0.15
        self.spline_alpha = 0.05

        self.reset_tracking_history()

    def reset_tracking_history(self):
        self.V_prev = 0.0
        self.om_prev = 0.0
        self.t_prev = 0.0

    def compute_heading_control(self, state, goal):
        error = wrap_angle(goal.theta - state.theta)

        return TurtleBotControl(
            v=0.0,
            omega=float(self.kp_heading * error),
        )

    def compute_trajectory_tracking_control(self, state, plan, t):
        dt = t - self.t_prev

        # Desired position, velocity, and acceleration.
        x_d = float(splev(t, plan.path_x_spline, der=0))
        xd_d = float(splev(t, plan.path_x_spline, der=1))
        xdd_d = float(splev(t, plan.path_x_spline, der=2))

        y_d = float(splev(t, plan.path_y_spline, der=0))
        yd_d = float(splev(t, plan.path_y_spline, der=1))
        ydd_d = float(splev(t, plan.path_y_spline, der=2))

        # Avoid a singular feedback-linearization matrix.
        if abs(self.V_prev) < self.V_PREV_THRES:
            self.V_prev = self.V_PREV_THRES

        c = np.cos(state.theta)
        s = np.sin(state.theta)

        xd = self.V_prev * c
        yd = self.V_prev * s

        u = np.array([
            xdd_d
            + self.kpx * (x_d - state.x)
            + self.kdx * (xd_d - xd),

            ydd_d
            + self.kpy * (y_d - state.y)
            + self.kdy * (yd_d - yd),
        ])

        J = np.array([
            [c, -self.V_prev * s],
            [s,  self.V_prev * c],
        ])

        acceleration, omega = np.linalg.solve(J, u)
        velocity = self.V_prev + acceleration * dt

        self.V_prev = float(velocity)
        self.om_prev = float(omega)
        self.t_prev = float(t)

        # The base controller applies command limits.
        return TurtleBotControl(
            v=float(velocity),
            omega=float(omega),
        )

    def compute_trajectory_plan(
        self, state, goal, occupancy, resolution, horizon
    ):
        start = np.array([state.x, state.y], dtype=float)
        target = np.array([goal.x, goal.y], dtype=float)

        # A square planning region centered on the robot.
        # Here horizon is the full side length.
        lower = start - horizon / 2.0
        upper = start + horizon / 2.0

        astar = AStar(
            statespace_lo=lower,
            statespace_hi=upper,
            x_init=start,
            x_goal=target,
            occupancy=occupancy,
            resolution=resolution,
        )

        if not astar.solve():
            self.get_logger().warn("A* could not find a path.")
            return None

        if len(astar.path) < 4:
            self.get_logger().warn(
                "Need at least four waypoints for a cubic spline."
            )
            return None

        self.reset_tracking_history()

        path = np.asarray(astar.path, dtype=float)

        segment_lengths = np.linalg.norm(
            np.diff(path, axis=0), axis=1
        )

        ts = np.concatenate([
            [0.0],
            np.cumsum(segment_lengths / self.v_desired),
        ])

        path_x_spline = splrep(
            ts, path[:, 0], k=3, s=self.spline_alpha
        )
        path_y_spline = splrep(
            ts, path[:, 1], k=3, s=self.spline_alpha
        )

        return TrajectoryPlan(
            path=path,
            path_x_spline=path_x_spline,
            path_y_spline=path_y_spline,
            duration=float(ts[-1]),
        )


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()

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