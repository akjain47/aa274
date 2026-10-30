#!/usr/bin/env python3

import typing as T

import numpy as np
import rclpy
from numpy import linalg
from scipy.interpolate import splev, splrep

from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.tf_utils import quaternion_to_yaw
from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState


class AStar(object):
    """Represents a motion planning problem to be solved using A* (from HW1 P1_astar.py)"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1,
                 heuristic="l2", heuristic_weight=1.0):
        self.statespace_lo = statespace_lo         # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = statespace_hi         # state space upper bound (e.g., [5, 5])
        self.occupancy = occupancy                 # occupancy grid (a StochOccupancyGrid2D object)
        self.resolution = resolution               # resolution of the discretization of state space (m)
        self.heuristic_type = heuristic            # norm used by the heuristic: "l1", "l2", or "linf"
        self.heuristic_weight = heuristic_weight   # weight w >= 1 on the heuristic (w > 1 gives weighted A*)
        self.x_offset = x_init
        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state

        self.closed_set = set()    # the set containing the states that have been visited
        self.open_set = set()      # the set containing the states that are condidate for future expension

        self.est_cost_through = {}  # dictionary of the estimated cost from start to goal passing through state (often called f score)
        self.cost_to_arrive = {}    # dictionary of the cost-to-arrive at state from start (often called g score)
        self.came_from = {}         # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.heuristic(self.x_init)

        self.path = None        # the final path as a list of states

    def is_free(self, x):
        x_snap = self.snap_to_grid(x)
        in_bounds = (
            x_snap[0] >= self.statespace_lo[0]
            and x_snap[1] >= self.statespace_lo[1]
            and x_snap[0] <= self.statespace_hi[0]
            and x_snap[1] <= self.statespace_hi[1]
        )
        # StochOccupancyGrid2D.is_free expects a numpy array, not a tuple
        return in_bounds and self.occupancy.is_free(np.asarray(x_snap))

    def distance(self, x1, x2):
        return np.linalg.norm(np.array(x1) - np.array(x2))

    def heuristic(self, x):
        if self.heuristic_type == "l1":
            return self.heuristic_weight * np.sum(np.abs(np.array(x) - np.array(self.x_goal)))
        elif self.heuristic_type == "l2":
            return self.heuristic_weight * np.linalg.norm(np.array(x) - np.array(self.x_goal))
        elif self.heuristic_type == "linf":
            return self.heuristic_weight * np.max(np.abs(np.array(x) - np.array(self.x_goal)))
        else:
            raise ValueError("Unrecognized heuristic type")

    def snap_to_grid(self, x):
        return (
            self.resolution * round((x[0] - self.x_offset[0]) / self.resolution) + self.x_offset[0],
            self.resolution * round((x[1] - self.x_offset[1]) / self.resolution) + self.x_offset[1],
        )

    def get_neighbors(self, x):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (x[0] + dx * self.resolution, x[1] + dy * self.resolution)
                if self.is_free(neighbor):
                    neighbors.append(self.snap_to_grid(neighbor))
        return neighbors

    def find_best_est_cost_through(self):
        return min(self.open_set, key=lambda x: self.est_cost_through[x])

    def reconstruct_path(self):
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))

    def solve(self):
        while len(self.open_set) > 0:
            x_current = self.find_best_est_cost_through()
            if x_current == self.x_goal:
                self.path = self.reconstruct_path()
                return True
            self.open_set.remove(x_current)
            self.closed_set.add(x_current)

            for x_neigh in self.get_neighbors(x_current):
                if x_neigh in self.closed_set:
                    continue
                tentative_cost_to_arrive = self.cost_to_arrive[x_current] + self.distance(x_current, x_neigh)
                if x_neigh not in self.open_set:
                    self.open_set.add(x_neigh)
                elif tentative_cost_to_arrive > self.cost_to_arrive[x_neigh]:
                    continue
                self.came_from[x_neigh] = x_current
                self.cost_to_arrive[x_neigh] = tentative_cost_to_arrive
                self.est_cost_through[x_neigh] = tentative_cost_to_arrive + self.heuristic(x_neigh)

        return False


class Navigator(BaseNavigator):
    def __init__(self):
        super().__init__("navigator")
        self.kp = 2.0
        # trajectory tracking gains and state
        self.kpx = 2.0
        self.kpy = 2.0
        self.kdx = 2.0
        self.kdy = 2.0
        self.V_PREV_THRES = 0.0001
        self.V_prev = 0.0
        self.om_prev = 0.0
        self.t_prev = 0.0

        # trajectory smoothing
        self.v_desired = 0.15    # nominal speed along the path [m/s], below v_max of 0.2
        self.spline_alpha = 0.05 # smoothing factor for splrep

    def compute_heading_control(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        error = wrap_angle(goal.theta - state.theta)
        control = TurtleBotControl()
        control.omega = self.kp * error
        return control

    def compute_trajectory_tracking_control(self, state: TurtleBotState, plan: TrajectoryPlan, t: float) -> TurtleBotControl:
        dt = t - self.t_prev
        x_d = splev(t, plan.path_x_spline, der=0)
        xd_d = splev(t, plan.path_x_spline, der=1)
        xdd_d = splev(t, plan.path_x_spline, der=2)
        y_d = splev(t, plan.path_y_spline, der=0)
        yd_d = splev(t, plan.path_y_spline, der=1)
        ydd_d = splev(t, plan.path_y_spline, der=2)

        x, y, th = state.x, state.y, state.theta

        # avoid singularity
        if abs(self.V_prev) < self.V_PREV_THRES:
            self.V_prev = self.V_PREV_THRES

        xd = self.V_prev * np.cos(th)
        yd = self.V_prev * np.sin(th)

        # compute virt ual ontrols
        u = np.array([xdd_d + self.kpx * (x_d - x) + self.kdx * (xd_d - xd),
                      ydd_d + self.kpy * (y_d - y) + self.kdy * (yd_d - yd)])

        # get actual cntrols 
        J = np.array([[np.cos(th), -self.V_prev * np.sin(th)],
                      [np.sin(th), self.V_prev * np.cos(th)]])
        a, om = linalg.solve(J, u)
        V = self.V_prev + a * dt

        # save the commands that were applied and the time
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        control = TurtleBotControl()
        control.v = float(V)
        control.omega = float(om)
        return control

    def compute_trajectory_plan(self, state: TurtleBotState, goal: TurtleBotState,
                                occupancy: StochOccupancyGrid2D, resolution: float,
                                horizon: float) -> T.Optional[TrajectoryPlan]:
        # A* path planning
        astar = AStar(
            (state.x - horizon, state.y - horizon),
            (state.x + horizon, state.y + horizon),
            (state.x, state.y),
            (goal.x, goal.y),
            occupancy,
            resolution,
        )
        if not astar.solve() or len(astar.path) < 4:
            return None

        # reset tracking controller history
        self.V_prev = 0.0
        self.om_prev = 0.0
        self.t_prev = 0.0

        # path time computation (constant velocity heuristic)
        path = np.asarray(astar.path)
        ds = np.linalg.norm(np.diff(path, axis=0), axis=1)
        ts = np.concatenate(([0.0], np.cumsum(ds / self.v_desired)))

        # trajectory smoothing
        path_x_spline = splrep(ts, path[:, 0], k=3, s=self.spline_alpha)
        path_y_spline = splrep(ts, path[:, 1], k=3, s=self.spline_alpha)

        return TrajectoryPlan(
            path=path,
            path_x_spline=path_x_spline,
            path_y_spline=path_y_spline,
            duration=ts[-1],
        )


if __name__ == "__main__":
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    rclpy.shutdown()
