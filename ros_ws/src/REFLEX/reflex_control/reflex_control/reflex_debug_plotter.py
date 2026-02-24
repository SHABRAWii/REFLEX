#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock

import matplotlib.pyplot as plt


def isfinite(x: float) -> bool:
    return isinstance(x, (int, float)) and math.isfinite(x)


class ReflexDebugPlotter(Node):
    """
    One big plot:
      - y=0 centered (symmetric ylim)
      - plots: theta (measured), u (pid output), and 0-line setpoint.
    Freezes updates when simulation is paused using /clock.
    """

    def __init__(self):
        super().__init__("reflex_debug_plotter")

        # ---- Parameters ----
        self.declare_parameter("debug_topic", "/reflex_balance/debug")
        self.declare_parameter("window_sec", 15.0)
        self.declare_parameter("max_points", 3000)
        self.declare_parameter("pause_hold_sec", 0.25)
        self.declare_parameter("plot_hz", 20.0)
        self.declare_parameter("ylim_margin", 0.15)  # 15% headroom
        self.declare_parameter("theta_scale", 1.0)   # keep 1.0 (rad). Set to 57.2958 to show deg.
        self.declare_parameter("u_scale", 1.0)       # if u is huge, you can scale for visibility.

        self.debug_topic = self.get_parameter("debug_topic").value
        self.window_sec = float(self.get_parameter("window_sec").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.pause_hold_sec = float(self.get_parameter("pause_hold_sec").value)
        self.plot_hz = float(self.get_parameter("plot_hz").value)
        self.ylim_margin = float(self.get_parameter("ylim_margin").value)
        self.theta_scale = float(self.get_parameter("theta_scale").value)
        self.u_scale = float(self.get_parameter("u_scale").value)

        # ---- Sim time / pause detection via /clock ----
        self.sim_t = None
        self.last_sim_t = None
        self.last_sim_wall = None
        self.paused = True

        qos_clock = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Clock, "/clock", self.on_clock, qos_clock)

        # ---- Debug subscription ----
        qos_dbg = QoSProfile(depth=10)
        self.create_subscription(Float64MultiArray, self.debug_topic, self.on_debug, qos_dbg)

        # ---- Data buffers ----
        self.t = deque(maxlen=self.max_points)
        self.theta = deque(maxlen=self.max_points)  # measured output
        self.theta_set = deque(maxlen=self.max_points)   # outer PID (clamped)
        self.u = deque(maxlen=self.max_points)      # pid output
        self.have_dbg = False

        # ---- Matplotlib UI ----
        plt.ion()
        self.fig = plt.figure(figsize=(12, 6))
        self.fig.canvas.manager.set_window_title("REFLEX: theta + u (0 centered)")
        self.ax = self.fig.add_subplot(1, 1, 1)

        self.ax.set_title("REFLEX Balance: theta (output) and u (PID output) with 0 setpoint")
        self.ax.set_xlabel("Sim time (s)")
        self.ax.set_ylabel("Value (scaled)")

        (self.l_theta,) = self.ax.plot([], [], label="theta (measured)", linewidth=2)
        (self.l_theta_set,) = self.ax.plot([], [], color="red", linewidth=2, label="theta_set (outer PID)")
        (self.l_u,) = self.ax.plot([], [], label="u (PID output)", linewidth=1.5)
        self.l_zero = self.ax.axhline(0.0, linewidth=1, linestyle="--", label="setpoint=0")

        self.ax.legend(loc="upper right")
        self.status_txt = self.fig.text(0.01, 0.01, "", fontsize=10)

        period = 1.0 / max(1e-6, self.plot_hz)
        self.create_timer(period, self.update_plot)

        self.get_logger().info(f"Plotting {self.debug_topic} (theta+u) with symmetric y-limits. Freezing on pause via /clock.")

    def on_clock(self, msg: Clock):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.sim_t = t

        now_wall = self.get_clock().now().nanoseconds * 1e-9
        if self.last_sim_t is None:
            self.last_sim_t = t
            self.last_sim_wall = now_wall
            self.paused = False
            return

        if t != self.last_sim_t:
            self.last_sim_t = t
            self.last_sim_wall = now_wall
            self.paused = False
        else:
            if self.last_sim_wall is not None and (now_wall - self.last_sim_wall) > self.pause_hold_sec:
                self.paused = True

    def on_debug(self, msg: Float64MultiArray):
        if self.sim_t is None:
            return

        # Freeze data collection when paused (your requirement)
        if self.paused:
            return

        d = list(msg.data)
        TARGET = 42
        if len(d) < TARGET:
            d = d + [float("nan")] * (TARGET - len(d))

        # Mapping (from your dashboard)
        # v_avg, v_set, theta, theta_set, u, wL_cmd, wR_cmd = d[0:7]
        theta = d[2]
        theta_set = d[3] 
        u = d[4]

        if not isfinite(theta) or not isfinite(u) or not isfinite(theta_set):
            return

        self.t.append(self.sim_t)
        self.theta.append(theta * self.theta_scale)
        self.theta_set.append(theta_set * self.theta_scale)
        self.u.append(u * self.u_scale)

        self.have_dbg = True
        self._trim_window()

    def _trim_window(self):
        if not self.t:
            return
        t_max = self.t[-1]
        t_min = t_max - self.window_sec
        while self.t and self.t[0] < t_min:
            self.t.popleft()
            self.theta.popleft()
            self.theta_set.popleft()
            self.u.popleft()

    def _set_symmetric_ylim(self):
        """Center y=0 and make +top == -bottom based on max abs of visible data."""
        vals = []
        vals.extend(self.theta)
        vals.extend(self.theta_set)
        vals.extend(self.u)

        # Filter finite
        vals = [v for v in vals if isfinite(v)]
        if len(vals) < 2:
            return

        max_abs = max(abs(v) for v in vals)
        if max_abs < 1e-9:
            max_abs = 1.0

        pad = 1.0 + max(0.0, self.ylim_margin)
        lim = max_abs * pad
        self.ax.set_ylim(-lim, lim)

    def update_plot(self):
        if not plt.fignum_exists(self.fig.number):
            rclpy.shutdown()
            return

        if not self.have_dbg or len(self.t) < 2:
            self.status_txt.set_text("Waiting for debug + /clock ...")
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            return

        t = list(self.t)
        self.l_theta.set_data(t, list(self.theta))
        self.l_theta_set.set_data(t, list(self.theta_set))
        self.l_u.set_data(t, list(self.u))

        # x-limits follow window
        self.ax.set_xlim(t[0], t[-1])

        # symmetric y around 0
        self._set_symmetric_ylim()

        st = "PAUSED" if self.paused else "RUNNING"
        self.status_txt.set_text(
            f"Sim: {st} | window={self.window_sec:.1f}s | points={len(t)} | "
            f"theta_scale={self.theta_scale:g} u_scale={self.u_scale:g}"
        )

        self.ax.grid(True, which="both", linestyle=":", linewidth=0.5)
        self.fig.tight_layout(rect=[0, 0.03, 1, 1])
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    node = ReflexDebugPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()