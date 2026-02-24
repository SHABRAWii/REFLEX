#!/usr/bin/env python3
import math
import sys
import time
from collections import deque
import shutil

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener

def term_width(default=110):
    try:
        return shutil.get_terminal_size((default, 20)).columns
    except Exception:
        return default
# ----------------- Terminal styling -----------------
ANSI = True

def esc(code: str) -> str:
    return f"\033[{code}m" if ANSI else ""

RESET = esc("0")
BOLD  = esc("1")
DIM   = esc("2")

FG_RED   = esc("31")
FG_GREEN = esc("32")
FG_YEL   = esc("33")
FG_BLUE  = esc("34")
FG_CYAN  = esc("36")
FG_GRAY  = esc("90")

def colorize(text: str, color: str) -> str:
    return f"{color}{text}{RESET}" if ANSI else text

def box_title(title: str, width: int) -> str:
    t = f" {title} "
    if len(t) >= width - 2:
        t = t[:width-4] + ".. "
    return "┌" + t + "─" * (width - 2 - len(t)) + "┐"

def box_bottom(width: int) -> str:
    return "└" + "─" * (width - 2) + "┘"

def row2(k1, v1, k2, v2, wkey=18, wval=16):
    return f"│ {k1:<{wkey}} {v1:>{wval}}  {k2:<{wkey}} {v2:>{wval}} │"

def fmt_col(x, w=9, p=3):
    """Colored number: nan/red, finite/gray."""
    if x is None or (isinstance(x, float) and (math.isnan(x) or math.isinf(x))):
        return colorize(f"{'nan':>{w}}", FG_RED)
    s = f"{x:>{w}.{p}f}"
    return colorize(s, FG_GRAY)

def deg(x):
    return math.degrees(x) if x is not None and math.isfinite(x) else float("nan")

def clamp_tag(flag_val: float) -> str:
    on = bool(flag_val) and flag_val > 0.5
    return colorize("CLAMP", FG_RED + BOLD) if on else colorize(" ok ", FG_GREEN)

def stale_tag(age: float, thresh: float) -> str:
    if not math.isfinite(age):
        return colorize(" NO ", FG_RED + BOLD)
    if age <= thresh:
        return colorize(" OK ", FG_GREEN)
    if age <= thresh * 2.0:
        return colorize(" OLD", FG_YEL + BOLD)
    return colorize("STALE", FG_RED + BOLD)

def spark(values, width=28):
    if not values:
        return ""
    vmin = min(values)
    vmax = max(values)
    if vmax - vmin < 1e-9:
        return "▁" * min(width, len(values))
    blocks = "▁▂▃▄▅▆▇█"
    vals = list(values)[-width:]
    out = []
    for v in vals:
        t = (v - vmin) / (vmax - vmin)
        idx = int(t * (len(blocks) - 1) + 1e-9)
        out.append(blocks[max(0, min(idx, len(blocks)-1))])
    return "".join(out)


# ----------------- Math helpers -----------------
def quat_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _finite(x) -> bool:
    return isinstance(x, (int, float)) and math.isfinite(float(x))


class StatusDashboard(Node):
    def __init__(self):
        super().__init__('reflex_status_dashboard')

        self.declare_parameter('left_joint', 'Left_Wheel_Joint')
        self.declare_parameter('right_joint', 'Right_Wheel_Joint')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('world_frame', 'odom')

        self.left_joint = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value
        self.base_frame = self.get_parameter('base_frame').value
        self.world_frame = self.get_parameter('world_frame').value

        self.wL = float('nan')
        self.wR = float('nan')
        self.roll = float('nan')
        self.pitch = float('nan')
        self.yaw = float('nan')

        self.t_js = None
        self.t_imu = None
        self.t_dbg = None

        # --- debug sample&hold (prevents NaN overwriting) ---
        self.TARGET = 42
        self.d_hold = [float('nan')] * self.TARGET         # last finite value per index
        self.d_t_hold = [None] * self.TARGET               # last update time per index
        self.dbg_last_len = 0
        self.dbg_frames = 0

        self.hist_v = deque(maxlen=40)
        self.hist_theta = deque(maxlen=40)
        self.hist_u = deque(maxlen=40)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(JointState, '/joint_states', self.on_js, 50)
        self.create_subscription(Imu, '/imu', self.on_imu, 50)
        self.create_subscription(Float64MultiArray, '/reflex_balance/debug', self.on_debug, 10)

        self.timer = self.create_timer(0.1, self.render)

        # clear once
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

    def on_js(self, msg: JointState):
        self.t_js = time.monotonic()
        idx = {n: i for i, n in enumerate(msg.name)}
        if self.left_joint in idx:
            i = idx[self.left_joint]
            if i < len(msg.velocity):
                self.wL = float(msg.velocity[i])
        if self.right_joint in idx:
            i = idx[self.right_joint]
            if i < len(msg.velocity):
                self.wR = float(msg.velocity[i])

    def on_imu(self, msg: Imu):
        self.t_imu = time.monotonic()
        r, p, y = quat_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.roll, self.pitch, self.yaw = r, p, y

    def on_debug(self, msg: Float64MultiArray):
        now = time.monotonic()
        self.t_dbg = now
        self.dbg_frames += 1
        self.dbg_last_len = len(msg.data)

        # Sample & hold: only update indices with finite values.
        upto = min(self.TARGET, len(msg.data))
        for i in range(upto):
            x = msg.data[i]
            if _finite(x):
                self.d_hold[i] = float(x)
                self.d_t_hold[i] = now
        # If msg is shorter than TARGET, do nothing for missing indices (hold last)

    def _age(self, t):
        if t is None:
            return float("inf")
        return time.monotonic() - t

    def _group_age(self, indices):
        """
        Age since last finite update among a set of indices.
        If none ever updated, returns inf.
        """
        ts = [self.d_t_hold[i] for i in indices if 0 <= i < self.TARGET and self.d_t_hold[i] is not None]
        if not ts:
            return float("inf")
        return time.monotonic() - max(ts)

    def render(self):
        # TF lookup
        pos = (float('nan'), float('nan'), float('nan'))
        rpy = (float('nan'), float('nan'), float('nan'))
        try:
            t = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame, Time())
            pos = (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
            q = t.transform.rotation
            rpy = quat_to_euler(q.x, q.y, q.z, q.w)
        except Exception:
            pass

        # Held debug frame
        d = self.d_hold[:]  # already TARGET length

        v_avg, v_set, theta, theta_set, u, wL_cmd, wR_cmd = d[0:7]
        dt, outer_decim, inner_rate, outer_rate, yaw_rate_set, w_turn = d[7:13]
        ev, v_p, v_i, v_d, v_derr, v_uncl, v_iterm, v_out_clamp, v_i_clamp, theta_set_clamp = d[13:23]
        e_theta, t_p, t_i, t_d, t_derr, t_uncl, t_iterm, t_out_clamp, t_i_clamp = d[23:32]
        vL, vR, wL_meas, wR_meas = d[32:36]
        v_kp, v_ki, v_kd, t_kp, t_ki, t_kd = d[36:42]

        # trends
        if math.isfinite(v_avg): self.hist_v.append(v_avg)
        if math.isfinite(theta): self.hist_theta.append(theta)
        if math.isfinite(u): self.hist_u.append(u)

        # ages
        a_js = self._age(self.t_js)
        a_imu = self._age(self.t_imu)
        a_dbg = self._age(self.t_dbg)

        # group ages: (tune thresholds based on your decimation)
        outer_age = self._group_age(list(range(13, 23)))   # outer PID block
        inner_age = self._group_age(list(range(23, 32)))   # inner PID block

        WIDTH = max(96, min(term_width(110), 140))  # wider, but capped
        sys.stdout.write("\033[H")

        header = f"{BOLD}REFLEX PID Insight Dashboard{RESET}"
        now_s = time.strftime("%H:%M:%S")
        sys.stdout.write(f"{header}  {DIM}{now_s}{RESET}\n")

        # ---------------- Status / Timing box ----------------
        sys.stdout.write(box_title("STATUS / TIMING", WIDTH) + "\n")
        sys.stdout.write(row2(
            "joint_states age", f"{fmt_col(a_js, w=7, p=2)} {stale_tag(a_js, 0.5)}",
            "imu age",         f"{fmt_col(a_imu, w=7, p=2)} {stale_tag(a_imu, 0.5)}",
            wkey=18, wval=22
        ) + "\n")
        sys.stdout.write(row2(
            "debug age",       f"{fmt_col(a_dbg, w=7, p=2)} {stale_tag(a_dbg, 1.0)}",
            "dt (s)",          f"{fmt_col(dt, w=9, p=5)}",
            wkey=18, wval=22
        ) + "\n")
        sys.stdout.write(row2(
            "outer fields",    f"{fmt_col(outer_age, w=7, p=2)} {stale_tag(outer_age, 0.25)}",
            "inner fields",    f"{fmt_col(inner_age, w=7, p=2)} {stale_tag(inner_age, 0.25)}",
            wkey=18, wval=22
        ) + "\n")
        sys.stdout.write(row2(
            "inner_rate (Hz)", fmt_col(inner_rate, w=9, p=1),
            "outer_decim",     fmt_col(outer_decim, w=9, p=0),
            wkey=18, wval=22
        ) + "\n")
        sys.stdout.write(row2(
            "outer_rate (Hz)", fmt_col(outer_rate, w=9, p=1),
            "dbg frames/len",  colorize(f"{self.dbg_frames:>6d}/{self.dbg_last_len:<3d}", FG_CYAN),
            wkey=18, wval=22
        ) + "\n")
        sys.stdout.write(box_bottom(WIDTH) + "\n")

        # ---------------- Wheels / Velocity box ----------------
        sys.stdout.write(box_title("WHEELS / VELOCITY", WIDTH) + "\n")
        sys.stdout.write(row2(
            "wL meas (rad/s)", fmt_col(self.wL),
            "wR meas (rad/s)", fmt_col(self.wR),
            wkey=18, wval=22
        ) + "\n")
        sys.stdout.write(row2(
            "vL (m/s)", fmt_col(vL),
            "vR (m/s)", fmt_col(vR),
            wkey=18, wval=22
        ) + "\n")
        sys.stdout.write(row2(
            "v_avg (m/s)", fmt_col(v_avg),
            "v_set (m/s)", fmt_col(v_set),
            wkey=18, wval=22
        ) + "\n")
        sys.stdout.write(row2(
            "ev", fmt_col(ev),
            "trend v_avg", colorize(spark(self.hist_v), FG_BLUE),
            wkey=18, wval=22
        ) + "\n")
        sys.stdout.write(box_bottom(WIDTH) + "\n")

        # ---------------- Outer PID box ----------------
        sys.stdout.write(box_title("OUTER PID: v -> theta_set", WIDTH) + "\n")
        sys.stdout.write(row2("P", fmt_col(v_p), "I", fmt_col(v_i), wkey=18, wval=22) + "\n")
        sys.stdout.write(row2("D", fmt_col(v_d), "dErr", fmt_col(v_derr), wkey=18, wval=22) + "\n")
        sys.stdout.write(row2("unclamped", fmt_col(v_uncl), "i_term", fmt_col(v_iterm), wkey=18, wval=22) + "\n")
        sys.stdout.write(row2("I clamp", clamp_tag(v_i_clamp), "Out clamp", clamp_tag(v_out_clamp), wkey=18, wval=22) + "\n")
        sys.stdout.write(row2("theta_set (rad)", fmt_col(theta_set), "theta_set clamp", clamp_tag(theta_set_clamp), wkey=18, wval=22) + "\n")
        sys.stdout.write(row2(
            "theta_set (deg)", fmt_col(deg(theta_set), w=9, p=2),
            "trend theta", colorize(spark(self.hist_theta), FG_BLUE),
            wkey=18, wval=22
        ) + "\n")
        sys.stdout.write(box_bottom(WIDTH) + "\n")

        # ---------------- PID GAINS ----------------
        sys.stdout.write(box_title("PID GAINS", WIDTH) + "\n")
        sys.stdout.write(row2("Outer Kp", fmt_col(v_kp), "Inner Kp", fmt_col(t_kp), wkey=18, wval=22) + "\n")
        sys.stdout.write(row2("Outer Ki", fmt_col(v_ki), "Inner Ki", fmt_col(t_ki), wkey=18, wval=22) + "\n")
        sys.stdout.write(row2("Outer Kd", fmt_col(v_kd), "Inner Kd", fmt_col(t_kd), wkey=18, wval=22) + "\n")
        sys.stdout.write(box_bottom(WIDTH) + "\n")

        # ---------------- IMU / Inner PID box ----------------
        sys.stdout.write(box_title("IMU / INNER PID: theta -> u", WIDTH) + "\n")
        sys.stdout.write(row2("roll (deg)", fmt_col(deg(self.roll), w=9, p=2),
                              "pitch (deg)", fmt_col(deg(self.pitch), w=9, p=2),
                              wkey=18, wval=22) + "\n")
        sys.stdout.write(row2("yaw (deg)", fmt_col(deg(self.yaw), w=9, p=2),
                              "theta (rad)", fmt_col(theta),
                              wkey=18, wval=22) + "\n")
        sys.stdout.write(row2("e_theta", fmt_col(e_theta),
                              "theta (deg)", fmt_col(deg(theta), w=9, p=2),
                              wkey=18, wval=22) + "\n")
        sys.stdout.write(row2("P", fmt_col(t_p), "I", fmt_col(t_i), wkey=18, wval=22) + "\n")
        sys.stdout.write(row2("D", fmt_col(t_d), "dErr", fmt_col(t_derr), wkey=18, wval=22) + "\n")
        sys.stdout.write(row2("unclamped", fmt_col(t_uncl), "i_term", fmt_col(t_iterm), wkey=18, wval=22) + "\n")
        sys.stdout.write(row2("I clamp", clamp_tag(t_i_clamp), "Out clamp", clamp_tag(t_out_clamp), wkey=18, wval=22) + "\n")
        sys.stdout.write(row2("u (rad/s)", fmt_col(u), "trend u", colorize(spark(self.hist_u), FG_BLUE), wkey=18, wval=22) + "\n")
        sys.stdout.write(box_bottom(WIDTH) + "\n")

        # ---------------- Outputs / Pose box ----------------
        # sys.stdout.write(box_title("OUTPUTS / POSE", WIDTH) + "\n")
        # sys.stdout.write(row2("yaw_rate_set", fmt_col(yaw_rate_set), "w_turn", fmt_col(w_turn), wkey=18, wval=22) + "\n")
        # sys.stdout.write(row2("wL_cmd", fmt_col(wL_cmd), "wR_cmd", fmt_col(wR_cmd), wkey=18, wval=22) + "\n")
        # sys.stdout.write(row2("x (m)", fmt_col(pos[0], w=9, p=3), "y (m)", fmt_col(pos[1], w=9, p=3), wkey=18, wval=22) + "\n")
        # sys.stdout.write(row2("z (m)", fmt_col(pos[2], w=9, p=3), "yaw (deg)", fmt_col(deg(rpy[2]), w=9, p=2), wkey=18, wval=22) + "\n")
        # sys.stdout.write(box_bottom(WIDTH) + "\n")

        sys.stdout.flush()


def main():
    rclpy.init()
    node = StatusDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()