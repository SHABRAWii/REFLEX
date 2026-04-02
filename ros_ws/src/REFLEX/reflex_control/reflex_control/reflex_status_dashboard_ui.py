#!/usr/bin/env python3
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from PySide6 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg


def isfinite(x):
    return isinstance(x, (int, float)) and math.isfinite(x)


class Ring:
    """Time series ring buffer."""
    def __init__(self, maxlen: int):
        self.t = deque(maxlen=maxlen)
        self.v = deque(maxlen=maxlen)

    def clear(self):
        self.t.clear()
        self.v.clear()

    def append(self, t, v):
        self.t.append(t)
        self.v.append(v)

    def data(self):
        return list(self.t), list(self.v)


class RateEstimator:
    """Estimate message rate per-topic using wall time (handles different Hz)."""
    def __init__(self, window=50):
        self.times = deque(maxlen=window)

    def tick(self):
        self.times.append(time.time())

    def hz(self):
        if len(self.times) < 5:
            return 0.0
        dt = self.times[-1] - self.times[0]
        if dt <= 1e-6:
            return 0.0
        return (len(self.times) - 1) / dt


class ReflexControlLabUI(Node):
    def __init__(self):
        super().__init__("reflex_control_lab_ui")

        # ---------------- Parameters ----------------
        self.declare_parameter("debug_topic", "/reflex_balance/debug")
        # tuner_node should be the node name (no leading slash) used by the PID tuner
        self.declare_parameter("tuner_node", "reflex_pid_tuner")
        # PID controller node (where live gains are kept)
        self.declare_parameter("pid_node", "/reflex_balance_pid")
        self.declare_parameter("window_sec", 15.0)
        self.declare_parameter("max_points", 20000)  # more points
        self.declare_parameter("sim_reset_threshold_sec", 0.5)

        self.declare_parameter("left_wheel_joint", "Left_Wheel_Joint")
        self.declare_parameter("right_wheel_joint", "Right_Wheel_Joint")

        self.debug_topic = self.get_parameter("debug_topic").value
        # normalize tuner node name (strip leading slash if provided)
        self.tuner_node = str(self.get_parameter("tuner_node").value).lstrip('/')
        # pid_node may be where the runtime gains live (strip leading slash)
        self.pid_node = str(self.get_parameter("pid_node").value).lstrip('/')
        self.window_sec = float(self.get_parameter("window_sec").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.sim_reset_thresh = float(self.get_parameter("sim_reset_threshold_sec").value)
        self.left_wheel_joint = self.get_parameter("left_wheel_joint").value
        self.right_wheel_joint = self.get_parameter("right_wheel_joint").value

        # ---------------- Sim time tracking ----------------
        self.sim_time = float("nan")
        self._last_sim_time = float("nan")
        self._last_clock_wall = None

        # ---------------- Rate tracking (wall) ----------------
        self.r_clock = RateEstimator()
        self.r_debug = RateEstimator()
        self.r_joint = RateEstimator()

        # ---------------- Buffers ----------------
        self.theta = Ring(self.max_points)
        self.theta_set = Ring(self.max_points)
        self.u = Ring(self.max_points)

        self.v_avg = Ring(self.max_points)
        self.v_set = Ring(self.max_points)

        self.wL = Ring(self.max_points)
        self.wR = Ring(self.max_points)
        self.tauL = Ring(self.max_points)
        self.tauR = Ring(self.max_points)

        # Optional: commands from debug (d[5], d[6])
        self.wL_cmd = Ring(self.max_points)
        self.wR_cmd = Ring(self.max_points)

        # ---------------- ROS Subscriptions ----------------
        self.create_subscription(Clock, "/clock", self.cb_clock, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, self.debug_topic, self.cb_debug, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.cb_joint, qos_profile_sensor_data)

        # ---------------- ROS Clients (PID + services) ----------------
        # AsyncParameterClient: prefer the PID node parameter service, fall back to tuner
        self.param_client = None
        for candidate in [self.pid_node, self.tuner_node]:
            try:
                client = AsyncParameterClient(self, candidate)
                client.wait_for_service(timeout_sec=0.5)
                self.get_logger().info(f"Using parameter service on '{candidate}'")
                self.param_client = client
                break
            except Exception:
                # try next candidate
                pass

        if self.param_client is None:
            # nothing available yet; create a client for pid_node and let callers wait later
            self.get_logger().warn("No parameter service available at startup; will retry when needed")
            self.param_client = AsyncParameterClient(self, self.pid_node)

        # service clients: we'll prefer node-scoped services (e.g. /<pid_node>/apply_gains)
        # and fall back to the global tuner services if needed. Create placeholders now.
        self.apply_gains_cli = None
        self.reset_pid_cli = None
        self.apply_pose_cli = None  # optional, if you want
        # default global names (fallbacks)
        self._default_apply_gains_name = '/apply_gains'
        self._default_reset_pid_name = '/reset_pid'
        self._default_apply_pose_name = '/apply_pose'

        # ---------------- UI / Theme ----------------
        self._init_ui()

        # Parameter service readiness flag
        self.param_service_ready = False

        # Start a background monitor that will enable Apply/Reload when the param service appears
        self._start_param_monitor()

    # ===================== Sim time + reset handling =====================
    def cb_clock(self, msg: Clock):
        self.r_clock.tick()
        self._last_clock_wall = time.time()
        t = msg.clock.sec + msg.clock.nanosec * 1e-9

        # Detect sim-time reset (Gazebo restarted)
        if isfinite(self._last_sim_time) and (t < self._last_sim_time - self.sim_reset_thresh):
            self.get_logger().warn(f"Sim time reset detected: {self._last_sim_time:.3f} -> {t:.3f}. Clearing buffers.")
            self._clear_all_buffers()

        self.sim_time = t
        self._last_sim_time = t

    def _clear_all_buffers(self):
        for ring in [
            self.theta, self.theta_set, self.u,
            self.v_avg, self.v_set,
            self.wL, self.wR, self.tauL, self.tauR,
            self.wL_cmd, self.wR_cmd
        ]:
            ring.clear()

        # Clear plots immediately
        for curve in [
            self.c_theta, self.c_theta_set, self.c_u,
            self.c_vavg, self.c_vset,
            self.c_wL, self.c_wR,
            self.c_tauL, self.c_tauR
        ]:
            curve.setData([], [])

        if self.chk_show_cmd.isChecked():
            self.c_wL_cmd.setData([], [])
            self.c_wR_cmd.setData([], [])

    # ===================== Topic callbacks =====================
    def cb_debug(self, msg: Float64MultiArray):
        self.r_debug.tick()

        if not isfinite(self.sim_time):
            return

        d = list(msg.data)
        TARGET = 42
        if len(d) < TARGET:
            d += [float("nan")] * (TARGET - len(d))

        # Mapping
        # v_avg, v_set, theta, theta_set, u, wL_cmd, wR_cmd = d[0:7]
        t = self.sim_time

        self.v_avg.append(t, d[0])
        self.v_set.append(t, d[1])

        self.theta.append(t, d[2])
        self.theta_set.append(t, d[3])
        self.u.append(t, d[4])

        self.wL_cmd.append(t, d[5])
        self.wR_cmd.append(t, d[6])

    def cb_joint(self, msg: JointState):
        self.r_joint.tick()

        if not isfinite(self.sim_time):
            return

        t = self.sim_time
        idx = {n: i for i, n in enumerate(msg.name)}
        li = idx.get(self.left_wheel_joint, None)
        ri = idx.get(self.right_wheel_joint, None)

        if li is not None:
            if li < len(msg.velocity):
                self.wL.append(t, msg.velocity[li])
            if li < len(msg.effort):
                self.tauL.append(t, msg.effort[li])

        if ri is not None:
            if ri < len(msg.velocity):
                self.wR.append(t, msg.velocity[ri])
            if ri < len(msg.effort):
                self.tauR.append(t, msg.effort[ri])

    # ===================== UI (Modern) =====================
    def _init_ui(self):
        # PyQtGraph modern dark
        pg.setConfigOptions(
            antialias=True,
            background=(12, 14, 18),
            foreground=(220, 220, 220),
        )

        # Create application and apply a robust dark palette + stylesheet so the UI
        # is not plain black/white on systems that ignore styles.
        self.app = QtWidgets.QApplication([])
        self.app.setStyle("Fusion")

        dark_palette = QtGui.QPalette()
        bg = QtGui.QColor(18, 20, 24)
        mid = QtGui.QColor(28, 30, 36)
        text = QtGui.QColor(230, 230, 230)
        disabled = QtGui.QColor(130, 130, 130)

        dark_palette.setColor(QtGui.QPalette.Window, bg)
        dark_palette.setColor(QtGui.QPalette.WindowText, text)
        dark_palette.setColor(QtGui.QPalette.Base, QtGui.QColor(12, 14, 18))
        dark_palette.setColor(QtGui.QPalette.AlternateBase, mid)
        dark_palette.setColor(QtGui.QPalette.ToolTipBase, text)
        dark_palette.setColor(QtGui.QPalette.ToolTipText, text)
        dark_palette.setColor(QtGui.QPalette.Text, text)
        dark_palette.setColor(QtGui.QPalette.Disabled, QtGui.QPalette.Text, disabled)
        dark_palette.setColor(QtGui.QPalette.Button, mid)
        dark_palette.setColor(QtGui.QPalette.ButtonText, text)
        dark_palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(70, 130, 180))
        dark_palette.setColor(QtGui.QPalette.HighlightedText, QtGui.QColor(240, 240, 240))

        self.app.setPalette(dark_palette)

        # Qt stylesheet for a modern feel
        self.app.setStyleSheet("""
            QWidget { font-size: 12px; }
            QGroupBox {
                border: 1px solid #2b313c; border-radius: 10px; margin-top: 10px;
                background: #141822;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 6px; color: #e6e6e6; }
            QLabel { color: #e6e6e6; }
            QPushButton {
                background: #1f2633; border: 1px solid #2b313c; border-radius: 8px; padding: 6px;
            }
            QPushButton:hover { background: #243043; }
            QDoubleSpinBox, QLineEdit {
                background: #0f1218; border: 1px solid #2b313c; border-radius: 8px; padding: 4px;
                color: #e6e6e6;
            }
            QCheckBox { color: #e6e6e6; }
        """)

        self.win = QtWidgets.QMainWindow()
        self.win.setWindowTitle("REFLEX Control Lab UI")
        self.win.resize(1550, 900)

        central = QtWidgets.QWidget()
        self.win.setCentralWidget(central)

        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        # Left: plots (2x2)
        self.plots_widget = pg.GraphicsLayoutWidget()
        root.addWidget(self.plots_widget, stretch=6)

        # Right: controls
        side_wrap = QtWidgets.QVBoxLayout()
        root.addLayout(side_wrap, stretch=3)

        # Status
        self.lbl_status = QtWidgets.QLabel("sim: -- | hz(clock/debug/joint): --/--/-- | state: --")
        self.lbl_status.setStyleSheet("font-family: monospace; font-size: 13px; padding: 6px;")
        side_wrap.addWidget(self.lbl_status)

        # Controls group
        ctrl = QtWidgets.QGroupBox("Controls")
        ctrl_l = QtWidgets.QVBoxLayout(ctrl)

        self.chk_show_cmd = QtWidgets.QCheckBox("Show wheel commands plot (wL_cmd / wR_cmd)")
        self.chk_show_cmd.setChecked(False)
        self.chk_show_cmd.stateChanged.connect(self._toggle_cmd_plot)
        ctrl_l.addWidget(self.chk_show_cmd)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_clear = QtWidgets.QPushButton("Clear Buffers")
        self.btn_clear.clicked.connect(self._clear_all_buffers)
        btn_row.addWidget(self.btn_clear)

        self.btn_reload = QtWidgets.QPushButton("Reload PID from node")
        self.btn_reload.clicked.connect(self._request_current_gains)
        btn_row.addWidget(self.btn_reload)
        # disable reload until parameter service is detected
        self.btn_reload.setEnabled(False)
        ctrl_l.addLayout(btn_row)

        side_wrap.addWidget(ctrl)

        # PID group
        pid = QtWidgets.QGroupBox("PID (edit -> Apply)")
        pid_l = QtWidgets.QGridLayout(pid)

        def add_spin(row, name):
            lab = QtWidgets.QLabel(name)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setDecimals(6)
            sp.setRange(-1e6, 1e6)
            sp.setSingleStep(0.1)
            pid_l.addWidget(lab, row, 0)
            pid_l.addWidget(sp, row, 1)
            return sp

        self.sp_v_kp = add_spin(0, "v_kp")
        self.sp_v_ki = add_spin(1, "v_ki")
        self.sp_v_kd = add_spin(2, "v_kd")
        self.sp_t_kp = add_spin(3, "t_kp")
        self.sp_t_ki = add_spin(4, "t_ki")
        self.sp_t_kd = add_spin(5, "t_kd")

        self.btn_apply = QtWidgets.QPushButton("Apply (set params -> /apply_gains -> /reset_pid)")
        self.btn_apply.clicked.connect(self._apply_pid_clicked)
        pid_l.addWidget(self.btn_apply, 6, 0, 1, 2)
        # disable apply until parameter service is detected
        self.btn_apply.setEnabled(False)

        side_wrap.addWidget(pid)
        side_wrap.addStretch(1)

        # ----- Build 2x2 plots -----
        grid_alpha = 0.20

        self.p_balance = self.plots_widget.addPlot(title="Balance: theta / theta_set / u (0-centered)")
        self.p_velocity = self.plots_widget.addPlot(title="Velocity: v_avg / v_set")
        self.plots_widget.nextRow()
        self.p_wheels = self.plots_widget.addPlot(title="Wheels: wL / wR (rad/s)")
        self.p_torque = self.plots_widget.addPlot(title="Torque: tauL / tauR (Nm)")

        self.all_plots = [self.p_balance, self.p_velocity, self.p_wheels, self.p_torque]

        for p in self.all_plots:
            p.showGrid(x=True, y=True, alpha=grid_alpha)
            p.setLabel("bottom", "Sim Time (s)")
            p.setClipToView(True)
            p.setDownsampling(auto=True, mode="peak")  # draws many points smoothly
            p.setAutoVisible(y=True)

        # Zero line for balance
        zero = pg.InfiniteLine(
            pos=0,
            angle=0,
            pen=pg.mkPen((80, 140, 255), width=1.5, style=QtCore.Qt.DashLine),
        )
        self.p_balance.addItem(zero)

        # Curves (modern colors) + legends
        self.p_balance.addLegend(offset=(10, 10))
        self.c_theta = self.p_balance.plot(pen=pg.mkPen((60, 120, 255), width=2), name='theta')
        self.c_theta_set = self.p_balance.plot(pen=pg.mkPen((255, 110, 110), width=2), name='theta_set')
        self.c_u = self.p_balance.plot(pen=pg.mkPen((255, 175, 90), width=2), name='u')

        self.p_velocity.addLegend(offset=(10, 10))
        self.c_vavg = self.p_velocity.plot(pen=pg.mkPen((0, 210, 210), width=2), name='v_avg')
        self.c_vset = self.p_velocity.plot(pen=pg.mkPen((210, 210, 0), width=2), name='v_set')

        self.p_wheels.addLegend(offset=(10, 10))
        self.c_wL = self.p_wheels.plot(pen=pg.mkPen((150, 200, 255), width=2), name='wL')
        self.c_wR = self.p_wheels.plot(pen=pg.mkPen((255, 160, 0), width=2), name='wR')

        self.p_torque.addLegend(offset=(10, 10))
        self.c_tauL = self.p_torque.plot(pen=pg.mkPen((150, 200, 255), width=2), name='tauL')
        self.c_tauR = self.p_torque.plot(pen=pg.mkPen((255, 160, 0), width=2), name='tauR')

        # Optional command plot (hidden by default)
        self.p_cmd = None
        self.c_wL_cmd = None
        self.c_wR_cmd = None

        # Render timer (fast)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_ui)
        self.timer.start(8)  # ~120 FPS

        self.win.show()

    def _toggle_cmd_plot(self):
        enabled = self.chk_show_cmd.isChecked()
        if enabled and self.p_cmd is None:
            # Add as a new row spanning 2 columns by placing it under existing grid:
            self.plots_widget.nextRow()
            self.p_cmd = self.plots_widget.addPlot(title="Commands: wL_cmd / wR_cmd (rad/s)")
            self.plots_widget.addItem(pg.LabelItem(""), 0, 1)  # spacer (keeps layout consistent)

            self.p_cmd.showGrid(x=True, y=True, alpha=0.20)
            self.p_cmd.setLabel("bottom", "Sim Time (s)")
            self.p_cmd.setClipToView(True)
            self.p_cmd.setDownsampling(auto=True, mode="peak")
            self.p_cmd.setAutoVisible(y=True)

            self.p_cmd.addLegend(offset=(10, 10))
            self.c_wL_cmd = self.p_cmd.plot(pen=pg.mkPen((170, 170, 255), width=2), name='wL_cmd')
            self.c_wR_cmd = self.p_cmd.plot(pen=pg.mkPen((255, 190, 140), width=2), name='wR_cmd')

        if not enabled and self.p_cmd is not None:
            # simplest: clear curves (keep plot)
            self.c_wL_cmd.setData([], [])
            self.c_wR_cmd.setData([], [])

    # ===================== Parameter service monitor =====================
    def _start_param_monitor(self):
        # Start a Qt timer that periodically checks whether the parameter service is available.
        self._param_timer = QtCore.QTimer()
        self._param_timer.timeout.connect(self._check_param_service)
        self._param_timer.start(500)  # check every 500 ms

    def _check_param_service(self):
        # Check the ROS graph for the parameter service names instead of blocking on wait_for_service.
        # This is more reliable in a UI thread because it doesn't depend on spinning the parameter client.
        try:
            svc_list = [name for name, _ in self.get_service_names_and_types()]
        except Exception as e:
            self.get_logger().debug(f"Param monitor: failed to get service list: {e}")
            return

        self.get_logger().debug(f"Param monitor: services: {svc_list}")
        # Only accept the PID node parameter service; do not adopt tuner services.
        svc_name = f"/{self.pid_node}/get_parameters"
        if svc_name in svc_list:
            try:
                client = AsyncParameterClient(self, self.pid_node)
                self.param_client = client
                self.param_service_ready = True
                self.get_logger().info(f"Parameter service detected on '{self.pid_node}' via service list")

                # Only create node-scoped apply/reset clients on the PID node.
                apply_name = f'/{self.pid_node}/apply_gains'
                reset_name = f'/{self.pid_node}/reset_pid'
                pose_name = f'/{self.pid_node}/apply_pose'

                if apply_name in svc_list:
                    try:
                        self.apply_gains_cli = self.create_client(Trigger, apply_name)
                    except Exception:
                        self.apply_gains_cli = None
                else:
                    self.apply_gains_cli = None

                if reset_name in svc_list:
                    try:
                        self.reset_pid_cli = self.create_client(Trigger, reset_name)
                    except Exception:
                        self.reset_pid_cli = None
                else:
                    self.reset_pid_cli = None

                if pose_name in svc_list:
                    try:
                        self.apply_pose_cli = self.create_client(Trigger, pose_name)
                    except Exception:
                        self.apply_pose_cli = None
                else:
                    self.apply_pose_cli = None

                # enable buttons
                try:
                    self.btn_reload.setEnabled(True)
                    self.btn_apply.setEnabled(True)
                except Exception:
                    pass

                # stop monitoring once PID node available and fetch current gains
                self._param_timer.stop()
                self._request_current_gains()
                return
            except Exception as e:
                self.get_logger().debug(f"Param monitor: failed to create client for '{self.pid_node}': {e}")
                # keep trying
                return

        # none found this check; keep trying
        self.get_logger().debug("Param monitor: no parameter services found this tick")
        return

    # ===================== PID apply =====================
    def _request_current_gains(self):
        # Use the param_client set up by the monitor (already running in the executor).
        # The monitor ensures it's the PID node's client, so we don't re-check.
        if self.param_client is None:
            self.get_logger().warn(f"Parameter client not yet available for '{self.pid_node}'")
            return
        try:
            fut = self.param_client.get_parameters(["v_kp", "v_ki", "v_kd", "t_kp", "t_ki", "t_kd"])
            fut.add_done_callback(self._on_gains_received)
            self.get_logger().info(f"Requested gains from PID node")
        except Exception as e:
            self.get_logger().warn(f"Failed to request gains: {e}")

    def _on_gains_received(self, future):
        try:
            res = future.result()
            if res is None or not hasattr(res, 'values'):
                self.get_logger().warn(f"Invalid parameter response: {res}")
                return
            # ParameterValue objects store the value differently depending on type
            # Try to extract the actual value from each ParameterValue
            vals = []
            for p in res.values:
                if hasattr(p, 'double_value'):
                    vals.append(p.double_value)
                elif hasattr(p, 'integer_value'):
                    vals.append(p.integer_value)
                elif hasattr(p, 'string_value'):
                    vals.append(float(p.string_value))
                else:
                    # Try direct access
                    vals.append(float(p))
            
            if len(vals) < 6:
                self.get_logger().warn(f"Received {len(vals)} values, expected 6: {vals}")
                return
            self.get_logger().info(f"Loaded gains from PID node: v_kp={vals[0]:.4f}, v_ki={vals[1]:.4f}, v_kd={vals[2]:.4f}, t_kp={vals[3]:.4f}, t_ki={vals[4]:.4f}, t_kd={vals[5]:.4f}")
        except Exception as e:
            self.get_logger().warn(f"Failed to unpack parameter response: {e}")
            return

        self.sp_v_kp.setValue(float(vals[0]))
        self.sp_v_ki.setValue(float(vals[1]))
        self.sp_v_kd.setValue(float(vals[2]))
        self.sp_t_kp.setValue(float(vals[3]))
        self.sp_t_ki.setValue(float(vals[4]))
        self.sp_t_kd.setValue(float(vals[5]))

    def _apply_pid_clicked(self):
        # Only write parameters to the PID node via the monitor's param_client (already spinning).
        # Do NOT create new clients or fall back to tuner.
        if self.param_client is None:
            self.get_logger().warn(f"Parameter client not yet available for '{self.pid_node}'")
            return

        params = [
            Parameter("v_kp", Parameter.Type.DOUBLE, float(self.sp_v_kp.value())),
            Parameter("v_ki", Parameter.Type.DOUBLE, float(self.sp_v_ki.value())),
            Parameter("v_kd", Parameter.Type.DOUBLE, float(self.sp_v_kd.value())),
            Parameter("t_kp", Parameter.Type.DOUBLE, float(self.sp_t_kp.value())),
            Parameter("t_ki", Parameter.Type.DOUBLE, float(self.sp_t_ki.value())),
            Parameter("t_kd", Parameter.Type.DOUBLE, float(self.sp_t_kd.value())),
        ]

        self.get_logger().info(f"Applying parameters to PID node '{self.pid_node}'")
        fut = self.param_client.set_parameters(params)
        # After setting parameters, call apply/reset on the PID node only.
        fut.add_done_callback(lambda _: self._call_apply_and_reset(prefer_pid_node=True))

    def _call_apply_and_reset(self, prefer_pid_node: bool = False):
        # call /apply_gains then /reset_pid (non-blocking)
        try:
            # ensure apply_gains client exists
            # If caller prefers PID-node scoped services, try those first.
            prefer_pid = False
            # backward compatible: function may be called with a keyword
            try:
                # inspect the function call arguments via closure if any
                prefer_pid = False
            except Exception:
                prefer_pid = False

            # Try to create node-scoped clients on the PID node. Do not fall
            # back to tuner/global apply/reset to avoid overwriting from tuner.
            try:
                name_apply = f"/{self.pid_node}/apply_gains"
                name_reset = f"/{self.pid_node}/reset_pid"
                if self.apply_gains_cli is None:
                    self.apply_gains_cli = self.create_client(Trigger, name_apply)
                if self.reset_pid_cli is None:
                    self.reset_pid_cli = self.create_client(Trigger, name_reset)
            except Exception:
                # If PID-node scoped services are not available, don't call tuner/global ones
                self.get_logger().warn("PID-node scoped apply/reset not available; skipping service calls to avoid tuner overriding values.")
                return

            # Call services if ready
            if self.apply_gains_cli is not None:
                if not self.apply_gains_cli.service_is_ready():
                    self.apply_gains_cli.wait_for_service(timeout_sec=0.5)
                self.apply_gains_cli.call_async(Trigger.Request())

            if self.reset_pid_cli is not None:
                if not self.reset_pid_cli.service_is_ready():
                    self.reset_pid_cli.wait_for_service(timeout_sec=0.5)
                self.reset_pid_cli.call_async(Trigger.Request())
        except Exception:
            pass

    # ===================== UI update loop =====================
    def _update_ui(self):
        now = time.time()
        sim = self.sim_time

        # clock running?
        state = "NO /clock"
        if self._last_clock_wall is not None:
            age = now - self._last_clock_wall
            state = "PAUSED" if age > 0.5 else "RUNNING"

        self.lbl_status.setText(
            f"sim: {sim:8.3f} | hz(clock/debug/joint): "
            f"{self.r_clock.hz():6.1f}/{self.r_debug.hz():6.1f}/{self.r_joint.hz():6.1f} | state: {state}"
        )

        if not isfinite(sim):
            return

        # Set data (each signal independent timestamps -> supports different Hz)
        self.c_theta.setData(*self.theta.data(), connect="finite")
        self.c_theta_set.setData(*self.theta_set.data(), connect="finite")
        self.c_u.setData(*self.u.data(), connect="finite")

        self.c_vavg.setData(*self.v_avg.data(), connect="finite")
        self.c_vset.setData(*self.v_set.data(), connect="finite")

        self.c_wL.setData(*self.wL.data(), connect="finite")
        self.c_wR.setData(*self.wR.data(), connect="finite")

        self.c_tauL.setData(*self.tauL.data(), connect="finite")
        self.c_tauR.setData(*self.tauR.data(), connect="finite")

        if self.chk_show_cmd.isChecked() and self.p_cmd is not None:
            self.c_wL_cmd.setData(*self.wL_cmd.data(), connect="finite")
            self.c_wR_cmd.setData(*self.wR_cmd.data(), connect="finite")
            self.p_cmd.setXRange(sim - self.window_sec, sim, padding=0.0)

        # X range scrolling
        x0 = sim - self.window_sec
        for p in self.all_plots:
            p.setXRange(x0, sim, padding=0.0)

        # keep balance centered around 0
        vals = []
        vals.extend([v for v in self.theta.v if isfinite(v)])
        vals.extend([v for v in self.theta_set.v if isfinite(v)])
        vals.extend([v for v in self.u.v if isfinite(v)])
        if vals:
            m = max(abs(v) for v in vals)
            m = max(m, 0.01)
            self.p_balance.setYRange(-1.2 * m, 1.2 * m, padding=0.0)


def main():
    rclpy.init()
    node = ReflexControlLabUI()
    exe = SingleThreadedExecutor()
    exe.add_node(node)

    try:
        while rclpy.ok():
            exe.spin_once(timeout_sec=0.002)
            QtWidgets.QApplication.processEvents()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()