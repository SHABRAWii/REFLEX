#!/usr/bin/env python3
import math
import subprocess
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter_client import AsyncParameterClient

def rpy_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """Convert RPY degrees -> quaternion (x,y,z,w)."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)

    # ROS standard (roll X, pitch Y, yaw Z)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class ReflexPidTuner(Node):
    """
    One node for:
      - Setting Gazebo model pose (teleport) using gz service.
      - Updating /reflex_balance_pid gains via ROS params.
      - Calling /reflex_balance_pid/reset (Trigger) to reset PID state.

    Services provided by this node:
      - /reflex_pid_tuner/apply_pose   (Trigger)
      - /reflex_pid_tuner/apply_gains  (Trigger)
      - /reflex_pid_tuner/reset_pid    (Trigger)  -> calls reflex_balance_pid reset srv

    Optional:
      - apply_pose_on_start (bool)
      - apply_gains_on_start (bool)
    """

    def __init__(self):
        super().__init__("reflex_pid_tuner")

        # ---- Gazebo pose params ----
        self.declare_parameter("gz_world", "default")
        self.declare_parameter("model_name", "reflex")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 1.0)

        # degrees
        self.declare_parameter("roll_deg", 0.0)
        self.declare_parameter("pitch_deg", 0.0)
        self.declare_parameter("yaw_deg", 0.0)

        self.declare_parameter("apply_pose_on_start", False)

        # ---- PID gain params (these will be written to /reflex_balance_pid) ----
        self.declare_parameter("v_kp", 0.6)
        self.declare_parameter("v_ki", 0.3)
        self.declare_parameter("v_kd", 0.0)
        self.declare_parameter("v_i_max", 0.3)
        self.declare_parameter("v_out_max", 0.25)

        self.declare_parameter("t_kp", 25.0)
        self.declare_parameter("t_ki", 0.0)
        self.declare_parameter("t_kd", 1.2)
        self.declare_parameter("t_i_max", 5.0)
        self.declare_parameter("t_out_max", 40.0)

        self.declare_parameter("apply_gains_on_start", False)

        # ---- Target PID node name ----
        self.declare_parameter("pid_node", "/reflex_balance_pid")
        self.pid_node = str(self.get_parameter("pid_node").value)

        # Parameter client to the PID node
        self.pid_param_client = AsyncParameterClient(self, self.pid_node)

        # Service client to reset PID node (you must add this service in reflex_balance_pid; see note below)
        self.reset_cli = self.create_client(Trigger, f"{self.pid_node}/reset")

        # Services exposed by tuner
        self.create_service(Trigger, "apply_pose", self.srv_apply_pose)
        self.create_service(Trigger, "apply_gains", self.srv_apply_gains)
        self.create_service(Trigger, "reset_pid", self.srv_reset_pid)

        # Optional auto-apply
        if bool(self.get_parameter("apply_pose_on_start").value):
            self.create_timer(0.2, self._apply_pose_once)

        if bool(self.get_parameter("apply_gains_on_start").value):
            self.create_timer(0.2, self._apply_gains_once)

        self.get_logger().info(
            "ReflexPidTuner ready.\n"
            "Services:\n"
            "  /reflex_pid_tuner/apply_pose  (Trigger)\n"
            "  /reflex_pid_tuner/apply_gains (Trigger)\n"
            "  /reflex_pid_tuner/reset_pid   (Trigger)\n"
        )

        self._pose_applied = False
        self._gains_applied = False

    # ----------------------- Pose control -----------------------
    def _apply_pose_once(self):
        if self._pose_applied:
            return
        ok, msg = self.apply_pose()
        if ok:
            self._pose_applied = True
        self.get_logger().info(msg)

    def apply_pose(self):
        world = str(self.get_parameter("gz_world").value)
        model = str(self.get_parameter("model_name").value)

        x = float(self.get_parameter("x").value)
        y = float(self.get_parameter("y").value)
        z = float(self.get_parameter("z").value)

        roll_deg = float(self.get_parameter("roll_deg").value)
        pitch_deg = float(self.get_parameter("pitch_deg").value)
        yaw_deg = float(self.get_parameter("yaw_deg").value)

        qx, qy, qz, qw = rpy_deg_to_quat(roll_deg, pitch_deg, yaw_deg)

        # Try /world/<world>/set_pose first (common in Gazebo Harmonic / gz)
        # If your setup uses a different service name, we’ll adapt (you can check `gz service -l`)
        service = f"/world/{world}/set_pose"

        req = (
            f'name: "{model}" '
            f'position: {{x: {x}, y: {y}, z: {z}}} '
            f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        )

        cmd = [
            "gz", "service",
            "-s", service,
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", req
        ]

        try:
            r = subprocess.run(cmd, capture_output=True, text=True, check=False)
        except FileNotFoundError:
            return False, "ERROR: `gz` command not found in PATH (is Gazebo/gz installed in this environment?)."

        if r.returncode != 0:
            err = (r.stderr or r.stdout or "").strip()
            return False, (
                f"Failed to set pose using {service}.\n"
                f"Command: {' '.join(cmd)}\n"
                f"Output: {err}\n"
                f"Tip: run `gz service -l | grep pose` to find the correct service name."
            )

        return True, (
            f"Pose applied to model='{model}' in world='{world}': "
            f"x={x:.3f}, y={y:.3f}, z={z:.3f}, "
            f"rpy(deg)=({roll_deg:.1f},{pitch_deg:.1f},{yaw_deg:.1f})"
        )

    def srv_apply_pose(self, request, response):
        ok, msg = self.apply_pose()
        response.success = bool(ok)
        response.message = msg
        return response

    # ----------------------- PID gains control -----------------------
    def _apply_gains_once(self):
        if self._gains_applied:
            return
        ok, msg = self.apply_gains()
        if ok:
            self._gains_applied = True
        self.get_logger().info(msg)

    def apply_gains(self):
        if not self.pid_param_client.service_is_ready():
            # don’t block forever; return clean error
            return False, f"PID param service not ready for {self.pid_node} (wait a bit and try again)."

        # Read our own params and push them to the PID node
        names = [
            "v_kp", "v_ki", "v_kd", "v_i_max", "v_out_max",
            "t_kp", "t_ki", "t_kd", "t_i_max", "t_out_max",
        ]
        params: List[Parameter] = []
        for n in names:
            params.append(Parameter(n, value=float(self.get_parameter(n).value)))

        fut = self.pid_param_client.set_parameters(params)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.done():
            return False, "Timed out setting PID parameters."

        result_list = fut.result()
        # result_list is a list of SetParametersResult
        bad = [r.reason for r in result_list if not r.successful]
        if bad:
            return False, "Some parameters were rejected by PID node: " + "; ".join(bad)

        return True, (
            f"Gains applied to {self.pid_node}: "
            f"v(kp,ki,kd)=({self.get_parameter('v_kp').value},"
            f"{self.get_parameter('v_ki').value},{self.get_parameter('v_kd').value}) "
            f"t(kp,ki,kd)=({self.get_parameter('t_kp').value},"
            f"{self.get_parameter('t_ki').value},{self.get_parameter('t_kd').value})"
        )

    def srv_apply_gains(self, request, response):
        ok, msg = self.apply_gains()
        response.success = bool(ok)
        response.message = msg
        return response

    # ----------------------- Reset PID -----------------------
    def srv_reset_pid(self, request, response):
        if not self.reset_cli.service_is_ready():
            response.success = False
            response.message = (
                f"Reset service not available: {self.pid_node}/reset\n"
                f"Add a Trigger service in reflex_balance_pid to support reset."
            )
            return response

        fut = self.reset_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.done():
            response.success = False
            response.message = "Timed out calling PID reset."
            return response

        res = fut.result()
        response.success = bool(res.success)
        response.message = str(res.message)
        return response


def main():
    rclpy.init()
    node = ReflexPidTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()