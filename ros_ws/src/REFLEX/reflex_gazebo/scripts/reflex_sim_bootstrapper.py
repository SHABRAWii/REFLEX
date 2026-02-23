#!/usr/bin/env python3
import time
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger, Empty

# Gazebo Sim (gz-sim / Harmonic) service type
try:
    from gz_msgs.srv import ControlWorld  # type: ignore
    HAS_GZ = True
except Exception:
    HAS_GZ = False


class ReflexSimBootstrapper(Node):
    """
    Bootstraps sim after reset:
      - ensure controllers active
      - call /reset (Trigger)
      - unpause simulation (gz-sim or gazebo classic)
    """

    def __init__(self):
        super().__init__("reflex_sim_bootstrapper")

        # Params
        self.declare_parameter("joint_state_controller", "joint_state_broadcaster")
        self.declare_parameter("diff_drive_controller", "diff_drive_controller")
        self.declare_parameter("reset_service", "/reset")

        # Gazebo Sim (gz-sim) world control service (override in launch)
        self.declare_parameter("gz_world_control_service", "/world/reflex_world/control")

        # Classic gazebo unpause service
        self.declare_parameter("classic_unpause_service", "/unpause_physics")

        # Behavior
        self.declare_parameter("run_once", True)          # do once at startup
        self.declare_parameter("retry_seconds", 0.5)      # retry interval
        self.declare_parameter("timeout_seconds", 60.0)   # overall timeout

        # “Detect reset” trigger topic/service (optional):
        # If you want an external component to trigger re-bootstrap, call this service.
        self.srv_rebootstrap = self.create_service(Trigger, "~/rebootstrap", self._on_rebootstrap)

        self._busy = False

        if self.get_parameter("run_once").value:
            # kick off automatically
            self.create_timer(0.2, self._startup_once)

    def _startup_once(self):
        # run only once then destroy timer by returning without scheduling again
        if self._busy:
            return
        self._busy = True
        self.get_logger().info("Bootstrapping: activating controllers -> /reset -> unpause sim")
        ok = self._do_bootstrap()
        if ok:
            self.get_logger().info("Bootstrap complete ✅")
        else:
            self.get_logger().error("Bootstrap failed ❌ (see logs)")
        # stop this timer by destroying all timers (simple approach)
        for t in list(self.timers):
            try:
                self.destroy_timer(t)
            except Exception:
                pass
        self._busy = False

    def _on_rebootstrap(self, req: Trigger.Request, res: Trigger.Response):
        if self._busy:
            res.success = False
            res.message = "Already bootstrapping; try again."
            return res
        self._busy = True
        ok = self._do_bootstrap()
        res.success = ok
        res.message = "Bootstrap complete" if ok else "Bootstrap failed"
        self._busy = False
        return res

    def _do_bootstrap(self) -> bool:
        joint = self.get_parameter("joint_state_controller").value
        diff = self.get_parameter("diff_drive_controller").value
        reset_srv = self.get_parameter("reset_service").value

        retry = float(self.get_parameter("retry_seconds").value)
        timeout = float(self.get_parameter("timeout_seconds").value)
        deadline = time.time() + timeout

        # 1) wait for controller manager to respond
        if not self._wait_for_controller_manager(deadline, retry):
            return False

        # 2) load + activate controllers (idempotent-ish)
        if not self._ensure_controller_active(joint, deadline, retry):
            return False
        if not self._ensure_controller_active(diff, deadline, retry):
            return False

        # 3) call /reset
        if not self._call_trigger(reset_srv, deadline, retry):
            return False

        # 4) unpause/start simulation
        if not self._unpause_sim(deadline, retry):
            self.get_logger().warn("Could not unpause simulation (service not found). Continuing anyway.")
        return True

    def _wait_for_controller_manager(self, deadline: float, retry: float) -> bool:
        self.get_logger().info("Waiting for controller_manager...")
        while time.time() < deadline:
            # controller_manager typically offers these services:
            # /controller_manager/list_controllers
            # we can just try `ros2 control list_controllers` via subprocess (simple & robust)
            rc, out = self._run(["ros2", "control", "list_controllers"])
            if rc == 0:
                return True
            time.sleep(retry)
        self.get_logger().error("Timed out waiting for controller_manager.")
        return False

    def _ensure_controller_active(self, name: str, deadline: float, retry: float) -> bool:
        self.get_logger().info(f"Ensuring controller active: {name}")
        while time.time() < deadline:
            rc, out = self._run(["ros2", "control", "list_controllers"])
            if rc == 0 and name in out:
                # If it exists, check state
                # output lines look like: "<name>\t<type>\t<state>"
                for line in out.splitlines():
                    if line.startswith(name + "\t") or line.startswith(name + " "):
                        # state is usually last token
                        state = line.split()[-1].strip()
                        if state == "active":
                            return True
                # exists but not active -> activate
                self._run(["ros2", "control", "set_controller_state", name, "active"])
            else:
                # doesn't exist -> load + set active (your exact command)
                self._run(["ros2", "control", "load_controller", "--set-state", "active", name])

            time.sleep(retry)

        self.get_logger().error(f"Timed out activating controller: {name}")
        return False

    def _call_trigger(self, srv_name: str, deadline: float, retry: float) -> bool:
        self.get_logger().info(f"Calling reset service: {srv_name}")
        client = self.create_client(Trigger, srv_name)
        while time.time() < deadline and not client.wait_for_service(timeout_sec=retry):
            self.get_logger().info(f"Waiting for {srv_name} ...")
        if not client.service_is_ready():
            self.get_logger().error(f"Reset service not available: {srv_name}")
            return False

        req = Trigger.Request()
        fut = client.call_async(req)

        # spin until complete or timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if fut.done():
                try:
                    resp = fut.result()
                    self.get_logger().info(f"/reset response: success={resp.success} msg='{resp.message}'")
                    return bool(resp.success)
                except Exception as e:
                    self.get_logger().error(f"/reset call failed: {e}")
                    return False

        self.get_logger().error("Timed out waiting for /reset response.")
        return False

    def _unpause_sim(self, deadline: float, retry: float) -> bool:
        # Try Gazebo Sim (gz-sim) first
        if HAS_GZ:
            gz_srv = self.get_parameter("gz_world_control_service").value
            self.get_logger().info(f"Trying gz-sim unpause via: {gz_srv}")
            gz_client = self.create_client(ControlWorld, gz_srv)
            if gz_client.wait_for_service(timeout_sec=1.0):
                req = ControlWorld.Request()
                # For gz_msgs/srv/ControlWorld: world_control has fields incl. pause (bool)
                req.world_control.pause = False
                fut = gz_client.call_async(req)
                while time.time() < deadline and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if fut.done():
                        _ = fut.result()
                        self.get_logger().info("gz-sim unpaused ✅")
                        return True
            else:
                self.get_logger().warn("gz-sim ControlWorld service not available.")

        # Fallback: classic gazebo_ros
        classic_srv = self.get_parameter("classic_unpause_service").value
        self.get_logger().info(f"Trying classic gazebo unpause via: {classic_srv}")
        cl_client = self.create_client(Empty, classic_srv)
        if cl_client.wait_for_service(timeout_sec=1.0):
            fut = cl_client.call_async(Empty.Request())
            while time.time() < deadline and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                if fut.done():
                    _ = fut.result()
                    self.get_logger().info("Classic gazebo unpaused ✅")
                    return True

        return False

    def _run(self, cmd) -> tuple[int, str]:
        try:
            p = subprocess.run(cmd, capture_output=True, text=True)
            out = (p.stdout or "") + (p.stderr or "")
            if p.returncode != 0:
                # log at debug-ish level
                self.get_logger().debug(f"Command failed ({p.returncode}): {' '.join(cmd)}\n{out}")
            return p.returncode, out
        except Exception as e:
            self.get_logger().debug(f"Command exception: {' '.join(cmd)} -> {e}")
            return 1, str(e)


def main():
    rclpy.init()
    node = ReflexSimBootstrapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()