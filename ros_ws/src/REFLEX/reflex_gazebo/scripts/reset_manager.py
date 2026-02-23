#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import ListControllers, LoadController, SwitchController
from std_srvs.srv import Trigger


class ControllerWatchdog(Node):
    def __init__(self):
        super().__init__("reflex_controller_watchdog")

        # DO NOT declare use_sim_time here (launch injects it)
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("controllers", ["joint_state_broadcaster", "diff_drive_controller"])
        self.declare_parameter("reset_service", "/reset")

        self.declare_parameter("check_period_s", 0.5)
        self.declare_parameter("service_wait_s", 0.2)
        self.declare_parameter("call_reset_after_recover", True)
        self.declare_parameter("log_throttle_s", 1.0)

        self.cm = self.get_parameter("controller_manager").value.rstrip("/")
        self.controllers = list(self.get_parameter("controllers").value)
        self.reset_srv_name = self.get_parameter("reset_service").value

        self.check_period_s = float(self.get_parameter("check_period_s").value)
        self.service_wait_s = float(self.get_parameter("service_wait_s").value)
        self.call_reset_after_recover = bool(self.get_parameter("call_reset_after_recover").value)
        self.log_throttle_s = float(self.get_parameter("log_throttle_s").value)

        self.cli_list = self.create_client(ListControllers, f"{self.cm}/list_controllers")
        self.cli_load = self.create_client(LoadController, f"{self.cm}/load_controller")
        self.cli_switch = self.create_client(SwitchController, f"{self.cm}/switch_controller")
        self.cli_reset = self.create_client(Trigger, self.reset_srv_name)

        self._busy = False
        self._was_healthy = False
        self._reset_called = False
        self._last_log_wall = 0.0

        self.timer = self.create_timer(self.check_period_s, self._tick)
        self.get_logger().info(
            f"Watchdog started. cm={self.cm}, controllers={self.controllers}, reset={self.reset_srv_name}"
        )

    def _tlog(self, level: str, msg: str):
        now = time.time()
        if now - self._last_log_wall >= self.log_throttle_s:
            self._last_log_wall = now
            getattr(self.get_logger(), level)(msg)

    def _services_ready(self) -> bool:
        ok = True
        ok &= self.cli_list.wait_for_service(timeout_sec=self.service_wait_s)
        ok &= self.cli_load.wait_for_service(timeout_sec=self.service_wait_s)
        ok &= self.cli_switch.wait_for_service(timeout_sec=self.service_wait_s)
        return ok

    def _tick(self):
        if self._busy:
            return

        if not self._services_ready():
            self._was_healthy = False
            self._reset_called = False
            self._tlog("warn", f"Waiting for {self.cm} services...")
            return

        self._busy = True
        fut = self.cli_list.call_async(ListControllers.Request())
        fut.add_done_callback(self._on_list_done)

    def _on_list_done(self, fut):
        self._busy = False
        try:
            res = fut.result()
        except Exception as e:
            self._was_healthy = False
            self._reset_called = False
            self._tlog("warn", f"list_controllers failed (cm restarting?): {e}")
            return

        state = {c.name: c.state for c in res.controller}
        missing = [c for c in self.controllers if c not in state]
        inactive = [c for c in self.controllers if state.get(c) != "active"]
        healthy = (not missing) and (not inactive)

        if healthy:
            if not self._was_healthy:
                self.get_logger().info("Controllers healthy (loaded + active).")
                if self.call_reset_after_recover and not self._reset_called:
                    self._call_reset_async()
            self._was_healthy = True
            return

        self._was_healthy = False
        self._reset_called = False

        if missing:
            self._load_one_async(missing[0])
            return

        if inactive:
            self._activate_all_async()
            return

    def _load_one_async(self, name: str):
        if not self.cli_load.service_is_ready():
            self._tlog("warn", f"Waiting for {self.cm}/load_controller ...")
            return

        self.get_logger().warn(f"Controller missing -> loading: {name}")
        req = LoadController.Request()
        req.name = name

        self._busy = True
        fut = self.cli_load.call_async(req)
        fut.add_done_callback(lambda f: self._on_load_done(name, f))

    def _on_load_done(self, name: str, fut):
        self._busy = False
        try:
            res = fut.result()
        except Exception as e:
            self._tlog("warn", f"load_controller({name}) failed: {e}")
            return

        if res.ok:
            self.get_logger().info(f"Loaded: {name}")
        else:
            self.get_logger().error(f"Failed to load: {name}")

    def _activate_all_async(self):
        if not self.cli_switch.service_is_ready():
            self._tlog("warn", f"Waiting for {self.cm}/switch_controller ...")
            return

        self.get_logger().warn(f"Controllers not active -> activating: {self.controllers}")

        req = SwitchController.Request()
        req.activate_controllers = list(self.controllers)
        req.deactivate_controllers = []
        req.strictness = SwitchController.Request.STRICT

        # Jazzy-safe: only set fields that exist in your interface version
        if hasattr(req, "start_asap"):
            req.start_asap = True
        if hasattr(req, "timeout"):
            req.timeout.sec = 2
            req.timeout.nanosec = 0

        self._busy = True
        fut = self.cli_switch.call_async(req)
        fut.add_done_callback(self._on_switch_done)

    def _on_switch_done(self, fut):
        self._busy = False
        try:
            res = fut.result()
        except Exception as e:
            self._tlog("warn", f"switch_controller failed: {e}")
            return

        if res.ok:
            self.get_logger().info("Activated controllers.")
        else:
            self.get_logger().error("Failed to activate controllers.")

    def _call_reset_async(self):
        if not self.cli_reset.wait_for_service(timeout_sec=self.service_wait_s):
            self._tlog("warn", f"Waiting for reset service {self.reset_srv_name} ...")
            return

        self.get_logger().info("Calling /reset ...")
        self._busy = True
        fut = self.cli_reset.call_async(Trigger.Request())
        fut.add_done_callback(self._on_reset_done)

    def _on_reset_done(self, fut):
        self._busy = False
        try:
            res = fut.result()
        except Exception as e:
            self._tlog("warn", f"/reset failed: {e}")
            return

        self._reset_called = True
        if res.success:
            self.get_logger().info(f"/reset OK: {res.message}")
        else:
            self.get_logger().warn(f"/reset returned success=False: {res.message}")


def main():
    rclpy.init()
    node = ControllerWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()