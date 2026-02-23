#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger


class ResetOnWorldReset(Node):
    """
    Detect Gazebo world reset by watching /clock go backwards, then call /reset (Trigger).
    """

    def __init__(self):
        super().__init__('reflex_reset_on_world_reset')

        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('reset_service', '/reset')

        # Don't trigger immediately at sim start (0 -> small).
        self.declare_parameter('arm_after_sec', 1.0)

        # How big a backwards jump counts as a reset (seconds).
        self.declare_parameter('backward_jump_sec', 0.2)

        # Optional: debounce so multiple clock messages during reset don't spam
        self.declare_parameter('cooldown_sec', 1.0)

        self.clock_topic = self.get_parameter('clock_topic').value
        self.reset_service = self.get_parameter('reset_service').value
        self.arm_after_sec = float(self.get_parameter('arm_after_sec').value)
        self.backward_jump_sec = float(self.get_parameter('backward_jump_sec').value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)

        self.last_t = None
        self.armed = False
        self.last_reset_wall_time = 0.0

        self.client = self.create_client(Trigger, self.reset_service)
        self.sub = self.create_subscription(Clock, self.clock_topic, self.on_clock, 50)

        self.get_logger().info(
            f"Listening on {self.clock_topic}; will call {self.reset_service} on world reset "
            f"(arm_after={self.arm_after_sec}s, backward_jump>{self.backward_jump_sec}s)"
        )

    def on_clock(self, msg: Clock):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9

        # Arm only after simulation has progressed a bit
        if not self.armed and t >= self.arm_after_sec:
            self.armed = True
            self.get_logger().info("Armed: will detect /clock backward jumps as world reset.")

        if self.last_t is None:
            self.last_t = t
            return

        # Detect backwards jump
        if self.armed and (t < self.last_t - self.backward_jump_sec):
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_reset_wall_time >= self.cooldown_sec:
                self.last_reset_wall_time = now
                self.get_logger().warn(
                    f"World reset detected: /clock jumped from {self.last_t:.3f} -> {t:.3f}. Calling {self.reset_service}."
                )
                self.call_reset()

        self.last_t = t

    def call_reset(self):
        if not self.client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"{self.reset_service} not available.")
            return

        req = Trigger.Request()
        fut = self.client.call_async(req)

        # no blocking spin here; just log when ready via callback
        fut.add_done_callback(self._on_reset_done)

    def _on_reset_done(self, fut):
        try:
            resp = fut.result()
            self.get_logger().info(f"/reset -> success={resp.success}, msg='{resp.message}'")
        except Exception as e:
            self.get_logger().error(f"/reset call failed: {e}")


def main():
    rclpy.init()
    node = ResetOnWorldReset()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()