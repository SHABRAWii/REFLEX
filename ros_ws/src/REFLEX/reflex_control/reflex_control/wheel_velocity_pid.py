#!/usr/bin/env python3
import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    i_max: float
    out_max: float

    i_term: float = 0.0
    prev_err: float = 0.0
    initialized: bool = False

    def reset(self):
        self.i_term = 0.0
        self.prev_err = 0.0
        self.initialized = False

    def step(self, err: float, dt: float) -> float:
        if dt <= 0.0 or math.isinf(dt) or math.isnan(dt):
            return 0.0

        if not self.initialized:
            self.prev_err = err
            self.initialized = True

        # Integral with clamp (anti-windup)
        self.i_term += err * dt
        self.i_term = max(-self.i_max, min(self.i_max, self.i_term))

        # Derivative
        d_err = (err - self.prev_err) / dt
        self.prev_err = err

        out = self.kp * err + self.ki * self.i_term + self.kd * d_err
        out = max(-self.out_max, min(self.out_max, out))
        return out


class WheelVelocityPIDNode(Node):
    """
    Subscribes:
      - /wheel_velocity_setpoint (Float64MultiArray)  [left, right] rad/s
      - /joint_states (sensor_msgs/JointState)

    Publishes:
      - /wheel_velocity_controller/commands (Float64MultiArray) [left_cmd, right_cmd] rad/s
    """

    def __init__(self):
        super().__init__('wheel_velocity_pid')

        # Joint names must match your URDF exactly
        self.declare_parameter('left_joint', 'Left_Wheel_Joint')
        self.declare_parameter('right_joint', 'Right_Wheel_Joint')

        self.left_joint = self.get_parameter('left_joint').get_parameter_value().string_value
        self.right_joint = self.get_parameter('right_joint').get_parameter_value().string_value

        # PID params
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 2.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('i_max', 5.0)
        self.declare_parameter('out_max', 20.0)  # command clamp (rad/s)

        kp = float(self.get_parameter('kp').value)
        ki = float(self.get_parameter('ki').value)
        kd = float(self.get_parameter('kd').value)
        i_max = float(self.get_parameter('i_max').value)
        out_max = float(self.get_parameter('out_max').value)

        self.pid_l = PID(kp, ki, kd, i_max, out_max)
        self.pid_r = PID(kp, ki, kd, i_max, out_max)

        self.setpoint_l = 0.0
        self.setpoint_r = 0.0
        self.meas_l = 0.0
        self.meas_r = 0.0

        self.last_time = self.get_clock().now()

        self.sub_setpoint = self.create_subscription(
            Float64MultiArray, '/wheel_velocity_setpoint', self.on_setpoint, 10
        )
        self.sub_joint = self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 50
        )

        self.pub_cmd = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10
        )

        self.declare_parameter('rate_hz', 250.0)
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)

        self.get_logger().info(
            f"WheelVelocityPID running. left_joint={self.left_joint}, right_joint={self.right_joint}"
        )

    def on_setpoint(self, msg: Float64MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn("wheel_velocity_setpoint needs 2 values: [left, right]")
            return
        self.setpoint_l = float(msg.data[0])
        self.setpoint_r = float(msg.data[1])

    def on_joint_state(self, msg: JointState):
        # Find joint velocities by name
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        if self.left_joint in name_to_idx:
            i = name_to_idx[self.left_joint]
            if i < len(msg.velocity):
                self.meas_l = float(msg.velocity[i])
        if self.right_joint in name_to_idx:
            i = name_to_idx[self.right_joint]
            if i < len(msg.velocity):
                self.meas_r = float(msg.velocity[i])

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        err_l = self.setpoint_l - self.meas_l
        err_r = self.setpoint_r - self.meas_r

        cmd_l = self.pid_l.step(err_l, dt)
        cmd_r = self.pid_r.step(err_r, dt)

        out = Float64MultiArray()
        out.data = [cmd_l, cmd_r]
        self.pub_cmd.publish(out)


def main():
    rclpy.init()
    node = WheelVelocityPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()