#!/usr/bin/env python3
import math
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray

from tf2_ros import Buffer, TransformListener


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

        self.wL = 0.0
        self.wR = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # debug: [v_avg, v_set, theta, theta_set, u, wL_cmd, wR_cmd]
        self.debug = [float('nan')] * 7

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(JointState, '/joint_states', self.on_js, 50)
        self.create_subscription(Imu, '/imu', self.on_imu, 50)
        self.create_subscription(Float64MultiArray, '/reflex_balance/debug', self.on_debug, 10)

        self.timer = self.create_timer(0.1, self.render)  # 10 Hz refresh

        # clear screen once
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

    def on_js(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if self.left_joint in idx:
            i = idx[self.left_joint]
            if i < len(msg.velocity): self.wL = float(msg.velocity[i])
        if self.right_joint in idx:
            i = idx[self.right_joint]
            if i < len(msg.velocity): self.wR = float(msg.velocity[i])

    def on_imu(self, msg: Imu):
        r, p, y = quat_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.roll, self.pitch, self.yaw = r, p, y

    def on_debug(self, msg: Float64MultiArray):
        if len(msg.data) >= 7:
            self.debug = list(msg.data[:7])

    def render(self):
        # TF lookup
        pos = (float('nan'), float('nan'), float('nan'))
        rpy = (float('nan'), float('nan'), float('nan'))
        try:
            t = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame, rclpy.time.Time())
            pos = (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
            q = t.transform.rotation
            rpy = quat_to_euler(q.x, q.y, q.z, q.w)
        except Exception:
            pass

        v_avg, v_set, theta, theta_set, u, wL_cmd, wR_cmd = self.debug

        # overwrite screen in-place
        sys.stdout.write("\033[H")  # cursor home
        sys.stdout.write("REFLEX Status Dashboard\n")
        sys.stdout.write("=" * 60 + "\n")
        sys.stdout.write(f"Wheels (rad/s):  wL={self.wL:8.3f}   wR={self.wR:8.3f}\n")
        sys.stdout.write(f"IMU RPY (deg):   roll={math.degrees(self.roll):7.2f}  pitch={math.degrees(self.pitch):7.2f}  yaw={math.degrees(self.yaw):7.2f}\n")
        sys.stdout.write("-" * 60 + "\n")
        sys.stdout.write(f"PID v_avg (m/s): v={v_avg:8.3f}   v_set={v_set:8.3f}\n")
        sys.stdout.write(f"PID theta (deg): theta={math.degrees(theta):7.2f}  theta_set={math.degrees(theta_set):7.2f}\n")
        sys.stdout.write(f"Inner output u:  u={u:8.3f}  cmds: wL_cmd={wL_cmd:8.3f}  wR_cmd={wR_cmd:8.3f}\n")
        sys.stdout.write("-" * 60 + "\n")
        sys.stdout.write(f"Base pose (world): x={pos[0]:7.3f} y={pos[1]:7.3f} z={pos[2]:7.3f}\n")
        sys.stdout.write(f"Base rpy  (deg):   r={math.degrees(rpy[0]):7.2f} p={math.degrees(rpy[1]):7.2f} y={math.degrees(rpy[2]):7.2f}\n")
        sys.stdout.write("=" * 60 + "\n")
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