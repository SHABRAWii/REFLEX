#!/usr/bin/env python3

"""
JOINT VELOCITY NODE (LOW-LEVEL ACTUATION LAYER)

This node is the lowest control layer that we implement for the robot.

ROLE IN ARCHITECTURE:
---------------------
This node acts as the bridge between:
    - High-level controllers (which output desired joint velocities qdot_des)
    - The simulator (Gazebo) or real hardware (ODrive)

It does NOT perform control logic like balance, IK, or PID.
It ONLY:
    1) Sends velocity commands to actuators
    2) Reads joint feedback
    3) Fixes joint position wrapping (critical for encoder-like behavior)

WHY THIS NODE EXISTS:
---------------------
Gazebo provides joint positions in the range [-pi, pi], which "wraps" when the joint rotates.
Real encoders do NOT wrap — they keep increasing or decreasing continuously.

Example:
    Gazebo:  3.13 → -3.13  (WRAP!)
    Real:     3.13 → 3.15   (CONTINUOUS)

This node "unwraps" the angle so higher-level controllers behave correctly.

DATA FLOW:
----------
INPUT:
    /joint_velocity_cmd   → desired joint velocities (qdot_des)

OUTPUT:
    /joint_group_velocity_controller/commands → sent to Gazebo

FEEDBACK:
    /joint_states        → raw Gazebo data
    /joint_states_out    → cleaned + unwrapped data

IN SHORT:
---------
This node = "Actuator Interface Layer"
- executes velocity
- provides clean state
- hides simulation quirks

It allows the rest of the system to work IDENTICALLY for:
    - Gazebo
    - Real robot (ODrive)

"""

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def unwrap_angle(q, q_prev):
    dq = q - q_prev
    if dq > math.pi:
        dq -= 2 * math.pi
    elif dq < -math.pi:
        dq += 2 * math.pi
    return q_prev + dq


class JointVelocityNode(Node):
    def __init__(self):
        super().__init__('joint_velocity_node')

        # ===== PARAMS =====
        self.declare_parameter('joint_names', [
            'Left_Hip_joint', 'Left_Knee_joint', 'Right_Hip_Joint', 'Right_Knee_joint'
        ])
        self.joint_names = self.get_parameter('joint_names').value

        self.n = len(self.joint_names)

        # ===== OFFSETS =====
        self.offset = [
            135 / 180 * math.pi, 
            (180-71) / 180 * math.pi, 
            0,
            135 / 180 * math.pi, 
            (180-71) / 180 * math.pi,
            0,
        ]
        
        # ===== STATE =====
        self.q = [0.0] * self.n
        self.qdot = [0.0] * self.n
        self.q_unwrapped = [0.0] * self.n
        self.prev_q_raw = [0.0] * self.n
        self.first = True

        # ===== SUB =====
        self.sub_cmd = self.create_subscription(
            Float64MultiArray,
            '/joint_velocity_cmd',
            self.on_cmd,
            10
        )

        self.sub_js = self.create_subscription(
            JointState,
            '/joint_states',
            self.on_joint_states,
            10
        )

        # ===== PUB =====
        self.pub_cmd = self.create_publisher(
            Float64MultiArray,
            '/leg_velocity_controller/commands',
            10
        )

        self.pub_state = self.create_publisher(
            JointState,
            '/joint_states_out',
            10
        )

        self.get_logger().info("Joint Velocity Node started")

    # =========================
    # Receive velocity command
    # =========================
    def on_cmd(self, msg: Float64MultiArray):
        if len(msg.data) != self.n:
            self.get_logger().warn("Invalid velocity command size")
            return

        out = Float64MultiArray()
        out.data = msg.data
        self.pub_cmd.publish(out)

    # =========================
    # Read joint states
    # =========================
    def on_joint_states(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        for i, jn in enumerate(self.joint_names):
            if jn not in name_to_idx:
                continue

            idx = name_to_idx[jn]

            q_raw = msg.position[idx] + self.offset[idx] if idx < len(msg.position) else 0.0
            qdot = msg.velocity[idx] if idx < len(msg.velocity) else 0.0

            # unwrap
            if self.first:
                self.q_unwrapped[i] = q_raw
                self.prev_q_raw[i] = q_raw
            else:
                self.q_unwrapped[i] = unwrap_angle(q_raw, self.prev_q_raw[i])
                self.prev_q_raw[i] = q_raw

            self.q[i] = self.q_unwrapped[i]
            self.qdot[i] = qdot

        self.first = False

        # publish clean state
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.q
        js.velocity = self.qdot

        self.pub_state.publish(js)


def main():
    rclpy.init()
    node = JointVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()