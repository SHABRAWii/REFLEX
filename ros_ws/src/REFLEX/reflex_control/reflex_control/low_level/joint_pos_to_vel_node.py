#!/usr/bin/env python3
"""
JOINT POSITION TO VELOCITY NODE (LOW-LEVEL JOINT TRACKING LAYER)

This node converts desired joint angles into desired joint velocities.

ARCHITECTURE ROLE:
------------------
This node sits above the pure velocity execution layer.

Higher layers (IK, height control, roll control, posture control) will produce:
    q_des = desired joint angles

This node converts those desired angles into:
    qdot_des = desired joint velocities

Those velocities are then sent to the low-level velocity execution node,
which publishes them to the Gazebo ros2_control velocity controller.

WHY THIS NODE EXISTS:
---------------------
The simulator and hardware interface are configured for VELOCITY COMMANDS,
not position commands.

So instead of commanding position directly, we use a cascaded structure:

    desired joint position
        ↓
    position-to-velocity controller  (this node)
        ↓
    velocity execution layer         (Step 1)
        ↓
    ros2_control / ODrive / Gazebo

CONTROL LAW:
------------
For each joint:

    e = q_des - q

    qdot_des = Kp * e + Ki * integral(e) - Kd * qdot

where:
    q_des   = desired joint angle
    q       = measured joint angle
    qdot    = measured joint velocity

TERM MEANING:
-------------
- Kp:
    Pushes the joint toward the target angle.
    Larger error -> larger velocity command.

- Ki:
    Slowly removes steady-state error.
    Useful when friction, gravity, or bias prevents the joint from reaching
    the exact target.

- Kd:
    Damps motion using measured velocity.
    Helps reduce overshoot and oscillation.

WHY Ki MUST BE USED CAREFULLY:
------------------------------
Integral action can improve final settling near the target, but if made too large,
it can accumulate error (windup), cause overshoot, and make the robot unstable.

So Ki should start small and always be clamped.

DATA FLOW:
----------
INPUTS:
    /joint_position_cmd   -> desired joint positions
    /joint_states_out     -> measured q and qdot from Step 1

OUTPUT:
    /joint_velocity_cmd   -> desired joint velocities

IN SHORT:
---------
This node answers:
    "Given the joint angle I want, what velocity should I command right now?"
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class JointPosToVelNode(Node):
    def __init__(self):
        super().__init__('joint_pos_to_vel_node')

        # ===== PARAMS =====
        self.declare_parameter('joint_names', [
            'Left_Hip_joint',
            'Left_Knee_joint',
            'Right_Hip_Joint',
            'Right_Knee_joint'
        ])
        self.declare_parameter('kp', 6.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('i_max', 1.0)
        self.declare_parameter('vel_limit', 10.0)
        
        self.declare_parameter('home_position', [2.35619, 1.9024, 2.35619, 1.9024])

        self.joint_names = self.get_parameter('joint_names').value
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.i_max = float(self.get_parameter('i_max').value)
        self.vel_limit = float(self.get_parameter('vel_limit').value)

        self.n = len(self.joint_names)

        # ===== STATE =====
        self.q = list(self.get_parameter('home_position').value)
        self.qdot = [0.0] * self.n
        self.q_des = list(self.get_parameter('home_position').value)
        self.i_term = [0.0] * self.n

        self.have_state = False
        self.have_cmd = True
        self.last_time = self.get_clock().now()

        # ===== SUB =====
        self.sub_js = self.create_subscription(
            JointState,
            '/joint_states_out',
            self.on_joint_states,
            10
        )

        self.sub_cmd = self.create_subscription(
            Float64MultiArray,
            '/joint_position_cmd',
            self.on_cmd,
            10
        )

        # ===== PUB =====
        self.pub_vel = self.create_publisher(
            Float64MultiArray,
            '/joint_velocity_cmd',
            10
        )

        self.get_logger().info("Joint Pos→Vel PID Node started")

    def on_cmd(self, msg: Float64MultiArray):
        if len(msg.data) != self.n:
            self.get_logger().warn("Invalid position command size")
            return

        self.q_des = list(msg.data)
        self.have_cmd = True
        self.compute()

    def on_joint_states(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        # self.get_logger().info(f"Received joint states: {msg.name}")
        for i, jn in enumerate(self.joint_names):
            # self.get_logger().info(f"Processing joint {jn}")
            if jn not in name_to_idx:
                continue

            idx = name_to_idx[jn]
            self.q[i] = msg.position[idx] if idx < len(msg.position) else 0.0
            self.qdot[i] = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
            # if i in (1, 3):
            #     self.q[i] = math.pi - self.q[i]
            
            # self.get_logger().info(f"Joint {jn}: q={self.q[i]}, qdot={self.qdot[i]}")
        self.have_state = True
        self.compute()

    def compute(self):
        if not (self.have_state and self.have_cmd):
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        qdot_des = []

        for i in range(self.n):
            error = self.q_des[i] - self.q[i]

            # Integral
            self.i_term[i] += error * dt
            self.i_term[i] = clamp(self.i_term[i], -self.i_max, self.i_max)

            # PID to velocity
            v = (
                self.kp * error +
                self.ki * self.i_term[i] -
                self.kd * self.qdot[i]
            )

            v = clamp(v, -self.vel_limit, self.vel_limit)
            qdot_des.append(v)

        msg = Float64MultiArray()
        msg.data = qdot_des
        self.pub_vel.publish(msg)


def main():
    rclpy.init()
    node = JointPosToVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()