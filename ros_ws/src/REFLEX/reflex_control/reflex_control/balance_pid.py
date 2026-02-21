#!/usr/bin/env python3
import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def quat_to_euler(ox, oy, oz, ow):
    """
    Returns roll, pitch, yaw (rad) from quaternion.
    ROS IMU uses x,y,z,w.
    """
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (ow * ox + oy * oz)
    cosr_cosp = 1.0 - 2.0 * (ox * ox + oy * oy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (ow * oy - oz * ox)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (ow * oz + ox * oy)
    cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


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
        if dt <= 0.0:
            return 0.0

        if not self.initialized:
            self.prev_err = err
            self.initialized = True

        # integral with clamp
        self.i_term += err * dt
        self.i_term = clamp(self.i_term, -self.i_max, self.i_max)

        # derivative
        d_err = (err - self.prev_err) / dt
        self.prev_err = err

        out = self.kp * err + self.ki * self.i_term + self.kd * d_err
        out = clamp(out, -self.out_max, self.out_max)
        return out


class ReflexBalancePID(Node):
    """
    Two-loop balancing controller:
      Outer PID: vavg -> theta_set
      Inner PID: theta_set -> wheel velocity commands

    Inputs:
      /cmd_vel (Twist): linear.x = vavg_set [m/s], angular.z = yaw_rate_set [rad/s]
      /joint_states (JointState): wheel joint velocities [rad/s]
      /imu (Imu): orientation quaternion

    Output:
      /wheel_velocity_controller/commands (Float64MultiArray): [wL_cmd, wR_cmd] [rad/s]
    """

    def __init__(self):
        super().__init__('reflex_balance_pid')

        # joints
        self.declare_parameter('left_joint', 'Left_Wheel_Joint')
        self.declare_parameter('right_joint', 'Right_Wheel_Joint')
        self.left_joint = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value

        # IMU + joint state msg recieved
        self.have_imu = False
        self.have_js = False

        # robot geometry
        self.declare_parameter('wheel_radius', 0.06)     # meters (set yours)
        self.declare_parameter('wheel_separation', 0.30) # meters (L-R distance)
        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_separation').value)

        # which angle to balance on: 'pitch' (recommended) or 'roll'
        self.declare_parameter('balance_axis', 'pitch')
        self.balance_axis = str(self.get_parameter('balance_axis').value).lower()

        # limits
        self.declare_parameter('theta_max', 0.25)    # rad
        self.declare_parameter('w_max', 40.0)        # rad/s clamp
        self.theta_max = float(self.get_parameter('theta_max').value)
        self.w_max = float(self.get_parameter('w_max').value)

        # outer loop PID (velocity -> theta_set)
        self.declare_parameter('v_kp', 0.6)
        self.declare_parameter('v_ki', 0.3)
        self.declare_parameter('v_kd', 0.0)
        self.declare_parameter('v_i_max', 0.3)
        self.declare_parameter('v_out_max', 0.25)  # theta_set clamp (rad)

        self.pid_v = PID(
            kp=float(self.get_parameter('v_kp').value),
            ki=float(self.get_parameter('v_ki').value),
            kd=float(self.get_parameter('v_kd').value),
            i_max=float(self.get_parameter('v_i_max').value),
            out_max=float(self.get_parameter('v_out_max').value),
        )

        # inner loop PID (theta -> wheel command u)
        self.declare_parameter('t_kp', 25.0)
        self.declare_parameter('t_ki', 0.0)
        self.declare_parameter('t_kd', 1.2)
        self.declare_parameter('t_i_max', 5.0)
        self.declare_parameter('t_out_max', 40.0)  # rad/s

        self.pid_t = PID(
            kp=float(self.get_parameter('t_kp').value),
            ki=float(self.get_parameter('t_ki').value),
            kd=float(self.get_parameter('t_kd').value),
            i_max=float(self.get_parameter('t_i_max').value),
            out_max=float(self.get_parameter('t_out_max').value),
        )

        # rates
        self.declare_parameter('inner_rate_hz', 250.0)
        self.declare_parameter('outer_rate_hz', 50.0)
        self.inner_rate = float(self.get_parameter('inner_rate_hz').value)
        self.outer_rate = float(self.get_parameter('outer_rate_hz').value)
        self.outer_decim = max(1, int(round(self.inner_rate / self.outer_rate)))

        # state
        self.v_set = 0.0
        self.yaw_rate_set = 0.0

        self.wL_meas = 0.0
        self.wR_meas = 0.0

        self.theta = 0.0
        self.theta_set = 0.0

        self._tick = 0
        self.last_time = self.get_clock().now()

        # subs/pubs
        qos_be = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.pub_cmd = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', qos_be)
        self.sub_js = self.create_subscription(JointState, '/joint_states', self.on_joint_states, qos_profile_sensor_data)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)

        self.pub_debug = self.create_publisher(Float64MultiArray, '/reflex_balance/debug', 10)
        self.timer = self.create_timer(1.0 / self.inner_rate, self.on_timer)

        self.get_logger().info(
            f"Balance PID started: axis={self.balance_axis}, r={self.r}, L={self.L}, "
            f"inner={self.inner_rate}Hz outer≈{self.outer_rate}Hz"
        )

    def on_cmd(self, msg: TwistStamped):
        self.v_set = float(msg.linear.x)       # m/s
        self.yaw_rate_set = float(msg.angular.z)  # rad/s

    def on_joint_states(self, msg: JointState):
        self.have_js = True
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        if self.left_joint in name_to_idx:
            i = name_to_idx[self.left_joint]
            if i < len(msg.velocity):
                self.wL_meas = float(msg.velocity[i])
        if self.right_joint in name_to_idx:
            i = name_to_idx[self.right_joint]
            if i < len(msg.velocity):
                self.wR_meas = float(msg.velocity[i])

    def on_imu(self, msg: Imu):
      ox = float(msg.orientation.x)
      oy = float(msg.orientation.y)
      oz = float(msg.orientation.z)
      ow = float(msg.orientation.w)

      # Reject invalid quaternion (all zeros) and NaNs
      if not (math.isfinite(ox) and math.isfinite(oy) and math.isfinite(oz) and math.isfinite(ow)):
          return

      n = ox*ox + oy*oy + oz*oz + ow*ow
      if n < 1e-12:
          return  # (0,0,0,0) -> ignore

      # Normalize quaternion
      inv = 1.0 / math.sqrt(n)
      ox *= inv; oy *= inv; oz *= inv; ow *= inv

      roll, pitch, _ = quat_to_euler(ox, oy, oz, ow)
      theta = pitch if self.balance_axis == 'pitch' else roll

      if not math.isfinite(theta):
          return

      self.theta = theta
      self.have_imu = True

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return
        if not (self.have_imu and self.have_js):
            return
        if not math.isfinite(self.theta):
            return
        if self.r <= 1e-6:
          return
        # measured vavg from wheels (m/s)
        vL = self.r * self.wL_meas
        vR = self.r * self.wR_meas
        v_avg = 0.5 * (vL + vR)

        # OUTER loop (decimated)
        self._tick += 1
        if (self._tick % self.outer_decim) == 0:
            ev = self.v_set - v_avg
            theta_cmd = self.pid_v.step(ev, dt * self.outer_decim)
            self.theta_set = clamp(theta_cmd, -self.theta_max, self.theta_max)

        # INNER loop
        e_theta = self.theta_set - self.theta
        u = self.pid_t.step(e_theta, dt)  # u in rad/s (forward wheel speed term)

        # turning term from yaw_rate_set (rad/s) => wheel differential (rad/s)
        # v = w*r; diff v = yaw_rate * L/2  => diff w = (yaw_rate * L/2) / r
        w_turn = (self.yaw_rate_set * (self.L / 2.0)) / max(1e-6, self.r)

        wL_cmd = clamp(u - w_turn, -self.w_max, self.w_max)
        wR_cmd = clamp(u + w_turn, -self.w_max, self.w_max)

        dbg = Float64MultiArray()
        dbg.data = [float(v_avg), float(self.v_set), float(self.theta), float(self.theta_set),
                    float(u), float(wL_cmd), float(wR_cmd)]
        self.pub_debug.publish(dbg)

        v_cmd = self.r * 0.5 * (wL_cmd + wR_cmd)
        yaw_cmd = (self.r / self.L) * (wR_cmd - wL_cmd)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(v_cmd)
        cmd.twist.angular.z = float(yaw_cmd)
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = ReflexBalancePID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()