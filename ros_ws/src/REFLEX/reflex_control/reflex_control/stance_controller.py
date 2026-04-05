import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
import math
import time
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


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
class RollStabilizer(Node):
    def __init__(self):
        super().__init__('stance_controller')

        # --- MECHANICAL CAD PARAMETERS ---
        self.L_thigh = 0.2  # Length from Hip to Knee (meters)
        self.L_calf = 0.2   # Length from Knee to Wheel/Ankle (meters)
        
        # --- CARTESIAN EQUILIBRIUM ---
        self.base_height = 0.32564 # The exact 32cm target height
        
        # --- ROLL PID PARAMETERS ---
        self.target_roll = 0.0
        self.current_roll = 0.0
        
        self.kp_roll = 0.1  
        self.ki_roll = 0.0
        self.kd_roll = 0.01 
        
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()

        # ---> NEW: Position Command Publisher <---
        self.pos_pub = self.create_publisher(Float64MultiArray, '/joint_position_cmd', 10)
        
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states_out', self.on_joint_states, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        
        

        self.get_logger().info("IK Stabilizer Running. Publishing raw angles to /joint_position_cmd.")

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.current_roll = math.atan2(sinr_cosp, cosr_cosp)

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
      theta = roll if self.balance_axis == 'roll' else pitch

      if not math.isfinite(theta):
          return

      # convert theta to deg then pass it
      self.theta = theta
      self.have_imu = True

    def compute_ik(self, target_z):
        """
        Inverse Kinematics Solver using the Law of Cosines.
        """
        max_reach = self.L_thigh + self.L_calf - 0.01
        min_reach = abs(self.L_thigh - self.L_calf) + 0.05
        z = max(min_reach, min(target_z, max_reach))

        cos_knee_interior = (self.L_thigh**2 + self.L_calf**2 - z**2) / (2 * self.L_thigh * self.L_calf)
        cos_knee_interior = max(-1.0, min(1.0, cos_knee_interior))
        knee_interior_angle = math.acos(cos_knee_interior)

        knee_angle = knee_interior_angle

        cos_hip = (self.L_thigh**2 + z**2 - self.L_calf**2) / (2 * self.L_thigh * z)
        cos_hip = max(-1.0, min(1.0, cos_hip))
        hip_angle = math.acos(cos_hip) + 99.5 * math.pi / 180.0

        return hip_angle, knee_angle

    def control_loop_callback(self, msg: JointState):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.0:
            return
        self.last_time = current_time

        # 1. Outer Loop: Calculate Delta Height (in meters)
        error = self.target_roll - self.current_roll
        self.integral_error += error * dt
        self.integral_error = max(-0.1, min(self.integral_error, 0.1)) 
        
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        delta_h = (self.kp_roll * error) + (self.ki_roll * self.integral_error) + (self.kd_roll * derivative)

        # 2. Middle Loop: Apply differential height & run IK Solver
        left_target_h = self.base_height - delta_h
        right_target_h = self.base_height + delta_h

        left_hip_tgt, left_knee_tgt = self.compute_ik(left_target_h)
        right_hip_tgt, right_knee_tgt = self.compute_ik(right_target_h)

        # 3. Output Translation (The New Execution Pipeline)
        # Pack the array strictly in the order: [Left_Hip, Left_Knee, Right_Hip, Right_Knee]
        target_positions = [
            float(left_hip_tgt),
            float(left_knee_tgt),
            float(right_hip_tgt),
            float(right_knee_tgt)
        ]

        # Publish the command
        command_msg = Float64MultiArray()
        command_msg.data = target_positions
        self.pos_pub.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RollStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()