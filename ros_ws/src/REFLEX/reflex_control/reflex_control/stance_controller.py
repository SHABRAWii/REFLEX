import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
import math
from rclpy.qos import qos_profile_sensor_data

def quat_to_euler(ox, oy, oz, ow):
    """
    Returns roll, pitch, yaw (rad) from quaternion.
    """
    sinr_cosp = 2.0 * (ow * ox + oy * oz)
    cosr_cosp = 1.0 - 2.0 * (ox * ox + oy * oy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (ow * oy - oz * ox)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (ow * oz + ox * oy)
    cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class RollStabilizer(Node):
    def __init__(self):
        super().__init__('stance_controller')

        # --- MECHANICAL CAD PARAMETERS ---
        self.L_thigh = 0.2  
        self.L_calf = 0.2   
        
        # --- CARTESIAN EQUILIBRIUM ---
        self.base_height = 0.32564 
        
        # --- ROLL PID PARAMETERS ---
        self.target_roll = 0.0
        self.current_roll = 0.0
        
        self.kp_roll = 0.1  
        self.ki_roll = 0.0
        self.kd_roll = 0.01 
        
        self.integral_error = 0.0
        self.prev_error = 0.0
        
        # Initialize with the ROS 2 Clock
        self.last_time = self.get_clock().now()
        
        self.have_imu = False

        # --- NETWORK INTERFACES ---
        self.pos_pub = self.create_publisher(Float64MultiArray, '/joint_position_cmd', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states_out', self.control_loop_callback, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        
        self.get_logger().info("IK Stabilizer Running. Sim Time Architecture applied.")

    def on_imu(self, msg: Imu):
        ox = float(msg.orientation.x)
        oy = float(msg.orientation.y)
        oz = float(msg.orientation.z)
        ow = float(msg.orientation.w)

        if not (math.isfinite(ox) and math.isfinite(oy) and math.isfinite(oz) and math.isfinite(ow)):
            return

        n = ox*ox + oy*oy + oz*oz + ow*ow
        if n < 1e-12:
            return  

        inv = 1.0 / math.sqrt(n)
        ox *= inv; oy *= inv; oz *= inv; ow *= inv

        roll, pitch, yaw = quat_to_euler(ox, oy, oz, ow)

        if not math.isfinite(roll):
            return

        self.current_roll = roll
        self.have_imu = True

    def compute_ik(self, target_z):
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
        if not self.have_imu:
            return

        # Extract ROS 2 time and calculate dt in seconds
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        
        if dt <= 0.0:
            return
            
        self.last_time = current_time

        error = self.target_roll - self.current_roll
        self.integral_error += error * dt
        self.integral_error = max(-0.1, min(self.integral_error, 0.1)) 
        
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        delta_h = (self.kp_roll * error) + (self.ki_roll * self.integral_error) + (self.kd_roll * derivative)

        left_target_h = self.base_height - delta_h
        right_target_h = self.base_height + delta_h

        left_hip_tgt, left_knee_tgt = self.compute_ik(left_target_h)
        right_hip_tgt, right_knee_tgt = self.compute_ik(right_target_h)

        target_positions = [
            float(left_hip_tgt),
            float(left_knee_tgt),
            float(right_hip_tgt),
            float(right_knee_tgt)
        ]

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