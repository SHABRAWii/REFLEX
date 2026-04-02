import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64

class StanceController(Node):
    def __init__(self):
        super().__init__('stance_controller')

        self.joint_names = [
            'Left_Hip_joint', 
            'Left_Knee_joint', 
            'Right_Hip_Joint', 
            'Right_Knee_joint'
        ]
        
        # Max velocity from your GIM6010-8 datasheet (rad/s)
        self.max_velocity = 58.01 

        self.pose_standing = {
            'Left_Hip_joint': 0.0, 'Left_Knee_joint': 0.0,
            'Right_Hip_Joint': 0.0, 'Right_Knee_joint': 0.0
        }
        self.pose_sitting = {
            'Left_Hip_joint': 1.57, 'Left_Knee_joint': 1.57,  
            'Right_Hip_Joint': 1.57, 'Right_Knee_joint': 1.57 
        }
        
        self.current_height_cmd = 0.0 

        # Proportional gain: How fast to move per radian of error.
        self.kp = 10.0 

        # Note the topic change to leg_velocity_controller
        self.vel_pub = self.create_publisher(Float64MultiArray, '/leg_velocity_controller/commands', 10)
        self.height_sub = self.create_subscription(Float64, '/reflex/target_height', self.height_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.control_loop_callback, 10)

        self.get_logger().info("Velocity Stance Controller Active. Send 0.0 to 1.0 to /reflex/target_height")

    def height_callback(self, msg: Float64):
        self.current_height_cmd = max(0.0, min(msg.data, 1.0))

    def control_loop_callback(self, msg: JointState):
        velocities = []

        for joint in self.joint_names:
            if joint in msg.name:
                idx = msg.name.index(joint)
                current_pos = msg.position[idx]

                # 1. Kinematic Mapping
                target_pos = self.pose_standing[joint] + self.current_height_cmd * (self.pose_sitting[joint] - self.pose_standing[joint])

                # 2. P-Control Law for Velocity
                error = target_pos - current_pos
                v_cmd = self.kp * error

                # 3. Velocity Saturation
                v_clipped = max(-self.max_velocity, min(v_cmd, self.max_velocity))
                velocities.append(v_clipped)
            else:
                velocities.append(0.0)

        # Publish commanded velocities
        command_msg = Float64MultiArray()
        command_msg.data = velocities
        self.vel_pub.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()