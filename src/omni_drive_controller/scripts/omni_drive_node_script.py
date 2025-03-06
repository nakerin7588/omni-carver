#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from std_msgs.msg import Float64MultiArray
import numpy as np

class forward_kinematics:
    def __init__(self, wheel_radius=0.05, wheel_base=0.2, wheel_track=0.15):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.wheel_track = wheel_track
        twist = np.array([0.0, 0.0, 0.0]) # linear velocity x, y and angular velocity z

    def calculate(self, wr, wl, wb):
        R = 2

        return 1
    
    def calculate_rpm(self, wheel_speed):
        wheel_rpm = [0, 0, 0]
        for i in range(3):
            wheel_rpm[i] = wheel_speed[i] / (2 * 3.14 * self.wheel_radius)

        return wheel_rpm

class inverse_kinematics:
    def __init__(self, wheel_base=0.04, wheel_radius = 0.05):
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        
    def calculate(self, w, vx, vy):
        """
        This function calculates the wheel speed based on the robot velocity from this equation:
        
        u = 1/r     [   -d -sqrt(3)/2  -1/2         [   w
                        -d  sqrt(3)/2  -1/2    @        vx
                        -d  0          1    ]           vy  ]

        where:
            u = [w_r, w_l, w_b]^T
            d = wheel_base
            r = wheel_radius
            w = angular velocity z
            vx = linear velocity x
            vy = linear velocity y
        
        Args:
            w (Float): angular velocity z
            vx (Float): linear velocity x
            vy (Float): linear velocity y

        Returns:
            float array: wheel speed
        """
        
        T = np.array([
                    [-self.wheel_base, -np.sqrt(3)/2, -1/2],
                    [-self.wheel_base, np.sqrt(3)/2, -1/2],
                    [-self.wheel_base, 0, 1]])
        
        u = 1/self.wheel_radius * T
        u = np.dot(u, np.array([w, vx, vy]))
        
        return u
    
class OmniDriveNode(Node):
    def __init__(self):
        super().__init__('omni_drive_node')
        
        self.declare_parameter('wheel_radius', 0.038)
        self.declare_parameter('wheel_base', 0.04)
        self.declare_parameter('wheel_track', 0.15)
        self.declare_parameter('wheel_delta', 30) # Degrees
        
        # Get the parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_track = self.get_parameter('wheel_track').value
        
        # Get wheels speed
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        class wheel_speed:
            def __init__(self):
                self.right = 0
                self.left = 0
                self.back = 0
        self.wheel_speed = wheel_speed()
        
        # Get the forward kinematics
        self.forward_kinematics = forward_kinematics(self.wheel_radius, self.wheel_base, self.wheel_track)
        
        # Get the inverse kinematics
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.inverse_kinematics = inverse_kinematics(self.wheel_base, self.wheel_radius)
        
        # Set joint control
        self.joints_control = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        
        # 
        
        self.get_logger().info('OmniDriveNode has been started')
        
    
    def publish_joint_speed(self, q):
        msg = Float64MultiArray()
        msg.data = q
        self.joints_control.publish(msg)
    
    def cmd_vel_callback(self, msg:Twist):
        q = self.inverse_kinematics.calculate(msg.angular.z, msg.linear.x, msg.linear.y)
        self.publish_joint_speed(q)
        self.get_logger().info('Wheel RPM: Right: %f, Left: %f, Back: %f' % (q[0], q[1], q[2]))
        self.get_logger().info('Robot velocity: x: %f, y: %f, z: %f' % (msg.linear.x, msg.linear.y, msg.angular.z))
        
    def joint_states_callback(self, msg:JointState):
        # Initialize indices
        index_r, index_l, index_b = None, None, None

        # Create a mapping of joint names to their respective indices
        indices = {name: i for i, name in enumerate(msg.name)}

        # Assign values if they exist in the message
        index_r = indices.get("rim_right_joint")
        index_l = indices.get("rim_left_joint")
        index_b = indices.get("rim_back_joint")
    
        self.wheel_speed.right = msg.velocity[index_r]
        self.wheel_speed.left = msg.velocity[index_l]
        self.wheel_speed.back = msg.velocity[index_b]
        
def main(args=None):
    rclpy.init(args=args)
    node = OmniDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
