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

    def calculate(self, vx, vy, wz):
        wheel_speed = [0, 0, 0]
        wheel_speed[0] = vx - vy - (self.wheel_base + self.wheel_track) * wz
        wheel_speed[1] = vx + vy + (self.wheel_base + self.wheel_track) * wz
        wheel_speed[2] = vx + vy - (self.wheel_base + self.wheel_track) * wz

        return wheel_speed

    def calculate_rpm(self, wheel_speed):
        wheel_rpm = [0, 0, 0]
        for i in range(3):
            wheel_rpm[i] = wheel_speed[i] / (2 * 3.14 * self.wheel_radius)

        return wheel_rpm

class inverse_kinematics:
    def __init__(self, wheel_base=0.04, wheel_delta=30):
        self.wheel_base = wheel_base
        self.wheel_delta = np.radians(wheel_delta)
        
    def calculate(self, u, v, w):
        # T = np.array([
        #             [-1,  0,  0],
        #             [ 0, -1,  0],
        #             [ 0,  0,  1] ]) @ np.array([
        #                                         [np.cos(self.wheel_delta), np.sin(self.wheel_delta), self.wheel_base],
        #                                         [-np.cos(self.wheel_delta), np.sin(self.wheel_delta), self.wheel_base],
        #                                         [0, -1, self.wheel_base]    ])
        T = np.array([
                    [np.cos(self.wheel_delta), np.sin(self.wheel_delta), self.wheel_base],
                    [-np.cos(self.wheel_delta), np.sin(self.wheel_delta), self.wheel_base],
                    [0, -1, self.wheel_base]    ])

        q = T@np.array([u, v, w])
        
        return q
    
class OmniDriveNode(Node):
    def __init__(self):
        super().__init__('omni_drive_node')
        
        self.declare_parameter('wheel_radius', 0.05)
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
        self.inverse_kinematics = inverse_kinematics(self.wheel_base, self.get_parameter('wheel_delta').value)
        
        # Set joint control
        self.joints_control = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        
        self.get_logger().info('OmniDriveNode has been started')
        
    
    def publish_joint_speed(self, q):
        msg = Float64MultiArray()
        msg.data = q
        self.joints_control.publish(msg)
    
    def cmd_vel_callback(self, msg:Twist):
        # wheel_speed = self.forward_kinematics.calculate(msg.linear.x, msg.linear.y, msg.angular.z)
        # wheel_rpm = self.forward_kinematics.calculate_rpm(wheel_speed)
        # self.get_logger().info('Wheel RPM: Right: %f, Left: %f, Back: %f' % (wheel_rpm[0], wheel_rpm[1], wheel_rpm[2]))
        q = self.inverse_kinematics.calculate(msg.linear.y, msg.linear.x, msg.angular.z)
        self.publish_joint_speed(q)
        self.get_logger().info('Wheel RPM: Right: %f, Left: %f, Back: %f' % (q[0], q[1], q[2]))
        
    def joint_states_callback(self, msg:JointState):
        self.wheel_speed.right = msg.velocity[0]
        self.wheel_speed.left = msg.velocity[1]
        self.wheel_speed.back = msg.velocity[2]
        
def main(args=None):
    rclpy.init(args=args)
    node = OmniDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
