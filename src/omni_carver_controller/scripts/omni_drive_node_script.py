#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Twist, PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
from geometry_msgs.msg import TransformStamped

class forward_kinematics:
    def __init__(self, wheel_base=0.065, wheel_radius = 0.075):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.twist = np.array([0.0, 0.0, 0.0]) # linear velocity x, y and angular velocity z

    def calculate(self, wr, wl, wb):
        """
        This function calculates the velocity at robot base frame based on the wheel velocity from this equation:
        
        vb = pinv(H) @ u

        where:
            H = r [   -d -sqrt(3)/2  -1/2 ]
                    [   -d  sqrt(3)/2  -1/2 ]
                    [   -d  0          1    ]
            u = [wr, wl, wb]^T
            d = wheel_base      (m)
            r = wheel_radius    (m)
            vb = [w, vx, vy]^T
            w = angular velocity z  (rad/s)
            vx = linear velocity x  (m/s)
            vy = linear velocity y  (m/s)
        Args:
            wr (Float): wheel velocity right    (rad/s)
            wl (Float): wheel velocity left     (rad/s)
            wb (Float): wheel velocity back     (rad/s)

        Returns:
            float array: robot velocity
        """
        
        H = np.array([
            [-self.wheel_base, -np.sqrt(3)/2, -1/2],
            [-self.wheel_base, np.sqrt(3)/2, -1/2],
            [-self.wheel_base, 0, 1]])
        
        H = 1/self.wheel_radius * H
        self.twist = np.dot(np.linalg.pinv(H), np.array([wr, wl, wb]))
        
        return self.twist

class inverse_kinematics:
    def __init__(self, wheel_base=0.065, wheel_radius = 0.075):
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        
    def calculate(self, w, vx, vy):
        """
        This function calculates the wheel velocity based on the robot velocity from this equation:
        
        u = 1/r     [   -d -sqrt(3)/2  -1/2 ]        [  w
                    [   -d  sqrt(3)/2  -1/2 ]   @       vx
                    [   -d  0          1    ]           vy  ]

        where:
            u = [wr, wl, wb]^T
            d = wheel_base      (m)
            r = wheel_radius    (m)
            w = angular velocity z  (rad/s)
            vx = linear velocity x  (m/s)
            vy = linear velocity y  (m/s)
        
        Args:
            w (Float): angular velocity z   (rad/s)
            vx (Float): linear velocity x   (m/s)
            vy (Float): linear velocity y   (m/s)

        Returns:
            float array: wheel speed    (rad/s)
        """
        
        H = np.array([
                    [-self.wheel_base, -np.sqrt(3)/2, -1/2],
                    [-self.wheel_base, np.sqrt(3)/2, -1/2],
                    [-self.wheel_base, 0, 1]])
        
        H = 1/self.wheel_radius * H
        u = np.dot(H, np.array([w, vx, vy]))
        
        return u
    
class OmniDriveNode(Node):
    def __init__(self):
        super().__init__('omni_drive_node')
        
        self.declare_parameter('wheel_radius', 0.075)
        self.declare_parameter('wheel_base', 0.065)
        self.declare_parameter('rate', 100) # Hz
        
        # Get the parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.rate = self.get_parameter('rate').value
        
        # Set the tf broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Get wheels speed
        self.create_subscription(JointState, '/omni_carver/joint_states', self.joint_states_callback, 10)
        class wheel_speed:
            def __init__(self):
                self.right = 0
                self.left = 0
                self.back = 0
        self.wheel_speed = wheel_speed()
        
        # Get the forward kinematics
        self.forward_kinematics = forward_kinematics(self.wheel_base, self.wheel_radius)
        
        # Get the inverse kinematics
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.inverse_kinematics = inverse_kinematics(self.wheel_base, self.wheel_radius)
        
        # Get the initial_2d message from either Rviz clicks or a manual pose publisher
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        
        # Set joints control
        self.joints_control = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        self.wheels_speed_pub = self.create_publisher(Float64MultiArray, "/wheel_controllers/commands", 10)
        
        # Set wheel odometry
        class Wheel_odometry:
            """
            This class defines the robot wheel odometry.
            """
            
            def __init__(self):
                self.theta = 0.0
                self.x = 0.0
                self.y = 0.0
                self.w = 0.0
                self.vx = 0.0
                self.vy = 0.0
                
        self.robot_wheelodom = Wheel_odometry() # Initialize the robot pose reference to world/odom/map/any_initial frame.
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)

        # Set the timer
        self.timer = self.create_timer(1/self.rate, self.timer_callback)
        
        self.get_logger().info('OmniDriveNode has been started')
    
    def timer_callback(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Get the forward kinematics
        self.robot_wheelodom.w, self.robot_wheelodom.vx, self.robot_wheelodom.vy = self.forward_kinematics.calculate(self.wheel_speed.right, self.wheel_speed.left, self.wheel_speed.back)
        
        # Update the robot odometry after get current robot velocity
        self.update_odometry()
        
        odom.pose.pose.position.x = self.robot_wheelodom.x
        odom.pose.pose.position.y = self.robot_wheelodom.y
        odom.pose.pose.position.z = 0.0
        
        
        q = quaternion_from_euler(0.0, 0.0, self.robot_wheelodom.theta)
        
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.twist.twist.linear.x = self.robot_wheelodom.vx
        odom.twist.twist.linear.y = self.robot_wheelodom.vy
        odom.twist.twist.angular.z = self.robot_wheelodom.w
        
        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = "odom"
        # t.child_frame_id = "base_footprint"

        # t.transform.translation.x = self.robot_wheelodom.x
        # t.transform.translation.y = self.robot_wheelodom.y
        # t.transform.translation.z = 0.0
        
        # q = quaternion_from_euler(0, 0, self.robot_wheelodom.theta)

        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]

        # self.tf_broadcaster.sendTransform(t)
        self.odom_pub.publish(odom)
        
    def publish_joint_speed(self, q):
        msg = Float64MultiArray()
        msg.data = q
        self.joints_control.publish(msg)
        self.wheels_speed_pub.publish(msg)
    
    def initial_pose_callback(self, msg:PoseWithCovarianceStamped):
        self.get_logger().info(f"Received initial pose: {msg.pose.pose}")
        self.robot_wheelodom.x = msg.pose.pose.position.x
        self.robot_wheelodom.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_wheelodom.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.get_logger().info(f"Robot pose: x: {self.robot_wheelodom.x}, y: {self.robot_wheelodom.y}, theta: {self.robot_wheelodom.theta}")
    
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
        index_r = indices.get("wheel_1_joint")
        index_l = indices.get("wheel_2_joint")
        index_b = indices.get("wheel_3_joint")
    
        self.wheel_speed.right = msg.velocity[index_r]
        self.wheel_speed.left = msg.velocity[index_l]
        self.wheel_speed.back = msg.velocity[index_b]
        
        # self.get_logger().info('Wheel RPM: Right: %f, Left: %f, Back: %f' % (self.wheel_speed.right, self.wheel_speed.left, self.wheel_speed.back))
    
    def update_odometry(self):
        """
        This function updates the robot odometry based on the robot velocity via this equation.

            x_t = x_t_1 + (vxcos(theta_t_1) - vysin(theta_t_1))*dt
            y_t = y_t_1 + (vxsin(theta_t_1) + vycos(theta_t_1))*dt
            theta_t = theta_t_1 + w*dt
        
        where:
            x_t = x position at time t
            y_t = y position at time t
            theta_t = orientation at time t
            v = linear velocity
            w = angular velocity
            dt = time step = 1/self.rate
        
        args:
            w (Float): angular velocity z   (rad/s)
            vx (Float): linear velocity x   (m/s)
            vy (Float): linear velocity y   (m/s)
        
        returns:
            odometry: robot wheel odometry
        """
        
        theta_t_1 = self.robot_wheelodom.theta
        x_t_1 = self.robot_wheelodom.x
        y_t_1 = self.robot_wheelodom.y
        
        self.robot_wheelodom.theta = theta_t_1 + self.robot_wheelodom.w * 1/self.rate
        self.robot_wheelodom.x = x_t_1 + (self.robot_wheelodom.vx*np.cos(theta_t_1) - self.robot_wheelodom.vy*np.sin(theta_t_1))*1/self.rate
        self.robot_wheelodom.y = y_t_1 + (self.robot_wheelodom.vx*np.sin(theta_t_1) + self.robot_wheelodom.vy*np.cos(theta_t_1))*1/self.rate
        
def main(args=None):
    rclpy.init(args=args)
    node = OmniDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
