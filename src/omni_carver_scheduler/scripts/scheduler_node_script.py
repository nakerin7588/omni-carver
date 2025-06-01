#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from omni_carver_interfaces.srv import OmniCarverMode
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler_node')
        
        self.create_service(OmniCarverMode, 'omni_carver/mode', self.omni_carver_mode_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.heartbeat_pub = self.create_publisher(Empty, 'omni_carver/heartbeat', 10)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.cmd_vel_smoothed_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self.cmd_vel_teleop_callback, 10)
        self.create_timer(5, self.timer_callback)
        
        self.mode = 'teleop'
        
        self.get_logger().info('Scheduler Node has been started.')
    
    def omni_carver_mode_callback(self, request:OmniCarverMode.Request, response:OmniCarverMode.Response):
        self.mode = request.mode
        if self.mode == 'Teleoperation':
            self.get_logger().info(f'Switched to teleoperation mode')
        elif self.mode == 'Autonomous':
            self.get_logger().info(f'Switched to autonomous mode')
        else:
            self.get_logger().warn(f'Unknown mode requested: {request.mode}')
        
        response.response =  "Success"
        return response
    
    def cmd_vel_smoothed_callback(self, msg:Twist):
        if self.mode == 'Autonomous':
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = msg.linear.x
            cmd_vel_msg.linear.y = msg.linear.y
            cmd_vel_msg.angular.z = msg.angular.z
            
            self.cmd_vel_pub.publish(cmd_vel_msg)
    
    def cmd_vel_teleop_callback(self, msg:Twist):
        if self.mode == 'Teleoperation':
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = msg.linear.x
            cmd_vel_msg.linear.y = msg.linear.y
            cmd_vel_msg.angular.z = msg.angular.z
            
            self.cmd_vel_pub.publish(cmd_vel_msg)
    
    def timer_callback(self):
        heartbeat_msg = Empty()
        self.heartbeat_pub.publish(heartbeat_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
