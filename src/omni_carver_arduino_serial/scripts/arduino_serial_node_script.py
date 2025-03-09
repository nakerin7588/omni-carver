#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import serial
import time

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/wheel_controllers/commands',
            self.callback,
            10)
        self.joints_state_publisher = self.create_publisher(
            JointState,
            '/omni_carver/joint_states',
            10)
        self.create_timer(0.01, self.timer_callback)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)
        self.get_logger().info('Arduino Serial Node initialized')
    
    def timer_callback(self):
        try:
            data_received = self.serial_port.readline()
            joint_states = [float(x) for x in data_received.decode().split(',')]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint']
            msg.position = joint_states[:3]
            msg.velocity = joint_states[3:6]
            self.joints_state_publisher.publish(msg)
            
        except Exception as e:
            error_msg = 'Error receiving data from microcontroller: %s' % str(e)
            self.get_logger().error(error_msg)
    
    def callback(self, msg: Float64MultiArray):
        
        data_to_send = f"{msg.data[0]},{msg.data[1]},{msg.data[2]}\n"
        try:
            self.serial_port.write(data_to_send.encode())
            
        except Exception as e:
            error_msg = 'Error sending data to microcontroller: %s' % str(e)
            self.get_logger().error(error_msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    time.sleep(0.01)
    main()
