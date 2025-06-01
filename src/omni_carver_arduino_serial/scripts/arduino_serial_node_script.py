#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import serial
import time
import struct

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
        self.create_timer(1/100, self.timer_callback)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)
        self.serial_port.reset_input_buffer()
        
        self.wheel_pos = [0.0, 0.0, 0.0]
        self.wheel_vel = [0.0, 0.0, 0.0]
        
        self.get_logger().info('Arduino Serial Node initialized')
    
    def timer_callback(self):
        if self.serial_port.in_waiting >= 6:
            raw = self.serial_port.read(6)
            v1, v2, v3 = struct.unpack('<3h', raw)
            self.wheel_vel = [v1/1000.0, v2/1000.0, v3/1000.0]
            for i in range(3):
                self.wheel_pos[i] += self.wheel_vel[i] / 100.0
            
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name     = ['wheel_1_joint','wheel_2_joint','wheel_3_joint']
            msg.position = self.wheel_pos
            msg.velocity = self.wheel_vel
            self.joints_state_publisher.publish(msg)
        except Exception as e:
            error_msg = 'Error publishing joint state: %s' % str(e)
            self.get_logger().error(error_msg)
    
    def callback(self, msg: Float64MultiArray):
        data_to_send = '{},{},{}\n'.format(int(msg.data[0]*1000), int(msg.data[1]*1000), int(msg.data[2]*1000))
        try:
            self.serial_port.write(data_to_send.encode())
            # self.get_logger().info('Data sent to microcontroller: %s' % data_to_send.strip())
            
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
