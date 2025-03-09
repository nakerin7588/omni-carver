#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from bno055_usb_stick_py import BnoUsbStick

class BNO055USBSTICKNode(Node):
    def __init__(self):
        super().__init__('bno055_usb_stick_node')
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.publisher = self.create_publisher(Imu, 'imu/data', qos_profile)
        self.timer = self.create_timer(1/30, self.timer_callback)
        
        self.bno_usb_stick = BnoUsbStick(port='/dev/ttyACM0')

    def timer_callback(self):
        self.bno_usb_stick.activate_streaming()
        for package in self.bno_usb_stick.recv_streaming_generator(num_packets=1):
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.orientation.x = package.quaternion[1]
            imu_msg.orientation.y = package.quaternion[2]
            imu_msg.orientation.z = package.quaternion[3]
            imu_msg.orientation.w = package.quaternion[0]
            imu_msg.angular_velocity.x = package.g[0]
            imu_msg.angular_velocity.y = package.g[1]
            imu_msg.angular_velocity.z = package.g[2]
            imu_msg.linear_acceleration.x = package.lin_a[0]
            imu_msg.linear_acceleration.y = package.lin_a[1]
            imu_msg.linear_acceleration.z = package.lin_a[2]
            self.publisher.publish(imu_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = BNO055USBSTICKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
