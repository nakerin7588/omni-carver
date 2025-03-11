#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from bno055_usb_stick_py import BnoUsbStick
import numpy as np
import threading

class BNO055USBSTICKNode(Node):
    def __init__(self):
        super().__init__('bno055_usb_stick_node')
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.publisher = self.create_publisher(Imu, 'imu/data', qos_profile)
        self.timer = self.create_timer(1/50, self.timer_callback)

        self.bno_usb_stick = BnoUsbStick(port='/dev/ttyACM0')
        
        self.latest_packet = None
        self.data_lock = threading.Lock()
        
        self.prev_package = None

        self.stream_thread = threading.Thread(target=self.imu_streaming, daemon=True)
        self.stream_thread.start()

    def imu_streaming(self):
        self.bno_usb_stick.activate_streaming()
        for package in self.bno_usb_stick.recv_streaming_generator():
            with self.data_lock:
                self.latest_packet = package

    def timer_callback(self):
        with self.data_lock:
            package = self.latest_packet

        if package is None:
            self.get_logger().info("No data received yet.")
            return

        # Check for abnormal gyro data: if any value >= 300, use previous valid data.
        if np.any(np.array(package.g) >= 300):
            package = self.prev_package
        else:
            self.prev_package = package

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation.x = self.prev_package.quaternion[1]
        imu_msg.orientation.y = self.prev_package.quaternion[2]
        imu_msg.orientation.z = self.prev_package.quaternion[3]
        imu_msg.orientation.w = self.prev_package.quaternion[0]
        imu_msg.angular_velocity.x = self.prev_package.g[0]
        imu_msg.angular_velocity.y = self.prev_package.g[1]
        imu_msg.angular_velocity.z = self.prev_package.g[2]
        imu_msg.linear_acceleration.x = self.prev_package.lin_a[0]
        imu_msg.linear_acceleration.y = self.prev_package.lin_a[1]
        imu_msg.linear_acceleration.z = self.prev_package.lin_a[2]
        
        self.publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BNO055USBSTICKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
