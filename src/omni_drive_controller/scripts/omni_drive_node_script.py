#!/usr/bin/python3

import rclpy
from rclpy.node import Node


class kinematics:
    def __init__(self):
        self.wheel_radius = 0.05
        self.wheel_base = 0.2
        self.wheel_track = 0.15

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

class OmniDriveNode(Node):
    def __init__(self):
        super().__init__('omni_drive_node')

def main(args=None):
    rclpy.init(args=args)
    node = OmniDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
