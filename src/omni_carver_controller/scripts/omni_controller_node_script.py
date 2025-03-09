#!/usr/bin/python3

import rclpy
from rclpy.node import Node


class OmniCarverControllerNode(Node):
    def __init__(self):
        super().__init__('omni_carver_controller_node')

def main(args=None):
    rclpy.init(args=args)
    node = OmniCarverControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
