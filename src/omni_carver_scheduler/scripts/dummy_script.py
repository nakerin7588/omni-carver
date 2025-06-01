#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from omni_carver_interfaces.srv import OmniCarverMode

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        
        self.mode = self.create_client(OmniCarverMode, 'omni_carver/mode')
        self.mode.wait_for_service()
        self.get_logger().info('Dummy Node has been started.')

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
