#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
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
        self.create_timer(0.1, self.timer_callback)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)  # Adjust serial port and baudrate
        self.get_logger().info('Arduino Serial Node initialized')
        
    def change_to_count(self, omega1,omega2,omega3):
        delta_time = 10  # millisec
        count_per_second1 = 600 
        count_per_second2 = 1000  # from encoder
        sec_to_millisec = 1000  # sec_to_millisec
        big_round = 4.6  # cm
        small_round = 0.63  # cm
        pi = 3.1416
        
        count1 = (omega1 * delta_time * count_per_second1 * big_round) / (sec_to_millisec * small_round * 2 * pi)
        count2 = (omega2 * delta_time * count_per_second2 * big_round) / (sec_to_millisec * small_round * 2 * pi)
        count3 = (omega3 * delta_time * count_per_second2 * big_round) / (sec_to_millisec * small_round * 2 * pi)
        # log_msg = 'Received Twist data from ROS: count1 = %.2f, count2 = %.2f, count3 = %.2f' % (count1, count2, count3) 
        # self.get_logger().info(log_msg)
        return count1,count2,count3

    def timer_callback(self):
        data_to_send = f"10,1,0\n"
        
        try:
            self.serial_port.write(data_to_send.encode('utf-8'))
            
        except Exception as e:
            error_msg = 'Error sending data to microcontroller: %s' % str(e)
            self.get_logger().error(error_msg)
    
    def callback(self, msg: Float64MultiArray):
        
        # msg.data[0] = 10;
        data_to_send = f"{msg.data[0]},{msg.data[1]},{msg.data[2]}\n"
        
        try:
            self.serial_port.write(data_to_send.encode('utf-8'))
            
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
