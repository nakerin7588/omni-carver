#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from bno055_usb_stick_py import BnoUsbStick
import numpy as np
import threading
from time import sleep
# import tf2_ros
# from geometry_msgs.msg import TransformStamped

class BNO055USBSTICKNode(Node):
    def __init__(self):
        super().__init__('bno055_usb_stick_node')
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.publisher = self.create_publisher(Imu, 'imu/data', qos_profile)
        
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.timer = self.create_timer(1/80, self.timer_callback)

        self.bno_usb_stick = BnoUsbStick(port='/dev/ttyACM0')
        
        # Reset the IMU
        try:
            self.imu_reset()
        except Exception as e:
            self.get_logger().error(f"Failed to reset IMU: {e}")
            return
        
        self.reading_thread = threading.Thread(target=self.imu_streaming)
        self.reading_thread.daemon = True  # Daemon thread will exit when the main thread exits
        self.reading_thread.start()

        self.imu_data = None  # Shared variable for storing the IMU data
        self.data_lock = threading.Lock()  # Lock for synchronizing access to IMU data
        
        self.get_logger().info("BNO055 USB Stick initialized successfully.")

    def imu_reset(self):
        # Reset and set the default configuration for the IMU
        opr_mode_addr = 0x3D
        mode = self.bno_usb_stick.read_register(opr_mode_addr)
        self.get_logger().info(f"Initial operation mode: {mode:08b}")
        
        sys_trigger_addr = 0x3F
        reset_val = 1 << 5
        self.bno_usb_stick.write_register(sys_trigger_addr, reset_val)
        sleep(1.0)
        
        # Set the BNO055 to CONFIG mode
        config_mode = 0b00000000
        self.bno_usb_stick.write_register(opr_mode_addr, config_mode)
        sleep(0.1)
        mode = self.bno_usb_stick.read_register(opr_mode_addr)
        self.get_logger().info(f"Config mode: {mode:08b}")
        
        # Set the Default UNITS
        unit_sel_addr = 0x3B
        unit_sel = self.bno_usb_stick.read_register(unit_sel_addr)
        self.get_logger().info(f"Initial unit selection: {unit_sel:08b}")
        
        si_units = 0b00000110
        self.bno_usb_stick.write_register(unit_sel_addr, si_units)
        sleep(0.1)
        
        # Set calibration parameters
        acc_offset_x_addr_LSB = 0x55
        acc_offset_x_addr_MSB = 0x56
        acc_offset_y_addr_LSB = 0x57
        acc_offset_y_addr_MSB = 0x58
        acc_offset_z_addr_LSB = 0x59
        acc_offset_Z_addr_MSB = 0x5A
        gyr_offset_x_addr_LSB = 0x61
        gyr_offset_x_addr_MSB = 0x62
        gyr_offset_y_addr_LSB = 0x63
        gyr_offset_y_addr_MSB = 0x64
        gyr_offset_z_addr_LSB = 0x65
        gyr_offset_z_addr_MSB = 0x66
        acc_radius_addr_LSB = 0x67
        acc_radius_addr_MSB = 0x68
        mag_radius_addr_LSB = 0x69
        mag_radius_addr_MSB = 0x6A
        
        self.bno_usb_stick.write_register(acc_offset_x_addr_LSB, 0xFA)
        self.bno_usb_stick.write_register(acc_offset_x_addr_MSB, 0xFF)
        self.bno_usb_stick.write_register(acc_offset_y_addr_LSB, 0xFD)
        self.bno_usb_stick.write_register(acc_offset_y_addr_MSB, 0xFF)
        self.bno_usb_stick.write_register(acc_offset_z_addr_LSB, 0xDF)
        self.bno_usb_stick.write_register(acc_offset_Z_addr_MSB, 0xFF)
        self.bno_usb_stick.write_register(gyr_offset_x_addr_LSB, 0xFF)
        self.bno_usb_stick.write_register(gyr_offset_x_addr_MSB, 0xFF)
        self.bno_usb_stick.write_register(gyr_offset_y_addr_LSB, 0x03)
        self.bno_usb_stick.write_register(gyr_offset_y_addr_MSB, 0x00)
        self.bno_usb_stick.write_register(gyr_offset_z_addr_LSB, 0x00)
        self.bno_usb_stick.write_register(gyr_offset_z_addr_MSB, 0x00)
        self.bno_usb_stick.write_register(acc_radius_addr_LSB, 0xE8)
        self.bno_usb_stick.write_register(acc_radius_addr_MSB, 0x03)
        self.bno_usb_stick.write_register(mag_radius_addr_LSB, 0xE0)
        self.bno_usb_stick.write_register(mag_radius_addr_MSB, 0x01)
        sleep(0.1)
           
        # Set the BNO055 to IMU mode
        imu_mode = 0b00001000 # 00000101
        self.bno_usb_stick.write_register(opr_mode_addr, imu_mode)
        sleep(0.1)
        for addr in [acc_offset_x_addr_LSB, acc_offset_x_addr_MSB, acc_offset_y_addr_LSB, acc_offset_y_addr_MSB,
                      acc_offset_z_addr_LSB, acc_offset_Z_addr_MSB, gyr_offset_x_addr_LSB, gyr_offset_x_addr_MSB,
                      gyr_offset_y_addr_LSB, gyr_offset_y_addr_MSB, gyr_offset_z_addr_LSB, gyr_offset_z_addr_MSB,
                      acc_radius_addr_LSB, acc_radius_addr_MSB, mag_radius_addr_LSB, mag_radius_addr_MSB]:
            value = self.bno_usb_stick.read_register(addr)
            self.get_logger().info(f"Calibration register {addr:02X}: {value:08b}")
            
        mode = self.bno_usb_stick.read_register(opr_mode_addr)
        self.get_logger().info(f"IMU mode: {mode:08b}")
        unit_sel = self.bno_usb_stick.read_register(unit_sel_addr)
        self.get_logger().info(f"Unit selection: {unit_sel:08b}")
    
    def combine_lsb_msb(self, lsb, msb):
        value = (msb << 8) | lsb
        if value >= 32768:
            value -= 65536
        return value
    
    def imu_streaming(self):
        while rclpy.ok():
            imu = self.get_imu_data()
            with self.data_lock:
                self.imu_data = imu
            sleep(1/100)
        
    def get_imu_data(self):
        # acc_x = self.combine_lsb_msb(self.bno_usb_stick.read_register(0x08), self.bno_usb_stick.read_register(0x09)) / 100
        # acc_y = self.combine_lsb_msb(self.bno_usb_stick.read_register(0x0A), self.bno_usb_stick.read_register(0x0B)) / 100
        # acc_z = self.combine_lsb_msb(self.bno_usb_stick.read_register(0x0C), self.bno_usb_stick.read_register(0x0D)) / 100
        gyr_x = self.combine_lsb_msb(self.bno_usb_stick.read_register(0x14), self.bno_usb_stick.read_register(0x15)) / 900
        gyr_y = self.combine_lsb_msb(self.bno_usb_stick.read_register(0x16), self.bno_usb_stick.read_register(0x17)) / 900
        gyr_z = self.combine_lsb_msb(self.bno_usb_stick.read_register(0x18), self.bno_usb_stick.read_register(0x19)) / 900
        qua_w = self.combine_lsb_msb(self.bno_usb_stick.read_register(0x20), self.bno_usb_stick.read_register(0x21)) / 16384
        qua_x = self.combine_lsb_msb(self.bno_usb_stick.read_register(0x22), self.bno_usb_stick.read_register(0x23)) / 16384
        qua_y = self.combine_lsb_msb(self.bno_usb_stick.read_register(0x24), self.bno_usb_stick.read_register(0x25)) / 16384
        qua_z = self.combine_lsb_msb(self.bno_usb_stick.read_register(0x26), self.bno_usb_stick.read_register(0x27)) / 16384
        lin_x = self.combine_lsb_msb(self.bno_usb_stick.read_register(0x28), self.bno_usb_stick.read_register(0x29)) / 100
        lin_y = self.combine_lsb_msb(self.bno_usb_stick.read_register(0x2A), self.bno_usb_stick.read_register(0x2B)) / 100
        lin_z = self.combine_lsb_msb(self.bno_usb_stick.read_register(0x2C), self.bno_usb_stick.read_register(0x2D)) / 100
        
        # streaming_data = [acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, qua_w, qua_x, qua_y, qua_z, lin_x, lin_y, lin_z]
        # streaming_data = [gyr_x, gyr_y, gyr_z, qua_w, qua_x, qua_y, qua_z, lin_x, lin_y, lin_z]
        
        return [gyr_x, gyr_y, gyr_z, qua_w, qua_x, qua_y, qua_z, lin_x, lin_y, lin_z]

            
    def timer_callback(self):
        with self.data_lock:
            if self.imu_data is not None:
                imu = self.imu_data
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_link'
                imu_msg.orientation.w = imu[3]
                imu_msg.orientation.x = imu[4]
                imu_msg.orientation.y = imu[5]
                imu_msg.orientation.z = imu[6]
                imu_msg.angular_velocity.x = imu[0]
                imu_msg.angular_velocity.y = imu[1]
                imu_msg.angular_velocity.z = imu[2]
                imu_msg.linear_acceleration.x = imu[7]
                imu_msg.linear_acceleration.y = imu[8]
                imu_msg.linear_acceleration.z = imu[9]
                self.publisher.publish(imu_msg)
                
                # t = TransformStamped()
                # t.header.stamp = self.get_clock().now().to_msg()
                # t.header.frame_id = "origin_link"
                # t.child_frame_id = "imu_link"

                # t.transform.translation.x = 
                # t.transform.translation.y = self.robot_wheelodom.y
                # t.transform.translation.z = 0.0
                
                # q = quaternion_from_euler(0, 0, self.robot_wheelodom.theta)

                # t.transform.rotation.x = q[0]
                # t.transform.rotation.y = q[1]
                # t.transform.rotation.z = q[2]
                # t.transform.rotation.w = q[3]

                # self.tf_broadcaster.sendTransform(t)
        
                # imu_msg = Imu()
                # imu_msg.header.stamp = self.get_clock().now().to_msg()
                # imu_msg.header.frame_id = 'imu_link'
                # imu_msg.orientation.x = self.prev_package.quaternion[1]
                # imu_msg.orientation.y = self.prev_package.quaternion[2]
                # imu_msg.orientation.z = self.prev_package.quaternion[3]
                # imu_msg.orientation.w = self.prev_package.quaternion[0]
                # imu_msg.angular_velocity.x = self.prev_package.g[0]
                # imu_msg.angular_velocity.y = self.prev_package.g[1]
                # imu_msg.angular_velocity.z = self.prev_package.g[2]
                # imu_msg.linear_acceleration.x = self.prev_package.lin_a[0]
                # imu_msg.linear_acceleration.y = self.prev_package.lin_a[1]
                # imu_msg.linear_acceleration.z = self.prev_package.lin_a[2]
                
                # imu_msg = Imu()
                # imu_msg.header.stamp = self.get_clock().now().to_msg()
                # imu_msg.header.frame_id = 'imu_link'
                # imu_msg.orientation.w = imu[6]
                # imu_msg.orientation.x = imu[7]
                # imu_msg.orientation.y = imu[8]
                # imu_msg.orientation.z = imu[9]
                # imu_msg.angular_velocity.x = imu[3]
                # imu_msg.angular_velocity.y = imu[4]
                # imu_msg.angular_velocity.z = imu[5]
                # imu_msg.linear_acceleration.x = imu[10]
                # imu_msg.linear_acceleration.y = imu[11]
                # imu_msg.linear_acceleration.z = imu[12]
    
def main(args=None):
    rclpy.init(args=args)
    node = BNO055USBSTICKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
