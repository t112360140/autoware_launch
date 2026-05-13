import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import Imu

import struct

import threading
import time
from .serial_sync import SERIAL_SYNC

import math

CAR_WHEEL_BASE = 1.64

class InterfaceNode(Node):
    def __init__(self):
        super().__init__('interface_imu_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.get_logger().info(f'Connecting to {port} at {baud}...')
        
        self.declare_parameter('gyro_base_x', 0.0)
        self.declare_parameter('gyro_base_y', 0.0)
        self.declare_parameter('gyro_base_z', 0.0)
        self.gyro_base_x = self.get_parameter('gyro_base_x').get_parameter_value().double_value
        self.gyro_base_y = self.get_parameter('gyro_base_y').get_parameter_value().double_value
        self.gyro_base_z = self.get_parameter('gyro_base_z').get_parameter_value().double_value

        self.frame_id = "base_link"

        self.serial_device = SERIAL_SYNC(PORT=port, BAUD_RATE=baud, event=self.serial_callback)
        
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 1)

        # self.timeout_timer = self.create_timer(0.01, self.timeout_callback)
        
        self.error_log_timer = self.create_timer(0.5, self.error_log_callback)

        self.is_running = True
        self._serial_thread = threading.Thread(target=self.serial_loop)
        self._serial_thread.daemon = True # 設為 Daemon，主程式結束時此執行緒會自動結束
        self._serial_thread.start()
        
        self.last_accel_data = None
    
    def serial_loop(self):
        while rclpy.ok() and self.is_running:
            try:
                self.serial_device.update()
                time.sleep(0.001) 
            except Exception as e:
                if self.is_running:
                    self.get_logger().error(f"Error in serial loop: {e}")
                for _ in range(10):
                    if not self.is_running:
                        break
                    time.sleep(0.1)
    
    def timeout_callback(self):
        if not self.serial_device.ok():
            self.get_logger().error(f"Serlai Error!")
            
    def error_log_callback(self):
        if not self.serial_device.ok():
            self.get_logger().error(f"Serial Error!")

    def serial_callback(self, id, data, len):
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self.frame_id)
        
        if id==0x300 and len==12:
            self.last_accel_data = data
        elif id==0x301 and len==12:
            if self.last_accel_data is None:
                return
                
            imu_msg = Imu()
            imu_msg.header = header
            imu_msg.linear_acceleration.x = struct.unpack("<f", self.last_accel_data[0:4])[0]
            imu_msg.linear_acceleration.y = struct.unpack("<f", self.last_accel_data[4:8])[0]
            imu_msg.linear_acceleration.z = struct.unpack("<f", self.last_accel_data[8:12])[0]
            imu_msg.angular_velocity.x = struct.unpack("<f", data[0:4])[0]-self.gyro_base_x
            imu_msg.angular_velocity.y = struct.unpack("<f", data[4:8])[0]-self.gyro_base_y
            imu_msg.angular_velocity.z = struct.unpack("<f", data[8:12])[0]-self.gyro_base_z
            imu_msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            imu_msg.linear_acceleration_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            imu_msg.orientation_covariance[0] = -1
            self.imu_pub.publish(imu_msg)
        
    def destroy_node(self):
        self.is_running = False
        self.serial_device.close()
        if self._serial_thread.is_alive():
            self._serial_thread.join(timeout=1.0)
            
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
