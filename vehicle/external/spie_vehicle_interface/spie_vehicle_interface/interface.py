import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from autoware_vehicle_msgs.msg import ControlModeReport

from autoware_control_msgs.msg import Control

import struct

import threading
import time
from .serial_sync import SERIAL_SYNC

class InterfaceNode(Node):
    def __init__(self):
        super().__init__('interface_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.get_logger().info(f'Connecting to {port} at {baud}...')

        self.serial_device = SERIAL_SYNC(PORT=port, BAUD_RATE=baud, event=self.serial_callback)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.ctrl_cmd_sub = self.create_subscription(Control, "/control/command/control_cmd", self.ctrl_cmd_callback, qos_profile)

        self.ctrl_mode_pub = self.create_publisher(ControlModeReport, "/vehicle/status/control_mode", 1)

        self._serial_thread = threading.Thread(target=self.serial_loop)
        self._serial_thread.daemon = True # 設為 Daemon，主程式結束時此執行緒會自動結束
        self._serial_thread.start()
    
    def serial_loop(self):
        while rclpy.ok():
            try:
                self.serial_device.update()
                time.sleep(0.001) 
            except Exception as e:
                self.get_logger().error(f"Error in serial loop: {e}")
                time.sleep(1)

    def serial_callback(self, id, data, len):
        stamp = self.get_clock().now().to_msg()
        if id==0x100 and len==1:
            ctrl_mode = ControlModeReport()
            ctrl_mode.stamp = stamp
            ctrl_mode.mode = struct.unpack("B", data)[0]

            self.ctrl_mode_pub.publish(ctrl_mode)
        elif id==0x101 and len==1:
            pass
    
    def ctrl_cmd_callback(self, msg: Control):
        self.serial_device.write(0x100, struct.pack(">f", msg.lateral.steering_tire_angle, msg.longitudinal.velocity), 8)



def main(args=None):
    rclpy.init(args=args)
    node = InterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()