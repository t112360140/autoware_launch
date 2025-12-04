import rclpy
from rclpy.node import Node

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
        pass


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