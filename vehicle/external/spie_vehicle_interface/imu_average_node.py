import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_average_node')
        
        # 訂閱 /imu 話題，隊列長度為 10
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.listener_callback,
            10)
        
        # 初始化統計變數
        self.sum_x = 0.0
        self.sum_y = 0.0
        self.sum_z = 0.0
        self.count = 0
        
        self.get_logger().info('IMU 平均值計算節點已啟動...')

    def listener_callback(self, msg):
        # 取得角速度 (Angular Velocity)
        gyro = msg.angular_velocity
        
        # 累加數值
        self.sum_x += gyro.x
        self.sum_y += gyro.y
        self.sum_z += gyro.z
        self.count += 1
        
        # 計算平均值
        avg_x = self.sum_x / self.count
        avg_y = self.sum_y / self.count
        avg_z = self.sum_z / self.count
        
        # 每收到 20 筆資料印出一次結果，避免洗板
        if self.count % 500 == 0:
            self.get_logger().info(
                f'已接收 {self.count} 筆 | 平均角速度: '
                f'x:{avg_x:.5f}, y:{avg_y:.5f}, z:{avg_z:.5f}'
            )

def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuSubscriber()
    
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        print("\n使用者停止執行。")
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
