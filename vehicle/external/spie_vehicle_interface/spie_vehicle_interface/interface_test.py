import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
# 新增 Imu 訊息格式
from sensor_msgs.msg import Imu

from autoware_vehicle_msgs.msg import GearCommand, TurnIndicatorsCommand, HazardLightsCommand, Engage
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
from autoware_vehicle_msgs.msg import ControlModeReport, VelocityReport, SteeringReport, GearReport, TurnIndicatorsReport, HazardLightsReport
from autoware_vehicle_msgs.srv import ControlModeCommand

from autoware_control_msgs.msg import Control

import struct

import threading
import time
from .serial_sync import SERIAL_SYNC

import math

CAR_WHEEL_BASE = 1.64

class InterfaceTestNode(Node):
    def __init__(self):
        super().__init__('interface_test_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.get_logger().info(f'Connecting to {port} at {baud}...')

        self.frame_id = "base_link"

        self.serial_device = SERIAL_SYNC(PORT=port, BAUD_RATE=baud, event=self.serial_callback)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.ctrl_cmd_sub = self.create_subscription(Control, "/control/command/control_cmd", self.ctrl_cmd_callback, qos_profile)
        self.gear_cmd_sub = self.create_subscription(GearCommand, "/control/command/gear_cmd", self.gear_cmd_callback, qos_profile)
        self.turn_indicators_cmd_sub = self.create_subscription(TurnIndicatorsCommand, "/control/command/turn_indicators_cmd", self.turn_indicators_cmd_callback, qos_profile)
        self.hazard_lights_cmd_sub = self.create_subscription(HazardLightsCommand, "/control/command/hazard_lights_cmd", self.hazard_lights_cmd_callback, qos_profile)
        self.engage_sub = self.create_subscription(Engage, "/vehicle/engage", self.engage_callback, qos_profile)
        self.emergency_cmd_sub = self.create_subscription(VehicleEmergencyStamped, "/control/command/emergency_cmd", self.emergency_cmd_callback, qos_profile)

        self.ctrl_mode_pub = self.create_publisher(ControlModeReport, "/vehicle/status/control_mode", 1)
        self.velo_repo_pub = self.create_publisher(VelocityReport, "/vehicle/status/velocity_status", 1)
        self.ster_repo_pub = self.create_publisher(SteeringReport, "/vehicle/status/steering_status", 1)
        self.gear_repo_pub = self.create_publisher(GearReport, "/vehicle/status/gear_status", 1)
        self.turn_repo_pub = self.create_publisher(TurnIndicatorsReport, "/vehicle/status/turn_indicators_status", 1)
        self.haza_repo_pub = self.create_publisher(HazardLightsReport, "/vehicle/status/hazard_lights_status", 1)
        
        self.srv_mode_req = self.create_service(ControlModeCommand, "/control/control_mode_request", self.on_control_mode_request)

        # 新增 IMU Publisher
        self.imu_pub = self.create_publisher(Imu, "/sensing/imu/imu_data", 1)

        self.timeout_timer = self.create_timer(0.01, self.timeout_callback)
        # self.fake_data_timer = self.create_timer(0.01, self.fake_data_callback)

        self._serial_thread = threading.Thread(target=self.serial_loop)
        self._serial_thread.daemon = True 
        self._serial_thread.start()
    
    def serial_loop(self):
        while rclpy.ok():
            try:
                self.serial_device.update()
                time.sleep(0.001) 
            except Exception as e:
                self.get_logger().error(f"Error in serial loop: {e}")
                time.sleep(1)
    
    def timeout_callback(self):
        if not self.serial_device.ok():
            ctrl_mode = ControlModeReport(
                stamp=self.get_clock().now().to_msg(),
                mode = ControlModeReport.NOT_READY
            )
            self.ctrl_mode_pub.publish(ctrl_mode)

    def on_control_mode_request(self, request, response):
        response.success = True
        self.get_logger().info(f"Autoware requested mode: {request.mode}, responding SUCCESS")
        return response

    def fake_data_callback(self):
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self.frame_id)

        speed = 1.0
        angle = 0.26

        ctrl_mode = ControlModeReport()
        ctrl_mode.stamp = stamp
        ctrl_mode.mode = ControlModeReport.MANUAL
        self.ctrl_mode_pub.publish(ctrl_mode)

        velo_repo = VelocityReport()
        velo_repo.header = header
        velo_repo.longitudinal_velocity = speed
        velo_repo.lateral_velocity = 0.0
        
        velo_repo.heading_rate = (velo_repo.longitudinal_velocity/CAR_WHEEL_BASE)*math.tan(angle)
        self.velo_repo_pub.publish(velo_repo)

        ster_repo = SteeringReport()
        ster_repo.stamp = stamp
        ster_repo.steering_tire_angle = angle
        
        self.ster_repo_pub.publish(ster_repo)

        gear_repo = GearReport()
        gear_repo.stamp = stamp
        gear_repo.report = GearReport.DRIVE
        self.gear_repo_pub.publish(gear_repo)

        turn_repo = TurnIndicatorsReport()
        turn_repo.stamp = stamp
        turn_repo.report = 1
        self.turn_repo_pub.publish(turn_repo)

        haza_repo = HazardLightsReport()
        haza_repo.stamp = stamp
        haza_repo.report = 3
        self.haza_repo_pub.publish(haza_repo)

        self.publish_fake_imu(stamp, velo_repo.heading_rate)



    def publish_fake_imu(self, stamp, yaw_rate):
        """
        根據當前的速度與方向盤角度，計算虛擬的 IMU 數據並發布
        """
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id  # 或是 "imu_link"，視您的 TF tree 而定

        imu_msg.angular_velocity.z = yaw_rate
        # 其他軸設為 0
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0

        # 線性加速度 (如果沒有真實數據，設為 0 即可，但 Covariance 不能為 0)
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81 # 模擬重力

        # 設定 Covariance (關鍵！EKF 需要這個來決定是否信任數據)
        # 對角線設為較小的值 (例如 0.01)，代表我們信任這個數據
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
        # Orientation 通常不準，設 -1 代表無效，或者設大的 Covariance
        imu_msg.orientation_covariance[0] = -1

        self.imu_pub.publish(imu_msg)

    def serial_callback(self, id, data, len):
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self.frame_id)
        
        if id==0x110 and len==1:
            ctrl_mode = ControlModeReport()
            ctrl_mode.stamp = stamp
            ctrl_mode.mode = struct.unpack("B", data)[0]
            self.ctrl_mode_pub.publish(ctrl_mode)

        elif id==0x111 and len==8:
            velo_repo = VelocityReport()
            velo_repo.header = header
            velo_repo.longitudinal_velocity = struct.unpack("<f", data[0:4])[0]
            velo_repo.lateral_velocity = 0.0
            velo_repo.heading_rate = (velo_repo.longitudinal_velocity/CAR_WHEEL_BASE)*math.tan(struct.unpack("<f", data[4:8])[0])
            self.velo_repo_pub.publish(velo_repo)
            # 發布同步的 Fake IMU
            self.publish_fake_imu(stamp, velo_repo.heading_rate)
        elif id==0x112 and len==4:
            ster_repo = SteeringReport()
            ster_repo.stamp = stamp
            ster_repo.steering_tire_angle = struct.unpack("<f", data)[0]
            self.ster_repo_pub.publish(ster_repo)
        elif id==0x113 and len==1:
            gear_repo = GearReport()
            gear_repo.stamp = stamp
            gear_repo.report = struct.unpack("B", data)[0]
            self.gear_repo_pub.publish(gear_repo)
        elif id==0x114 and len==1:
            turn_repo = TurnIndicatorsReport()
            turn_repo.stamp = stamp
            turn_repo.report = struct.unpack("B", data)[0]
            self.turn_repo_pub.publish(turn_repo)
        elif id==0x115 and len==1:
            haza_repo = HazardLightsReport()
            haza_repo.stamp = stamp
            haza_repo.report = struct.unpack("B", data)[0]
            self.haza_repo_pub.publish(haza_repo)
    
    def ctrl_cmd_callback(self, msg: Control):
        self.serial_device.write(0x100, struct.pack("<f", msg.lateral.steering_tire_angle)+struct.pack("<f", msg.longitudinal.velocity), 8)
    def gear_cmd_callback(self, msg: GearCommand):
        self.serial_device.write(0x101, struct.pack("B", msg.command), 1)
    def turn_indicators_cmd_callback(self, msg: TurnIndicatorsCommand):
        self.serial_device.write(0x102, struct.pack("B", msg.command), 1)
    def hazard_lights_cmd_callback(self, msg: HazardLightsCommand):
        self.serial_device.write(0x103, struct.pack("B", msg.command), 1)
    def engage_callback(self, msg: Engage):
        self.serial_device.write(0x104, struct.pack("B", msg.engage), 1)
    def emergency_cmd_callback(self, msg: VehicleEmergencyStamped):
        self.serial_device.write(0x105, struct.pack("B", msg.emergency), 1)

def main(args=None):
    rclpy.init(args=args)
    node = InterfaceTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
