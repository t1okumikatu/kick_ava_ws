## 3. 簡単なテストスクリプトで確認
# lidar_test.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LidarTest(Node):
    def __init__(self):
        super().__init__('lidar_test')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(LaserScan, '/scan', self.callback, qos)
        print("LiDAR待機中...")

    def callback(self, msg):
        valid = [r for r in msg.ranges if 0.05 < r < 10.0]
        min_dist = min(valid) if valid else None
        print(f"✅ LiDAR受信OK | 点群数: {len(msg.ranges)} | 最小距離: {min_dist:.2f}m")

rclpy.init()
node = LidarTest()
rclpy.spin(node)

