# -*- coding: utf-8 -*-
import sys, os, math, threading, json, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
import paho.mqtt.client as mqtt

class RobotHardware:
    def __init__(self): print("!!!! Robot Hardware Initialized !!!!")
    def run(self, left_rpm, right_rpm): print(f"Drive -> L: {int(left_rpm)} | R: {int(right_rpm)}")
    def run_stop(self): print("Drive -> STOP")

class MainGeometryNode(Node):
    def __init__(self):
        super().__init__('main_geometry_node')
        self.max_rpm = 1500
        self.watchdog_count = 0
        self.slowdown_factor = 1.0
        self.robot = RobotHardware()

        # MQTT受信設定
        self.mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.subscribe("robot/joystick")
        threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()

        # LiDAR設定
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        self.create_timer(0.05, self.timer_callback)

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            x, y = payload.get("x", 0.0), -payload.get("y", 0.0)
            v = y * self.max_rpm * self.slowdown_factor
            w = x * (self.max_rpm / 2)
            self.robot.run(v - w, v + w)
            self.watchdog_count = 0
        except: pass

    def lidar_callback(self, msg):
        d = min([r for r in msg.ranges if 0.05 < r < 2.0] + [2.0])
        self.slowdown_factor = 0.0 if d < 0.3 else 0.25 if d < 0.6 else 0.5 if d < 1.0 else 1.0

    def timer_callback(self):
        if self.watchdog_count > 10: self.robot.run_stop()
        self.watchdog_count += 1

if __name__ == "__main__":
    rclpy.init()
    node = MainGeometryNode()
    try: rclpy.spin(node)
    except: rclpy.shutdown()
