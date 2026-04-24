# -*- coding: utf-8 -*-
import sys
import os
import math
import threading
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
import paho.mqtt.client as mqtt
from time import sleep

# Robot2WD クラスをインポート
from robot_2wd_new import Robot2WD

class MainGeometryNode(Node):
    def __init__(self):
        super().__init__('main_geometry_node')
        
        # --- 設定 ---
        self.max_rpm = 1500  # 最高速度
        self.target_x = 0.0
        self.target_y = 0.0
        self.watchdog_count = 0
        self.WATCHDOG_MAX = 10 # 0.5秒間信号が途絶えたら停止
        self.slowdown_factor = 1.0 # LiDARによる減速係数

        # --- KeiganMotor初期化 (by-id パスを使用) ---
        # 動作実績のある固有IDパスを指定することで、入れ替わりや競合を防ぎます
        L_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0001KJH-if00-port0"
        R_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B003725S-if00-port0"

        try:
            print(f">>> Connecting to Left: {L_PORT}")
            print(f">>> Connecting to Right: {R_PORT}")
            self.robot = Robot2WD(port_left=L_PORT, port_right=R_PORT)
            
            # 確実に有効化するために少し待機
            sleep(0.5)
            self.robot.enable()
            print(">>> KeiganMotor: Connected and Enabled using by-id paths.")
        except Exception as e:
            print(f">>> KeiganMotor Connection Error: {e}")
            print("Hint: Check if the USB cables are connected and you have permissions.")
            sys.exit(1)

        # --- MQTT受信設定 ---
        self.mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_message = self.on_mqtt_message
        try:
            self.mqtt_client.connect("100.124.195.2", 1883, 60)
            self.mqtt_client.subscribe("robot/joystick", qos=0)
            threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()
            print(">>> MQTT Joystick Receiver: Online (Port 1883)")
        except Exception as e:
            print(f"MQTT Connection failed: {e}")

        # --- ROS 2 LiDAR設定 ---
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        
        # 制御ループ (20Hz)
        self.create_timer(0.05, self.control_loop)

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            self.target_x = payload.get("x", 0.0)
            self.target_y = -payload.get("y", 0.0) # 上下反転
            self.watchdog_count = 0 
        except Exception as e:
            print(f"MQTT Parse Error: {e}")

    def lidar_callback(self, msg):
        # 前方2m以内の最小距離を取得
        valid_ranges = [r for r in msg.ranges if 0.05 < r < 2.0]
        min_dist = min(valid_ranges) if valid_ranges else 2.0
        
        if min_dist < 0.3:
            self.slowdown_factor = 0.0
        elif min_dist < 0.6:
            self.slowdown_factor = 0.3
        elif min_dist < 1.0:
            self.slowdown_factor = 0.6
        else:
            self.slowdown_factor = 1.0

    def control_loop(self):
        # 通信途絶時の安全停止
        if self.watchdog_count > self.WATCHDOG_MAX:
            self.robot.run_stop()
            self.watchdog_count += 1
            return

        # 差動二輪モデルの回転数計算
        v = self.target_y * self.max_rpm * self.slowdown_factor
        w = self.target_x * (self.max_rpm * 0.5)
        
        left_rpm = v - w
        right_rpm = v + w

        # デッドゾーン判定
        if abs(v) < 50 and abs(w) < 50:
            self.robot.run_stop()
        else:
            # 実際のモーター命令
            self.robot.run(left_rpm, right_rpm)

        self.watchdog_count += 1

    def quit_app(self):
        print("Stopping robot and exiting...")
        try:
            self.robot.run_stop()
            self.robot.disable()
        except:
            pass
        rclpy.shutdown()

if __name__ == "__main__":
    rclpy.init()
    node = MainGeometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.quit_app()