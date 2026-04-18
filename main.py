# -*- coding: utf-8 -*-
import sys
import os
import math
import threading
import requests
import json 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
from enum import Enum, auto
import paho.mqtt.client as mqtt
from time import sleep

# ==========================================
# ロボットコマンド定義
# ==========================================
class RobotCmd(Enum):
    DISABLE = auto()
    RUN_FORWARD = auto()
    RUN_BACKWARD = auto()
    RUN_TURN_LEFT = auto()
    RUN_TURN_RIGHT = auto()
    RUN_STOP = auto()

# 実機がない環境用のダミー（必要に応じて実機クラスへ差し替え）
class MockRobot:
    def __init__(self): print("Mock Robot Initialized")
    def enable(self): pass
    def disable(self): pass
    def run_straight(self, rpm): print(f"Motor: Straight {rpm} RPM")
    def run_pivot_turn(self, rpm): print(f"Motor: Turn {rpm} RPM")
    def run_stop(self): print("Motor: STOP")

# ==========================================
# メイン制御クラス
# ==========================================
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # 状態管理
        self.robot_cmd = RobotCmd.RUN_STOP
        self.robot_speed_rpm = 1500
        self.watchdog_count = 0
        self.WATCHDOG_MAX = 10 # 0.5秒間信号が途絶えたら停止
        
        # 障害物フラグ
        self.in_zone_1 = self.in_zone_2 = self.in_zone_3 = False

        # ロボット初期化
        self.robot = MockRobot()
        self.robot.enable()

        # --- MQTT受信設定 ---
        self.mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_message = self.on_mqtt_message
        try:
            self.mqtt_client.connect("localhost", 1883, 60)
            self.mqtt_client.subscribe("robot/joystick", qos=0)
            threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()
            print(">>> MQTT Joystick Receiver: OK (Listening on 1883)")
        except Exception as e:
            print(f"MQTT Connection failed: {e}")

        # --- ROS 2 Lidar設定 ---
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        
        # メインループ (20Hz = 0.05s)
        self.create_timer(0.05, self.robot_loop)

    def on_mqtt_message(self, client, userdata, msg):
        """ジョイスティック信号をロボットコマンドに変換"""
        try:
            data = json.loads(msg.payload.decode())
            axes = data.get("axes", [])
            if len(axes) > 1:
                lx, ly = axes[0], axes[1]
                
                if ly < -0.5:   self.robot_cmd = RobotCmd.RUN_FORWARD
                elif ly > 0.5:  self.robot_cmd = RobotCmd.RUN_BACKWARD
                elif lx < -0.5: self.robot_cmd = RobotCmd.RUN_TURN_LEFT
                elif lx > 0.5:  self.robot_cmd = RobotCmd.RUN_TURN_RIGHT
                else:           self.robot_cmd = RobotCmd.RUN_STOP
                
                self.watchdog_count = 0 # 信号受信でリセット
        except Exception as e:
            print(f"MQTT Parse Error: {e}")

    def lidar_callback(self, msg):
        """障害物検知ロジック"""
        self.in_zone_1 = self.in_zone_2 = self.in_zone_3 = False
        angle = msg.angle_min
        for r in msg.ranges:
            if 0.05 < r < 2.0:
                x = r * math.cos(angle); y = r * math.sin(angle)
                if x > 0: # 前方のみ
                    if x <= 0.3 and abs(y) <= 0.2: self.in_zone_3 = True # 至近
                    elif x <= 0.6 and abs(y) <= 0.3: self.in_zone_2 = True # 近
                    elif x <= 1.0 and abs(y) <= 0.5: self.in_zone_1 = True # 遠
            angle += msg.angle_increment

    def robot_loop(self):
        """安全装置と移動実行"""
        # ウォッチドッグ (通信途絶時に停止)
        if self.watchdog_count > self.WATCHDOG_MAX:
            self.robot.run_stop()
            return

        # 速度調整
        speed = self.robot_speed_rpm
        if self.in_zone_3 and self.robot_cmd in [RobotCmd.RUN_FORWARD, RobotCmd.RUN_TURN_LEFT, RobotCmd.RUN_TURN_RIGHT]:
            self.robot.run_stop() # 緊急停止
            return
        elif self.in_zone_2: speed *= 0.3
        elif self.in_zone_1: speed *= 0.6

        # 実行
        if self.robot_cmd == RobotCmd.RUN_FORWARD:   self.robot.run_straight(speed)
        elif self.robot_cmd == RobotCmd.RUN_BACKWARD:  self.robot.run_straight(-speed/2)
        elif self.robot_cmd == RobotCmd.RUN_TURN_LEFT:  self.robot.run_pivot_turn(-speed/2)
        elif self.robot_cmd == RobotCmd.RUN_TURN_RIGHT: self.robot.run_pivot_turn(speed/2)
        elif self.robot_cmd == RobotCmd.RUN_STOP:       self.robot.run_stop()

        self.watchdog_count += 1

    def quit(self):
        self.robot.run_stop()
        rclpy.shutdown()

if __name__ == "__main__":
    rclpy.init()
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.quit()