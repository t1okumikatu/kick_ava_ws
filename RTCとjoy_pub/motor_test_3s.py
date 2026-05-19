import time
from pykeigan import uartcontroller
# 既存の robot_2wd_new.py を流用します
#import motor_test.robot_2wd_new as robot_2wd_new
#import robot_2wd_new as robot_2wd_new  #　 同じフォルダー内にいるので motor_test. は不要
import robot_2wd_new

def simple_move_test():
    # 1. 動作実績のあるポートIDを指定
    L_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0001KJH-if00-port0"
    R_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B003725S-if00-port0"

    print("--- モーター動作テスト開始 (3秒間前進) ---")
    
    try:
        # 2. ロボットの初期化
        # 内部で enable_action() と set_acc/dec 等が実行されます
        robot = robot_2wd_new.Robot2WD(L_PORT, R_PORT)
        robot.enable()
        
        # 3. 前進命令 (RPM指定)
        # 動きが分かりやすいよう 100 RPM で設定
        test_rpm = 100
        print(f"走行開始: {test_rpm} RPM")
        robot.run(test_rpm, test_rpm)
        
        # 4. 3秒間待機
        time.sleep(3.0)
        
        # 5. 停止
        print("停止命令")
        robot.run_stop()
        robot.disable()
        
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    
    print("--- テスト終了 ---")

if __name__ == "__main__":
    simple_move_test()