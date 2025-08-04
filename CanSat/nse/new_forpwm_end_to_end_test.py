import serial
import time
import math
import threading
import datetime
import pigpio
import csv
import os
import cv2  # [修正] cv2ライブラリをインポート
import pynmea2
from library import BNO055
from library import BMP180
from library import detect_corn as dc
from picamera2 import Picamera2
import sys

# --- 定数 ---
# [整理] 各種設定値を整理
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'log')
TARGET_LAT = 38.266285
TARGET_LNG = 140.855498
CALIBRATION_MILLITIME = 20 * 1000
DATA_SAMPLING_RATE = 0.1  # 10Hz

# ピン定義
AIN1, AIN2, PWMA = 8, 25, 18
BIN1, BIN2, PWMB = 9, 11, 19
STBY = 22
LED1, LED2 = 16, 20
TRIG_PIN, ECHO_PIN = 15, 14

# モーターデューティ
MAX_DUTY = 106
TURN_DUTY = 90
ROTATE_DUTY = 90

# --- グローバル変数 ---
pi = None
detector = None
picam2 = None # [追加] カメラオブジェクト用
bno = None    # [変更] 初期値はNoneに
bmp = None    # [変更] 初期値はNoneに

# センサー・GPSデータ
acc, gyro, mag = [0.0]*3, [0.0]*3, [0.0]*3
calibBias, calibRange = [0.0]*3, [1.0]*3
lat, lng, alt, pres = 0.0, 0.0, 0.0, 0.0
gps_detect = False

# 計算・状態データ
distance_to_goal = 0.0
angle_to_goal = 0.0
azimuth = 0.0
direction = 360.0  # 360:停止, 0-359:角度
phase = 0

# [追加] 共有リソース
log_file_name = ""
data_lock = threading.Lock() # 共有データ保護のためのロック


def main():
    global phase, direction
    Setup()
    phase = 1 # 開始フェーズ
    try:
        while True:
            # 各スレッドが自律的に動作するため、mainループは状態を監視・制御するだけになる
            with data_lock:
                current_phase = phase
                dist = distance_to_goal
                current_azimuth = azimuth
                goal_angle = angle_to_goal

            if current_phase == 1:
                print("Phase 1: Parachute Separation")
                with data_lock: direction = -400.0
                time.sleep(4)
                with data_lock: phase = 2
            
            elif current_phase == 2:
                print("Phase 2: Calibration")
                with data_lock: direction = -400.0
                calibration()
                with data_lock: phase = 3
            
            elif current_phase == 3:
                print(f"Phase 3: GPS Navigation (Dist: {dist:.2f} m)")
                if dist < 4.0:
                    print("GPS goal reached. Switching to cone search.")
                    with data_lock: phase = 4
                    continue
                
                # GPS誘導ロジック
                diff_angle = (goal_angle - current_azimuth + 180) % 360 - 180
                new_direction = diff_angle if abs(diff_angle) >= 10.0 else 0.0
                with data_lock: direction = new_direction

            elif current_phase == 4:
                # このフェーズのロジックはvision_threadに移行した
                # vision_threadがphaseを5に変更するのを待つ
                print("Phase 4: Cone Search & Track (vision_thread is active)")
                time.sleep(1) # 待機中の負荷を下げる
            
            elif current_phase == 5:
                print("Phase 5: Goal! Mission accomplished.")
                with data_lock: direction = 360.0
                # ゴール後はループを抜けて終了
                break

            time.sleep(0.1)
    finally:
        print("Stopping motors and releasing resources.")
        if pi and pi.connected:
            pi.write(STBY, 0)
            pi.stop()
        if picam2:
            picam2.stop()
        print("All resources released.")

def Setup():
    global pi, bno, bmp, detector, picam2, log_file_name
    
    # ログディレクトリ作成
    os.makedirs(LOG_DIR, exist_ok=True)

    # pigpio初期化
    pi = pigpio.pi()
    if not pi.connected: sys.exit("pigpio connection failed.")

    # カメラ初期化
    print("Camera setup starting...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    time.sleep(2.0)
    print("Camera setup OK.")

    # センサー初期化 (piオブジェクト生成後に行う)
    print("BNO055 setup starting...")
    bno = BNO055.BNO055(pi)
    if not bno.setUp(): sys.exit("BNO055 setup failed.")
    
    print("BMP180 setup starting...")
    bmp = BMP180.BMP180(pi)
    if not bmp.setUp(): sys.exit("BMP180 setup failed.")
    print("Sensors setup OK.")
    
    # モーターピン設定
    for pin in [AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY, LED1, LED2, TRIG_PIN]:
        pi.set_mode(pin, pigpio.OUTPUT)
    pi.set_mode(ECHO_PIN, pigpio.INPUT)
    pi.set_PWM_frequency(PWMA, 5000)
    pi.set_PWM_frequency(PWMB, 5000)
    pi.write(STBY, 1)

    # 画像認識器の初期化
    roi_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'library', 'roi_red_cone.png')
    roi_img = cv2.imread(roi_path)
    if roi_img is None: sys.exit(f"Error: Could not load ROI image at {roi_path}")
    detector = dc.detector()
    detector.set_roi_img(roi_img)

    # ログファイル準備 (ファイル名をグローバル変数に保存)
    log_file_name = os.path.join(LOG_DIR, 'testlog_' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S') + '.csv')
    with open(log_file_name, 'w', newline="") as f: # 'a'から'w'に変更し、初回書き込み
        csv.writer(f).writerow(['MilliTime','Phase','AccX','AccY','AccZ','GyroX','GyroY','GyroZ','MagX','MagY','MagZ','LAT','LNG','ALT','Distance','Azimuth','Angle','Direction'])

    # 全てのスレッドを開始
    print("Starting threads...")
    threading.Thread(target=moveMotor_thread, daemon=True).start()
    threading.Thread(target=data_thread, daemon=True).start()
    threading.Thread(target=gps_thread, daemon=True).start()
    threading.Thread(target=vision_thread, daemon=True).start() # vision_threadを開始

    pi.write(LED2, 1)
    print("Setup OK")

# [整理] データ取得・計算関数群
def get_all_data():
    global acc, gyro, mag, alt, pres, distance_to_goal, angle_to_goal, azimuth
    acc, gyro, mag = bno.getAcc(), bno.getGyro(), bno.getMag()
    for i in range(3):
        mag[i] = (mag[i] - calibBias[i]) / calibRange[i] if calibRange[i] != 0 else 0
    alt, pres = bmp.getAltitude() + 60, bmp.getPressure()
    if gps_detect:
        # ゴールまでの距離計算
        EARTH_RADIUS = 6378136.59
        dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng) * math.cos(math.radians((TARGET_LAT + lat) / 2))
        dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        # ゴールまでの方位角計算
        y = math.sin(math.radians(TARGET_LNG - lng)) * math.cos(math.radians(TARGET_LAT))
        x = math.cos(math.radians(lat)) * math.sin(math.radians(TARGET_LAT)) - math.sin(math.radians(lat)) * math.cos(math.radians(TARGET_LAT)) * math.cos(math.radians(TARGET_LNG - lng))
        angle_to_goal = (math.degrees(math.atan2(y, x)) + 360) % 360
    # 機体の向き（磁北基準）
    azimuth = (math.degrees(math.atan2(mag[1], mag[0] if mag[0]!=0 else 0.00001)) + 360) % 360

def calibration():
    # (内容は変更なし)
    global calibBias, calibRange, mag
    max_val, min_val = [0.0] * 3, [0.0] * 3
    print("Calibration starting...")
    start_time = round(time.time() * 1000)
    while (round(time.time() * 1000) - start_time) < CALIBRATION_MILLITIME:
        get_all_data()
        for i in range(3):
            if mag[i] > max_val[i]: max_val[i] = mag[i]
            if mag[i] < min_val[i]: min_val[i] = mag[i]
        time.sleep(0.01)
    for i in range(3):
        calibBias[i] = (max_val[i] + min_val[i]) / 2
        calibRange[i] = (max_val[i] - min_val[i]) / 2
        if calibRange[i] == 0: calibRange[i] = 1.0
    print("Calibration complete!")

# [整理] スレッド関数群
def gps_thread():
    global lat, lng, gps_detect
    s = serial.Serial('/dev/serial0', 115200, timeout=1)
    while True:
        try:
            line = s.readline().decode('utf-8', errors='ignore')
            if line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)
                with data_lock: # ロックして共有変数を更新
                    if msg.latitude != 0.0:
                        lat, lng, gps_detect = msg.latitude, msg.longitude, True
                    else: gps_detect = False
            else:
                with data_lock: gps_detect = False
        except Exception as e:
            with data_lock: gps_detect = False
            # print(f"GPS Error: {e}") # デバッグ用にエラー表示を入れると良い
        time.sleep(0.1)

def data_thread():
    # log_file_name はグローバル変数を使用
    while True:
        # ロックして共有変数を一貫した状態で取得・計算・保存
        with data_lock:
            get_all_data()
            with open(log_file_name, 'a', newline="") as f:
                csv.writer(f).writerow([round(time.time()*1000), round(phase, 1), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], lat, lng, alt, distance_to_goal, azimuth, angle_to_goal, direction])
        time.sleep(DATA_SAMPLING_RATE)

def vision_thread():
    global phase, direction
    while True:
        # 現在のフェーズが4 (コーン探索) でなければ何もしない
        with data_lock:
            current_phase = phase
        if current_phase != 4:
            time.sleep(0.2)
            continue

        # 画像を取得してコーンを検出
        frame = picam2.capture_array()
        # detectorにフレームを渡す方法はライブラリの仕様に合わせてください
        # ここではdetectorオブジェクトが内部でフレームを保持すると仮定
        detector.frame = frame 
        detector.detect_cone()

        # 検出結果に基づき、次の行動を決定
        new_direction = -400.0 # デフォルトは探索回転
        next_phase = 4 # デフォルトはフェーズ維持

        if detector.is_reached:
            print("Cone Reached! Goal!")
            new_direction = 360.0 # 停止
            next_phase = 5
        elif detector.is_detected:
            # コーン中心からのズレで方向決定
            if detector.cone_direction > 0.55: new_direction = 45  # 右へ
            elif detector.cone_direction < 0.45: new_direction = -45 # 左へ
            else: new_direction = 0.0 # 前進
        
        # 共有変数をロックして更新
        with data_lock:
            direction = new_direction
            phase = next_phase
        
        time.sleep(0.1)

def moveMotor_thread():
    while True:
        # ロックしてdirection変数を安全に読み取る
        with data_lock:
            current_direction = direction
        
        # 読み取った値に基づいてモーターを制御
        if current_direction == 360.0:  # 停止
            pi.write(AIN1, 0); pi.write(AIN2, 0); pi.write(BIN1, 0); pi.write(BIN2, 0)
            pi.set_PWM_dutycycle(PWMA, 0); pi.set_PWM_dutycycle(PWMB, 0)
        elif current_direction == 0.0:  # 前進
            pi.write(AIN1, 0); pi.write(AIN2, 1); pi.write(BIN1, 1); pi.write(BIN2, 0)
            pi.set_PWM_dutycycle(PWMA, MAX_DUTY); pi.set_PWM_dutycycle(PWMB, MAX_DUTY)
        elif current_direction == -400.0:  # その場右回転
            pi.write(AIN1, 0); pi.write(AIN2, 1); pi.write(BIN1, 0); pi.write(BIN2, 1)
            pi.set_PWM_dutycycle(PWMA, ROTATE_DUTY); pi.set_PWM_dutycycle(PWMB, ROTATE_DUTY)
        elif current_direction > 0:  # 右へカーブ
            pi.write(AIN1, 0); pi.write(AIN2, 1); pi.write(BIN1, 1); pi.write(BIN2, 0)
            pi.set_PWM_dutycycle(PWMA, MAX_DUTY); pi.set_PWM_dutycycle(PWMB, TURN_DUTY)
        elif current_direction < 0:  # 左へカーブ
            pi.write(AIN1, 0); pi.write(AIN2, 1); pi.write(BIN1, 1); pi.write(BIN2, 0)
            pi.set_PWM_dutycycle(PWMA, TURN_DUTY); pi.set_PWM_dutycycle(PWMB, MAX_DUTY)
        
        time.sleep(0.02)

if __name__ == '__main__':
    main()
