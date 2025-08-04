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
acc, gyro, mag = [0.0]*3, [0.0]*3, [0.0]*3
calibBias, calibRange = [0.0]*3, [1.0]*3
lat, lng, alt, pres = 0.0, 0.0, 0.0, 0.0
distance_to_goal = 0.0
angle_to_goal = 0.0
azimuth = 0.0
direction = 360.0  # 360:停止, 0-359:角度
phase = 0
gps_detect = False

# --- インスタンス ---
bno = BNO055.BNO055()
bmp = BMP180.BMP180()

def main():
    global phase, direction
    Setup()
    phase = 1 # 開始フェーズ
    try:
        while True:
            if phase == 1:
                print("Phase 1: Parachute Separation")
                direction = -400.0 # [変更] 右回転を示す値（後述のmoveMotorで処理）
                time.sleep(4)
                phase = 2
            
            elif phase == 2:
                print("Phase 2: Calibration")
                direction = -400.0 # キャリブレーション中も回転
                calibration()
                phase = 3
            
            elif phase == 3:
                print(f"Phase 3: GPS Navigation (Dist: {distance_to_goal:.2f} m)")
                if distance_to_goal < 4.0:
                    phase = 4
                    print("GPS goal reached. Switching to cone search.")
                    continue
                # GPS誘導ロジック
                diff_angle = (angle_to_goal - azimuth + 180) % 360 - 180
                if abs(diff_angle) < 10.0:
                    direction = 0.0 # 前進
                else:
                    direction = diff_angle # 差分角度で旋回
            
            elif phase == 4:
                print("Phase 4: Cone Search & Track")
                detector.detect_cone()
                if detector.is_reached:
                    print("Cone Reached! Goal!")
                    phase = 5
                    continue
                
                if detector.is_detected:
                    # コーン中心からのズレで方向決定 (0.5が中心)
                    if detector.cone_direction > 0.55:
                        direction = 45 # 右へ
                    elif detector.cone_direction < 0.45:
                        direction = -45 # 左へ
                    else:
                        direction = 0.0 # 前進
                else:
                    direction = -400.0 # 見つからない->右回転で探索
            
            elif phase == 5:
                print("Phase 5: Goal!")
                direction = 360.0 # 停止
                # ゴール後は何もしない
                while True: time.sleep(1)

            time.sleep(0.1)
    finally:
        print("Stopping motors and releasing resources.")
        if pi and pi.connected:
            pi.write(STBY, 0)
            pi.stop()

def Setup():
    global detector, pi
    os.makedirs(LOG_DIR, exist_ok=True)

    pi = pigpio.pi()
    if not pi.connected: sys.exit("pigpio connection failed.")

    # モーターピン設定
    for pin in [AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY, LED1, LED2, TRIG_PIN]:
        pi.set_mode(pin, pigpio.OUTPUT)
    pi.set_mode(ECHO_PIN, pigpio.INPUT)
    pi.set_PWM_frequency(PWMA, 5000)
    pi.set_PWM_frequency(PWMB, 5000)
    pi.write(STBY, 1)

    # センサー初期化
    bno.setUp()
    # ROI画像の準備
    roi_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'library', 'roi_red_cone.png')
    roi_img = cv2.imread(roi_path)
    if roi_img is None: sys.exit(f"Error: Could not load ROI image at {roi_path}")
    detector = dc.detector()
    detector.set_roi_img(roi_img)

    # ログファイル準備
    fileName = os.path.join(LOG_DIR, 'testlog_' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S') + '.csv')
    with open(fileName, 'a', newline="") as f:
        csv.writer(f).writerow(['MilliTime','Phase','AccX','AccY','AccZ','GyroX','GyroY','GyroZ','MagX','MagY','MagZ','LAT','LNG','ALT','Distance','Azimuth','Angle','Direction'])

    # スレッド開始
    threading.Thread(target=moveMotor_thread, daemon=True).start()
    threading.Thread(target=data_thread, daemon=True).start()
    threading.Thread(target=gps_thread, daemon=True).start()

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
    azimuth = (math.degrees(math.atan2(mag[2], mag[1] if mag[1]!=0 else 0.00001)) + 360) % 360

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
            line = s.readline().decode('utf-8')
            if line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)
                if msg.latitude != 0.0:
                    lat, lng, gps_detect = msg.latitude, msg.longitude, True
                else: gps_detect = False
        except: gps_detect = False
        time.sleep(0.1)

def data_thread():
    fileName = os.path.join(LOG_DIR, 'testlog_' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S') + '.csv')
    while True:
        get_all_data()
        with open(fileName, 'a', newline="") as f:
            csv.writer(f).writerow([round(time.time()*1000), round(phase, 1), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], lat, lng, alt, distance_to_goal, azimuth, angle_to_goal, direction])
        time.sleep(DATA_SAMPLING_RATE)

def moveMotor_thread():
    # [整理] モーター制御ロジックを単純化
    while True:
        if direction == 360.0:  # 停止
            pi.write(AIN1, 0); pi.write(AIN2, 0); pi.write(BIN1, 0); pi.write(BIN2, 0)
            pi.set_PWM_dutycycle(PWMA, 0); pi.set_PWM_dutycycle(PWMB, 0)
        elif direction == 0.0:  # 前進
            pi.write(AIN1, 0); pi.write(AIN2, 1); pi.write(BIN1, 1); pi.write(BIN2, 0)
            pi.set_PWM_dutycycle(PWMA, MAX_DUTY); pi.set_PWM_dutycycle(PWMB, MAX_DUTY)
        elif direction == -400.0:  # その場右回転
            pi.write(AIN1, 0); pi.write(AIN2, 1); pi.write(BIN1, 0); pi.write(BIN2, 1)
            pi.set_PWM_dutycycle(PWMA, ROTATE_DUTY); pi.set_PWM_dutycycle(PWMB, ROTATE_DUTY)
        elif direction > 0:  # 右へカーブ（正の角度）
            pi.write(AIN1, 0); pi.write(AIN2, 1); pi.write(BIN1, 1); pi.write(BIN2, 0)
            pi.set_PWM_dutycycle(PWMA, MAX_DUTY); pi.set_PWM_dutycycle(PWMB, TURN_DUTY)
        elif direction < 0:  # 左へカーブ（負の角度）
            pi.write(AIN1, 0); pi.write(AIN2, 1); pi.write(BIN1, 1); pi.write(BIN2, 0)
            pi.set_PWM_dutycycle(PWMA, TURN_DUTY); pi.set_PWM_dutycycle(PWMB, MAX_DUTY)
        time.sleep(0.02)

if __name__ == '__main__':
    main()
