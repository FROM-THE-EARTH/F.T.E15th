# 必要なライブラリをインポート
import serial
import time
import math
import threading
import datetime
import pigpio # RPi.GPIOに代わり、pigpioを使用
import csv
import os
import cv2

# カスタムライブラリをインポート
from library import BNO055
from library import BMP180
from library import detect_corn as dc
import pynmea2
from picamera2 import Picamera2

# --- グローバル定数 ---

# -- 運用設定 --
TARGET_LAT = 38.260469  # ゴール緯度
TARGET_LNG =  140.854215 # ゴール経度
DATA_LOGGING_INTERVAL = 0.1     # データ記録間隔 (秒)
CALIBRATION_DURATION_S = 20     # 地磁気キャリブレーションの時間 (秒)
OBSTACLE_THRESHOLD_CM = 30.0 #30cm以内に障害物があれば回避

# -- 超音波センサー Pinアサイン --
TRIG_PIN = 23 # トリガーピンのGPIO番号
ECHO_PIN = 24 # エコーピンのGPIO番号

# -- モーター制御関連 --
KICK_PEAK_DUTY = 150 # AINモーターの固着解消用ピークデューティ
# PWM周波数
PWM_FREQUENCY = 5000 # pigpioで推奨される周波数の一つ
# デューティサイクル (0-255)
# 12V電源から6Vモーターを駆動するため、デューティ比を約42% (106/255)に調整
DUTY_FORWARD = 106
DUTY_SLOW = 70
DUTY_FAST = 128
DUTY_SPIN = 128

# -- 進行方向を定義する定数 (マジックナンバーの排除) --
DIRECTION_STOP = 0
DIRECTION_FORWARD = 1
DIRECTION_SPIN_RIGHT = 2    # 右旋回（キャリブレーション用）
DIRECTION_TURN_RIGHT = 3    # 右旋回
DIRECTION_TURN_LEFT = 4     # 左旋回
DIRECTION_PARA_SEPARATION = 5 # パラシュート分離用の特殊旋回

# -- Pinアサイン 右モーター
AIN1 = 8   # AIN1
AIN2 = 25  # AIN2
PWMA = 18  # PWMA
# 左モータ
BIN1 = 9    # BIN1
BIN2 = 11   # BIN2
PWMB = 19   # PWMB
# ドライバ有効化ピン
STBY = 22

# --- グローバル変数 ---
# pigpioインスタンス
pi = None

# センサーデータ
acc = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]
mag = [0.0, 0.0, 0.0]
euler = [0.0, 0.0, 0.0] # オイラー角 (Heading, Roll, Pitch)
fall = 0.0 # 合成加速度

# GPSデータ
lat = 0.0
lng = 0.0

# 気圧センサーデータ
alt = 0.0

# 計算値
distance = 0.0 # ゴールまでの距離
target_angle = 0.0 # ゴールの方角
azimuth = 0.0 # 機体の向き（方位角）
direction = DIRECTION_STOP # モーター制御用の最終的な進行方向

# 状態管理
phase = 0
upside_down_Flag = 0
stuck_GPS_Flag = 0

# --- ★★★ここから故障管理フラグを追加★★★ ---
bno_ok = False
bmp_ok = False
camera_ok = False

# カメラ関連
cone_direction = 0.5
cone_probability = 1.0

# ログファイル名
nowTime = datetime.datetime.now()
fileName = f"./log/log_{nowTime.strftime('%Y%m%d-%H%M%S')}.csv"

# --- メインロジック ---
def main():
    global phase, direction, fall # directionをmainで直接制御する
    
    # フラグ
    searching_Flag = False
    camera_failed = False
    is_phase0_start_time_set = False # phase0の開始時刻を一度だけ記録するためのフラグ
    is_evading = False # 回避行動中かどうかのフラグ
    
    # 時間計測用
    time_start_searching_cone = 0
    time_start_phase0 = 0 # タイムアウト用の開始時刻を保持する変数
    evasion_end_time = 0 # 回避行動を終了する時刻

    try:
        Setup()
        print("Setup complete. Starting mission.")
        phase = 0

        while True:
            if phase == 0:  # 投下検知
                direction = DIRECTION_STOP

                # このフェーズに入った最初のループで開始時刻を記録
                if not is_phase0_start_time_set:
                    time_start_phase0 = time.time()
                    is_phase0_start_time_set = True
                    print("Phase 0: Awaiting launch/fall detection...")

                # --- 落下検知ロジック ---
                # 条件1: 合成加速度(fall)が30m/s^2を超えた場合 (射出の衝撃を検知)
                # 条件2: 300秒経過した場合 (タイムアウト)
                if fall > 30.0 or (time.time() - time_start_phase0 > 300):
                    print(f"Fall detected! (Trigger: {'Acceleration' if fall > 30.0 else 'Timeout'})")
                    phase = 1 # 次のフェーズへ移行

            elif phase == 1:  # パラシュート分離
                direction = DIRECTION_PARA_SEPARATION
                print("Phase 1: Para-separation sequence initiated.")
                time.sleep(15)
                phase = 2

            elif phase == 2:  # 地磁気センサーキャリブレーション
                direction = DIRECTION_SPIN_RIGHT
                print("Phase 2: Magnetometer calibration start.")
                calibration() # calibration関数内でのdirection設定は不要にする
                direction = DIRECTION_STOP
                print("Calibration complete.")
                phase = 3

            elif phase == 3: # GPS誘導
                # BNO055が故障している場合、方位が分からないため直進しかできない
                if not bno_ok:
                    print("Phase 3: GPS nav (IMU FAILED - proceeding straight).")
                    direction = DIRECTION_FORWARD
                else:
                    print(f"Phase 3: GPS nav. Dist: {distance:.2f}m, Target: {target_angle:.1f}, Azimuth: {azimuth:.1f}, Obstacle: {obstacle_distance:.1f}cm")
                    # (障害物回避ロジックなどもこのelseブロックの中に含める)
                    angle_diff = target_angle - azimuth
                    if angle_diff > 180: angle_diff -= 360
                    if angle_diff < -180: angle_diff += 360

                    if abs(angle_diff) < 15:
                        direction = DIRECTION_FORWARD
                    elif angle_diff > 0:
                        direction = DIRECTION_TURN_RIGHT
                    else:
                        direction = DIRECTION_TURN_LEFT
                
                # --- フェーズ遷移判定 ---
                # カメラが正常で、距離が5m未満になったらカメラフェーズへ
                if distance < 5.0 and camera_ok:
                    phase = 4
                # カメラが故障している場合、1m未満になったらゴールとみなす
                elif distance < 1.0 and not camera_ok:
                    print("Camera failed, proceeding to goal based on GPS distance.")
                    phase = 6

            elif phase == 4: # カメラ：コーン探索
                # このフェーズに来る時点でcamera_okはTrueのはずだが、念のため
                if not camera_ok:
                    print("Camera is not available. Cannot search for cone. Proceeding to goal.")
                    phase = 6
                    continue # ループの最初に戻る
                
                direction = DIRECTION_SPIN_RIGHT

            elif phase == 5: # カメラ：コーン追尾
                print(f"Phase 5: Tracking cone. Cone direction: {cone_direction:.2f}")
                cone_detect()

                if detector.is_reached:
                    print("Goal reached!")
                    phase = 6
                elif not detector.is_detected:
                    print("Cone lost. Returning to search phase.")
                    phase = 4
                else:
                    # コーン追尾のロジックをここに記述
                    if cone_direction > 0.6:
                        direction = DIRECTION_TURN_RIGHT
                    elif cone_direction < 0.4:
                        direction = DIRECTION_TURN_LEFT
                    else:
                        direction = DIRECTION_FORWARD

            elif phase == 6: # ゴール
                direction = DIRECTION_STOP
                print("Phase 6: Mission accomplished. Goal!")
                while True: time.sleep(100)
            
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nMission aborted by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        # 終了処理
        if pi:
            # PWMを停止
            pi.write(AIN1, 0)
            pi.write(AIN2, 0)
            pi.write(BIN1, 0)
            pi.write(BIN2, 0)
            pi.set_PWM_dutycycle(PWMA, 0)
            pi.set_PWM_dutycycle(PWMB, 0)
            # ドライバを無効化
            pi.write(STBY, 0)
            pi.stop()
            print("pigpio stopped.")

# --- 初期化 ---
def Setup():
    global pi, bno, bmp, detector, picam2
    global bno_ok, bmp_ok, camera_ok # 故障フラグをグローバル宣言
    
    # pigpioの初期化
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("Could not connect to pigpio daemon.")
    
    # モーターピンを出力に設定
    motor_pins = [AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY]
    for pin in motor_pins:
        pi.set_mode(pin, pigpio.OUTPUT)

    # PWM周波数を設定
    pi.set_PWM_frequency(PWMA, 5000) 
    pi.set_PWM_frequency(PWMB, 5000)

    # ドライバを有効化
    pi.write(STBY, 1)

    # --- BNO055 (IMU) の初期化 ---
    try:
        bno = BNO055.BNO055()
        if not bno.setUp():
            raise RuntimeError("BNO055 setup method failed.")
        bno_ok = True
        print("BNO055 initialized successfully.")
    except Exception as e:
        print(f"!!! CRITICAL WARNING: BNO055 (IMU) initialization failed: {e}")
        print("    -> Navigation will be disabled.")

    # --- BMP180 (気圧計) の初期化 ---
    try:
        bmp = BMP180.BMP180()
        if not bmp.setUp():
            raise RuntimeError("BMP180 setup method failed.")
        bmp_ok = True
        print("BMP180 initialized successfully.")
    except Exception as e:
        print(f"!!! WARNING: BMP180 (Altimeter) initialization failed: {e}")
        print("    -> Altitude data will be unavailable.")
    
    try:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (640, 480)})
        picam2.configure(config)
        picam2.start()
        time.sleep(2)

        detector = dc.detector(picam2)
        roi_img = cv2.imread("./library/roi_red_cone.png")
        if roi_img is None:
            raise FileNotFoundError("ROI image './library/roi_red_cone.png' not found.")
        detector.set_roi_img(roi_img)
        camera_ok = True
        print("Camera and Cone Detector initialized successfully.")
    except Exception as e:
        print(f"!!! CRITICAL WARNING: Camera initialization failed: {e}")
        print("    -> Cone detection and tracking will be disabled.")

    # ログファイルのヘッダー書き込み
    os.makedirs("./log", exist_ok=True)
    with open(fileName, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Time", "Phase", "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ", 
            "MagX", "MagY", "MagZ", "EulerH", "EulerR", "EulerP",
            "Lat", "Lng", "Alt", "Distance", "TargetAngle", "Azimuth", 
            "Direction", "Fall", "ConeDirection", "ConeProbability"
        ])
    print(f"Log file created: {fileName}")

    # 各種スレッドの開始 (スレッド自体はエラーに関わらず開始する)
    threading.Thread(target=moveMotor_thread, daemon=True).start()
    threading.Thread(target=dataProcessing_thread, daemon=True).start()
    threading.Thread(target=gps_thread, daemon=True).start()
    threading.Thread(target=ultrasonic_thread, daemon=True).start()
    print("All threads started.")

# --- データ処理・計算関連 ---
def dataProcessing_thread():
    """センサーデータ取得、計算、ロギングを定期的に行う"""
    while True:
        # データ取得
        getBnoData()
        getBmpData()
        
        # 計算
        calc_target_angle()
        calc_azimuth()
        calc_distance()
        
        # ログ書き込み
        with open(fileName, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.datetime.now().strftime('%H:%M:%S.%f'), phase,
                acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2],
                mag[0], mag[1], mag[2], euler[0], euler[1], euler[2],
                lat, lng, alt, distance, target_angle, azimuth,
                direction, fall, cone_direction, cone_probability
            ])
        
        time.sleep(DATA_LOGGING_INTERVAL)

def getBnoData():
    global acc, gyro, mag, euler, fall
    acc = bno.getAcc()
    gyro = bno.getGyro()
    mag = bno.getMag()
    euler = bno.getEuler() # ★オイラー角を取得
    fall = math.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)

def getBmpData():
    global alt
    alt = bmp.getAltitude() 
    # 必要に応じて高度の補正値を加える
    # alt += 60 

def calc_target_angle():
    global target_angle
    if lat == 0.0 or lng == 0.0:
        target_angle = 0.0
        return
        
    y = math.sin(math.radians(TARGET_LNG - lng)) * math.cos(math.radians(TARGET_LAT))
    x = math.cos(math.radians(lat)) * math.sin(math.radians(TARGET_LAT)) - \
        math.sin(math.radians(lat)) * math.cos(math.radians(TARGET_LAT)) * math.cos(math.radians(TARGET_LNG - lng))
    
    angle_rad = math.atan2(y, x)
    target_angle = (math.degrees(angle_rad) + 360) % 360 # 0-360度の範囲に正規化

def calc_azimuth():
    global azimuth
    # BNO055のオイラー角(Heading)を機体の方位角として使用（チルト補正済み）
    azimuth = euler[0]

def calc_distance():
    global distance
    if lat == 0.0 or lng == 0.0:
        distance = 9999
        return
        
    d_lat = math.radians(TARGET_LAT - lat)
    d_lon = math.radians(TARGET_LNG - lng)
    a = math.sin(d_lat/2)**2 + math.cos(math.radians(lat)) * math.cos(math.radians(TARGET_LAT)) * math.sin(d_lon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = 6371 * 1000 * c # 単位: メートル

def calibration():
    print(f"Rotate the CanSat for {CALIBRATION_DURATION_S} seconds to calibrate the magnetometer.")
    
    # BNO055の内部キャリブレーションステータスを監視
    start_time = time.time()
    while time.time() - start_time < CALIBRATION_DURATION_S:
        status = bno.getCalibrationStatus()
        print(f"  Calibration status (Sys, Gyro, Accel, Mag): {status}")
        # 全てが3になるのが理想だが、地磁気(mag)が1以上になればある程度信頼できる
        if status[3] >= 1:
            print("Magnetometer has been partially calibrated.")
        time.sleep(1)
    
    # BNO055がNDOFモードの場合、センサーフュージョンが自動で補正を続けるため、
    # ここでの手動キャリブレーションは必須ではないが、初期精度向上のために行う。

# --- ハードウェア制御 ---
def cone_detect():
    global cone_direction, cone_probability
    detector.detect_cone()
    cone_direction = detector.cone_direction if detector.cone_direction is not None else 0.5
    cone_probability = detector.probability if detector.probability is not None else 1.0


def gps_thread():
    global lat, lng
    try:
        s = serial.Serial("/dev/serial0", 115200, timeout=1)
        while True:
            try:
                # 1行ずつデータを読み込む
                sentence = s.readline().decode("utf-8", errors='ignore')
                
                # データがあれば解析を試みる
                if sentence.startswith('$'):
                    msg = pynmea2.parse(sentence)
                    
                    # GGAまたはRMCメッセージに緯度経度情報があれば更新する
                    if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                        # is_validでデータの品質も確認できる
                        if hasattr(msg, 'is_valid') and msg.is_valid:
                            lat = msg.latitude
                            lng = msg.longitude
                            # print(f"GPS Updated: Lat={lat}, Lng={lng}") # デバッグ用
                        
            except pynmea2.ParseError as e:
                # 解析エラーは無視して次の行へ
                # print(f"Parse error: {e}")
                pass

    except serial.SerialException as e:
        print(f"GPS Error: {e}")
        lat, lng = 0.0, 0.0 # エラー時は0に

def ultrasonic_thread():
    """超音波センサーで障害物までの距離を継続的に測定する"""
    global obstacle_distance
    
    pi.set_mode(TRIG_PIN, pigpio.OUTPUT)
    pi.set_mode(ECHO_PIN, pigpio.INPUT)
    pi.write(TRIG_PIN, 0)
    time.sleep(0.1) # センサーの安定待ち

    while True:
        try:
            # トリガーパルスを送信
            pi.write(TRIG_PIN, 1)
            time.sleep(0.00001)
            pi.write(TRIG_PIN, 0)

            # エコー信号の開始を待つ
            start_time = time.time()
            timeout_start = start_time
            while pi.read(ECHO_PIN) == 0:
                start_time = time.time()
                if start_time - timeout_start > 0.02: # タイムアウト
                    break
            
            if pi.read(ECHO_PIN) == 0: # タイムアウトした場合
                obstacle_distance = 999
                time.sleep(0.1)
                continue

            # エコー信号の終了を待つ
            end_time = time.time()
            timeout_start = end_time
            while pi.read(ECHO_PIN) == 1:
                end_time = time.time()
                if end_time - timeout_start > 0.02: # タイムアウト
                    break
            
            # 距離を計算 (音速 343m/s)
            duration = end_time - start_time
            distance = (duration * 34300) / 2
            
            if distance < 400: # センサーの最大測定距離を超える値は除外
                obstacle_distance = distance
            else:
                obstacle_distance = 999

        except Exception as e:
            # print(f"Ultrasonic sensor error: {e}")
            obstacle_distance = 999
        
        time.sleep(0.1) # 100msごとに測定

def moveMotor_thread():
    """
    ソフトスタート/ストップを実装。
    AIN(右)モーターの始動不良対策として、始動時に一時的に
    KICK_PEAK_DUTYまでデューティ比を上げてから目標値に戻すロジックを搭載。
    """
    global direction

    # 各モーターの現在のデューティ比
    current_duty_A = 0
    current_duty_B = 0

    # AIN(右)モーターの状態管理用変数
    # 'IDLE': 停止中, 'KICK_UP': ピークへ上昇中, 'KICK_DOWN': 目標値へ下降中, 'NORMAL': 通常運転
    motor_A_state = 'IDLE'

    # 加減速のステップ値
    STEP = 2 # 少し加速を速めるためステップを2に変更（必要に応じて調整）

    while True:
        if not pi or not pi.connected:
            time.sleep(0.1)
            continue

        # --- 1. 目標デューティ比と回転方向を決定 ---
        target_duty_A = 0
        target_duty_B = 0
        target_A1, target_A2 = 0, 0
        target_B1, target_B2 = 0, 0


        if direction == DIRECTION_STOP:
            pass
        elif direction == DIRECTION_FORWARD:
            target_A1, target_A2 = 1, 0
            target_B1, target_B2 = 0, 1
            target_duty_A = DUTY_FORWARD
            target_duty_B = DUTY_FORWARD
        elif direction == DIRECTION_SPIN_RIGHT:
            target_A1, target_A2 = 1, 0
            target_B1, target_B2 = 0, 1
            target_duty_A = DUTY_SLOW
            target_duty_B = DUTY_SPIN
        elif direction == DIRECTION_TURN_RIGHT:
            target_A1, target_A2 = 1, 0
            target_B1, target_B2 = 0, 1
            target_duty_A = DUTY_SLOW
            target_duty_B = DUTY_FORWARD
        elif direction == DIRECTION_TURN_LEFT:
            target_A1, target_A2 = 1, 0
            target_B1, target_B2 = 0, 1
            target_duty_A = DUTY_FORWARD
            target_duty_B = DUTY_SLOW
        elif direction == DIRECTION_PARA_SEPARATION:
            target_A1, target_A2 = 1, 0
            target_B1, target_B2 = 0, 1
            target_duty_A = DUTY_SLOW
            target_duty_B = DUTY_SPIN

        # --- 2. AIN(右)モーターのデューティ比を状態に応じて制御 ---

        # 状態遷移の決定
        if motor_A_state == 'IDLE' and target_duty_A > 0:
            motor_A_state = 'KICK_UP'
        elif target_duty_A == 0:
            motor_A_state = 'NORMAL' # 停止指令が来たら通常減速モードへ

        # 状態ごとのデューティ比操作
        if motor_A_state == 'KICK_UP':
            current_duty_A = min(current_duty_A + STEP, KICK_PEAK_DUTY)
            if current_duty_A >= KICK_PEAK_DUTY:
                motor_A_state = 'KICK_DOWN' # ピークに達したら下降モードへ
        elif motor_A_state == 'KICK_DOWN':
            current_duty_A = max(current_duty_A - STEP, target_duty_A)
            if current_duty_A <= target_duty_A:
                motor_A_state = 'NORMAL' # 目標値に達したら通常モードへ
        else: # 'NORMAL' または 'IDLE'
            if current_duty_A < target_duty_A:
                current_duty_A = min(current_duty_A + STEP, target_duty_A)
            elif current_duty_A > target_duty_A:
                current_duty_A = max(current_duty_A - STEP, target_duty_A)

        if current_duty_A == 0:
            motor_A_state = 'IDLE'

        # --- 3. BIN(左)モーターのデューティ比を制御（通常ソフトスタート） ---
        if current_duty_B < target_duty_B:
            current_duty_B = min(current_duty_B + STEP, target_duty_B)
        elif current_duty_B > target_duty_B:
            current_duty_B = max(current_duty_B - STEP, target_duty_B)

        # --- 4. ハードウェアへ指令を送信 ---
        # (安全な方向転換ロジックは前回と同じ)
        if current_duty_A == 0:
            pi.write(AIN1, 0); pi.write(AIN2, 0)
        if current_duty_A > 0 and pi.read(AIN1) == 0 and pi.read(AIN2) == 0:
            pi.write(AIN1, target_A1); pi.write(AIN2, target_A2)
        pi.set_PWM_dutycycle(PWMA, current_duty_A)

        if current_duty_B == 0:
            pi.write(BIN1, 0); pi.write(BIN2, 0)
        if current_duty_B > 0 and pi.read(BIN1) == 0 and pi.read(BIN2) == 0:
            pi.write(BIN1, target_B1); pi.write(BIN2, target_B2)
        pi.set_PWM_dutycycle(PWMB, current_duty_B)

        time.sleep(0.01)

# --- プログラム実行 ---
if __name__ == "__main__":
    main()
