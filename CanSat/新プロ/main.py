import serial
import time
import math
import threading
import datetime
import csv
import os
import RPi.GPIO as GPIO
from library import BNO055
from micropyGPS import MicropyGPS

# --- 定数 ---
# 科学技術計算
MAG_CONST = 8.53  # 地磁気補正用の偏角（設置場所に合わせて要調整）
CALIBRATION_MILLITIME = 10 * 1000  # キャリブレーションにかける時間（ミリ秒）
DATA_SAMPLING_RATE = 0.1 # データサンプリング周期（秒）

# 目標座標
TARGET_LAT = 40.14230733
TARGET_LNG = 139.98738483
GOAL_DISTANCE_THRESHOLD = 3.0 # ゴールと判定する距離（メートル）

# GPIOピン設定
LED1 = 16
PWMA = 18  # 右モーターPWM
AIN1 = 25  # 右モーター制御1
AIN2 = 8   # 右モーター制御2
PWMB = 19  # 左モーターPWM
BIN1 = 9   # 左モーター制御1
BIN2 = 11  # 左モーター制御2

# モーター制御関連
FREQUENCY = 50 # PWM周波数
STUCK_JUDGE_TIME = 10 # GPSでのスタック判定時間（秒）
STUCK_JUDGE_DISTANCE = 1.0 # GPSでのスタック判定距離（メートル）

# モーターの動作モードを定義
DIRECTION_STOP = 360.0          # 停止
DIRECTION_FORWARD = -360.0        # 直進
DIRECTION_ROTATE_LEFT = -400.0    # 左回転（キャリブレーション、スタック時）
DIRECTION_BACK_LEFT = 500.0       # 左後退（スタック脱出時）
DIRECTION_BACK_RIGHT = 600.0      # 右後退（スタック脱出時）


# --- グローバル変数 ---
# センサーデータ
acc = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]
mag = [0.0, 0.0, 0.0]
calibBias = [0.0, 0.0, 0.0]
calibRange = [1.0, 1.0, 1.0]
lat = 0.0
lng = 0.0
fall = 0.0

# 計算値
distance = 999.9 # ゴールまでの距離
angle = 0.0      # ゴールへの方位
azimuth = 0.0    # 現在の機体の方位
direction = 0.0  # モーターへの指示値

# 状態管理
phase = -1 # 制御フェーズ
gps_detect = 0 # GPS受信フラグ
stuck_GPS_Flag = 0 # GPSによるスタック検知フラグ


# --- BNO055センサーのインスタンス化 ---
bno = BNO055.BNO055()

# --- ログファイルの設定 ---
try:
    home_dir = os.path.expanduser("~")
    log_dir = os.path.join(home_dir, "cansat_logs")
    os.makedirs(log_dir, exist_ok=True)
    nowTime = datetime.datetime.now()
    file_base_name = 'log_' + nowTime.strftime('%Y%m%d_%H%M%S') + '.csv'
    fileName = os.path.join(log_dir, file_base_name)
    print(f"ログファイル: {fileName}")
except Exception as e:
    print(f"ログファイルの設定中にエラーが発生しました: {e}")


def main():
    """
    メインの制御ループ
    フェーズに基づいてCanSatの動作を管理する
    """
    global phase
    
    try:
        GPIO.setwarnings(False)
        Setup()
        phase = 0 # 初期フェーズをキャリブレーションに設定

        while True:
            print(f"--- Phase: {phase}, Distance: {distance:.2f} m, Direction: {direction:.2f} ---")

            if phase == 0:  # キャリブレーション
                print("Phase 0: キャリブレーションを開始します。機体をゆっくり回転させてください。")
                calibration()
                print("キャリブレーションが完了しました。")
                phase = 1

            elif phase == 1:  # GPS誘導走行
                print("Phase 1: GPS誘導走行を開始します。")
                # ゴール距離に達するまでこのフェーズを継続
                if distance < GOAL_DISTANCE_THRESHOLD:
                    phase = 2

            elif phase == 2: # ゴール
                print("Phase 2: ゴールに到達しました。")
                # 処理を終了するために無限ループ
                while True:
                    time.sleep(1)
            
            # スタックした場合の割り込み処理
            if stuck_GPS_Flag == 1 and phase == 1:
                print("スタックを検知しました。脱出シーケンスを開始します。")
                # set_direction関数内で脱出処理が行われる
            
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nプログラムが中断されました。")
    finally:
        GPIO.cleanup()
        print("GPIOクリーンアップ完了。")


def Setup():
    """
    ハードウェアとスレッドの初期設定
    """
    # BNO055セットアップ
    if not bno.setUp():
         print("BNO055のセットアップに失敗しました。プログラムを終了します。")
         exit()

    # GPIO設定
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED1, GPIO.OUT)
    
    # ログファイルのヘッダー書き込み
    try:
        with open(fileName, 'w', newline='') as f: # 'a'から'w'に変更し、初回起動時にファイルを新規作成
            writer = csv.writer(f)
            writer.writerow(['Time','Phase','AccX','AccY','AccZ','GyroX','GyroY','GyroZ',
                             'MagX','MagY','MagZ','LAT','LNG','Distance','Azimuth','Angle','Direction'])
    except Exception as e:
        print(f"ログファイルのヘッダー書き込み中にエラー: {e}")

    # 各種処理をスレッドで開始
    threading.Thread(target=moveMotor_thread, daemon=True).start()
    threading.Thread(target=setData_thread, daemon=True).start()
    threading.Thread(target=GPS_thread, daemon=True).start()

    print("セットアップ完了")
    GPIO.output(LED1, HIGH) # 準備完了をLEDで通知
    time.sleep(1)
    GPIO.output(LED1, LOW)


def getBnoData():
    """ BNO055から加速度、ジャイロ、地磁気データを取得・補正 """
    global acc, gyro, mag, fall
    acc = bno.getAcc()
    gyro = bno.getGyro()
    mag = bno.getMag()
    
    # 軸の調整（機体の搭載方向に合わせて変更が必要な場合がある）
    mag[1] = mag[1]
    mag[2] = -mag[2]
    
    fall = math.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)
    
    for i in range(3):
        # キャリブレーション値を適用
        if calibRange[i] != 0:
            mag[i] = (mag[i] - calibBias[i]) / calibRange[i]


def detect_stuck_by_GPS():
    """ 
    GPSデータを用いてスタックを検出するジェネレータ
    一定時間、移動距離が閾値以下の場合にフラグを立てる
    """
    global stuck_GPS_Flag
    last_check_time = time.time()
    last_distance = distance

    while True:
        # yieldで一度処理を中断し、次の呼び出しで再開
        yield

        current_time = time.time()
        if current_time - last_check_time > STUCK_JUDGE_TIME:
            if abs(distance - last_distance) < STUCK_JUDGE_DISTANCE:
                stuck_GPS_Flag = 1
            last_check_time = current_time
            last_distance = distance


def calibration():
    """ 地磁気センサーのキャリブレーション """
    global calibBias, calibRange
    
    print("キャリブレーション中...")
    max_mag = [-float('inf'), -float('inf'), -float('inf')]
    min_mag = [float('inf'), float('inf'), float('inf')]

    start_time = time.time()
    while time.time() - start_time < CALIBRATION_MILLITIME / 1000.0:
        getBnoData()
        for i in range(3):
            if max_mag[i] < mag[i]: max_mag[i] = mag[i]
            if min_mag[i] > mag[i]: min_mag[i] = mag[i]
        time.sleep(0.01)
    
    # バイアスとレンジを計算
    for i in range(3):
        calibBias[i] = (max_mag[i] + min_mag[i]) / 2
        calibRange[i] = (max_mag[i] - min_mag[i]) / 2
        if calibRange[i] == 0: # ゼロ除算を防止
            calibRange[i] = 1.0

    print("キャリブレーション完了値:")
    print(f"  Bias: {calibBias}")
    print(f"  Range: {calibRange}")
    
    # LEDで完了を通知
    for _ in range(3):
        GPIO.output(LED1, True)
        time.sleep(0.2)
        GPIO.output(LED1, False)
        time.sleep(0.2)


def calcdistance():
    """ 2点間の緯度経度から距離を計算（ヒュベニの公式） """
    global distance
    if lat == 0.0 or lng == 0.0: # GPS未受信の場合は計算しない
        distance = 999.9
        return

    POLE_RADIUS = 6356752.314245  # 極半径
    EQUATOR_RADIUS = 6378137.0  # 赤道半径
    
    d_lat = math.radians(TARGET_LAT - lat)
    d_lon = math.radians(TARGET_LNG - lng)
    
    lat1 = math.radians(lat)
    lat2 = math.radians(TARGET_LAT)
    
    e2 = (EQUATOR_RADIUS**2 - POLE_RADIUS**2) / EQUATOR_RADIUS**2
    
    w = math.sqrt(1.0 - e2 * math.sin((lat1 + lat2) / 2)**2)
    m = POLE_RADIUS * (1 - e2) / w**3  # 子午線曲率半径
    n = EQUATOR_RADIUS / w  # 卯酉線曲率半径
    
    distance = math.sqrt((d_lat * m)**2 + (d_lon * n * math.cos((lat1 + lat2) / 2))**2)


def calcAngle():
    """ 2点間の緯度経度から方位角を計算 """
    global angle
    if lat == 0.0 or lng == 0.0: # GPS未受信の場合は計算しない
        angle = 0.0
        return

    d_lon = math.radians(TARGET_LNG - lng)
    
    y = math.sin(d_lon)
    x = math.cos(math.radians(lat)) * math.tan(math.radians(TARGET_LAT)) - math.sin(math.radians(lat)) * math.cos(d_lon)
    
    bearing = math.degrees(math.atan2(y, x))
    if bearing < 0:
        bearing += 360
    angle = bearing


def calcAzimuth():
    """ 地磁気データから機体の現在の方位角を計算 """
    global azimuth
    # Y軸を北とした場合のX軸との角度を計算
    azimuth = math.degrees(math.atan2(mag[1], mag[0])) + MAG_CONST
    if azimuth < 0:
        azimuth += 360
    if azimuth > 360:
        azimuth -= 360


def GPS_thread():
    """ GPSモジュールからデータを継続的に読み取る """
    global lat, lng, gps_detect
    stuck_detector = detect_stuck_by_GPS()

    try:
        # シリアルポートは環境に合わせて '/dev/ttyAMA0' や '/dev/serial0' に変更
        s = serial.Serial('/dev/serial0', 9600, timeout=10)
        gps = MicropyGPS(9, 'dd')

        while True:
            try:
                sentence = s.readline().decode('utf-8', errors='ignore')
                if sentence and sentence[0] == '$':
                    for x in sentence:
                        gps.update(x)
                
                if gps.latitude[0] > 0 and gps.longitude[0] > 0:
                    lat = gps.latitude[0]
                    lng = gps.longitude[0]
                    gps_detect = 1
                else:
                    gps_detect = 0
                
                # スタック検出処理を呼び出し
                next(stuck_detector)

            except serial.SerialException as e:
                print(f"GPSシリアルエラー: {e}")
                time.sleep(5)
            except Exception as e:
                print(f"GPSスレッドで予期せぬエラー: {e}")
                
    except serial.SerialException as e:
        print(f"GPSポートが開けません: {e}。プログラムを終了します。")
        os._exit(1)


def setData_thread():
    """ 
    定期的に各センサーデータを更新し、CSVファイルに記録する
    """
    while True:
        getBnoData()
        calcAngle()
        calcAzimuth()
        set_direction()
        calcdistance()

        try:
            with open(fileName, 'a', newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3],
                    phase,
                    f"{acc[0]:.2f}", f"{acc[1]:.2f}", f"{acc[2]:.2f}",
                    f"{gyro[0]:.2f}", f"{gyro[1]:.2f}", f"{gyro[2]:.2f}",
                    f"{mag[0]:.2f}", f"{mag[1]:.2f}", f"{mag[2]:.2f}",
                    lat, lng,
                    f"{distance:.2f}",
                    f"{azimuth:.2f}",
                    f"{angle:.2f}",
                    direction
                ])
        except Exception as e:
            print(f"ログ書き込みエラー: {e}")

        time.sleep(DATA_SAMPLING_RATE)


def moveMotor_thread():
    """
    `direction`変数の値に応じてモーターを制御する
    """
    GPIO.setmode(GPIO.BCM)
    pins = [PWMA, AIN1, AIN2, PWMB, BIN1, BIN2]
    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)

    M_pwmA = GPIO.PWM(PWMA, FREQUENCY)
    M_pwmB = GPIO.PWM(PWMB, FREQUENCY)
    M_pwmA.start(0)
    M_pwmB.start(0)
    
    def set_motor(pwm_a, in1_a, in2_a, pwm_b, in1_b, in2_b):
        M_pwmA.ChangeDutyCycle(pwm_a)
        GPIO.output(AIN1, in1_a)
        GPIO.output(AIN2, in2_a)
        M_pwmB.ChangeDutyCycle(pwm_b)
        GPIO.output(BIN1, in1_b)
        GPIO.output(BIN2, in2_b)

    while True:
        if direction == DIRECTION_STOP: # 停止
            set_motor(0, False, False, 0, False, False)
        elif direction == DIRECTION_FORWARD: # 直進
            set_motor(100, True, False, 100, True, False)
        elif direction == DIRECTION_ROTATE_LEFT: # 左回転
            set_motor(100, False, True, 100, True, False)
        elif direction == DIRECTION_BACK_LEFT: # 左後退
            set_motor(0, False, False, 100, False, True)
        elif direction == DIRECTION_BACK_RIGHT: # 右後退
            set_motor(100, False, True, 0, False, False)
        elif 0 < direction <= 180: # 右旋回
            duty = 100 - (direction / 180) * 80 # 角度が大きいほど低速に
            set_motor(100, True, False, duty, True, False)
        elif -180 <= direction < 0: # 左旋回
            duty = 100 - (abs(direction) / 180) * 80 # 角度が大きいほど低速に
            set_motor(duty, True, False, 100, True, False)
        
        time.sleep(0.02) # CPU負荷軽減


def set_direction():
    """ 
    現在のフェーズや状態に応じて、`direction`変数を設定する
    """
    global direction, stuck_GPS_Flag
    
    # GPSが未受信の場合は停止
    if gps_detect == 0 and phase > 0:
        direction = DIRECTION_STOP
        print("GPS信号を待っています...")
        return

    # スタックからの脱出処理を最優先
    if stuck_GPS_Flag == 1 and phase == 1:
        print("スタック脱出シーケンス実行中")
        # 交互に後退して方向転換
        for _ in range(2):
            direction = DIRECTION_BACK_LEFT
            time.sleep(0.5)
            direction = DIRECTION_BACK_RIGHT
            time.sleep(0.5)
        # 少し回転して進行方向を変える
        direction = DIRECTION_ROTATE_LEFT
        time.sleep(1.5)
        stuck_GPS_Flag = 0 # フラグをリセット
        return

    # フェーズごとの処理
    if phase == -1: # 初期化中
        direction = DIRECTION_STOP
    elif phase == 0:  # キャリブレーション
        direction = DIRECTION_ROTATE_LEFT
    elif phase == 1:  # GPS誘導
        # 目標方位と現在方位の差を計算
        diff_angle = angle - azimuth
        # 角度差を-180度から180度の範囲に正規化
        if diff_angle > 180:
            diff_angle -= 360
        if diff_angle < -180:
            diff_angle += 360
        
        direction = diff_angle
        
        # ほぼ目標方位を向いている場合は直進
        if abs(direction) < 10.0:
            direction = DIRECTION_FORWARD

    elif phase == 2:  # ゴール
        direction = DIRECTION_STOP


if __name__ == '__main__':
    main()
