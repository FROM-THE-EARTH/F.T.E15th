import serial
import time
import math
import threading
import datetime
import pigpio
import csv
import os
import cv2
import RPi.GPIO as GPIO
import pynmea2
from library import BNO055
from library import BMP180
from library import detect_corn as dc
from picamera2 import Picamera2
import matplotlib.pyplot as plt
import sys


# --- 定数 (グローバル) ---
MAG_CONST = 8.53
CALIBRATION_MILLITIME = 20 * 1000
TARGET_LAT = 38.266285
TARGET_LNG = 140.855498
TARGET_ALTITUDE = 20
DATA_SAMPLING_RATE = 0.00001
# ALTITUDE_CONST1 = 30 # 落下検知用のため削除
# ALTITUDE_CONST2 = 5 # 落下検知用のため削除
LED1 = 16
LED2 = 20
HIGH = 1
LOW = 0

# --- 新しいモータードライバーのピン定義 (TB6612FNGなどを想定) ---
AIN1 = 8
AIN2 = 25
PWMA = 18
BIN1 = 9
BIN2 = 11
PWMB = 19
STBY = 22

# 超音波センサーのピン定義
trig_pin = 15
echo_pin = 14
speed_of_sound = 34370

# --- 変数 (グローバル) ---
acc = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]
mag = [0.0, 0.0, 0.0]
calibBias = [0.0, 0.0, 0.0]
calibRange = [1.0, 1.0, 1.0]
lat = 0.0
lng = 0.0
alt = 0.0
# maxAlt = 0.0 # 落下検知用のため削除
# minAlt = 0.0 # 落下検知用のため削除
pres = 0.0
distance = 0
angle = 0.0
azimuth = 0.0
direction = 0.0
phase = 0
gps_detect = 0
cone_direction = 0
cone_probability = 0
upside_down_Flag = 0
stuck_uss_Flag = 0
stuck_GPS_Flag = 0

# --- pigpioインスタンス (グローバル) ---
pi = None

# --- センサーインスタンス ---
bno = BNO055.BNO055()
bmp = BMP180.BMP180()

# --- ログファイルのパスを動的に生成 ---
log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'log')
os.makedirs(log_dir, exist_ok=True)
nowTime = datetime.datetime.now()
fileName = os.path.join(log_dir, 'testlog_' + nowTime.strftime('%Y-%m%d-%H%M%S') + '.csv')


def main():
    global phase

    GPIO.setwarnings(False)
    Setup()
    # 開始フェーズを1に設定
    phase = 1
    n = 0

    try:
        while True:
            # phase == 0 のブロックは削除

            if phase == 1: # パラ分離フェーズ
                print("phase1 : para-separation phase")
                time.sleep(2) # サーボ動作の代わりの待機
                phase = 2

            elif phase == 2:  # キャリブレーション
                print("phase2 : calibration start")
                calibration()
                phase = 3

            elif phase == 3: # GPS走行
                print(f"phase3 : GPS navigation (distance: {distance:.2f} m)")
                if distance < 4.0:
                    phase = 4

            elif phase == 4: # コーンサーチ
                print("phase4 : camera start")
                cone_detect()
                if cone_probability < 1:
                    phase = 5
                if n > 5:
                    phase = 6
                n += 1

            elif phase == 5: # コーン追跡
                print("phase5: cone tracking")
                cone_detect()
                if detector.is_reached:
                    print("reached cone")
                    phase = 6

            elif phase == 6:
                print("phase6 : Goal. Program finished. Halting.")
                # ゴールしたのでモーターを停止させる
                direction = 360.0
                # プログラムを終了させず、無限ループで待機させる
                while True:
                    time.sleep(1)
                    break

            while ((stuck_uss_Flag == 1 or stuck_GPS_Flag == 1) and phase >= 3):
                pass
            time.sleep(0.1)

    finally:
        print("Stopping motors and releasing resources.")
        pi.write(STBY, 0)
        pi.stop()
        GPIO.cleanup()


def currentMilliTime():
    return round(time.time() * 1000)


def Setup():
    global detector, pi

    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio connection failed.")
        sys.exit()

    MOTOR_PINS = [AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY]
    for pin in MOTOR_PINS:
        pi.set_mode(pin, pigpio.OUTPUT)

    pi.set_PWM_frequency(PWMA, 5000)
    pi.set_PWM_frequency(PWMB, 5000)
    pi.write(STBY, 1)

    bno.setUp()

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED1, GPIO.OUT)
    GPIO.setup(LED2, GPIO.OUT)
    GPIO.setup(trig_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)

    with open(fileName, 'a', newline="") as f:
        writer = csv.writer(f)
        # ログヘッダーから'Fall'を削除
        writer.writerow(['MilliTime','Phase','AccX','AccY','AccZ','GyroX','GyroY','GyroZ','MagX','MagY','MagZ','LAT','LNG','ALT','Distance','Object_Distance','Azimuth','Angle','Direction'])

    getThread = threading.Thread(target=moveMotor_thread, daemon=True)
    getThread.start()

    dataThread = threading.Thread(target=setData_thread, daemon=True)
    dataThread.start()

    gpsThread = threading.Thread(target=GPS_thread, daemon=True)
    gpsThread.start()

    # --- ROI画像のパスを動的に生成 ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    roi_path = os.path.join(script_dir, 'library', 'roi_red_cone.png')
    roi_img = cv2.imread(roi_path)

    if roi_img is None:
        print(f"Error: Could not load ROI image at {roi_path}")
        sys.exit()
    
    detector = dc.detector()
    roi_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2RGB)
    detector.set_roi_img(roi_img)

    GPIO.output(LED2, HIGH)
    print("Setup OK")


def getBnoData():
    global acc, gyro, mag
    acc = bno.getAcc()
    gyro = bno.getGyro()
    mag = bno.getMag()
    # fallの計算を削除
    for i in range(3):
        mag[i] = (mag[i] - calibBias[i]) / calibRange[i] if calibRange[i] != 0 else 0


def getBmpData():
    global alt, pres
    alt = bmp.getAltitude()
    pres = bmp.getPressure()
    alt = alt + 60

# flying()関数を削除

def get_object_distance():
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)
    t1 = time.time()
    t2 = t1
    while not GPIO.input(echo_pin):
        t1 = time.time()
    while GPIO.input(echo_pin):
        t2 = time.time()
    return (t2 - t1) * speed_of_sound / 2


def stuck_GPS():
    global stuck_GPS_Flag
    while True:
        ti = time.time()
        distance_buff = distance
        yield
        while time.time() - ti < 10:
            yield
        if abs(distance - distance_buff) < 1:
            stuck_GPS_Flag = 1

def upside_down():
    global upside_down_Flag, acc
    if acc[1] > 0:
        upside_down_Flag = 1

def calibration():
    global calibBias, calibRange, mag
    max_val = [0.0] * 3
    min_val = [0.0] * 3
    
    print("Calibration starting. Please rotate the CanSat on all axes.")
    
    start_time = currentMilliTime()
    while (currentMilliTime() - start_time) < CALIBRATION_MILLITIME:
        getBnoData()
        for i in range(3):
            if mag[i] > max_val[i]: max_val[i] = mag[i]
            if mag[i] < min_val[i]: min_val[i] = mag[i]
        time.sleep(0.01)

    for i in range(3):
        calibBias[i] = (max_val[i] + min_val[i]) / 2
        calibRange[i] = (max_val[i] - min_val[i]) / 2
        if calibRange[i] == 0: calibRange[i] = 1.0

    print("Calibration complete!")
    GPIO.output(LED1, HIGH)
    time.sleep(2)
    GPIO.output(LED1, LOW)


def calcdistance():
    global distance
    EARTH_RADIUS = 6378136.59
    dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng) * math.cos(math.radians((TARGET_LAT + lat) / 2))
    dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
    distance = math.sqrt(dx**2 + dy**2)


def calcAngle():
    global angle
    rad_lat1 = math.radians(lat)
    rad_lat2 = math.radians(TARGET_LAT)
    rad_lng_diff = math.radians(TARGET_LNG - lng)
    y = math.sin(rad_lng_diff) * math.cos(rad_lat2)
    x = math.cos(rad_lat1) * math.sin(rad_lat2) - math.sin(rad_lat1) * math.cos(rad_lat2) * math.cos(rad_lng_diff)
    angle = (math.degrees(math.atan2(y, x)) + 360) % 360


def calcAzimuth():
    global azimuth
    if mag[1] == 0: mag[1] = 0.000001
    azimuth = (math.degrees(math.atan2(mag[2], mag[1])) + 360) % 360


def LED_Checker(num):
    for _ in range(num):
        GPIO.output(LED1, HIGH)
        time.sleep(0.1)
        GPIO.output(LED1, LOW)
        time.sleep(0.1)

def GPS_thread():
    global lat, lng, gps_detect, stuck_GPS_Flag
    stuck_GPS_detection = stuck_GPS()
    s = serial.Serial('/dev/serial0', 115200, timeout=1) #9600のケースも

    while True:
        try:
            sentence = s.readline().decode('utf-8')
            if sentence.startswith('$GPGGA'):
                msg = pynmea2.parse(sentence)
                if msg.latitude != 0.0:
                    lat = msg.latitude
                    lng = msg.longitude
                    gps_detect = 1
                    next(stuck_GPS_detection)
                else:
                    gps_detect = 0
            if s.in_waiting > 1024:
                s.reset_input_buffer()
        except Exception as e:
            gps_detect = 0
            time.sleep(0.1)


def cone_detect():
    global detector, cone_direction, cone_probability
    detector.detect_cone()
    cone_direction = detector.cone_direction
    cone_probability = detector.probability


def setData_thread():
    object_distance_list = []
    while True:
        getBnoData()
        getBmpData()
        if gps_detect:
            calcAngle()
            calcAzimuth()
            calcdistance()
        set_direction()
        object_distance = get_object_distance()

        global stuck_uss_Flag
        object_distance_list.append(object_distance)
        if len(object_distance_list) > 5:
            object_distance_list.pop(0)
        if all(x <= 10 for x in object_distance_list if x is not None):
            stuck_uss_Flag = 1

        with open(fileName, 'a', newline="") as f:
            writer = csv.writer(f)
            # ログデータからfallを削除
            writer.writerow([currentMilliTime(), round(phase, 1), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], lat, lng, alt, distance, object_distance, azimuth, angle, direction])
        time.sleep(DATA_SAMPLING_RATE)


def moveMotor_thread():
    global pi, direction
    MAX_DUTY = 106
    TURN_DUTY = 80
    ROTATE_DUTY = 90

    while True:
        if direction == 360.0:  # 停止 (Brake)
            pi.write(AIN1, 1); pi.write(AIN2, 1)
            pi.write(BIN1, 1); pi.write(BIN2, 1)
            pi.set_PWM_dutycycle(PWMA, 0)
            pi.set_PWM_dutycycle(PWMB, 0)

        elif direction == -360.0:  # 前進
            pi.write(AIN1, 0); pi.write(AIN2, 1)
            pi.write(BIN1, 1); pi.write(BIN2, 0)
            pi.set_PWM_dutycycle(PWMA, MAX_DUTY)
            pi.set_PWM_dutycycle(PWMB, MAX_DUTY)

        elif direction == -400.0:  # その場回転 (右回転)
            pi.write(AIN1, 0); pi.write(AIN2, 1)
            pi.write(BIN1, 0); pi.write(BIN2, 1)
            pi.set_PWM_dutycycle(PWMA, ROTATE_DUTY)
            pi.set_PWM_dutycycle(PWMB, ROTATE_DUTY)

        elif direction == 500.0:  # 左後退
            pi.write(AIN1, 1); pi.write(AIN2, 0)
            pi.write(BIN1, 1); pi.write(BIN2, 1)
            pi.set_PWM_dutycycle(PWMA, TURN_DUTY)
            pi.set_PWM_dutycycle(PWMB, 0)

        elif direction == 600.0:  # 右後退
            pi.write(AIN1, 1); pi.write(AIN2, 1)
            pi.write(BIN1, 1); pi.write(BIN2, 0)
            pi.set_PWM_dutycycle(PWMA, 0)
            pi.set_PWM_dutycycle(PWMB, TURN_DUTY)

        elif direction == 700: # 後退
            pi.write(AIN1, 1); pi.write(AIN2, 0)
            pi.write(BIN1, 0); pi.write(BIN2, 1)
            pi.set_PWM_dutycycle(PWMA, MAX_DUTY)
            pi.set_PWM_dutycycle(PWMB, MAX_DUTY)

        elif direction > 0.0 and direction <= 180.0:  # 左へカーブ
            pi.write(AIN1, 0); pi.write(AIN2, 1)
            pi.write(BIN1, 1); pi.write(BIN2, 0)
            pi.set_PWM_dutycycle(PWMA, TURN_DUTY)
            pi.set_PWM_dutycycle(PWMB, MAX_DUTY)

        elif direction < 0.0 and direction >= -180.0:  # 右へカーブ
            pi.write(AIN1, 0); pi.write(AIN2, 1)
            pi.write(BIN1, 1); pi.write(BIN2, 0)
            pi.set_PWM_dutycycle(PWMA, MAX_DUTY)
            pi.set_PWM_dutycycle(PWMB, TURN_DUTY)
        
        else: # それ以外のdirection値（phase 0, 1など）では停止
            pi.write(AIN1, 1); pi.write(AIN2, 1)
            pi.write(BIN1, 1); pi.write(BIN2, 1)
            pi.set_PWM_dutycycle(PWMA, 0)
            pi.set_PWM_dutycycle(PWMB, 0)

        time.sleep(0.02)


def set_direction():
    global direction, phase, stuck_uss_Flag, stuck_GPS_Flag, upside_down_Flag

    if upside_down_Flag == 1 and phase >= 3:
        direction = 700
        time.sleep(3)
        upside_down_Flag = 0
    elif (stuck_uss_Flag == 1 or stuck_GPS_Flag == 1) and phase >= 3:
        for _ in range(2):
            direction = 500; time.sleep(0.5)
            direction = 600; time.sleep(0.5)
        direction = 90; time.sleep(2)
        direction = -360; time.sleep(2)
        stuck_uss_Flag = 0
        stuck_GPS_Flag = 0
    elif phase == 1:
        direction = 360
    elif phase == 2:
        direction = -400.0
    elif phase == 3:
        diff_angle = (angle - azimuth + 180) % 360 - 180
        if abs(diff_angle) < 10.0:
            direction = -360.0
        else:
            direction = diff_angle
    elif phase == 4:
        direction = -400.0
    elif phase == 5:
        if cone_direction > 0.55:
            direction = 180
        elif cone_direction < 0.45:
            direction = -180
        else:
            direction = -360
    elif phase == 6:
        direction = 360


if __name__ == '__main__':
    main()
