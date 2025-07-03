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
from library import BMX055
from library import BMP085
from micropyGPS import MicropyGPS
from library import detect_corn as dc
from picamera2 import Picamera2



# 定数　上書きしない
MAG_CONST = 8.53  # 地磁気補正用の偏角
CALIBRATION_MILLITIME = 10 * 1000
TARGET_LAT = 40.14230733
TARGET_LNG = 139.98738483
# TARGET_LAT = 40.14247083
# TARGET_LNG = 139.98780116
DATA_SAMPLING_RATE = 0.00001
ALTITUDE_CONST1 = 30
LED1 = 16
LED2 = 20
LED3 = 21
HIGH = 1
LOW = 0
PWMA=18
AIN1=25
AIN2=8
PWMB=19
BIN1=9
BIN2=11
ROTATION_DIFF = 80
MOTOR_COFFICIENT = 0.5                          
OBJECT_DISTANCE = 10 # if Object is in 10 meter,Rover stuck 

# 変数
acc = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]
mag = [0.0, 0.0, 0.0]
calibBias = [0.0, 0.0, 0.0]
calibRange = [1.0, 1.0, 1.0]
lat = 0.0
lng = 0.0
pres = 0.0
distance = 0
angle = 0.0
azimuth = 0.0
direction = 0.0
frequency = 50
phase = 0
gps_detect = 0
cone_direction = 0
cone_probability = 0
restTime = 0.0
diff_rot = 0.4
object_distance = 0.0
upside_down_Flag = 0 # judge the upside down by acc(bmx)
stuck_uss_Flag=0    # judge the stuck by ultrasonic sensor
stuck_GPS_Flag = 0  # judge the stuck by GPS : no obstacle distance_Flag = 0, if CanSat stucked distance_Flag = 1


bmx = BMX055.BMX055()
bmp = BMP085.BMP085()
servo = pigpio.pi()

nowTime = datetime.datetime.now()
fileName = '/home/karisora/FTE14/NSE2024/log/testlog_' + nowTime.strftime('%Y-%m%d-%H%M%S') + '.csv'


def main():
    global phase
    global restTime
    global start
    global gps_detect

    GPIO.setwarnings(False)
    Setup()
    phase = 0
    n = 0

    while True:
        elif phase == 0:  # キャリブレーション
            print("phase0 : calibration start")
            direction = -360
            time.sleep(3)
            calibration()
            phase = 1

        elif phase == 1:
            print("phase1 : GPS start")
            if distance < 5:  # GPS座標との距離 < m以内
                phase = 2
        
        while ( (stuck_uss_Flag == 1 or stuck_GPS_Flag == 1) and phase >= 3):      # フラグ折り待ちループ
            pass
            
        time.sleep(0.1)

        elif phase == 2:
            print("phase2: GOAL!")

def currentMilliTime():
    return round(time.time() * 1000)


def Setup():
    global detector
    bmx.setUp()

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED1, GPIO.OUT)
    GPIO.setup(LED2, GPIO.OUT)
    GPIO.setup(LED3, GPIO.OUT)

    with open(fileName, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(['MilliTime','Phase','AccX','AccY','AccZ','GyroX','GyroY','GyroZ','MagX','MagY','MagZ','LAT','LNG','ALT',"distance",'Azimuth','Angle','Direction'])
        
    getThread = threading.Thread(target=moveMotor_thread, args=())
    getThread.daemon = True
    getThread.setDaemon(True)
    getThread.start()

    dataThread = threading.Thread(target=setData_thread, args=())
    dataThread.daemon = True
    dataThread.setDaemon(True)
    dataThread.start()

    gpsThread = threading.Thread(target=GPS_thread, args=())
    gpsThread.daemon = True
    gpsThread.setDaemon(True)
    gpsThread.start()

    GPIO.output(LED2, HIGH)
    print("Setup OK")


def getBmxData():  # get BMX data
    global acc
    global gyro
    global mag
    global fall
    acc = bmx.getAcc()
    gyro = bmx.getGyro()
    mag = bmx.getMag()
    mag[1] = mag[1]
    mag[2] = -mag[2]
    fall = math.sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2] )
    for i in range(3):
        mag[i] = (mag[i] - calibBias[i]) / calibRange[i]





def detect_stuck_by_GPS(): #generation構文
    global stuck_GPS_Flag #0:not stuck　1:stuck
    while_Flag=True
    t1=0.0

    while True:
        if while_Flag:
            t1=time.time()
            distance_buff= distance
            while_Flag= False
            yield
        if time.time()-t1 < 10:
            yield
            continue
        if abs(distance - distance_buff) < 1:
            stuck_GPS_Flag = 1
        while_Flag = True  

# def detect_upside_down():
#     global upside_down_Flag
#     global acc
#     if acc[1] > 0:
#         upside_down_Flag=1
#         print("Now Rover looks the sky;;")
#         


def calcdistance():  # 距離計算用関
    global distance
    EARTH_RADIUS = 6378136.59
    dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng)
    dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
    distance = math.sqrt(dx * dx + dy * dy)


def calcAngle():  # 角度計算用関数 : north=0 east=90 west = -90
    global angle
    forEAstAngle = 0.0
    EARTH_RADIUS = 6378136.59

    dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng)
    dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
    if dx == 0 and dy == 0:
        forEastAngle = 0.0
    else:
        forEastAngle = (180 / math.pi) * math.atan2(dy, dx)  # arctan
    angle = forEastAngle - 90
    if angle < -180:
        angle += 360
    if angle > 180:
        angle -= 360
    angle = -angle   


def calcAzimuth():  # 方位角計算用関数
    global azimuth

    if mag[1] == 0.0:
        mag[1] = 0.0000001
    azimuth = -(180 / math.pi) * math.atan(mag[2] / mag[1])
    if mag[1] > 0:
        azimuth = 90 + azimuth
    elif mag[1] < 0:
        azimuth = -90 + azimuth

def LED_Checker(num):
    for _ in range(num):
        GPIO.output(LED1, HIGH)
        time.sleep(0.1)
        GPIO.output(LED1, LOW)
        time.sleep(0.1)


def GPS_thread():  # GPSモジュールを読み、GPSオブジェクトを更新する
    global lat
    global lng
    global gps_detect
    global stuck_GPS_Flag
    stuck_GPS_detection = detect_stuck_by_GPS()  #genetration 

    s = serial.Serial('/dev/ttyAMA0', 115200)
    s.readline()  # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    gps = MicropyGPS(9, 'dd')
    
    while True:
        sentence = s.readline().decode('utf-8')  # GPSデーターを読み、文字列に変換する

        if s.in_waiting > 64: # バッファを削除
            s.reset_input_buffer()
        if sentence[0] != '$':  # 先頭が'$'でなければ捨てる
            continue
        for x in sentence:  # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
            gps.update(x)
        lat = gps.latitude[0]
        lng = gps.longitude[0]

        if lat == 0.0:
            gps_detect = 0
            print("None gnss value")
            continue
        gps_detect = 1
        #stuck_GPS_Flag立てちゃうよ^^
        stuck_GPS_detection.__next__() 
    

            
      
def setData_thread():
    while True:
        getBmxData()
        calcAngle()
        calcAzimuth()
        set_direction()
        calcdistance()

        with open(fileName, 'a', newline="") as f:
            writer = csv.writer(f)
            writer.writerow([currentMilliTime(), round(phase,1), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[1], mag[1], mag[2], lat, lng,distance, azimuth, angle, direction, fall])
        time.sleep(DATA_SAMPLING_RATE)


def moveMotor_thread():
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(PWMA, GPIO.OUT)
    GPIO.setup(AIN1, GPIO.OUT)
    GPIO.setup(AIN2, GPIO.OUT)
    GPIO.setup(PWMB, GPIO.OUT)
    GPIO.setup(BIN1, GPIO.OUT)
    GPIO.setup(BIN2, GPIO.OUT)

    M_pwmA = GPIO.PWM(PWMA, frequency)
    M_pwmB = GPIO.PWM(PWMB, frequency)

    M_pwmA.start(100) # right tire pwm
    M_pwmB.start(100)

    
    while True:
        if direction == 360.0:  # stop
            M_pwmA.ChangeDutyCycle(0)
            M_pwmB.ChangeDutyCycle(0)
            GPIO.output(AIN1,LOW)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,LOW)
            GPIO.output(BIN2,LOW) 

        elif direction == 500.0:  # left 
            M_pwmA.ChangeDutyCycle(0)
            M_pwmB.ChangeDutyCycle(100)
            GPIO.output(AIN1,LOW)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,HIGH)
            GPIO.output(BIN2,LOW) 

        elif direction == 600.0:  # right 
            M_pwmA.ChangeDutyCycle(100)
            M_pwmB.ChangeDutyCycle(0)
            GPIO.output(AIN1,HIGH)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,LOW)
            GPIO.output(BIN2,LOW) 

        elif direction == -360.0:  #forward
            M_pwmA.ChangeDutyCycle(100)
            M_pwmB.ChangeDutyCycle(100)
            GPIO.output(AIN1,HIGH)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,HIGH)
            GPIO.output(BIN2,LOW) 

        elif direction == -400.0:  # rotate  left
            M_pwmA.ChangeDutyCycle(30)
            M_pwmB.ChangeDutyCycle(100)
            GPIO.output(AIN1,HIGH)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,HIGH)
            GPIO.output(BIN2,LOW)  
            
        elif direction > 0.0 and direction <= 180.0:  # turn left
            M_pwmA.ChangeDutyCycle(20)
            M_pwmB.ChangeDutyCycle(100)
            GPIO.output(AIN1,HIGH)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,HIGH)
            GPIO.output(BIN2,LOW) 

        elif direction < 0.0 and direction >= -180.0:  # turn right
            M_pwmA.ChangeDutyCycle(100)
            M_pwmB.ChangeDutyCycle(20)            
            GPIO.output(AIN1,HIGH)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,HIGH)
            GPIO.output(BIN2,LOW) 

def set_direction():  # -180<direction<180  #rover move to right while direction > 0
    global direction
    global phase
    global stuck_uss_Flag
    global stuck_GPS_Flag
    global upside_down_Flag
    
#         #フラグ処理を優先
#     if upside_down_Flag == 1 and phase >= 3:
#         direction = -360
#         time.sleep(3)
#         upside_down_Flag=0


#     while gps_detect == 0: # if not GNSS is updated
#         direction = 360
#         continue

    if (stuck_GPS_Flag==1) and phase ==1:
        for _ in range(4):
            direction = 500
            time.sleep(0.3)
            direction = 600
            time.sleep(0.3)
        direction = 90
        time.sleep(2)
        direction = -360
        time.sleep(2)
        stuck_GPS_Flag=0


    elif phase == 0:  # キャリブレーション
        direction = -400.0  # right

    elif phase == 1:
        if (angle - azimuth) > 180:
            theta = angle - 360
        elif (azimuth - angle) > 180:
            theta = angle + 360
        else:
            theta = angle

        direction = theta - azimuth

        if abs(direction) < 5.0: # margin of the target angle
            direction = -360.0


    elif phase == 2:
        direction = 360.0
        exit()    


if __name__ == '__main__':
    main()
    time.sleep(100)
