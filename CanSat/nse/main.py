import serial
import time
import math
import threading
import datetime
import csv
import os
import cv2
import RPi.GPIO as GPIO
import sys
sys.path.append("/home/raspberry/Desktop/library")
import detect_corn as dc
from micropyGPS import MicropyGPS
import bno055
import bmp180
from picamera2 import Picamera2



# 定数　上書きしない
MAG_CONST = 8.53  # 地磁気補正用の偏角
CALIBRATION_MILLITIME = 10 * 1000
TARGET_LAT = 40.14230733
TARGET_LNG = 139.98738483
# TARGET_LAT = 40.14247083
# TARGET_LNG = 139.98780116
TARGET_ALTITUDE = 20
DATA_SAMPLING_RATE = 0.00001
ALTITUDE_CONST1 = 30
ALTITUDE_CONST2 = 5
SERVO_PIN = 14
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
TRIG = 23                     
ECHO = 24                           
SPEED_OF_SOUND = 34767  #音速（気温27℃）
OBJECT_DISTANCE = 10 # if Object is in 10 meter,Rover stuck 

# 変数
acc = [0.0, 0.0, 0.0]
pre_z=0
gyro = [0.0, 0.0, 0.0]
mag = [0.0, 0.0, 0.0]
calibBias = [0.0, 0.0, 0.0]
calibRange = [1.0, 1.0, 1.0]
lat = 0.0
lng = 0.0
alt = 0.0
maxAlt = 0.0
minAlt = 0.0
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
diff_rot = 0
object_distance = 0.0
upside_down_Flag = 0 # judge the upside down by acc(bmx)
stuck_uss_Flag=0    # judge the stuck by ultrasonic sensor
stuck_GPS_Flag = 0  # judge the stuck by GPS : no obstacle distance_Flag = 0, if CanSat stucked distance_Flag = 1


bmp = bmp180.BMP180(oss=3)
bmp.setUp()
bno=bno055.BNO055()
bno.setUp()

nowTime = datetime.datetime.now()
fileName = '/home/raspberry/Desktop/log/ete/testlog' + nowTime.strftime('%Y-%m%d-%H%M%S') + '.csv'


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
        print("-----------------------------------")
        if phase == 0:  # 投下
            print("phase0 : falling")
            start = time.time()
            #getbnoData()
            while True:
                restTime = time.time() - start
                if fall > 22:
                    time.sleep(5)
                    break
                if flying() == False:
                    break
                if restTime > 5 * 60:# 5 * 60
                    phase = 1
                    break
                time.sleep(0.1)
            phase = 1

        elif phase == 1: # パラ分離
            print("phase1 : remove para")
            time.sleep(10)
            phase = 2

        elif phase == 2:  # キャリブレーション
            print("phase2 : calibration start")
            direction = -360
            time.sleep(3)
            calibration()
            phase = 3
        elif phase == 3:
            print("phase3 : GPS start")
#            print(direction)
#            print(distance)
#            print("--------------")
            if distance < 3.0:  # GPS座標との距離 < m以内
                phase = 4
                
        elif phase == 4:
            print("phase4 : camera start")
            start2 = time.time()
            cone_detect()
            if cone_probability < 1:
                phase = 5

            
            if n > 5:
                phase = 6
                break
            n = n+1

        elif phase == 5:
            print("phase5")
            while True:
                cone_detect()
#                 if detector.is_detected:
#                     print("cone detected")
#                     print(detector.detected[cv2.CC_STAT_AREA])
                if detector.is_reached: # if rover reached cone
                    print("reached")
                    phase = 6
                    break
                if object_distance < 30:
                    phase = 6
                    break
                print(object_distance)
#                 if cone_probability > 1: # detect cone probability
#                     n += 1
#                     phase = 4
                    
        elif phase == 6:
            print("phase6 : Goal")
            time.sleep(10000)
        
        while ( stuck_uss_Flag == 1 and phase >= 3):      # フラグ折り待ちループ
            pass
            
        time.sleep(0.1)



def currentMilliTime():
    return round(time.time() * 1000)


def Setup():
    global detector

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED1, GPIO.OUT)
    GPIO.setup(LED2, GPIO.OUT)
    GPIO.setup(LED3, GPIO.OUT)
    GPIO.setup(TRIG, GPIO.OUT)          # Trigピン出力モード設定
    GPIO.setup(ECHO, GPIO.IN)           # Echoピン入力モード設定
    
    


    with open(fileName, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(['MilliTime','Phase','AccX','AccY','AccZ','GyroX','GyroY','GyroZ','MagX','MagY','MagZ','ALT','Distance','Azimuth','Direction','Fall'])
        
    getThread = threading.Thread(target=moveMotor_thread, args=())
    getThread.daemon = True
    getThread.setDaemon(True)
    getThread.start()

    UssThread = threading.Thread(target=setUss_thread, args=())
    UssThread.daemon = True
    UssThread.setDaemon(True)
    UssThread.start()
    
    dataThread = threading.Thread(target=setData_thread, args=())
    dataThread.daemon = True
    dataThread.setDaemon(True)
    dataThread.start()

    detector = dc.detector()
    roi_img = cv2.imread("/home/raspberry/Desktop/library/roi_red_v2.JPG")
    
    detector.set_roi_img(roi_img)

    GPIO.output(LED2, HIGH)
    print("Setup OK")


def getbnoData():  # get BNO data
    global acc
    global gyro
    global mag
    global fall
    global pre_z
    acc = bno.getAcc()
    if abs(acc[2])>200:
        acc[2]=pre_z
    else:
        pre_z=acc[2]
        print(pre_z)
    gyro = bno.getGyro()
    mag = bno.getMag()
    mag[1] = mag[1]
    mag[2] = -mag[2]
    fall = math.sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2] )
    for i in range(3):
        mag[i] = (mag[i] - calibBias[i]) / calibRange[i]


def getBmpData():
    global alt
    global pres
    alt = bmp.getAltitude()
    pres = bmp.getPressure()
    
    alt = alt + 60 # calibration

def get_object_distance(timeout=0.01):  #超音波での障害物との距離計算関数 #0.003秒で約50cm
    #Trigピンを10μsだけHIGHにして超音波の発信開始
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.000010)
    GPIO.output(TRIG, GPIO.LOW)
    start_time=time.time()
    while not GPIO.input(ECHO):
        #if time.time() - start_time > timeout:
            #return 100  
        pass
    t1 = time.time() # 超音波発信時刻（EchoピンがHIGHになった時刻）格納
    while GPIO.input(ECHO):
        if time.time() - start_time > timeout:
            return 50
    t2 = time.time() # 超音波受信時刻（EchoピンがLOWになった時刻）格納
    return (t2 - t1) * SPEED_OF_SOUND / 2 # 時間差から対象物までの距離計算




# def detect_upside_down():
#     global upside_down_Flag
#     global acc
#     if acc[1] > 0:
#         upside_down_Flag=1
#         print("Now Rover looks the sky;;")
#         
def flying(): #落下検知関数 :飛んでいるときはTrueを返し続ける
    #この関数は何回も繰り返されることを想定
    global maxAlt
    global minAlt
    

    if maxAlt < alt:
        maxAlt = alt
    if minAlt > alt:
        minAlt = alt
    subAlt = maxAlt-minAlt
    absAlt = abs(alt-minAlt)

    if subAlt > ALTITUDE_CONST1  and absAlt < ALTITUDE_CONST2:
        print("bmp : reached ground")
        time.sleep(10)
        return False
     
    else:
        True


def calibration():  # calibrate BMX raw data
    global calibBias
    global calibRange
    max = [0.0, 0.0, 0.0]
    min = [0.0, 0.0, 0.0]
    max[1] = mag[1]
    max[2] = mag[2]
    min[1] = mag[1]
    min[2] = mag[2]

    complete = False;
    while complete == False:
        before = currentMilliTime()
        after = before
        while (after - before) < CALIBRATION_MILLITIME:
            if max[1] < mag[1]:
                max[1] = mag[1]
            elif min[1] > mag[1]:
                min[1] = mag[1]
            elif max[2] < mag[2]:
                max[2] = mag[2]
            elif min[2] > mag[2]:
                min[2] = mag[2]
            after = currentMilliTime()
        if (max[1] - min[1]) > 20 and (max[2] - min[2] > 20):
            print("calibration(): Complete!")
            GPIO.output(LED1, HIGH)
            complete = True
            time.sleep(0.5)
            calibBias[1] = (max[1] + min[1]) / 2
            calibBias[2] = (max[2] + min[2]) / 2

            calibRange[1] = (max[1] - min[1]) / 2
            calibRange[2] = (max[2] - min[2]) / 2
            GPIO.output(LED1, LOW)
            time.sleep(0.5)

def calcAzimuth():  # 方位角計算用関数
    global azimuth

    if mag[1] == 0.0:
        mag[1] = 0.0000001
    azimuth = -(180 / math.pi) * math.atan(mag[2] / mag[1])
    if mag[1] > 0:
        azimuth = 90 + azimuth
    elif mag[1] < 0:
        azimuth = -90 + azimuth


def servoMotor(angle):
    assert 0 <= angle <= 180, '角度は0から180の間でなければなりません'

    # 角度を500から2500のパルス幅にマッピングする
    pulse_width = (angle / 180) * (2500 - 500) + 500
    # パルス幅を設定してサーボを回転させる
    servo.set_servo_pulsewidth(SERVO_PIN, pulse_width)

def LED_Checker(num):
    for _ in range(num):
        GPIO.output(LED1, HIGH)
        time.sleep(0.1)
        GPIO.output(LED1, LOW)
        time.sleep(0.1)
    
def cone_detect():
    global detector
    global cone_direction
    global cone_probability
       
    
    detector.detect_cone()
    cone_direction = 1 - detector.cone_direction #flip
    cone_probability = detector.probability
    print((cone_direction, cone_probability))

    
def setUss_thread():
    object_distance_list=[]
    #Stuck_uss_Flag立てちゃうよ　＾＾
    global stuck_uss_Flag
    global object_distance
    
    while True:
        object_distance=get_object_distance()
        object_distance_list.append(object_distance)
        if len(object_distance_list)>5:
            object_distance_list.pop(0)   
        if all( x <= OBJECT_DISTANCE for x in object_distance_list):
            stuck_uss_Flag = 1
        time.sleep(0.5)

def setData_thread():
    while True:
        getbnoData()
        calcAzimuth()
        set_direction()

        with open(fileName, 'a', newline="") as f:
            writer = csv.writer(f)
            writer.writerow([currentMilliTime(), round(phase,1), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[1], mag[1], mag[2], alt, distance, object_distance, azimuth,direction, fall])
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

    if (stuck_uss_Flag==1 or stuck_GPS_Flag==1) and phase >=3:
        for _ in range(4):
            direction = 500
            time.sleep(0.3)
            direction = 600
            time.sleep(0.3)
        direction = 90
        time.sleep(2)
        direction = -360
        time.sleep(2)
        stuck_uss_Flag=0
        stuck_GPS_Flag=0


    if phase == 0:  # 投下
        direction = 360

    elif phase == 1:
        direction = -360

    elif phase == 2:  # キャリブレーション
        direction = -400.0  # right

    elif phase == 4:
        direction = -400.0
        
    elif phase == 5:
        if cone_direction > 0.7:
            direction = -180
        elif cone_direction <= 0.7 and cone_direction >= 0.3:
            direction = -360
        elif cone_direction < 0.3:
            direction = 180
        
    elif phase == 6:
        direction = 360
    
        
             
            


if __name__ == '__main__':
    main()
    time.sleep(100)
