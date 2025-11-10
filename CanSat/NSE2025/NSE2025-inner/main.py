import serial
import time
import math
import threading
import datetime
import csv
import os
import cv2
import RPi.GPIO as GPIO

# import wiringpi as pi
import BNO055
import bmp180
from micropyGPS import MicropyGPS
import detect_corn as dc
from picamera2 import Picamera2

# import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import sys


# 定数　上書きしない
MAG_CONST = 8.9  # 地磁気補正用の偏角
CALIBRATION_MILLITIME = 20 * 1000
TARGET_LAT = 38.26052
TARGET_LNG = 140.8544151
TARGET_ALTITUDE = 20
DATA_SAMPLING_RATE = 0.00001
ALTITUDE_CONST1 = 30
ALTITUDE_CONST2 = 5
HIGH = 1
LOW = 0
# Pin number
heating_wire = 10
PWMA=18
AIN1=8
AIN2=25
PWMB=19
BIN1=9
BIN2=11


# 変数
acc = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]
mag = [0.0, 0.0, 0.0]
calibBias = [0.0, 0.0, 0.0]
calibRange = [1.0, 1.0, 1.0]
lat = 0.0  # from GPS sensor
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
diff_rot = 1
upside_down_Flag = 0  # judge the upside down by acc(bmx)
stuck_GPS_Flag = 0  # judge the stuck by GPS : no obstacle distance_Flag = 0, if CanSat stucked distance_Flag = 1


bmx = BNO055.BNO055()
bmp = bmp180.BMP180(oss=3)
bmp.setUp()

nowTime = datetime.datetime.now()
fileName = "./log/testlog_" + nowTime.strftime("%Y-%m%d-%H%M%S") + ".csv"


def main():
    global phase
    global restTime
    global start
    global gps_detect
    # Flag
    searching_Flag = False
    camera_failed = False
    count_cone_lost = False
    # measure time
    time_start_searching_cone = 0
    time_searching_cone = 0
    time_camera_start = 0
    time_camera_detecting = 0

    GPIO.setwarnings(False)
    Setup()
    phase = 3
    
    while True:
        if phase == 0:  # 投下
            print("phase0 : falling")
            start = time.time()
            while True:
                getBmxData()
                # print(fall)
                if fall > 25:
                    print("para released")
                    time.sleep(10)
                    break
                if time.time() - start > 5 * 60:  # ********  fix later **********
                    print("failed to detect falling")
                    break
                # time.sleep(0.1)
            phase = 1

        elif phase == 1:  # パラ分離
            print("phase1 : remove para")
            print("fire")
            direction=-400.0
            time.sleep(10)
        
            phase = 3

        elif phase == 3:
            direction=360.0
            #print("phase3 : GPS start")
            #print("angle: ", angle)
            #print("azimuth", azimuth)
            #print("direction", direction)
            #if camera_failed == False:
            #    if distance < 5.0:
            #        phase = 4
            #elif camera_failed == True:  # not using camera mode and get closer
            #    if distance < 1.0:
            #        phase = 6  # goal

            #            if upside_down == True:
            #                phase = -2
            # if distance < 1.0:  # GPS座標との距離 < m以内　　#スタック優先
            #    phase = 6

        elif phase == 4:
            print("phase4 : camera start")
            cone_detect()
            if searching_Flag == False:
                searching_Flag = True
                time_start_searching_cone = time.time()
            elif searching_Flag == True:
                time_searching_cone = time.time()
                if (
                    time_searching_cone - time_start_searching_cone >= 20
                ):  # something is wrong with the camera
                    camera_failed = True
                    searching_Flag = False
                    phase = 3  # restart GPS mode and get closer
            if cone_probability < 1:
                phase = 5

        elif phase == 5:
            print("phase5")
            time_camera_start = time.time()
            count_cone_lost = 0
            while True:
                cone_detect()
                time_camera_detecting = time.time()

                if time_camera_detecting - time_camera_start >= 10:
                    if detector.is_detected == False:
                        count_cone_lost += 1
                        print("count_cone_lost", count_cone_lost)
                    if count_cone_lost >= 10:  # camera lost the cone for a long time
                        phase = 4  # restart at phase4
                        break

                if (
                    time_camera_detecting - time_camera_start >= 45
                ):  # camera has been detected the cone for a long time, but Goal has not been judged.
                    print("already reached and being stuck")
                    phase = 6
                    break
                if detector.is_reached:  # if rover reached cone
                    print("reached")
                    phase = 6
                    break

        elif phase == 6:
            print("phase6 : Goal")
            time.sleep(10000)
        #         elif phase==-1:
        #             print("phase-1 : stuck")
        #             if object_distance_Flag ==0:
        #                 phase = -1
        #             elif object_distance_Flag == 1:
        #                 phase = 3
        #                 object_distance_Flag = 0
        #         elif phase ==-2:
        #             print("phase-2 : upside down")
        #             if  upside_down_Flag == 0:
        #                 phase = -2
        #             elif upside_down_Flag == 1:
        #                 phase = 3
        #

        time.sleep(0.1)


def currentMilliTime():
    return round(time.time() * 1000)


def Setup():
    global detector
    bmx.setUp()

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(heating_wire, GPIO.OUT)

    with open(fileName, "a") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "MilliTime",
                "Phase",
                "AccX",
                "AccY",
                "AccZ",
                "GyroX",
                "GyroY",
                "GyroZ",
                "MagX",
                "MagY",
                "MagZ",
                "LAT",
                "LNG",
                "ALT",
                "Distance",
                "Azimuth",
                "Angle",
                "Direction",
                "Fall",
                "cone direction",
                "cone probability"
            ]
        )

    getThread = threading.Thread(target=moveMotor_thread, args=())
    getThread.daemon = True
    getThread.start()

    dataThread = threading.Thread(target=setData_thread, args=())
    dataThread.daemon = True
    dataThread.start()

    gpsThread = threading.Thread(target=GPS_thread, args=())
    gpsThread.daemon = True
    gpsThread.start()

    detector = dc.detector()
    roi_img = cv2.imread("./log/captured.png")

    roi_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2RGB)
    detector.set_roi_img(roi_img)

    print("Setup OK")


def getBmxData():  # get BMX data
    global acc
    global gyro
    global mag
    global fall
    acc = bmx.getAcc()
    gyro = bmx.getGyro()
    mag = bmx.getMag()
    # mag[1] = mag[1]
    # mag[2] = mag[2]
    fall = math.sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2])
    #for i in range(3):
    #    mag[i] = (mag[i] - calibBias[i]) / calibRange[i]


def getBmpData():
    global alt
    global pres
    alt = bmp.getAltitude()
    pres = bmp.getPressure()


def flying():  # 落下検知関数 :飛んでいるときはTrueを返し続ける
    # この関数は何回も繰り返されることを想定
    global maxAlt
    global minAlt

    if maxAlt < alt:
        maxAlt = alt
    if minAlt > alt:
        minAlt = alt
    subAlt = maxAlt - minAlt
    absAlt = abs(alt - minAlt)

    if subAlt > ALTITUDE_CONST1 and absAlt < ALTITUDE_CONST2:
        print("bmp : reached ground")
        time.sleep(10)
        return False

    else:
        True


def upside_down():
    global upside_down_Flag
    global acc
    if acc[0] > 0:
        upside_down_Flag = 1


def calibration():  # calibrate BMX raw data
    global calibBias
    global calibRange
    max = [0.0, 0.0, 0.0]
    min = [0.0, 0.0, 0.0]
    mag = bmx.getMag()
    max[0] = mag[0]
    max[1] = mag[1]
    #max[2] = mag[2]
    min[0] = mag[0]
    min[1] = mag[1]
    #min[2] = mag[2]

    complete = False
    while complete == False:
        before = currentMilliTime()
        after = before
        while (after - before) < CALIBRATION_MILLITIME:
            mag = bmx.getMag()
            if max[0] < mag[0]:
                max[0] = mag[0]
            elif min[0] > mag[0]:
                min[0] = mag[0]
            elif max[1] < mag[1]:
                max[1] = mag[1]
            elif min[1] > mag[1]:
                min[1] = mag[1]
            after = currentMilliTime()
        if (max[0] - min[0]) > 3 and (max[1] - min[1] > 3):
            print("calibration(): Complete!")
            complete = True
            time.sleep(1)
            calibBias[0] = (max[0] + min[0]) / 2
            calibBias[1] = (max[1] + min[1]) / 2

            calibRange[0] = (max[0] - min[0]) / 2
            calibRange[1] = (max[1] - min[1]) / 2
            print("calibBias", calibBias, "calibRange", calibRange)
            time.sleep(1)


def calcdistance():  # 距離計算用関
    global distance
    EARTH_RADIUS = 6378136.59
    dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng)
    dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
    distance = math.sqrt(dx * dx + dy * dy)
    # print(f"distance: {distance}")


def calcAngle():  # 角度計算用関数 : north=0 east=90 west = -90
    global angle
    forEAstAngle = 0.0
    EARTH_RADIUS = 6378136.59

    dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng)
    dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
    angle = 90 - math.degrees(math.atan2(dy, dx))
    angle %= 360.0
    # if dx == 0 and dy == 0:
    #    forEastAngle = 0.0
    # else:
    #    forEastAngle = (180 / math.pi) * math.atan2(dy, dx)  # arctan
    # angle = forEastAngle - 90
    # if angle < -180:
    #    angle += 360
    # if angle > 180:
    #    angle -= 360
    # angle = -angle
    # print(f"angle: {angle}")


def calcAzimuth():  # 方位角計算用関数
    global azimuth

    azimuth = 90 - math.degrees(math.atan2(mag[1], mag[0]))
    azimuth *= -1
    azimuth % 360  # 上のazimuthはCanSatからみた北の方位
    # print(f"azimuth: {azimuth}")
    # if mag[1] == 0.0:
    #     mag[1] = 0.0000001
    # azimuth = -(180 / math.pi) * math.atan(mag[2] / mag[1])
    # if mag[1] > 0:
    #     azimuth = 90 + azimuth
    # elif mag[1] < 0:
    #     azimuth = -90 + azimuth


def GPS_thread():  # GPSモジュールを読み、GPSオブジェクトを更新する
    global lat
    global lng
    global gps_detect

    s = serial.Serial("/dev/serial0", 115200)
    s.readline()  # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    gps = MicropyGPS(9, "dd")

    while True:
        sentence = s.readline().decode("utf-8")  # GPSデーターを読み、文字列に変換する

        if s.in_waiting > 64:  # バッファを削除
            s.reset_input_buffer()
        if sentence[0] != "$":  # 先頭が'$'でなければ捨てる
            continue
        for (
            x
        ) in (
            sentence
        ):  # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
            gps.update(x)
        lat = gps.latitude[0]
        lng = gps.longitude[0]

        if lat > 0:
            gps_detect = 1
        elif lat == 0.0:
            gps_detect = 0
        # print(lat)
        # print(lng)


def cone_detect():
    global detector
    global cone_direction
    global cone_probability

    detector.detect_cone()
    try:
        cone_direction = 1 - detector.cone_direction
        cone_probability = detector.probability
    except:
        cone_direction = 0.5
        cone_probability = 10
#    print("direction",130.960084 cone_direction)
#    print("prob.", cone_probability)


def setData_thread():
    while True:
        getBmxData()
        getBmpData()
        calcAngle()
        calcAzimuth()
        set_direction()
        calcdistance()
        with open(fileName, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    currentMilliTime(),
                    round(phase, 1),
                    acc[0],
                    acc[1],
                    acc[2],
                    gyro[0],
                    gyro[1],
                    gyro[2],
                    mag[0],
                    mag[1],
                    mag[2],
                    lat,
                    lng,
                    alt,
                    distance,
                    azimuth,
                    angle,
                    direction,
                    fall,
                    cone_direction,
                    cone_probability
                ]
            )
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
            M_pwmA.ChangeDutyCycle(85)
            M_pwmB.ChangeDutyCycle(100)
            GPIO.output(AIN1,HIGH)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,HIGH)
            GPIO.output(BIN2,LOW) 

        elif direction == -400.0:  # rotate  left
            M_pwmA.ChangeDutyCycle(60)
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
    global object_distance_Flag
    global upside_down_Flag

    if phase == 0:  # 投下
        direction = 360

    elif phase == 1:
        direction = -400.0

    elif phase == 2:  # キャリブレーション
        direction = -400.0  # right

    elif phase == 3:
        direction=360.0
        #direction = azimuth - angle
        #direction %= 360
        #if (direction > 180):
        #    direction -= 360
        #if abs(direction) < 5.0:
        #    direction = -360
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
    elif phase == -1:
        for _ in range(4):
            direction = 500
            time.sleep(0.3)
            direction = 600
            time.sleep(0.3)
        direction = 90
        time.sleep(2)
        direction = -360
        time.sleep(2)
        stuck_uss_Flag = 0
        stuck_GPS_Flag = 0
    elif phase == -2:
        direction = 700
        time.sleep(3)
        upside_down_Flag = 0


if __name__ == "__main__":
    main()
    time.sleep(100)
