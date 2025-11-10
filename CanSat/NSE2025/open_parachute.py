import RPi.GPIO as GPIO
from library import bmp180
import math
import datetime
import csv
import time
nowTime = datetime.datetime.now()
bmp = bmp180.BMP180(oss=3)
bmp.setUp()
ground_alt=37.3
fileName = '/home/raspberry/Desktop/log/open_parachute/testlog_' + nowTime.strftime('%Y-%m%d-%H%M%S') + '.csv'
start=time.time()
while True:
    alt = bmp.getAltitude()
    print(f'alt:{alt}')
    end=time.time()
    with open(fileName, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([round(end-start,3),(alt-ground_alt)])
