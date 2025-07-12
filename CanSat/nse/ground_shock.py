import RPi.GPIO as GPIO
from library import bno055
import math
import datetime
import csv
import time
nowTime = datetime.datetime.now()
bno=bno055.BNO055()
bno.setUp()
fileName = '/home/raspberry/Desktop/log/testlog_' + nowTime.strftime('%Y-%m%d-%H%M%S') + '.csv'
start=time.time()
while True:
    acc = bno.getAcc()
    fall = math.sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2] )
    end=time.time()
    with open(fileName, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([round(end-start,3),fall])
