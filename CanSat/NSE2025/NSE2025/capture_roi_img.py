import cv2
import numpy as np
from picamera2 import Picamera2 # camera
import time

print("Capture start")
picam2 = Picamera2()
fullReso= picam2.camera_properties["PixelArraySize"]
cam_conf = picam2.create_preview_configuration({"format" : "BGR888", "size": fullReso})
picam2.configure(cam_conf)
t1 = time.time()
picam2.start()
print(time.time() - t1)
input_img = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_RGB2BGR)

picam2.capture_file("./log/captured_roi_img.png")
