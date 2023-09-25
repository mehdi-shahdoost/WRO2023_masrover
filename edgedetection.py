# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.rotation = 180
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera)
# allow the camera to warmup
time.sleep(0.1)
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    img = frame.array
    # show the frame
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 100, 70)
    ret, mask = cv2.threshold(canny, 70, 255, cv2.THRESH_BINARY)
    rho = 1  # distance resolution in pixels of the Hough grid
    theta = np.pi / 180  # angular resolution in radians of the Hough grid
    threshold = 10  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 300  # minimum number of pixels making up a line
    max_line_gap = 30  # maximum gap in pixels between connectable line segments
    line_image = np.copy(mask) * 0  # creating a blank to draw lines on
    
  # Run Hough on edge detected image
  # Output "lines" is an array containing endpoints of detected line segments
    lines = cv2.HoughLinesP(mask, rho, theta, threshold, np.array([]),
                      min_line_length, max_line_gap)
    try:
      for line in lines:
        for x1,y1,x2,y2 in line:
          x = cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)
          cv2.imshow('Video feed', x)
    except:
      print("no line")
  
    
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break