import RPi.GPIO as GPIO
import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time


s0 = 15
s1 = 14
s2 = 3
s3 = 2
signal = 17
NUM_CYCLES = 30
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.rotation = 180
camera.resolution = (640, 480)
camera.framerate = 32
camera.brightness = 55
rawCapture = PiRGBArray(camera)
# allow the camera to warmup
time.sleep(0.1)

    
servo_pwm = 19
motor_pwm = 13
button = 26

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_pwm,GPIO.OUT)
v = GPIO.PWM(motor_pwm,1000)
v.start(0)
GPIO.setup(servo_pwm,GPIO.OUT)
s = GPIO.PWM(servo_pwm,50)
s.start(0)
GPIO.setup(button,GPIO.IN)
sensor = 14

GPIO.setup(signal,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(s0,GPIO.OUT)
GPIO.setup(s1,GPIO.OUT)
GPIO.setup(s2,GPIO.OUT)
GPIO.setup(s3,GPIO.OUT)

GPIO.output(s0,GPIO.LOW)
GPIO.output(s1,GPIO.HIGH)
GPIO.output(s2,GPIO.LOW)
GPIO.output(s3,GPIO.LOW)
red_seen = False
green_seen = False

def color():  
    start = time.time()
    for impulse_count in range(NUM_CYCLES):
      GPIO.wait_for_edge(signal, GPIO.FALLING)
    duration = time.time() - start      #seconds to run for loop
    red  = NUM_CYCLES / duration   #in Hz
    return round(red,1)


def servo(x):
    x = int(x)
    if x > 100: x = 100
    elif x < -100: x = -100
    
    if red_seen and x < 0: x = 0
    elif green_seen and x > 0: x = 0
    d = (x/100)*3
    
    s.ChangeDutyCycle(6.5 + d)
    
        
def speed(velocity):
    if velocity > 100: velocity = 100
    elif velocity < 0: velocity = 0
    v.ChangeDutyCycle(velocity)
    
while GPIO.input(button) == 0:
    speed(0)
    servo(0)

lr = -1
r = 0
last_check = time.time()
start_time = time.time()
last_stop = time.time()
red_seen_time = time.time()
green_seen_time = time.time()
red_area = 0
green_area = 0
for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    
    imageFrame = frame.array
    imageFrame = imageFrame[100:,:]
    colorFrame = imageFrame
    hsvFrame = cv2.cvtColor(colorFrame, cv2.COLOR_BGR2HSV)
    
    imageFrame = cv2.blur(imageFrame, (5,5))
    imageFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2GRAY)
    
   
    full_thresh = cv2.threshold(imageFrame,60,255, cv2.THRESH_BINARY_INV)[1]
    left_thresh = full_thresh[:,:270]
    right_thresh = full_thresh[:,370:640]
    center_thresh = full_thresh[:,160:480]
    
    #cv2.imshow("Multiple Color Detection in Real-TIme", hsvFrame)
    
    cnts_l = cv2.findContours(left_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_l = cnts_l[0] if len(cnts_l) == 2 else cnts_l[1]

    cnts_r = cv2.findContours(right_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_r = cnts_r[0] if len(cnts_r) == 2 else cnts_r[1]

    cnts_c = cv2.findContours(center_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_c = cnts_c[0] if len(cnts_c) == 2 else cnts_c[1]
    red_lower = np.array([110,100,70], np.uint8)
    red_upper = np.array([180,255,180], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    
    green_lower = np.array([30, 50, 50], np.uint8)
    green_upper = np.array([90, 200, 120], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    kernal = np.ones((20,20), "uint8")
      
    # For red color
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(colorFrame, colorFrame, mask = red_mask)
    
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(colorFrame, colorFrame, mask = green_mask)
   
    # Creating contour to track red color
    r_contours, r_hierarchy = cv2.findContours(red_mask,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    g_contours, g_hierarchy = cv2.findContours(green_mask,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    r_flag = False
    for pic, contour in enumerate(r_contours):
        r_areas = [cv2.contourArea(c) for c in r_contours]
        max_index = np.argmax(r_areas)
        cnt=r_contours[max_index]
        M = cv2.moments(contour)
        r_area = cv2.contourArea(contour)
        red_area = 0
        if(r_area > 1000):
            x, y, w, h = cv2.boundingRect(cnt)
            if 1.6 >= h/w >= 1.2:
                print("red")
                imageFrame = cv2.rectangle(colorFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                red_center = x
                r_flag = True
                red_seen = True
                red_seen_time = time.time()
                red_area = r_area
        
            

    g_flag = False
    for pic, contour in enumerate(g_contours):
        g_areas = [cv2.contourArea(c) for c in g_contours]
        max_index = np.argmax(g_areas)
        cnt=g_contours[max_index]
        M = cv2.moments(contour)
        g_area = cv2.contourArea(contour)
        green_area = 0 
        if(g_area > 1000):
            x, y, w, h = cv2.boundingRect(cnt)
            if 1.6 >= h/w >= 1.2:
                print("green")
                imageFrame = cv2.rectangle(colorFrame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                green_center = x
                g_flag = True
                green_seen = True
                green_seen_time = time.time()
                green_area = g_area
    
    cv2.imshow("Multiple Color Detection in Real-TIme", colorFrame)

    if red_seen and green_seen:
        if green_area > red_area:
            red_seen = False
        else:
            green_seen = False

    if time.time() - red_seen_time >= 1:
        red_seen = False
    elif time.time() - green_seen_time >= 1:
        green_seen = False
    
            
    area_r = 0
    area_l = 0
    area_c = 0
    for c in cnts_l:
       area_l = round(cv2.contourArea(c),1)/1000
    for c in cnts_r:
       area_r = round(cv2.contourArea(c),1)/1000
    for c in cnts_c:
       area_c = round(cv2.contourArea(c),1)/1000
    
    
    
    if r_flag == True and red_center >= 200:
        servo((red_area/10))
        print((red_area/10))
    elif g_flag == True and green_center <= 500:
        servo(-(green_area/10))
    elif time.time() - start_time <= 1 :
        servo(0)
    elif area_c >= 25 :
        servo(lr * 100)
    else:
        servo((area_l- area_r)*4)

    speed(100)
    c = color()
    
    if c <65 and lr == 0:
        lr = -1
        print(-1)
    elif c<77 and lr == 0:
        lr = 1
        print(1)
    if c < 77 and time.time() - last_check >= 5:
        r +=1
        last_check = time.time()
        if r == 12:
            last_stop = time.time()
            r = 13
        if r == 13 and time.time()-last_stop >= 8:
            speed(0)
            servo(0)
            break
    
    rawCapture.truncate(0)


    if cv2.waitKey(10) & 0xFF == ord('q'):
        webcam.release()
        cv2.destroyAllWindows()
        break
