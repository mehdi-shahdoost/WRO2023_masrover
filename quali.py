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
    d = (x/100)*3
    s.ChangeDutyCycle(6.5 + d)
    
        
def speed(velocity):
    if velocity > 100: velocity = 100
    elif velocity < 0: velocity = 0
    v.ChangeDutyCycle(velocity)
    
while GPIO.input(button) == 0:
    speed(0)
    servo(0)

lr = 0
r = 0
last_check = time.time()
start_time = time.time()
last_stop = time.time()

for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    
    imageFrame = frame.array
    imageFrame = cv2.blur(imageFrame, (5,5))
    imageFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2GRAY)
    
   
    full_thresh = cv2.threshold(imageFrame,60,255, cv2.THRESH_BINARY_INV)[1]
    full_thresh = full_thresh[100:,:]
    left_thresh = full_thresh[:,:270]
    right_thresh = full_thresh[:,370:640]
    center_thresh = full_thresh[:,160:480]
    
    #cv2.imshow("Multiple Color Detection in Real-TIme", full_thresh)
    
    cnts_l = cv2.findContours(left_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_l = cnts_l[0] if len(cnts_l) == 2 else cnts_l[1]

    cnts_r = cv2.findContours(right_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_r = cnts_r[0] if len(cnts_r) == 2 else cnts_r[1]

    cnts_c = cv2.findContours(center_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_c = cnts_c[0] if len(cnts_c) == 2 else cnts_c[1]

    area_r = 0
    area_l = 0
    area_c = 0
    for c in cnts_l:
       area_l = round(cv2.contourArea(c),1)/1000
    for c in cnts_r:
       area_r = round(cv2.contourArea(c),1)/1000
    for c in cnts_c:
       area_c = round(cv2.contourArea(c),1)/1000

    if time.time() - start_time <= 1 :
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