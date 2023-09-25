import RPi.GPIO as GPIO
from time import sleep
steering_gpio = 24
steering_pwm = 13
move_gpio = 23
move_pwm = 12
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(move_pwm,GPIO.OUT)
v = GPIO.PWM(move_pwm,1000)
GPIO.setup(steering_pwm,GPIO.OUT)
steer_signal = GPIO.PWM(steering_pwm,1000)
GPIO.setup(steering_gpio,GPIO.OUT)
v.start(0)
steer_signal.start(0)

def steer(x):
    if x > 100: x = 100
    elif x < -100: x = -100
    
    
    if x < 0:
        GPIO.output(steering_gpio, GPIO.LOW)
        steer_signal.ChangeDutyCycle(abs(x))
    elif x > 0:
        GPIO.output(steering_gpio, GPIO.HIGH)
        steer_signal.ChangeDutyCycle(100-x)
    else:
        GPIO.output(steering_gpio, GPIO.LOW)
        steer_signal.ChangeDutyCycle(0)
        
        
def speed(velocity):
    v.ChangeDutyCycle(velocity)
    

    
    
    
            
