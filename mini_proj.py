import RPi.GPIO as GPIO 
import time
import numpy as np
import imutils
import cv2
from picamera import PiCamera 
from picamera.array import PiRGBArray
from pivideostream import PiVideoStream

GPIO.setmode(GPIO.BOARD)
pin_number_1 = 11
pin_number_2 = 12

GPIO.setup(pin_number_1, GPIO.OUT)
GPIO.setup(pin_number_2, GPIO.OUT)


frequency_hertz = 50
pwm_1 = GPIO.PWM(pin_number_1, frequency_hertz)
pwm_2 = GPIO.PWM(pin_number_2, frequency_hertz)
ms_per_cycle = 1000 / frequency_hertz

def servo(angle):
    position = ((1.9)/180)*angle+.4
    duty_cycle_percentage = (position / ms_per_cycle)*100
    return duty_cycle_percentage

lower = {'red':(166, 84, 141), 'green':(66, 122, 129), 'blue':(97, 100, 117), 'yellow':(23, 59, 119), 'orange':(0, 50, 80)} 
upper = {'red':(186,255,255), 'green':(86,255,255), 'blue':(117,255,255), 'yellow':(54,255,255), 'orange':(20,255,255)}
 
colors = {'red':(0,0,255), 'green':(0,255,0), 'blue':(255,0,0), 'yellow':(0, 255, 217), 'orange':(0,140,255)}

x = servo(180)
pwm_1.start(x)
time.sleep(.5)
pwm_1.stop()

vs=PiVideoStream()

vs.start()
time.sleep(2.0)

while True:

    pwm_2.start(servo(30))
    time.sleep(3)
    pwm_2.stop()

    pwm_2.start(servo(135))
    time.sleep(2)
    pwm_2.stop()
    
    frame = vs.read()
    frame = imutils.resize(frame, width=800)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    for key, value in upper.items():
        kernel = np.ones((9,9),np.uint8)
        mask = cv2.inRange(hsv, lower[key], upper[key])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 0.5:
                cv2.circle(frame, (int(x), int(y)), int(radius), colors[key], 2)
                if key == 'red':
                    cv2.putText(frame,"red", (int(x-radius),int(y-radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[key],2)
                    pwm_1.start(servo(135))
                    time.sleep(.5)
                    pwm_1.stop()
                    GPIO.cleanup()

                elif key == 'yellow':
                    cv2.putText(frame,"veg", (int(x-radius),int(y-radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[key],2)
                    pwm_1.start(servo(75))
                    time.sleep(.5)
                    pwm_1.stop()
                    GPIO.cleanup()
                         
                elif key == 'green':
                    cv2.putText(frame,key, (int(x-radius),int(y-radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[key],2)
                    pwm_1.start(servo(45))
                    time.sleep(.5)
                    pwm_1.stop()
                    GPIO.cleanup()

    cv2.imshow("Frame", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break
camera.release()
cv2.destroyAllWindows()           
                          
                 
           
