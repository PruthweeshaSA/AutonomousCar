from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
from picamera import PiCamera
import cv2
import numpy as np
import math
import time

a=0
b=0
IR=8
TRIG=16
ECHO=18
TRIG2=11
ECHO2=13
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.cleanup()
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
GPIO.setup(TRIG2,GPIO.OUT)
GPIO.setup(ECHO2,GPIO.IN)
GPIO.setup(IR,GPIO.IN)
motor1S=35
motor1B=38
motor1C=37
motor1D=36
ptime=0
stime=0
GPIO.setup(motor1S,GPIO.OUT)
GPIO.setup(motor1B,GPIO.OUT)
GPIO.setup(motor1C,GPIO.OUT)
GPIO.setup(motor1D,GPIO.OUT)
turn_exec=0

p = GPIO.PWM(35,100)
q = GPIO.PWM(36,100)
r = GPIO.PWM(37,100)
s = GPIO.PWM(38,100)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(35,GPIO.OUT)
GPIO.setup(36,GPIO.OUT)
GPIO.setup(37,GPIO.OUT)
GPIO.setup(38,GPIO.OUT)
motor1S=35
motor1B=38
motor1C=37
motor1D=36

def left():
	print " moving left  "

        #GPIO.output(motor1S,GPIO.HIGH)
        #GPIO.output(motor1B,GPIO.LOW)
        #GPIO.output(motor1C,GPIO.LOW)
        #GPIO.output(motor1D,GPIO.LOW)
	q.stop()
	r.stop()
	s.stop()
	p.start(100)
	time.sleep(0.1)
        #p.stop()
        #GPIO.output(motor1S,GPIO.LOW)
        #GPIO.output(motor1B,GPIO.LOW)
        #GPIO.output(motor1C,GPIO.LOW)
        #GPIO.output(motor1D,GPIO.LOW)

def right():
	print "TURNING RIGHT"

        #GPIO.output(motor1S,GPIO.LOW)
        #GPIO.output(motor1B,GPIO.HIGH)
        #GPIO.output(motor1C,GPIO.LOW)
        #GPIO.output(motor1D,GPIO.LOW)
	p.stop()
	q.stop()
	r.stop()
	s.start(100)
	time.sleep(0.1)
        #s.stop()
        #GPIO.output(motor1S,GPIO.LOW)
        #GPIO.output(motor1B,GPIO.LOW)
        #GPIO.output(motor1C,GPIO.LOW)
        #GPIO.output(motor1D,GPIO.LOW)

def checkdist():
	GPIO.output(TRIG,GPIO.LOW)
	time.sleep(0.01)
	GPIO.output(TRIG,GPIO.HIGH)
	time.sleep(0.00001)
	GPIO.output(TRIG,GPIO.LOW)
	echosignal=0
	while(echosignal==0):
		#print "echo=0"
		echosignal=GPIO.input(ECHO)
		stime=time.time()
	while(echosignal==1):
		ptime=time.time() - stime
		echosignal=GPIO.input(ECHO)
		if ptime>0.1:
			echosignal=0
        # print "rtime : %f" %ptime
	distance=165*ptime
	print "distance:%f" %distance
	return distance

def checkdist2():
        GPIO.output(TRIG2,GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(TRIG2,GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG2,GPIO.LOW)
        echosignal=0
        while(echosignal==0):
                #print "echo=0"
                echosignal=GPIO.input(ECHO2)
                stime=time.time()
        while(echosignal==1):
                ptime=time.time() - stime
                echosignal=GPIO.input(ECHO2)
                if ptime>0.1:
                        echosignal=0
        # print "rtime : %f" %ptime
        distance2=165*ptime
        print "distance2:%f" %distance2
        return distance2

def straight(intensity):

        #GPIO.output(motor1S,GPIO.HIGH)
        #GPIO.output(motor1B,GPIO.HIGH)
        #GPIO.output(motor1C,GPIO.LOW)
        #GPIO.output(motor1D,GPIO.LOW)
	q.stop()
	r.stop()
	p.start(intensity)
	s.start(intensity)
        #time.sleep(0.1)
        #p.stop()
        #s.stop()

def pause():
	distance=checkdist()
	p.stop()
	q.stop()
	r.stop()
	s.stop()
	waitstart=time.time()
	while(distance<0.3):
		distance=checkdist()
		time.sleep(0.1)
		if((time.time()-waitstart)>2):
			leftstart=time.time()
			while(time.time()-leftstart<2):
				left()
			p.stop()
			q.stop()
			r.stop()
			s.stop()
			uturn=1
			break



theta=0
minLineLength = 5
maxLineGap = 10
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 15
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(blurred, 85, 85)
	lines = cv2.HoughLinesP(edged,1,np.pi/180,10,minLineLength,maxLineGap)
	if(lines !=None):
		for x in range(0, len(lines)):
			for x1,y1,x2,y2 in lines[x]:
				cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)
				theta=theta+math.atan2((y2-y1),(x2-x1))
   #print(theta)GPIO pins were connected to arduino for servo steering control
	threshold=6
	distance=checkdist()
	distance2=checkdist2()
	intensity=distance*50
	if(intensity>100):
		intensity=100
	if(intensity<3):
		intensity=0

	if(theta>threshold):
   	# GPIO.output(7,True)
      	# GPIO.output(8,False)
		print("left")
		#GPIO.output(motor1S,GPIO.HIGH)
		#GPIO.output(motor1B,GPIO.LOW)  
		#GPIO.output(motor1C,GPIO.LOW)
		#GPIO.output(motor1D,GPIO.LOW)

		#time.sleep(1)

		#GPIO.output(motor1S,GPIO.LOW)
		#GPIO.output(motor1B,GPIO.LOW)
		#GPIO.output(motor1C,GPIO.LOW)
		#GPIO.output(motor1D,GPIO.LOW)

   	if(theta<-threshold):
      # GPIO.output(8,True)
      # GPIO.output(7,False)
		print("right")
		#GPIO.output(motor1S,GPIO.LOW)
		#GPIO.output(motor1B,GPIO.HIGH)  
		#GPIO.output(motor1C,GPIO.LOW)
		#GPIO.output(motor1D,GPIO.LOW)

		#time.sleep(1)

		#GPIO.output(motor1S,GPIO.LOW)
		#GPIO.output(motor1B,GPIO.LOW)
		#GPIO.output(motor1C,GPIO.LOW)
		#GPIO.output(motor1D,GPIO.LOW)

	if(abs(theta)<threshold):

      #GPIO.output(8,False)
      #GPIO.output(7,False)
		print "straight"
		#GPIO.output(motor1S,GPIO.HIGH)
		#GPIO.output(motor1B,GPIO.HIGH)  
		#GPIO.output(motor1C,GPIO.LOW)
		#GPIO.output(motor1D,GPIO.LOW)

		#time.sleep(1)

		#GPIO.output(motor1S,GPIO.LOW)
		#GPIO.output(motor1B,GPIO.LOW)  
		#GPIO.output(motor1C,GPIO.LOW)
		#GPIO.output(motor1D,GPIO.LOW)

		time.sleep(0.1)

		if distance>0.3:
			if distance<0.6:
				print"no object Detected:moving forward slowly "
			else:
                                print"moving forward"
			turn_exec=0
                        #GPIO.output(motor1S,GPIO.HIGH)
                        #GPIO.output(motor1B,GPIO.HIGH)
                        #GPIO.output(motor1C,GPIO.LOW)
                        #GPIO.output(motor1D,GPIO.LOW)
			straight(intensity)
                        #time.sleep(0.1)
                        #GPIO.cleanup
		else:
			if turn_exec==0:
				b=0
				print" Object. NO movement"
				p.stop()
				q.stop()
				r.stop()
				s.stop()
                                #GPIO.output(motor1S,GPIO.LOW)
                                #GPIO.output(motor1B,GPIO.LOW)
                                #GPIO.output(motor1C,GPIO.LOW)
                                #GPIO.output(motor1D,GPIO.LOW)
				time.sleep(0.2)
				distance=checkdist()
				distance2-checkdist2()
				turnstart=time.time()
				while (distance<0.3):
					right()
					distance=checkdist()
					distance2=checkdist2()
					turnend=time.time()
					#print "%f" %(turnend-turnstart)
					if (turnend-turnstart)>2:
						break
					a=1
					b=0
				p.stop()
				q.stop()
				r.stop()
				s.stop()
				time.sleep(2)
				right_check=1
				turnstart=time.time()
				while (distance<0.3):
					#if(left_check==0):
					left()
					distance=checkdist()
					distance2=checkdist2()
					turnend=time.time()
					if (turnend-turnstart)>4:
						break
				b=1
				a=0
				p.stop()
				q.stop()
				r.stop()
				s.stop()
				time.sleep(2)
				turnstart=time.time()
				while (distance<0.3):
					left()
					distance=checkdist()
					turnend=time.time()
					if (turnend-turnstart)>2:
						break
				p.stop()
				q.stop()
				r.stop()
				s.stop()
				time.sleep(2)





	theta=0
	cv2.imshow("Frame",image)
	key = cv2.waitKey(1) & 0xFF
	rawCapture.truncate(0)
	if key == ord("q"):
		break

GPIO.cleanup()