import RPi.GPIO as GPIO

PWMA=18
AIN1=8
AIN2=25
PWMB=19
BIN1=9
BIN2=11
HIGH=1
LOW=0

GPIO.setmode(GPIO.BCM)
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(AIN2,GPIO.OUT)
GPIO.setup(BIN1,GPIO.OUT)
GPIO.setup(BIN2,GPIO.OUT)
GPIO.setup(PWMA,GPIO.OUT)
GPIO.setup(PWMB,GPIO.OUT)
M_pwmA=GPIO.PWM(PWMA,50)
M_pwmB=GPIO.PWM(PWMB,50)
M_pwmA.start(100)
M_pwmB.start(100)

while True:
	M_pwmA.ChangeDutyCycle(85)
	M_pwmB.ChangeDutyCycle(100)
	GPIO.output(AIN1,HIGH)
	GPIO.output(AIN2,LOW)
	GPIO.output(BIN1,HIGH)
	GPIO.output(BIN2,LOW)
