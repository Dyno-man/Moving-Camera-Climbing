import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

GPIO.setup(3, GPIO.OUT)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)


pwm1=GPIO.PWM(7, 100)
pwm2=GPIO.PWM(15, 100)

pwm1.start(0)
pwm2.start(0)

GPIO.output(3, True)
GPIO.output(5, False)

pwm1.ChangeDutyCycle(50)

GPIO.output(7, True)

sleep(3)

GPIO.output(7, False)

GPIO.output(3, False)
GPIO.output(5, True)

GPIO.output(7, True)

sleep(3)

GPIO.output(7, False)

pwm1.stop()

GPIO.output(11, True)
GPIO.output(13, False)

pwm2.ChangeDutyCycle(50)

GPIO.output(15, True)

sleep(3)

GPIO.output(15, False)

GPIO.output(11, False)
GPIO.output(13, True)

GPIO.output(15, True)

sleep(3)

GPIO.output(15, False)

pwm2.stop()

GPIO.cleanup()
