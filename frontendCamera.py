import cv2
import socket
import struct
import pickle
import RPi.GPIO as GPIO
from time import sleep

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# Define GPIO pins for motor control
motor_x_pin1 = 3
motor_x_pin2 = 5
motor_x_enable = 7
motor_y_pin1 = 11
motor_y_pin2 = 13
motor_y_enable = 15

# Set up GPIO pins
GPIO.setup(motor_x_pin1, GPIO.OUT)
GPIO.setup(motor_x_pin2, GPIO.OUT)
GPIO.setup(motor_x_enable, GPIO.OUT)
GPIO.setup(motor_y_pin1, GPIO.OUT)
GPIO.setup(motor_y_pin2, GPIO.OUT)
GPIO.setup(motor_y_enable, GPIO.OUT)

# Set up PWM
pwm_x = GPIO.PWM(motor_x_enable, 100)
pwm_y = GPIO.PWM(motor_y_enable, 100)

pwm_x.start(0)
pwm_y.start(0)

# Function to stop all motors
def stop_motors():
    GPIO.output(motor_x_pin1, GPIO.LOW)
    GPIO.output(motor_x_pin2, GPIO.LOW)
    pwm_x.ChangeDutyCycle(0)
    GPIO.output(motor_y_pin1, GPIO.LOW)
    GPIO.output(motor_y_pin2, GPIO.LOW)
    pwm_y.ChangeDutyCycle(0)

# Function to move the camera based on received commands
def move_camera(diff_x, diff_y):
    threshold = 15  # Threshold to determine when to stop the motors
    duty_cycle = 50  # Example duty cycle
    angle_step = 2  # Small degree change per step

    # Move in X direction
    if diff_x > threshold:
        GPIO.output(motor_x_pin1, GPIO.LOW)
        GPIO.output(motor_x_pin2, GPIO.HIGH)
        pwm_x.ChangeDutyCycle(duty_cycle)
        GPIO.output(motor_x_enable, GPIO.HIGH)
    elif diff_x < -threshold:
        GPIO.output(motor_x_pin1, GPIO.HIGH)
        GPIO.output(motor_x_pin2, GPIO.LOW)
        pwm_x.ChangeDutyCycle(duty_cycle)
        GPIO.output(motor_x_enable, GPIO.HIGH)
    else:
        GPIO.output(motor_x_enable, GPIO.LOW)
        pwm_x.ChangeDutyCycle(0)

    # Move in Y direction
    if diff_y > threshold:
        GPIO.output(motor_y_pin1, GPIO.LOW)
        GPIO.output(motor_y_pin2, GPIO.HIGH)
        pwm_y.ChangeDutyCycle(duty_cycle)
        GPIO.output(motor_y_enable, GPIO.HIGH)
    elif diff_y < -threshold:
        GPIO.output(motor_y_pin1, GPIO.HIGH)
        GPIO.output(motor_y_pin2, GPIO.LOW)
        pwm_y.ChangeDutyCycle(duty_cycle)
        GPIO.output(motor_y_enable, GPIO.HIGH)
    else:
        GPIO.output(motor_y_enable, GPIO.LOW)
        pwm_y.ChangeDutyCycle(0)

# Set up socket for communication
HOST = 'your_computer_ip'  # IP address of the powerful computer
PORT = 5000  # Port to connect to the computer

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Serialize frame
    data = pickle.dumps(frame)
    message_size = struct.pack("L", len(data))

    # Send frame
    client_socket.sendall(message_size + data)

    # Receive motor commands
    command_size = struct.unpack("L", client_socket.recv(struct.calcsize("L")))[0]
    command_data = client_socket.recv(command_size)
    diff_x, diff_y = pickle.loads(command_data)

    # Move the camera based on received commands
    move_camera(diff_x, diff_y)

    sleep(0.1)

cap.release()
stop_motors()
GPIO.cleanup()
client_socket.close()
