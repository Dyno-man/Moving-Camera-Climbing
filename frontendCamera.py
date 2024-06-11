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
    GPIO.output(motor_x_enable, GPIO.LOW)
    pwm_x.ChangeDutyCycle(0)
    GPIO.output(motor_y_pin1, GPIO.LOW)
    GPIO.output(motor_y_pin2, GPIO.LOW)
    GPIO.output(motor_y_enable, GPIO.LOW)
    pwm_y.ChangeDutyCycle(0)

# Function to move the camera based on received commands
def move_camera(diff_x, diff_y):
    threshold = 150  # Threshold to determine when to stop the motors
    duty_cycle = 20  # Example duty cycle
    move_duration = 0.1  # Duration to move motors in seconds

    # Move in X direction
    if diff_x > threshold:
        GPIO.output(motor_x_pin1, GPIO.LOW)
        GPIO.output(motor_x_pin2, GPIO.HIGH)
        pwm_x.ChangeDutyCycle(duty_cycle)
        GPIO.output(motor_x_enable, GPIO.HIGH)
        sleep(move_duration)
        GPIO.output(motor_x_enable, GPIO.LOW)
        pwm_x.ChangeDutyCycle(0)
    elif diff_x < -threshold:
        GPIO.output(motor_x_pin1, GPIO.HIGH)
        GPIO.output(motor_x_pin2, GPIO.LOW)
        pwm_x.ChangeDutyCycle(duty_cycle)
        GPIO.output(motor_x_enable, GPIO.HIGH)
        sleep(move_duration)
        GPIO.output(motor_x_enable, GPIO.LOW)
        pwm_x.ChangeDutyCycle(0)

    # Move in Y direction
    if diff_y > threshold:
        GPIO.output(motor_y_pin1, GPIO.LOW)
        GPIO.output(motor_y_pin2, GPIO.HIGH)
        pwm_y.ChangeDutyCycle(duty_cycle)
        GPIO.output(motor_y_enable, GPIO.HIGH)
        sleep(move_duration)
        GPIO.output(motor_y_enable, GPIO.LOW)
        pwm_y.ChangeDutyCycle(0)
    elif diff_y < -threshold:
        GPIO.output(motor_y_pin1, GPIO.HIGH)
        GPIO.output(motor_y_pin2, GPIO.LOW)
        pwm_y.ChangeDutyCycle(duty_cycle)
        GPIO.output(motor_y_enable, GPIO.HIGH)
        sleep(move_duration)
        GPIO.output(motor_y_enable, GPIO.LOW)
        pwm_y.ChangeDutyCycle(0)

# Set up socket for communication
HOST = '10.120.60.30'  # IP address of the powerful computer
PORT = 5000  # Port to connect to the computer

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Resize the frame to reduce quality
        frame = cv2.resize(frame, (320, 240))

        # Convert the frame to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Compress the frame using JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # 90 is the quality of the JPEG compression
        result, frame = cv2.imencode('.jpg', gray_frame, encode_param)
        data = pickle.dumps(frame, 0)

        # Serialize frame
        message_size = struct.pack("Q", len(data))

        # Send frame
        client_socket.sendall(message_size + data)
        print(f"Sent frame of size: {len(data)}")

        # Receive motor commands
        command_data = recv_all(client_socket, struct.calcsize("Q"))
        if command_data is None:
            print("Failed to receive command size")
            break

        command_size = struct.unpack("Q", command_data)[0]
        command_data = recv_all(client_socket, command_size)
        if command_data is None:
            print("Failed to receive command data")
            break

        diff_x, diff_y = pickle.loads(command_data)
        print(f"Received commands: diff_x={diff_x}, diff_y={diff_y}")

        # Move the camera based on received commands
        move_camera(diff_x, diff_y)

        sleep(0.1)
except Exception as e:
    print(f"Error: {e}")
finally:
    cap.release()
    stop_motors()
    GPIO.cleanup()
    client_socket.close()

# Helper function to receive all data
def recv_all(sock, size):
    data = b''
    while len(data) < size:
        packet = sock.recv(size - len(data))
        if not packet:
            return None
        data += packet
    return data
