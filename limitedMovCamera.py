import cv2
import numpy as np
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

# Initial motor positions (in degrees)
current_angle_x = 90  # Assume starting at middle position (90 degrees)
current_angle_y = 50  # Assume starting at middle position (50 degrees)

# Function to stop all motors
def stop_motors():
    GPIO.output(motor_x_pin1, GPIO.LOW)
    GPIO.output(motor_x_pin2, GPIO.LOW)
    pwm_x.ChangeDutyCycle(0)
    GPIO.output(motor_y_pin1, GPIO.LOW)
    GPIO.output(motor_y_pin2, GPIO.LOW)
    pwm_y.ChangeDutyCycle(0)

# Function to move the camera based on the difference between the center and the person's location
def move_camera(diff_x, diff_y):
    global current_angle_x, current_angle_y
    threshold = 15  # Threshold to determine when to stop the motors
    duty_cycle = 50  # Example duty cycle
    angle_step = 2  # Small degree change per step

    # Calculate new positions
    new_angle_x = current_angle_x
    new_angle_y = current_angle_y

    # Move in X direction
    if diff_x > threshold and current_angle_x < 180:
        new_angle_x += angle_step
    elif diff_x < -threshold and current_angle_x > 0:
        new_angle_x -= angle_step

    # Move in Y direction
    if diff_y > threshold and current_angle_y < 100:
        new_angle_y += angle_step
    elif diff_y < -threshold and current_angle_y > 0:
        new_angle_y -= angle_step

    # Update motor positions if within limits
    if 0 <= new_angle_x <= 180:
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
        current_angle_x = new_angle_x

    if 0 <= new_angle_y <= 100:
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
        current_angle_y = new_angle_y

# Load YOLO
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
layer_names = net.getLayerNames()
try:
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
except IndexError:
    output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
classes = []
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Get the index of the 'person' class
person_index = classes.index('person')

# Open the camera using VideoCapture
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Cannot open camera")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame (stream end?). Exiting ...")
        break

    height, width, channels = frame.shape

    # Detecting objects
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # Showing information on the screen
    class_ids = []
    confidences = []
    boxes = []
    centers = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5 and class_id == person_index:
                # Object detected is a person
                person_center_x = int(detection[0] * width)
                person_center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(person_center_x - w / 2)
                y = int(person_center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
                centers.append((person_center_x, person_center_y))

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    for i in range(len(boxes)):
        if i in indexes:
            person_center_x, person_center_y = centers[i]

            # Calculate the difference between the frame center and the person's center
            frame_center_x = width // 2
            frame_center_y = height // 2
            diff_x = frame_center_x - person_center_x
            diff_y = frame_center_y - person_center_y

            # Print the results to the console
            print(f"Frame center: ({frame_center_x}, {frame_center_y})")
            print(f"Person center: ({person_center_x}, {person_center_y})")
            print(f"Difference: (x: {diff_x}, y: {diff_y})")

            # Move the camera based on the difference
            move_camera(diff_x, diff_y)

    # Sleep for a short duration to reduce CPU usage
    sleep(0.1)

# When everything is done, release the capture and stop the motors
cap.release()
stop_motors()
GPIO.cleanup()
