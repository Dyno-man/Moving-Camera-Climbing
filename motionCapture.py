import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define GPIO pins for motor control
motor_x_pin1 = 17
motor_x_pin2 = 18
motor_y_pin1 = 22
motor_y_pin2 = 23

GPIO.setup(motor_x_pin1, GPIO.OUT)
GPIO.setup(motor_x_pin2, GPIO.OUT)
GPIO.setup(motor_y_pin1, GPIO.OUT)
GPIO.setup(motor_y_pin2, GPIO.OUT)

# Function to move the camera
def move_camera(center_x, center_y, frame_width, frame_height):
    x_threshold = frame_width // 4
    y_threshold = frame_height // 4

    # Horizontal movement
    if center_x < frame_width // 2 - x_threshold:
        GPIO.output(motor_x_pin1, GPIO.HIGH)
        GPIO.output(motor_x_pin2, GPIO.LOW)
    elif center_x > frame_width // 2 + x_threshold:
        GPIO.output(motor_x_pin1, GPIO.LOW)
        GPIO.output(motor_x_pin2, GPIO.HIGH)
    else:
        GPIO.output(motor_x_pin1, GPIO.LOW)
        GPIO.output(motor_x_pin2, GPIO.LOW)

    # Vertical movement
    if center_y < frame_height // 2 - y_threshold:
        GPIO.output(motor_y_pin1, GPIO.HIGH)
        GPIO.output(motor_y_pin2, GPIO.LOW)
    elif center_y > frame_height // 2 + y_threshold:
        GPIO.output(motor_y_pin1, GPIO.LOW)
        GPIO.output(motor_y_pin2, GPIO.HIGH)
    else:
        GPIO.output(motor_y_pin1, GPIO.LOW)
        GPIO.output(motor_y_pin2, GPIO.LOW)

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

# Access the camera
cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
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
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
                centers.append((center_x, center_y))

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            confidence = confidences[i]
            color = (0, 255, 0)  # Green for bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Draw the center of the rectangle
            center_x, center_y = centers[i]
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            # Print the center coordinates
            print(f"Center: ({center_x}, {center_y})")

            # Use center_x and center_y to control DC motors
            move_camera(center_x, center_y, width, height)

    cv2.imshow("Image", frame)

    key = cv2.waitKey(1)
    if key == 27:  # Esc key to stop
        break

cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()
