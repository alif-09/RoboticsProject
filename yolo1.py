import cv2
from ultralytics import YOLO
import serial

# Load the custom model
model = YOLO('best(2).pt')  # Ganti dengan path model yang sesuai

# Open a connection to the default camera (0)
cap = cv2.VideoCapture(1)

# Open a serial connection to Arduino
arduino = serial.Serial('COM20', 9600)  # Sesuaikan baud rate sesuai dengan Arduino Anda

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Loop to continuously get frames from the camera
while True:
    # Capture each frame from the camera
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Get frame dimensions
    frame_height, frame_width, _ = frame.shape
    frame_center_x = frame_width // 2

    # Use the model to predict objects in the frame
    results = model(frame)

    # Get the first result
    result = results[0]

    # Filter detections with confidence scores above 0.8
    ball_detected = False
    command = 'L'  # Default command (L for no detection)
    for box in result.boxes:
        if box.conf > 0.8:  # Confidence threshold
            ball_detected = True
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
            box_center_x = (x1 + x2) // 2
            box_area = (x2 - x1) * (y2 - y1)  # Calculate bounding box area

            # Determine the action based on bounding box area and position
            if box_area >= (frame_width * frame_height) // 8 and abs(box_center_x - frame_center_x) <= 50:
                command = 'S'  # Stop if the ball is large enough and near the vertical center
            elif abs(box_center_x - frame_center_x) > 20:  # Ball is not centered
                if box_center_x < frame_center_x:  # Ball is on the left
                    command = 'L'
                else:  # Ball is on the right
                    command = 'R'
            else:  # Ball is centered, move forward
                command = 'F'
            break

    # If no ball is detected, command is 'L'
    if not ball_detected:
        command = 'L'

    # Send the command to Arduino
    arduino.write(command.encode())

    # Draw filtered boxes on the frame
    for box in result.boxes:
        if box.conf > 0.8:  # Confidence threshold
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
            conf = box.conf.item()  # Convert confidence score to a scalar
            label = f'{box.cls} {conf:.2f}'

            # Draw the bounding box and label on the frame
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the frame with detections
    cv2.imshow('YOLOv8 Camera Feed', frame)

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera, close serial, and close any OpenCV windows
cap.release()
arduino.close()
cv2.destroyAllWindows()
