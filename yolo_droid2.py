import cv2
from ultralytics import YOLO
import serial

# Load the custom model
model = YOLO('best(2).pt')  # Ganti dengan path model yang sesuai

# Open a connection to the default camera (0)
cap = cv2.VideoCapture(0)

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

    # Rotate the frame to change orientation from landscape to portrait
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)  # Rotate 90 degrees clockwise

    # Get frame dimensions
    frame_height, frame_width, _ = frame.shape
    frame_center_x = frame_width // 2

    # Use the model to predict objects in the frame
    results = model(frame)

    # Get the first result
    result = results[0]

    # Filter detections with confidence scores above 0.8
    ball_detected = False
    command = 'S'  # Default command
    for box in result.boxes:
        if box.conf > 0.8:  # Confidence threshold
            ball_detected = True
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
            box_center_x = (x1 + x2) // 2
            box_area = (x2 - x1) * (y2 - y1)  # Calculate bounding box area

            # Calculate distance to bottom frame
            distance_to_bottom = frame_height - y2

            # Logic for commands based on bounding box position
            if y2 >= frame_height - 10 and abs(box_center_x - frame_center_x) <= 50:  # Centered and near bottom
                command = 'S'  # Stop
            elif distance_to_bottom <= 50:  # Very close to the bottom
                command = 'f'  # Slow forward
            elif abs(box_center_x - frame_center_x) > 50:  # Ball is not centered
                if box_center_x < frame_center_x:
                    command = 'L'  # Turn left
                else:
                    command = 'R'  # Turn right
            else:
                command = 'F'  # Move forward at normal speed
            break

    # If no ball is detected, search by turning
    if not ball_detected:
        command = 'S'  # Rotate left if no ball is detected

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
