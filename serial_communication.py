from ultralytics import YOLO
import cv2
import serial
import time

# Initialize YOLO model (use 'yolov8n.pt' for small, 'yolov8m.pt' for medium)
model = YOLO("yolov8n.pt")

# Initialize Arduino communication (replace COM6 with your port)
arduino = serial.Serial('COM6', 9600)
time.sleep(2)  # Wait for Arduino connection

# Open webcam
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO object detection
    results = model(frame)

    # Extract detected objects
    detected_classes = [model.names[int(box.cls)] for box in results[0].boxes]

    # Define electronic items category
    electronic_items = ["cell phone", "laptop", "tv", "mouse", "remote", "keyboard"]

    # Check if any detected object is electronic
    if any(item in detected_classes for item in electronic_items):
        print("Electronic device detected!")
        arduino.write(b'1')  # Send '1' to turn LED ON
    else:
        arduino.write(b'0')  # Send '0' to turn LED OFF

    # Display results
    frame = results[0].plot()
    cv2.imshow("YOLOv8 Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
arduino.close()
