YOLO Object Detection with LED Control using Arduino  
Project Description
This project uses YOLOv8 for object detection and an Arduino Uno (or Mega) to control an LED. If an electronic device is detected in the camera feed, the LED glows. The system integrates computer vision (YOLOv8) and hardware control (Arduino) to demonstrate real-time object detection and physical interaction.

Project Features
✔ Object Detection: Uses YOLOv8 to detect various objects.
✔ Electronic Device Filtering: Detects specific electronic items (like phones, laptops, TVs, keyboards, etc.).
✔ Hardware Integration: Sends signals to Arduino to control an LED.
✔ Real-time Processing: Uses OpenCV to display detected objects live

Circuit Diagram
![Yolov_diagram](https://github.com/user-attachments/assets/b5fc5863-2f49-4ee5-9727-dbad6629f9c7)
Connections:
LED: Connected to pin D13 on Arduino.
GND: LED negative leg to GND of Arduino.
USB: Connect Arduino to PC via USB.
Requirements
Hardware:
🔹 Arduino Uno / Mega
🔹 LED
🔹 USB Cable
🔹 Computer with a camera

Software:
🔸 Python 3.8+
🔸 OpenCV
🔸 Ultralytics YOLOv8
🔸 Arduino IDE
🔸 PySerial

Installation and Setup
1️⃣ Install Python & Dependencies
bash
Copy
Edit
pip install ultralytics opencv-python pyserial
2️⃣ Upload Arduino Code
Open Arduino IDE.
Upload the following code to Arduino:
cpp
Copy
Edit
const int ledPin = 13;
void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char received = Serial.read();
    if (received == '1') {
      digitalWrite(ledPin, HIGH);
    } else if (received == '0') {
      digitalWrite(ledPin, LOW);
    }
  }
}
3️⃣ Run YOLO Object Detection
Save this Python script as yolo_arduino.py and run it:

python
Copy
Edit
from ultralytics import YOLO
import cv2
import serial
import time

# Load YOLO model
model = YOLO("yolov8n.pt")

# Initialize Arduino
arduino = serial.Serial('COM6', 9600)
time.sleep(2)

# Open camera
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    detected_classes = [model.names[int(box.cls)] for box in results[0].boxes]
    
    electronic_items = ["cell phone", "laptop", "tv", "mouse", "remote", "keyboard"]
    
    if any(item in detected_classes for item in electronic_items):
        print("Electronic device detected!")
        arduino.write(b'1')
    else:
        arduino.write(b'0')

    frame = results[0].plot()
    cv2.imshow("YOLOv8 Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()
4️⃣ Run the Project
bash
Copy
Edit
python yolo_arduino.py
Usage Instructions
Start the Python script to detect objects.
Point the camera at an electronic device (like a mobile phone or laptop).
If an electronic device is detected, the LED will turn ON.
If no electronic device is detected, the LED will turn OFF.
Press 'q' to exit the program.
Troubleshooting
✔ Arduino not responding? Ensure the correct COM port is used (COM6 in the Python script).
✔ YOLO model not loading? Ensure yolov8n.pt is downloaded. Try pip install ultralytics again.
✔ LED not glowing? Check the circuit connections and resistor value.

Contributors
Sharveswar
License
This project is open-source under the MIT License.
