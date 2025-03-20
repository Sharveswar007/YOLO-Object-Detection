const int ledPin = 13;
void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);  // Start serial communication
}

void loop() {
  if (Serial.available() > 0) {
    char received = Serial.read();  // Read data from Python
    if (received == '1') {
      digitalWrite(ledPin, HIGH);   // Turn LED ON
    } else if (received == '0') {
      digitalWrite(ledPin, LOW);    // Turn LED OFF
    }
  }
}
