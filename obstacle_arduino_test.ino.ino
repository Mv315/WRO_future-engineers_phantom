#include <Servo.h>

// Define pin assignments
const int servoPin = 9;          // PWM pin connected to the servo
const int ledPin = 13;  // Built-in LED pin

Servo steeringServo;

// Variables to store incoming serial data
char incomingChar;
String receivedString = "";

// Steering angle constraints
const int NEUTRAL_ANGLE = 120;    // Neutral position
const int MAX_LEFT_ANGLE = 70;     // 120 - 50
const int MAX_RIGHT_ANGLE = 170;   // 120 + 50

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize LED pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // Turn off LED initially
  
  // Initialize servo
  steeringServo.attach(servoPin);
  steeringServo.write(NEUTRAL_ANGLE); // Set to neutral position
}

void loop() {
  // Check if data is available on Serial
  if (Serial.available() > 0) {
    incomingChar = Serial.read();

    if (incomingChar == 'n') {
      // No block detected
      digitalWrite(ledPin, HIGH); 
      delay(500);
      digitalWrite(ledPin,LOW); // Turn on LED
      Serial.println("LED ON: No block detected.");
      
      // Reset servo to neutral
      steeringServo.write(NEUTRAL_ANGLE);
    }
    else if (incomingChar == 'r' || incomingChar == 'g') {
      // Block detected: Read the next 4 characters for angle (e.g., '+30\n' or '-15\n')
      receivedString = "";
      while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
          break;
        }
        receivedString += c;
      }
      
      // Parse the angle
      int angle = receivedString.toInt(); // This handles the sign automatically
      
      // Calculate the steering angle
      int steeringAngle = NEUTRAL_ANGLE + angle;
      steeringAngle = constrain(steeringAngle, MAX_LEFT_ANGLE, MAX_RIGHT_ANGLE);
      
      // Debug print
      Serial.print("Block: ");
      Serial.print(incomingChar);
      Serial.print(", Angle: ");
      Serial.print(angle);
      Serial.print(", Steering Angle: ");
      Serial.println(steeringAngle);
      
      // Set servo to the calculated angle
      steeringServo.write(steeringAngle);
      
      // Blink the LED to indicate action
      digitalWrite(ledPin, LOW); // Turn off LED
      delay(100);
      digitalWrite(ledPin, HIGH); // Turn on LED
      delay(100);
      digitalWrite(ledPin, LOW); // Turn off LED
    }
    else {
      // Unknown command, ignore or handle accordingly
      Serial.print("Unknown command: ");
      Serial.println(incomingChar);
    }
  }
}
void setup() {
  // Start the serial communication at 9600 baud rate
  Serial.begin(9600);
  while (!Serial) {
    // Wait for the serial port to connect
  }
}

void loop() {
  // Check if data is available to read from the Raspberry Pi
  if (Serial.available() > 0) {
    // Read the incoming message from Raspberry Pi
    String message = Serial.readStringUntil('\n');
    
    // Send the message back to the Raspberry Pi
    Serial.println(message);
  }
}
