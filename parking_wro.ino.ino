// to check line 242 for which axis of gyro we will read
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Initialize MPU6050
Adafruit_MPU6050 mpu;

// Define Servo for Steering
Servo steeringServo;
const int STEERING_PIN = 8;
const int SERVO_LEFT = 0;    // Maximum left angle
const int SERVO_RIGHT = 180; // Maximum right angle
int currentSteeringAngle = 90; // Start with straight

// Define Motor Pins
const int IN1 = 2; // Left Motor Forward
const int IN2 = 3; // Left Motor Backward
const int ENA = 5; // Left Motor Speed (PWM)

const int IN3 = 4;  // Right Motor Forward
const int IN4 = 7;  // Right Motor Backward
const int ENB = 6;  // Right Motor Speed (PWM)

// Define Ultrasonic Sensors
// Front Ultrasonic
const int FRONT_TRIG_PIN = 9;
const int FRONT_ECHO_PIN = 10;

// Back Ultrasonic
const int BACK_TRIG_PIN = 11;
const int BACK_ECHO_PIN = 12;

// Define TCS3200 Color Sensor Pins
const int S0 = A0;
const int S1 = A1;
const int S2 = A2;
const int S3 = A3;
const int OUT = 13;

// Define thresholds and constants
const float YAW_THRESHOLD = 45.0; // Degrees
const float MIN_DISTANCE = 10.0;   // Centimeters (Adjust based on requirements)
const unsigned long TIMEOUT = 5000; // in milliseconds

// Variables for Yaw Control
float initialYaw = 0.0;
float currentYaw = 0.0;
float integratedYaw = 0.0; // To accumulate yaw over time

// Variables for Parking Logic
bool parkingComplete = false;

// Timing Variables
unsigned long startTime;

// Function Prototypes
void initializeSensors();
float readYaw();
bool detectColorWall();
float readUltrasonic(int trigPin, int echoPin);
void setMotorSpeed(int leftSpeed, int rightSpeed);
void stopMotors();
void turnSteering(int angle);
void moveBackward(int speed);
void moveForward(int speed);
bool verifyPosition();
void adjustSteeringDuringStraightening();
void resetYawIntegration();

void setup() {
  Serial.begin(115200);

  // Initialize Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize Ultrasonic Pins
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(BACK_TRIG_PIN, OUTPUT);
  pinMode(BACK_ECHO_PIN, INPUT);

  // Initialize TCS3200 Pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  // Initialize Steering Servo
  steeringServo.attach(STEERING_PIN);
  steeringServo.write(currentSteeringAngle); // Start with straight

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("MPU6050 initialized.");

  // Initialize TCS3200
  // Set frequency scaling to 20% as an example (can be adjusted)
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);

  // Allow sensors to stabilize
  delay(1000);

  Serial.println("Initialization Complete.");
}

void loop() {
  if (!parkingComplete) {
    // Step 1: Detect Front Wall using Color Sensor
    if (detectColorWall()) {
      Serial.println("Front Wall Detected. Initiating Parking Maneuver.");
      
      // Capture Initial Yaw
      resetYawIntegration(); // Reset integrated yaw before starting
      initialYaw = integratedYaw;
      Serial.print("Initial Yaw: ");
      Serial.println(initialYaw);

      // Step 2: Turn Steering to Max Right and Move Backwards
      turnSteering(SERVO_RIGHT);
      startTime = millis();
      while (true) {
        moveBackward(150); // Adjust speed as needed
        float deltaYaw = readYaw();
        Serial.print("Delta Yaw: ");
        Serial.println(deltaYaw);

        // Check stopping conditions
        float yawChange = abs(integratedYaw - initialYaw);
        float backDistance = readUltrasonic(BACK_TRIG_PIN, BACK_ECHO_PIN);
        bool backWallDetected = detectColorWall();

        if (yawChange >= YAW_THRESHOLD || (backDistance != -1.0 && backDistance <= MIN_DISTANCE) || backWallDetected) {
          Serial.println("Stopping Backward Movement after Turning Right.");
          stopMotors();
          break;
        }

        // Timeout to prevent infinite loop
        if (millis() - startTime > TIMEOUT) {
          Serial.println("Timeout Reached during Turning Right.");
          stopMotors();
          break;
        }
      }

      // Step 3: Turn Steering to Max Left and Move Backwards to Straighten
      float straighteningInitialYaw = integratedYaw;
      turnSteering(SERVO_LEFT);
      startTime = millis();
      while (true) {
        moveBackward(150); // Adjust speed as needed
        float deltaYaw = readYaw();
        Serial.print("Delta Yaw during Straightening: ");
        Serial.println(deltaYaw);

        float yawChange = abs(integratedYaw - straighteningInitialYaw);
        float backDistance = readUltrasonic(BACK_TRIG_PIN, BACK_ECHO_PIN);
        bool backWallDetected = detectColorWall();

        // Proportional Steering Adjustment
        if (yawChange < YAW_THRESHOLD) {
          int steeringAdjustment = map(yawChange, 0, YAW_THRESHOLD, SERVO_LEFT, 90);
          turnSteering(steeringAdjustment);
        } else {
          turnSteering(SERVO_LEFT);
        }

        if (yawChange >= YAW_THRESHOLD || (backDistance != -1.0 && backDistance <= MIN_DISTANCE) || backWallDetected) {
          Serial.println("Straightening Complete.");
          stopMotors();
          break;
        }

        // Timeout
        if (millis() - startTime > TIMEOUT) {
          Serial.println("Timeout Reached during Straightening.");
          stopMotors();
          break;
        }
      }

      // Step 4: Verify Position with Ultrasonic Sensors
      if (verifyPosition()) {
        Serial.println("Parking Successful!");
        parkingComplete = true;
      } else {
        Serial.println("Adjusting Position to Equalize Distances.");
        // Implement Adjustment Logic (e.g., Move Forward or Backward to Equalize)
        // For simplicity, moving forward slightly and rechecking
        moveForward(100);
        delay(1000);
        stopMotors();

        if (verifyPosition()) {
          Serial.println("Adjustment Successful. Parking Complete.");
          parkingComplete = true;
        } else {
          Serial.println("Further Adjustment Required. Restarting Parking Maneuver.");
          // Optionally, you could implement additional logic or retries here
        }
      }
    }
  }

  // Once parking is complete, you can add additional behaviors or halt.
  // For this example, we'll stop all actions.
  if (parkingComplete) {
    stopMotors();
    Serial.println("Parking Complete and Motors Stopped.");
    while (1) {
      // Halt the program
      delay(1000);
    }
  }
}

// Function to read and integrate yaw from MPU6050
float readYaw() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Get gyro Z axis rate in degrees per second
  float gyroZ = g.gyro.z;

  // Assume loop runs at roughly 100Hz (10ms)
  // In a real scenario, calculate the actual delta time
  float deltaTime = 0.01; // 10 ms

  // Integrate gyroZ to get yaw; positive for clockwise, negative for counter-clockwise
  integratedYaw += gyroZ * deltaTime;

  // Optionally, implement drift correction or reset based on other sensors

  return integratedYaw;
}

// Function to reset integrated yaw
void resetYawIntegration() {
  integratedYaw = 0.0;
}

// Function to detect wall using TCS3200 Color Sensor
bool detectColorWall() {
  // Configure TCS3200 to read specific color frequency
  // For example, set to detect pink wall by checking for specific color
  // This is a simplified example; actual implementation may require calibration

  // Example: Read frequency and determine if it matches pink
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  unsigned long duration = pulseIn(OUT, LOW, 1000000); // Timeout after 1 second
  if (duration == 0) {
    // No pulse received
    return false;
  }
  
  // Calculate frequency in Hz
  float frequency = 1000000.0 / duration;

  Serial.print("Color Frequency: ");
  Serial.println(frequency);

  // Define frequency range for pink color
  // These values need to be calibrated based on your specific environment and wall color
  const float PINK_MIN_FREQ = 400.0; // Example value
  const float PINK_MAX_FREQ = 600.0; // Example value

  if (frequency > PINK_MIN_FREQ && frequency < PINK_MAX_FREQ) {
    return true;
  }
  return false;
}

// Function to read distance from ultrasonic sensors
float readUltrasonic(int trigPin, int echoPin) {
  // Trigger the ultrasonic pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pulse
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
  if (duration == 0) {
    // Timeout or no echo received
    Serial.println("Ultrasonic Timeout or No Echo.");
    return -1.0;
  }

  // Calculate distance in centimeters
  float distance = (duration / 2.0) * 0.0343;
  Serial.print("Ultrasonic Distance (cm): ");
  Serial.println(distance);
  return distance;
}

// Function to set motor speeds
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

// Function to stop all motors
void stopMotors() {
  setMotorSpeed(0, 0);
  // Also stop backward movement if any direction is active
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Function to control steering servo
void turnSteering(int angle) {
  angle = constrain(angle, 0, 180);
  steeringServo.write(angle);
  currentSteeringAngle = angle;
  delay(200); // Allow servo to move
  Serial.print("Steering Angle Set To: ");
  Serial.println(angle);
}

// Function to move backward
void moveBackward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  setMotorSpeed(speed, speed);
  Serial.println("Moving Backward.");
}

// Function to move forward
void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setMotorSpeed(speed, speed);
  Serial.println("Moving Forward.");
}

// Function to verify parking position
bool verifyPosition() {
  float frontDistance = readUltrasonic(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  float backDistance = readUltrasonic(BACK_TRIG_PIN, BACK_ECHO_PIN);

  Serial.print("Front Distance: ");
  Serial.println(frontDistance);
  Serial.print("Back Distance: ");
  Serial.println(backDistance);

  if (frontDistance != -1.0 && backDistance != -1.0) {
    if (frontDistance < MIN_DISTANCE && backDistance < MIN_DISTANCE) {
      // Define a tolerance for equality
      float tolerance = 2.0; // cm
      if (abs(frontDistance - backDistance) <= tolerance) {
        return true;
      }
    }
  }
  return false;
}