// to check line 242 for which axis of gyro we will read
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Initialize MPU6050
Adafruit_MPU6050 mpu;
// Define Parking States
enum ParkingState {
  IDLE,
  TURN_RIGHT_BACKWARD,
  STRAIGHTEN_BACKWARD,
  TURN_RIGHT_FORWARD,
  STRAIGHTEN_FORWARD,
  PARKING_COMPLETE
};

// Initialize current state
ParkingState currentState = IDLE;
// Define Servo for Steering
Servo steeringServo;
const int STEERING_PIN = 8;
const int SERVO_LEFT = 0;    // Maximum left angle
const int SERVO_RIGHT = 180; // Maximum right angle
int currentSteeringAngle = 90; // Start with straight
const int MPU_addr=0x68;
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

int16_t AcZ,AcY,AcX;
double x,y,z;


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

//prev time/

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
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);

  Serial.println("Initialization Complete.");
}

void loop() {
  switch (currentState) {
    case IDLE:
      // Wait until the front wall is detected
      if (detectColorWall()) {
        Serial.println("Front Wall Detected. Initiating Parking Maneuver.");
        resetYawIntegration(); // Reset yaw before starting
        initialYaw = integratedYaw;
        currentState = TURN_RIGHT_BACKWARD;
      }
      break;
      
    case TURN_RIGHT_BACKWARD:
      Serial.println("State: TURN_RIGHT_BACKWARD");
      turnSteering(SERVO_RIGHT);
      moveBackward(150); // Adjust speed as needed
      
      
      while (true) {
        startTime = millis();
        float deltaYaw = readYaw();
        Serial.print("Delta Yaw: ");
        Serial.println(deltaYaw);
        
        float yawChange = abs(integratedYaw - initialYaw);
        float backDistance = readUltrasonic(BACK_TRIG_PIN, BACK_ECHO_PIN);
        bool backWallDetected = detectColorWall();
        
        if (yawChange >= YAW_THRESHOLD || (backDistance != -1.0 && backDistance <= MIN_DISTANCE) || backWallDetected) {
          Serial.println("Stopping Backward Movement after Turning Right.");
          stopMotors();
          currentState = STRAIGHTEN_BACKWARD;
          break;
        }
         
      }
      break;
      
    case STRAIGHTEN_BACKWARD:
      Serial.println("State: STRAIGHTEN_BACKWARD");
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
          int steeringAdjustment = map(yawChange * 10, 0, YAW_THRESHOLD * 10, SERVO_LEFT, 90); // Multiply to maintain resolution
          steeringAdjustment = constrain(steeringAdjustment, SERVO_LEFT, 90);
          turnSteering(steeringAdjustment);
        } 
        
        if (yawChange >= YAW_THRESHOLD || (backDistance != -1.0 && backDistance <= MIN_DISTANCE) || backWallDetected) {
          Serial.println("Straightening Complete.");
          stopMotors();
          if(verifyPosition()){
            currentState = PARKING_COMPLETE;
            break
          }
          currentState = TURN_RIGHT_FORWARD;
          break;
        }

      }
      break;
      
    case TURN_RIGHT_FORWARD:
      Serial.println("State: TURN_RIGHT_FORWARD");
      turnSteering(SERVO_RIGHT);
      moveForward(150); // Adjust speed as needed
      startTime = millis();
      
      while (true) {
        float deltaYaw = readYaw();
        Serial.print("Delta Yaw: ");
        Serial.println(deltaYaw);
        
        float yawChange = abs(integratedYaw - initialYaw);
        float frontDistance = readUltrasonic(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
        bool frontWallDetected = detectColorWall();
        
        if (yawChange >= YAW_THRESHOLD || (frontDistance != -1.0 && frontDistance <= MIN_DISTANCE) || frontWallDetected) {
          Serial.println("Stopping Forward Movement after Turning Right.");
          stopMotors();
          currentState = STRAIGHTEN_FORWARD;
          break;
        }
        
        }
      
      break;
      
    case STRAIGHTEN_FORWARD:
      Serial.println("State: STRAIGHTEN_FORWARD");
      float straightenForwardInitialYaw = integratedYaw;
      turnSteering(SERVO_LEFT);
      startTime = millis();
      
      while (true) {
        moveForward(150); // Adjust speed as needed
        float deltaYaw = readYaw();
        Serial.print("Delta Yaw during Straightening Forward: ");
        Serial.println(deltaYaw);
        
        float yawChange = abs(integratedYaw - straightenForwardInitialYaw);
        float frontDistance = readUltrasonic(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
        bool frontWallDetected = detectColorWall();
        
        // Proportional Steering Adjustment
        if (yawChange < YAW_THRESHOLD) {
          int steeringAdjustment = map(yawChange * 10, 0, YAW_THRESHOLD * 10, SERVO_LEFT, 90);
          steeringAdjustment = constrain(steeringAdjustment, SERVO_LEFT, 90);
          turnSteering(steeringAdjustment);
        } else {
          turnSteering(SERVO_LEFT);
        }
        
        if (yawChange >= YAW_THRESHOLD || (frontDistance != -1.0 && frontDistance <= MIN_DISTANCE) || frontWallDetected) {
          Serial.println("Straightening Forward Complete.");
          stopMotors();
          // Proceed to verify position
          if (verifyPosition()) {
            Serial.println("Parking Successful!");
            currentState = PARKING_COMPLETE;
          } else {
            Serial.println("Position Verification Failed. Adjusting...");
            currentState = TURN_RIGHT_BACKWARD; // Optionally implement further adjustment steps
          }
          break;
        }
      }
      break;
      
    case PARKING_COMPLETE:
      Serial.println("State: PARKING_COMPLETE");
      stopMotors();
      Serial.println("Parking Complete and Motors Stopped.");
      while (1) {
        // Halt the program
        delay(1000);
      }
      break;
      
    default:
      // Handle unexpected states
      Serial.println("Unknown State. Resetting to IDLE.");
      currentState = IDLE;
      break;
  }
}

// Function to read and integrate yaw from MPU6050
float readYaw() {
Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);  // Starting with the first gyroscope register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);

  int16_t GyX = Wire.read() << 8 | Wire.read();
  int16_t GyY = Wire.read() << 8 | Wire.read();
  int16_t GyZ = Wire.read() << 8 | Wire.read();

  // Convert gyroscope data to degrees per second (assuming full scale range ±250°/s)
  double gz = GyZ / 131.0;

 
  unsigned long currentTime = millis();
  double deltaTime = currentTime - startTime;
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
      if (abs(frontDistance 2- backDistance) <= tolerance) {
        return true;
      }
    }
  }
  return false;
}
