#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// Constants
#define NUM_OF_COLORS 7
const int MPU_ADDR = 0x68;
const int TRIG_L = 35;
const int ECHO_L = 34;
const int TRIG_R = 31;
const int ECHO_R = 30;

// Color Sensor Pins
const int S0 = 9;
const int S1 = 8;
const int S2 = 10;
const int S3 = 11;
const int COLOR_OUT = 12;

// Servo Pin
const int SERVO_PIN = 45;

// Calibration Values
int distinctRGB[NUM_OF_COLORS][3] = {
  {250, 250, 250}, // White
  {0, 0, 0},       // Black
  {142, 34, 41},    // Red
  {166, 125, 71},   // Yellow
  {35, 55, 38},     // Green
  {150, 50, 43},    // Orange
  {22, 25, 45}      // Blue
};

String distinctColors[NUM_OF_COLORS] = {
  "white", "black", "red", "yellow", "green", "orange", "blue"
};

// Variables
int redValue = 0, greenValue = 0, blueValue = 0;
unsigned long t_old = 0.0, t_new = 0.0;

Servo servo;
double leftDistance, rightDistance;
double PID_error;
double steering_angle_degrees, prev_err = 0;
const double std_steering_angle = 113.0; // standard steering angle
const double Kp = 0.08, Kd = 0.15, Ki  = 0.0;
double sum_err = 0;
int pos = 0;
int color_counter = 0, no_of_turns = 0;
double Gyro_angle = 0;

// MPU6050 Object
Adafruit_MPU6050 mpu;

// Function Prototypes
double readUltrasonic(int trigPin, int echoPin);
double PID(double err);
void detectRed();
void detectGreen();
void detectBlue();
void colorDetect();
double getGyroReading();

// Setup Function
void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);

  // Initialize Servo
  servo.attach(SERVO_PIN);
  for (; pos <= std_steering_angle; pos++) {
    servo.write(pos);
    delay(6);
  }

  // Initialize Color Sensor Pins
  pinMode(TRIG_L, OUTPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(ECHO_R, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  pinMode(15,OUTPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// Main Loop Function
void loop() {
 // t = millis();

  // Color Detection and Turning Logic
  if (no_of_turns <= 12) {
    detectRed();
    detectGreen();
    detectBlue();

    // Example condition based on detected colors
    if (((redValue >= 35 && redValue <= 45) && 
         (greenValue >= 38 && greenValue <= 42) &&
         (blueValue >= 48 && blueValue <= 55)) ||
        ((redValue >= 51 && redValue <= 55) && 
         (greenValue >= 41 && greenValue <= 45) && 
         (blueValue >= 41 && blueValue <= 45))) {
      
      if (color_counter == 0) {
        color_counter++;
        no_of_turns++;

        // Right Turn Example
        if (std_steering_angle + 72 < pos) { // Right turn
          for (; pos >= std_steering_angle + 72; pos--) {
            servo.write(pos);
            delay(5); // Added small delay for smoother motion
          }
        } else { // Left turn
          for (; pos <= std_steering_angle + 72; pos++) {
            servo.write(pos);
            delay(5); // Added small delay for smoother motion
          }
        }
       
        // Gyro Angle Handling
        Gyro_angle = 0;
        unsigned long startTime = millis();
        while (Gyro_angle <= 90 && (millis() - startTime) < 8000) {
          
         
          t_old = micros() ;// Timeout after 2 seconds
          Gyro_angle += getGyroReading();
        }
        Gyro_angle = 0;
      }
    } else {
      color_counter = 0; // Reset counter if condition not met
      leftDistance = readUltrasonic(TRIG_L, ECHO_L);
      rightDistance = readUltrasonic(TRIG_R, ECHO_R);
      PID_error = leftDistance - rightDistance; // Positive: turn left, Negative: turn right
      steering_angle_degrees = (30 * PID(PID_error)) + std_steering_angle;
      prev_err = PID_error;

      // Clamp the steering angle
      if (steering_angle_degrees > (std_steering_angle + 45))
        steering_angle_degrees = std_steering_angle + 45;
      else if (steering_angle_degrees < (std_steering_angle - 45))
        steering_angle_degrees = std_steering_angle - 45;

      // Move Servo to the new steering angle
      if (steering_angle_degrees > pos) {
        for (; pos <= steering_angle_degrees; pos++) {
          servo.write(pos);
          delay(5); // Added small delay for smoother motion
        }
      } else if (steering_angle_degrees < pos) {
        for (; pos >= steering_angle_degrees; pos--) {
          servo.write(pos);
          delay(5); // Added small delay for smoother motion
        }
      }
    }
  
  }
  digitalWrite(15,HIGH); //write to the second arduino that will take it as a digital input
}

// Function to Read Ultrasonic Distance
double readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(50);
  digitalWrite(trigPin, LOW);
  double duration = pulseIn(echoPin, HIGH, 30000); // Timeout after ~30ms
  if (duration == 0) {
    return -1; // Indicate no object detected within range
  }
  return (duration * 0.034 / 2); // Convert to centimeters
}

// PID Controller Function
double PID(double err) {
  sum_err += err;
  return ((Kp * err) + (Ki * sum_err) + (Kd * (err - prev_err)));
}

// Functions to Detect Red, Green, and Blue
void detectRed() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redValue = pulseIn(COLOR_OUT, LOW, 1000); // Timeout after 1ms
  // Convert pulse duration to frequency or another meaningful metric if needed
  delay(20);
}

void detectGreen() {
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenValue = pulseIn(COLOR_OUT, LOW, 1000); // Timeout after 1ms
  delay(20);
}

void detectBlue() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueValue = pulseIn(COLOR_OUT, LOW, 1000); // Timeout after 1ms
  delay(20);
}

// Function to Handle Color Detection Logic (If Needed Separately)
void colorDetect() {
  detectRed();
  detectGreen();
  detectBlue();
}

// Function to Get Gyro Reading using Adafruit MPU6050 Library
double getGyroReading() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g,&temp);
  // Assuming you want the Z-axis gyro data
  // Convert gyro data to degrees per millisecond
  // mpu.getEvent updates g.gyro.z in degrees per second
  // To get degrees since last call, multiply by delta time in seconds
  // Here, we'll calculate delta time based on loop iterations
  // For more accurate timing, consider using interruptions or precise timers
  t_new = micros();
  unsigned long deltaTime = (t_new-t_old)/(1000000); 
  //t_old = t_new;
  return g.gyro.z * deltaTime;
}