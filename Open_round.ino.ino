//blue - left turn, orange - right turn
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
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

// Variables
int redValue = 0, greenValue = 0, blueValue = 0, clearValue = 0;
unsigned long t_old = 0.0, t_new = 0.0;

Servo servo;
double leftDistance, rightDistance;
double PID_error;
double steering_angle_degrees, prev_err = 0;
const double std_steering_angle = 130.0; // standard steering angle
const double Kp = 0.08, Kd = 0.15, Ki  = 0.0;
double sum_err = 0;
int pos = 0;
int no_of_turns = 0;
double Gyro_angle = 0;

// MPU6050 Object
Adafruit_MPU6050 mpu;
Adafruit_TCS34725 tcs = Adafruit_TCS34725();
// Function Prototypes
double readUltrasonic(int trigPin, int echoPin);
double PID(double err);
double getGyroReading();
bool isOrange(int red, int green, int blue);
bool isBlue(int red, int green, int blue);


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
  pinMode(4,OUTPUT);//in 1 or 3
  pinMode(6,OUTPUT);// en A or B
  pinMode(5,OUTPUT);// in 2 or 4
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
  if (no_of_turns <= 12) {
    moveforward();
    tcs.getRawData(&redValue, &greenValue, &blueValue, &clearValue);

    if (isOrange(redValue, greenValue, blueValue) || isBlue(redValue, greenValue, blueValue)) {
      no_of_turns++;
      moveforwardturn();
      int target_angle;
      int turn_direction;

      if (isOrange(redValue, greenValue, blueValue)) {
        // Right turn
        target_angle = std_steering_angle + 90;  // 220 degrees
        turn_direction = 1;
      } else {
        // Left turn
        target_angle = std_steering_angle - 90;  // 40 degrees
        turn_direction = -1;
      }

      // Perform the turn
      while (pos != target_angle) {
        pos += turn_direction;
        servo.write(pos);
        delay(5);  // Small delay for smooth motion
      }

      // Use gyro to confirm turn
      Gyro_angle = 0;
      unsigned long startTime = millis();
      while (abs(Gyro_angle) < 90 && (millis() - startTime) < 8000) {
        t_old = micros();
        double gyroReading = getGyroReading();
        // Apply the correction factor to the gyro reading
        Gyro_angle += gyroReading *  turn_direction;
      }
Gyro_angle = 0;

      // Return to center position after turn
      while (pos != std_steering_angle) {
        pos += (pos < std_steering_angle) ? 1 : -1;
        servo.write(pos);
        delay(5);
      }
    }
    else {
      // Normal line following logic
      leftDistance = readUltrasonic(TRIG_L, ECHO_L);
      rightDistance = readUltrasonic(TRIG_R, ECHO_R);
      PID_error = leftDistance - rightDistance;
      steering_angle_degrees = (30 * PID(PID_error)) + std_steering_angle;
      prev_err = PID_error;

      // Clamp the steering angle
      steering_angle_degrees = constrain(steering_angle_degrees, std_steering_angle - 45, std_steering_angle + 45);

      // Move Servo to the new steering angle
      while (pos != (int)steering_angle_degrees) {
        pos += (pos < steering_angle_degrees) ? 1 : -1;
        servo.write(pos);
        delay(5);
      }
    }
  }
  else{
    stop();
    while(1);

  }
}

// Function to Read Ultrasonic Distance
double readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(50);
  digitalWrite(trigPin, LOW);
  double duration = pulseIn(echoPin, HIGH, 30000);
  return (duration == 0) ? -1 : (duration * 0.034 / 2);
}

double PID(double err) {
  sum_err += err;
  return ((Kp * err) + (Ki * sum_err) + (Kd * (err - prev_err)));
}

double getGyroReading() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  t_new = micros();
  unsigned long deltaTime = (t_new - t_old) / 1000000.0;
  return g.gyro.z * deltaTime;
}

bool isOrange(int red, int green, int blue) {
  // Adjust these values based on your sensor's readings for orange
  return (red > 100 && green > 50 && green < 100 && blue < 50);
}

bool isBlue(int red, int green, int blue) {
  // Adjust these values based on your sensor's readings for blue
  return (red < 50 && green < 100 && blue > 100);
}
void moveforward(){
  digitalWrite(4,LOW);
  digitalWrite(5,HIGH);
  analogWrite(6,255);
}
void moveforwardturn(){
  digitalWrite(4,LOW);
  digitalWrite(5,HIGH);
  analogWrite(6,180);
}
void stop(){
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  analogWrite(6,0);
}
