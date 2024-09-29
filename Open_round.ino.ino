#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>

// Constants
const int TRIG_L = 35;
const int ECHO_L = 34;
const int TRIG_R = 31;
const int ECHO_R = 30;

// Servo Pin
const int SERVO_PIN = 45;

// Motor Pins
const int IN1 = 4;
const int IN2 = 5;
const int ENA = 6;

// Variables
uint16_t redValue = 0, greenValue = 0, blueValue = 0, clearValue = 0;
Servo servo;
double leftDistance, rightDistance;
double PID_error;
double steering_angle_degrees, prev_err = 0;
const double std_steering_angle = 130.0; // standard steering angle
const double Kp = 0.08, Kd = 0.15, Ki = 0.0;
double sum_err = 0;
int pos = std_steering_angle;
int no_of_turns = 0;
float heading;

// State Machine States
enum State {
  FOLLOW_LINE,
  TURN_LEFT,
  TURN_RIGHT,
  STOP
};

State currentState = FOLLOW_LINE;

// Objects
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Function Prototypes
double readUltrasonic(int trigPin, int echoPin);
double PID(double err);
void updateHeading();
bool isOrange(uint16_t red, uint16_t green, uint16_t blue);
bool isBlue(uint16_t red, uint16_t green, uint16_t blue);
void moveForward(int speed);
void stop();
void turnServo(int angle);
void readColor();
void setupColorInterrupt();
bool checkColorInterrupt();

void setup() {
  Serial.begin(115200);

  // Initialize Servo
  servo.attach(SERVO_PIN);
  turnServo(std_steering_angle);

  // Initialize Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Initialize Ultrasonic Pins
  pinMode(TRIG_L, OUTPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(ECHO_R, INPUT);

  // Initialize Color Sensor
  if (!tcs.begin()) {
    Serial.println("Error initializing TCS34725 sensor");
    while (1);
  }

  // Initialize HMC5883L Compass
  if (!mag.begin()) {
    Serial.println("Failed to initialize HMC5883L");
    while (1);
  }

  // Set up TCS34725 interrupt
  setupColorInterrupt();
}

void loop() {
  if (checkColorInterrupt()) {
    readColor();
    
    if (isBlue(redValue, greenValue, blueValue) && currentState == FOLLOW_LINE) {
      currentState = TURN_LEFT;
    } else if (isOrange(redValue, greenValue, blueValue) && currentState == FOLLOW_LINE) {
      currentState = TURN_RIGHT;
    }
    
    // Clear the interrupt
    tcs.clearInterrupt();
  }

  switch (currentState) {
    case FOLLOW_LINE:
      moveForward(255);
      leftDistance = readUltrasonic(TRIG_L, ECHO_L);
      rightDistance = readUltrasonic(TRIG_R, ECHO_R);
      PID_error = leftDistance - rightDistance;
      steering_angle_degrees = (30 * PID(PID_error)) + std_steering_angle;
      steering_angle_degrees = constrain(steering_angle_degrees, std_steering_angle - 45, std_steering_angle + 45);
      turnServo(steering_angle_degrees);
      break;

    case TURN_LEFT:
      moveForward(180);
      turnServo(std_steering_angle - 90);
      updateHeading();
      float startHeading = heading;
      while (abs(heading - startHeading) < 90) {
        updateHeading();
      }
      turnServo(std_steering_angle);
      no_of_turns++;
      currentState = FOLLOW_LINE;
      break;

    case TURN_RIGHT:
      moveForward(180);
      turnServo(std_steering_angle + 90);
      updateHeading();
      float startHeading = heading;
      while (abs(heading - startHeading) < 90) {
        updateHeading();
      }
      turnServo(std_steering_angle);
      no_of_turns++;
      currentState = FOLLOW_LINE;
      break;

    case STOP:
      stop();
      while (1);
      break;
  }

  if (no_of_turns >= 12) {
    currentState = STOP;
  }
}

void setupColorInterrupt() {
  tcs.setInterrupt(true);
  tcs.clearInterrupt();
  tcs.setIntLimits(0x0F00, 0x0F00);  // Adjust these thresholds as needed
}

bool checkColorInterrupt() {
  return tcs.getInterrupt();
}

void readColor() {
  tcs.getRawData(&redValue, &greenValue, &blueValue, &clearValue);
}


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
  double result = ((Kp * err) + (Ki * sum_err) + (Kd * (err - prev_err)));
  prev_err = err;
  return result;
}

void updateHeading() {
  sensors_event_t event;
  mag.getEvent(&event);
  heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
  if (heading < 0) {
    heading += 360;
  }
}

bool isOrange(int red, int green, int blue) {
  return (red > 100 && green > 50 && green < 100 && blue < 50);
}

bool isBlue(int red, int green, int blue) {
  return (red < 50 && green < 100 && blue > 100);
}

void moveForward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

void turnServo(int angle) {
  angle = constrain(angle, 0, 180);
  if (pos != angle) {
    while (pos != angle) {
      pos += (pos < angle) ? 1 : -1;
      servo.write(pos);
      delay(5);
    }
  }
}
// ... (rest of the functions remain the same)
