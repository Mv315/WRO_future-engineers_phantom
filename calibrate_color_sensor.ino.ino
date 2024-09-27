const int S0 = 9;        // S0 pin of the color sensor
const int S1 = 8;        // S1 pin of the color sensor
const int S2 = 10;       // S2 pin of the color sensor
const int S3 = 11;       // S3 pin of the color sensor
const int COLOR_OUT = 12; // Output pin of the color sensor

// Function Prototypes
long readColor(int s2, int s3);

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  Serial.println("Color Sensor Calibration Started");

  // Initialize Color Sensor Pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);

  // Set Frequency Scaling to 100% (can be adjusted as needed)
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
}

void loop() {
  // Read Red, Green, and Blue Values
  long red = readColor(LOW, LOW);    // Red
  long green = readColor(HIGH, HIGH); // Green
  long blue = readColor(LOW, HIGH);   // Blue

  // Print the RGB Values to Serial Monitor
  Serial.print("Red: ");
  Serial.print(red);
  Serial.print(" | Green: ");
  Serial.print(green);
  Serial.print(" | Blue: ");
  Serial.println(blue);

  // Wait for 1 Second before next reading
  delay(1000);
}

/**
 * Reads the color value based on S2 and S3 pin settings.
 * @param s2: The state to set on S2 pin (HIGH or LOW)
 * @param s3: The state to set on S3 pin (HIGH or LOW)
 * @return: Duration of the pulse in microseconds
 */
long readColor(int s2, int s3) {
  // Set S2 and S3 to filter the desired color
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);

  // Give time for the color sensor to stabilize
  delay(20); // 20ms delay

  // Read the pulse duration on COLOR_OUT pin
  // Measure how long the pulse stays HIGH
  long duration = pulseIn(COLOR_OUT, LOW, 1000); // Timeout after 1000 microseconds

  // Optional: Handle timeout (duration == 0)
  if (duration == 0) {
    Serial.println("Warning: PulseIn timed out!");
  }

  return duration;
}