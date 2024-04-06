#include "Adafruit_APDS9960.h"
#include <Wire.h>
#include <Servo.h>

int Ain1 = 4;     // PB5
int Ain2 = 10;    // PB6
int sleepPin = 9; // PC7
int Bin1 = 8;     // PA9
int Bin2 = 5;     // PB4
int button = 23;
int LED = 13;

int baseSpeed = 180, leftMotorSpeed = 0, rightMotorSpeed = 0;
unsigned long startTime = 0, currentTime = 0, count = 0;
int woodGreen = 27;

// Define PID parameters
double error = 0, lastError = 0;
double Kp = 6.66, Ki = 0.0065, Kd = 3.41; // PID gains
double integral = 0, derivative = 0;
double dt = 0.1; // time interval for derivative calculation
int servoPin = 11;

TwoWire Wire2(PB3, PB10); // Creating a new instance of the Wire library for I2C communication
// Creating an instance of the APDS9960 sensor
Adafruit_APDS9960 apds; //left
Adafruit_APDS9960 apds2;//right
Servo claw;

void setup() {

  Serial.begin(115200);
  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  Wire2.begin();
  pinMode(Ain1, OUTPUT);
  pinMode(Ain2, OUTPUT);
  pinMode(sleepPin, OUTPUT);
  pinMode(Bin1, OUTPUT);
  pinMode(Bin2, OUTPUT);
  claw.attach(servoPin);
  pinMode(button, INPUT);
  pinMode(LED, OUTPUT);

  // Initializing the APDS9960 sensor with Wire2
  if (!apds.begin(10, APDS9960_AGAIN_4X, APDS9960_ADDRESS, &Wire2)) { // Passing the Wire2 instance to the begin() function
    Serial.println("Failed to initialize device! Please check your wiring.1");
  } else {
    Serial.println("Device initialized!");
  }
  if (!apds2.begin()) { // Passing the Wire2 instance to the begin() function
    Serial.println("Failed to initialize device! Please check your wiring.2");
  } else {
    Serial.println("Device initialized!");
  }

  // Enable color sensing mode
  apds.enableColor(true);
  apds2.enableColor(true);
}

void loop() {
  // Wait for color data to be ready
  while (!apds.colorDataReady() || !apds2.colorDataReady()) {
    delay(5);
  }
  // Get current time
  currentTime = millis();

  // 1. start movement
  if (!digitalRead(button)) {
    performMovement();
  }

  // 2. Get the data
  uint16_t r1, g1, b1, c1, r2, g2, b2, c2;
  apds.getColorData(&r1, &g1, &b1, &c1);
  int leftSensorGreen = g1;

  apds2.getColorData(&r2, &g2, &b2, &c2);
  int rightSensorGreen = g2;

  // 3. end run at double red and only run code after 42 seconds
  if((leftSensorGreen < 15 && rightSensorGreen < 15) && (currentTime-startTime) > 42000) {
    performEnd();
  }

  // 4. pick up lego man after 23 seconds (only do this once since count will be 1 after first run)
  if((b1 > 36 && b2 > 36) && ((currentTime-startTime) > 28500 && count == 0)) {
    count = 1;
    performPickUp();
  }
  
  // 5. equate both motors if detecting wood
  if (leftSensorGreen > woodGreen) {
    leftSensorGreen = 30;
  }
  if (rightSensorGreen > woodGreen) {
    rightSensorGreen = 30;
  }

  // 6. find difference (error) and input it to PID function
  double diff = leftSensorGreen - rightSensorGreen;
  double PID = computePID(diff);

  // 7. Adjust motor speed values based on PID output (floor it to 255 if needed)

  leftMotorSpeed = min(baseSpeed + PID, 255.0);
  rightMotorSpeed = min(baseSpeed - PID, 255.0);

  // 8. set motor speed (if motor speed is < 0, turn direction of rotation of said motor)
  // Left motor PWM
  if (leftMotorSpeed >= 0) {
      analogWrite(Ain1, 0);
      analogWrite(Ain2, leftMotorSpeed);
  }
  else {
      analogWrite(Ain2, 0);
      analogWrite(Ain1, abs(leftMotorSpeed));
  }
    // Right motor PWM
    if (rightMotorSpeed >= 0) {
        analogWrite(Ain1, 0);
        analogWrite(Bin2, rightMotorSpeed);
  }
  else {
      analogWrite(Bin2, 0);
      analogWrite(Bin1, abs(rightMotorSpeed));
  }

  // 9. print different rgb channels and PID and motor values
  // Serial.print("Left:");
  // Serial.print("r:");
  // Serial.print(r1);
  // Serial.print(" g:");
  // Serial.print(g1);
  // Serial.print(" b:");
  // Serial.print(b1);
  // Serial.print(" c:");
  // Serial.println(c1);

  // Serial.print("Right:");
  // Serial.print("r:");
  // Serial.print(r2);
  // Serial.print(" g:");
  // Serial.print(g2);
  // Serial.print(" b:");
  // Serial.print(b2);
  // Serial.print(" c:");
  // Serial.println(c2);
  // Serial.println();

  // Serial.print("Left Sensor: ");
  // Serial.print(leftSensorGreen);
  // Serial.print(" Right Sensor: ");
  // Serial.print(rightSensorGreen);
  // Serial.print(" Left PID Output: ");
  // Serial.print(PID);
  // Serial.print(" Right PID Output: ");
  // Serial.println(-PID);

  // Serial.print("Left Motor Speed: ");
  // Serial.println(leftMotorSpeed);
  // Serial.print("Right Motor Speed: ");
  // Serial.println(rightMotorSpeed);
  // delay(150);
}


void performMovement() {
  // Set start time for future comparison
  startTime = millis();
  //start motors
  digitalWrite(sleepPin, HIGH);
  // Run full speed for 0.4 seconds for stall torque
  analogWrite(Ain2, 0);
  analogWrite(Ain2, 255);
  analogWrite(Bin2, 0);
  analogWrite(Bin2, 255);
  delay(400);
}

void performPickUp() {
  Serial.println("Picking up");
  // Stop and move forward very slightly
  analogWrite(Ain1, 0);
  analogWrite(Ain2, 0);
  analogWrite(Bin1, 0);
  analogWrite(Bin2, 0);
  delay(300);
  analogWrite(Ain2, 255);
  analogWrite(Bin2, 255);
  delay(40);

  // Stop and pick up Lego man and make the 180° turn
  analogWrite(Ain1, 0);
  analogWrite(Ain2, 0);
  analogWrite(Bin1, 0);
  analogWrite(Bin2, 0);
  delay(6000);
  claw.write(10);
  delay(1000);
  analogWrite(Ain2, 255);
  analogWrite(Ain1, 0);
  analogWrite(Bin2, 0);
  analogWrite(Bin1, 255);
  delay(750);
  analogWrite(Ain1, 0);
  analogWrite(Ain2, 0);
  analogWrite(Bin1, 0);
  analogWrite(Bin2, 0);
  delay(200);

  // Go back slightly and full speed for 90ms to overcome stall torque and line follow again
  analogWrite(Ain1, 255);
  analogWrite(Bin1, 255);
  delay(100);
  analogWrite(Ain1, 0);
  analogWrite(Ain2, 0);
  analogWrite(Bin1, 0);
  analogWrite(Bin2, 0);
  delay(100);
  analogWrite(Ain2, 255);
  analogWrite(Bin2, 255);
  delay(90);
}

void performEnd() {
  // Stop motors and open claw
  analogWrite(Ain1, 0);
  analogWrite(Ain2, 0);
  analogWrite(Bin1, 0);
  analogWrite(Bin2, 0);
  claw.write(90);
  digitalWrite(sleepPin, LOW);
  Serial.println("Ending Run");
  exit(0);
}
 //PID function
  double computePID(double error) {
    integral += error * dt;
    derivative = (error - lastError) / dt;
    double pidOutput = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;
    return pidOutput;
  }