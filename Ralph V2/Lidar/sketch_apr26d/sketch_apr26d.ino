#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>

// Pin and address
const int servoPin = 8;
const int rotationRate = 90 + 6;

// Global variables
Servo servo;
VL53L0X tof1;
unsigned long avgRotTime;
#define HIGH_SPEED
const int maxPoints = 1000;
int readings[maxPoints];


bool isInRange(int value, int min, int max) {
  return (value >= min && value <= max);
}

void Home() {
  int range = tof1.readRangeSingleMillimeters();
  if (!isInRange(range, 22, 28)) {
    do {
    servo.write(rotationRate);
    range = tof1.readRangeSingleMillimeters();
    //Serial.println(range);
    //delay(10);
  } while (!isInRange(range, 22, 28));
  servo.write(0);
  delay(1);
  servo.write(90); // Stop rotation
  } else {
    return;
  }
  
}
  
float calibrate() {
  Home();
  // Rotate the base positive until tof1 finds the homing pole
  servo.write(rotationRate);
  int range;
  // Perform calibration rotation
  unsigned long startTime = millis();
  servo.write(rotationRate);
  do {
    range = tof1.readRangeSingleMillimeters();
    //Serial.println(range);
    //delay(10);
  } while (!isInRange(range, 22, 28));
  servo.write(0);
  delay(1);
  servo.write(90); // Stop rotation

  unsigned long rotationTime = millis() - startTime;
  Serial.print("Rotation time: ");
  Serial.print(rotationTime);
  Serial.println(" ms");
  
  return rotationTime;
}

void setup() {
  // Setup serial communication
  Serial.begin(115200);

  // Connect the servo to pin 8
  servo.attach(servoPin);

  // Initialize I2C communication
  Wire.begin();
  
  // Initialize the TOF sensor
  tof1.init();
    tof1.setMeasurementTimingBudget(20000);
    Home();
  // Perform zeroing and calibration rotation 3 times
  /*unsigned long sumRotTime = 0;
  for (int i = 0; i < 3; i++) {
    sumRotTime += calibrate(true);
  }
  avgRotTime = sumRotTime / 3;

  Serial.print("Average rotation time: ");
  Serial.print(avgRotTime);
  Serial.println(" ms");*/
}

void loop() {
  method1();
}

void method1() {
  // Reset the position to the home position
  Home();

  // Start rotation
  servo.write(rotationRate);

  // Read TOF1 sensor during rotation and store the values
  int pointCount = 0;
  int range;
  unsigned long startTime = millis();
  do {
    range = tof1.readRangeSingleMillimeters();
    readings[pointCount] = range;
    pointCount++;

    if (pointCount >= maxPoints) {
      break;
    }

    //delay(10);
  } while (!isInRange(range, 22, 28) || millis() - startTime < avgRotTime);

  // Stop rotation
  servo.write(0);
  delay(1);
  servo.write(90);
  
  // Send data via serial
  Serial.print(pointCount);
  for (int i = 0; i < pointCount; i++) {
    Serial.print(",");
    Serial.print(readings[i]);
  }
  Serial.println();
}
