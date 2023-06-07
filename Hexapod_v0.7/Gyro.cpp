#include "SensorFusion.h"
#include "Gyro.h"
#include <math.h>

 SF fusion;

Gyro::Gyro(uint8_t MPD, TwoWire &w) : _MPD(MPD), _wire(&w) {}

void Gyro::writeRegister(uint8_t reg, uint8_t data) {
  _wire->beginTransmission(_MPD);
  _wire->write(reg);
  _wire->write(data);
  _wire->endTransmission(true);
}

void Gyro::begin() {
 
  writeRegister(0x6B, 0); // PWR_MGMT_1 Reg    -   Wake up
  writeRegister(0x19, 0); // SMPLRT_DIV Reg    -   Sets to maximum sample rate
  writeRegister(0x1A, 6); // CONFIG Reg        -   Set DLPF_CFG to 6 (5Hz bandwidth)
  writeRegister(0x1B, 0); // GYRO_CONFIG Reg   -   Set FS_SEL to 0 (±250 °/s)
  writeRegister(0x1C, 0); // ACCEL_CONFIG Reg  -   Set AFS_SEL to 0 (±2g)
  calibrate();
}

void Gyro::calibrate() {
  int16_t AcXcalArr[100];
  int16_t AcYcalArr[100];
  int16_t AcZcalArr[100];
  int16_t GyXcalArr[100];
  int16_t GyYcalArr[100];
  int16_t GyZcalArr[100];
  float pitchArr[100];
  float rollArr[100];
  float yawArr[100];
  AcXcal = 0;
  AcYcal = 0;
  AcZcal = 0;
  GyXcal = 0;
  GyYcal = 0;
  GyZcal = 0;
  pitchCal = 0;
  rollCal = 0;

  Serial.print("Calibrating:");
  for (int i = 0; i < 50; i++) {
    _wire->beginTransmission(_MPD);
    _wire->write(0x3B);
    _wire->endTransmission(false);
    _wire->requestFrom(_MPD, 14, true);
    AcXcalArr[i] = _wire->read() << 8 | _wire->read();
    AcYcalArr[i] = _wire->read() << 8 | _wire->read();
    AcZcalArr[i] = _wire->read() << 8 | _wire->read();
    int Tmp2 = _wire->read() << 8 | _wire->read();
    GyXcalArr[i] = _wire->read() << 8 | _wire->read();
    GyYcalArr[i] = _wire->read() << 8 | _wire->read();
    GyZcalArr[i] = _wire->read() << 8 | _wire->read();
    getAngleAccel(AcXcalArr[i], AcYcalArr[i], AcZcalArr[i]);
    pitchArr[i] = getPitch();
    rollArr[i] = getRoll();
    //_wire->endTransmission(true);
    Serial.print(".");
    //delay(50);
  }
  Serial.println(":Data captured");
  
  for (int i = 0; i < 50; i++) {
    AcXcal += AcXcalArr[i];
    AcYcal += AcYcalArr[i];
    AcZcal += AcZcalArr[i];
    GyXcal += GyXcalArr[i];
    GyYcal += GyYcalArr[i];
    GyZcal += GyZcalArr[i];
    pitchCal += pitchArr[i];
    rollCal += rollArr[i];
  }
  AcXcal = AcXcal / 50;
  AcYcal = AcYcal / 50;
  AcZcal = AcZcal / 50;
  GyXcal = GyXcal / 50;
  GyYcal = GyYcal / 50;
  GyZcal = GyZcal / 50;
  pitchCal = pitchCal / 50;
  rollCal = rollCal / 50;
/*
  getAngle(AcX, AcY, AcZ);

  // Gravity value
  int16_t gravity = 16384;

  // Calculate the gravity components for each axis
  int16_t gravityX = gravity * sin(roll);
  int16_t gravityY = gravity * sin(pitch);
  int16_t gravityZ = gravity * cos(pitch) * cos(roll);

  // Subtract the gravity components from the accelerometer readings
  AcXcal = (AcXcal - gravityX);
  AcYcal = (AcYcal - gravityY);
  AcZcal = (AcZcal - gravityZ);
*/
  // Print the final result of each 'cal' value
  Serial.println("Final calibration values:");
  Serial.print("AcXcal: ");
  Serial.println(AcXcal);
  Serial.print("AcYcal: ");
  Serial.println(AcYcal);
  Serial.print("AcZcal: ");
  Serial.println(AcZcal);
  Serial.print("GyXcal: ");
  Serial.println(GyXcal);
  Serial.print("GyYcal: ");
  Serial.println(GyYcal);
  Serial.print("GyZcal: ");
  Serial.println(GyZcal);
  //delay(5000);
}

void Gyro::update() {
  unsigned long startMicros = micros();

  _wire->beginTransmission(_MPD);
  _wire->write(0x3B);
  _wire->endTransmission(false);
  _wire->requestFrom(_MPD, 14, true);

  int16_t tcal = -1600;

  if (micros() - startMicros > 2500) { // Check if more than 2.5ms has passed
    writeRegister(0x6B, 0); // PWR_MGMT_1 Reg - Wake up
    return; // Exit the function early
  }

  AcX = (_wire->read() << 8 | _wire->read()) - (AcXcal < 0 ? -AcXcal : AcXcal);
  AcY = (_wire->read() << 8 | _wire->read()) - (AcYcal < 0 ? AcYcal : -AcYcal);
  AcZ = (_wire->read() << 8 | _wire->read()) - (AcZcal < 0 ? AcZcal : -AcZcal);
  Tmp = (_wire->read() << 8 | _wire->read()) - tcal;
  GyX = (_wire->read() << 8 | _wire->read()) - (GyXcal < 0 ? -GyXcal : GyXcal);
  GyY = (_wire->read() << 8 | _wire->read()) - (GyYcal < 0 ? GyYcal : -GyYcal);
  GyZ = (_wire->read() << 8 | _wire->read()) - (GyZcal < 0 ? GyZcal : -GyZcal);

  //_wire->endTransmission(true);

  tx = Tmp + tcal;
  t = tx / 340 + 36.53;
  tf = (t * 9 / 5) + 32;

  getAngleAccel(AcX, AcY, AcZ);

  // Gravity value
  //int16_t gravity = 16384;
/*
  // Calculate the gravity components for each axis
  int16_t gravityX = gravity * sin(roll);
  int16_t gravityY = gravity * sin(pitch);
  int16_t gravityZ = gravity * cos(pitch) * cos(roll);

  // Subtract the gravity components from the accelerometer readings
  //AcX = ((AcX - AcXcal) - gravityX);
  //AcY = ((AcY - AcYcal) - gravityY);
  //AcZ = ((AcZ - AcZcal) - gravityZ);

  AcX_noG = AcX - gravityX;
  AcY_noG = AcY - gravityY;
  AcZ_noG = AcZ - gravityZ;
*/
  /*AcX_g = AcX / (float)gravity;
  AcY_g = AcY / (float)gravity;
  AcZ_g = AcZ / (float)gravity;
  //GyX_dg = GyX / 131.0;
  //GyY_dg = GyY / 131.0;
  //GyZ_dg = GyZ / 131.0;
  GyX_rad = GyX_dg * DEG_TO_RAD;
  GyY_rad = GyY_dg * DEG_TO_RAD;
  GyZ_rad = GyZ_dg * DEG_TO_RAD;
  //getAngleGyro(GyX_dg, GyY_dg, GyZ_dg);*/
}

void Gyro::getAngleAccel(int16_t Ax, int16_t Ay, int16_t Az) {
  double x = Ax;
  double y = Ay;
  double z = Az;
  pitch = atan(x / sqrt((y * y) + (z * z)));
  roll = atan(y / sqrt((x * x) + (z * z)));
  pitch *= RAD_TO_DEG;
  roll *= RAD_TO_DEG;
  //pitch = pitch - pitchCal;
  //roll = roll - pitchCal;
}

void Gyro::getAngleGyro(int16_t gx, int16_t gy, int16_t gz) {
  // Convert gyroscope values to degrees/sec
  double gyroXrate = gx / 131.0;
  double gyroYrate = gy / 131.0;
  double gyroZrate = gz / 131.0;


  pitchGyro += gy;
  rollGyro += gx;

  // Note: This simple integration method accumulates error over time,
  // in other words; this sucks
}

void Gyro::updateFusedData() {
  update();
  fgx = getGyX_rad();
  fgy = getGyY_rad();
  fgz = getGyZ_rad();
  fax = getAcX_g();
  fay = getAcY_g();
  faz = getAcZ_g();
  deltat = fusion.deltatUpdate();
  fusion.MadgwickUpdate(fgx, fgy, fgz, fax, fay, faz, deltat);
  fPitch = fusion.getPitch();
  fRoll = fusion.getRoll();
  fYaw = fusion.getYaw();
}

void Gyro::updatePR() {
    update();
    
    fRoll = getPitch();
    fPitch = getRoll();

    //fRoll *= DEG_TO_RAD;
    //fPitch *= DEG_TO_RAD;
}

double Gyro::getFusedPitch() {
  return fPitch;
}

double Gyro::getFusedRoll() {
  return fRoll;
}

double Gyro::getFusedYaw() {
  return fYaw;
}

double Gyro::getPitch() {
  return pitch;
}
double Gyro::getRoll() {
  return roll;
}
double Gyro::getPitchGyro() {
  return pitchGyro;
}
double Gyro::getRollGyro() {
  return rollGyro;
}
double Gyro::getTempC() {
  return t;
}
double Gyro::getTempF() {
  return tf;
}
int16_t Gyro::getAcX() {
  return AcX;
}
int16_t Gyro::getAcY() {
  return AcY;
}
int16_t Gyro::getAcZ() {
  return AcZ;
}
int16_t Gyro::getGyX() {
  return GyX;
}
int16_t Gyro::getGyY() {
  return GyY;
}
int16_t Gyro::getGyZ() {
  return GyZ;
}
float Gyro::getAcX_g() {
  return AcX_g;
}
float Gyro::getAcY_g() {
  return AcY_g;
}
float Gyro::getAcZ_g() {
  return AcZ_g;
}
float Gyro::getAcX_noG() {
  return AcX_g;
}
float Gyro::getAcY_noG() {
  return AcY_noG;
}
float Gyro::getAcZ_noG() {
  return AcZ_noG;
}
float Gyro::getGyX_dg() {
  return GyX_dg;
}
float Gyro::getGyY_dg() {
  return GyY_dg;
}
float Gyro::getGyZ_dg() {
  return GyZ_dg;
}
float Gyro::getGyX_rad() {
  return GyX_rad;
}
float Gyro::getGyY_rad() {
  return GyY_rad;
}
float Gyro::getGyZ_rad() {
  return GyZ_rad;
}
