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

uint8_t Gyro::readRegister(uint8_t reg) {
  _wire->beginTransmission(_MPD);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom(_MPD, (uint8_t)1);
  if (_wire->available()) {
    return _wire->read();
  } else {
    return 0; // Return 0 if no data available
  }
}

void Gyro::begin() {
  writeRegister(0x6B, 0x80); // PWR_MGMT_1 Reg    -   Reset
  delay(100);
  writeRegister(0x6B, 0x00); // PWR_MGMT_1 Reg    -   Wake up
  delay(100);
  writeRegister(0x6B, 0x03); // PWR_MGMT_1 Reg    -   Set Clock
  delay(200);
  writeRegister(0x1A, 0x03); // CONFIG Reg        -   Set to 250Hz bandwidth
  writeRegister(0x19, 0x03); // SMPLRT_DIV Reg    -   Set to 1k sample rate
  
  uint8_t c = readRegister(0x1B);
  c = c & ~0x03;
  c = c & ~0x18; 
  c = c | (uint8_t)3 << 3;
  writeRegister(0x1B, c); // GYRO_CONFIG Reg   -   Set Gyro range
  
  c = readRegister(0x1C);
  c = c & ~0x18; 
  c = c | (uint8_t)3 << 3;
  writeRegister(0x1C, c); // ACCEL_CONFIG Reg  -   Set Accel range
  
  writeRegister(0x1A, 0x03);  // CONFIG Reg    -   Set to 250Hz again(per FastIMU)
  writeRegister(0x37, 0x22);  // INT_CONFIG
  writeRegister(0x38, 0x01); // INT_ENABLE Reg -   Enable interrupts
  delay(100);
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
    _wire->endTransmission(true);
    _wire->requestFrom(_MPD, 14, false);
    if (_wire->available() >= 14) {
    AcXcalArr[i] = _wire->read() << 8 | _wire->read();
    AcYcalArr[i] = _wire->read() << 8 | _wire->read();
    AcZcalArr[i] = _wire->read() << 8 | _wire->read();
    int Tmp2 = _wire->read() << 8 | _wire->read();
    GyXcalArr[i] = _wire->read() << 8 | _wire->read();
    GyYcalArr[i] = _wire->read() << 8 | _wire->read();
    GyZcalArr[i] = _wire->read() << 8 | _wire->read();
    } else {
      Serial.println("Gyro error, restarting.");
      begin();
      return; // Try again
    }
    getAngleAccel(AcXcalArr[i], AcYcalArr[i], AcZcalArr[i]);
    pitchArr[i] = getPitch();
    rollArr[i] = getRoll();
    _wire->endTransmission(true);
    Serial.print(".");
    delay(50);
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
  _wire->beginTransmission(_MPD);
  _wire->write(0x3B);
  _wire->endTransmission(true);
  _wire->requestFrom(_MPD, 14, false);

  if (_wire->available() >= 14) {
  AcX = (_wire->read() << 8 | _wire->read()) - (AcXcal < 0 ? -AcXcal : AcXcal);
  AcY = (_wire->read() << 8 | _wire->read()) - (AcYcal < 0 ? AcYcal : -AcYcal);
  AcZ = (_wire->read() << 8 | _wire->read()) - (AcZcal < 0 ? AcZcal : -AcZcal);
  Tmp = (_wire->read() << 8 | _wire->read()) - tcal;
  GyX = (_wire->read() << 8 | _wire->read()) - (GyXcal < 0 ? -GyXcal : GyXcal);
  GyY = (_wire->read() << 8 | _wire->read()) - (GyYcal < 0 ? GyYcal : -GyYcal);
  GyZ = (_wire->read() << 8 | _wire->read()) - (GyZcal < 0 ? GyZcal : -GyZcal);

  _wire->endTransmission(true);

int16_t gravity = 16384;
AcX_g = AcX / (float)gravity; //Convert accel from raw to G's
  AcY_g = AcY / (float)gravity;
  AcZ_g = AcZ / (float)gravity;
  GyX_dg = GyX / 131.0; // Convert Gyro from raw to degrees
  GyY_dg = GyY / 131.0;
  GyZ_dg = GyZ / 131.0;
  GyX_rad = GyX_dg * DEG_TO_RAD; //Gyro in Radians
  GyY_rad = GyY_dg * DEG_TO_RAD;
  GyZ_rad = GyZ_dg * DEG_TO_RAD;
/*
  if (abs(AcX - prevAcX) > threshold) {
    AcX = alpha * AcX + (1 - alpha) * prevAcX;
    prevAcX = AcX;
  } else { AcX = prevAcX; }
  if (abs(AcY - prevAcY) > threshold) {
    AcY = alpha * AcY + (1 - alpha) * prevAcY;
    prevAcY = AcY;
  } else { AcY = prevAcY; }
  if (abs(AcZ - prevAcZ) > threshold) {
    AcZ = alpha * AcZ + (1 - alpha) * prevAcZ;
    prevAcZ = AcZ;
  } else { AcZ = prevAcZ; }
//Serial.println("AcX: " + String(AcX) + " AcY: " + String(AcY) + " AcZ: " + String(AcZ));
    */
  tx = Tmp + tcal;
  t = tx / 340 + 36.53;
  tf = (t * 9 / 5) + 32;
  //Serial.print("Gyro data obtained  ");
  getAngleAccel(AcX, AcY, AcZ);
  updateFusedData();
} else {
  Serial.println("Gyro has shit the bed, again.");
  begin();
  return;
}
  

  //int16_t gravityX = gravity * sin(roll);
  //int16_t gravityY = gravity * sin(pitch);
  //int16_t gravityZ = gravity * cos(pitch) * cos(roll);

  //AcX = ((AcX - AcXcal) - gravityX);
  //AcY = ((AcY - AcYcal) - gravityY);
  //AcZ = ((AcZ - AcZcal) - gravityZ);

  //AcX_noG = AcX - gravityX;
  //AcY_noG = AcY - gravityY;
  //AcZ_noG = AcZ - gravityZ;

  
  //getAngleGyro(GyX_dg, GyY_dg, GyZ_dg);
}

void Gyro::getAngleAccel(int16_t Ax, int16_t Ay, int16_t Az) {
  float x = Ax;
  float y = Ay;
  float z = Az;
  if (x != 0.00f && (y != 0.00f || z != 0.00f)) {
    float a = sqrt((y * y) + (z * z));
    float tempPitch = atan2(x, a);
    pitch = alpha * tempPitch + (1 - alpha) * pitch;
    prevPitch = pitch;
  } else {
    pitch = prevPitch;
  }
  if (y != 0.00f && (x != 0.00f || z != 0.00f)) {
    float  a = sqrt((x * x) + (z * z));
    float tempRoll = atan2(y, a);
    roll = alpha * tempRoll + (1 - alpha) * roll;
    prevRoll = roll;
  } else {
    roll = prevRoll;
  }
  //pitch *= RAD_TO_DEG;
  //roll *= RAD_TO_DEG;
  
}

void Gyro::getAngleGyro(int16_t gx, int16_t gy, int16_t gz) {
  // Convert gyroscope values to degrees/sec
  float gyroXrate = gx / 131.0;
  float gyroYrate = gy / 131.0;
  float gyroZrate = gz / 131.0;


  pitchGyro += gy;
  rollGyro += gx;

  // this integration method accumulates a lot of error over time, fusion only
}

void Gyro::updateFusedData() {
  //update();
  fgx = getGyX_rad();
  fgy = getGyY_rad();
  fgz = getGyZ_rad();
  fax = getAcX_g();
  fay = getAcY_g();
  faz = getAcZ_g();
  deltat = fusion.deltatUpdate();
  fusion.MadgwickUpdate(fgx, fgy, fgz, fax, fay, faz, deltat);
  fPitch = fusion.getPitchRadians();
  fRoll = fusion.getRollRadians();
  fYaw = fusion.getYawRadians();
}

void Gyro::updatePR() {
    update();
    
    fRoll = getPitch();
    fPitch = getRoll();

    //fRoll *= DEG_TO_RAD;
    //fPitch *= DEG_TO_RAD;
}

void Gyro::setAlpha(float newAlpha) {
  alpha = newAlpha;
}

bool Gyro::dataAvailable() { 
  return (readRegister(0x3A) & 0x01);
}

float Gyro::getFusedPitch() {
  return fPitch;
}

float Gyro::getFusedRoll() {
  return fRoll;
}

float Gyro::getFusedYaw() {
  return fYaw;
}

float Gyro::getPitch() {
  return pitch;
}
float Gyro::getRoll() {
  return roll;
}
float Gyro::getPitchGyro() {
  return pitchGyro;
}
float Gyro::getRollGyro() {
  return rollGyro;
}
float Gyro::getTempC() {
  return t;
}
float Gyro::getTempF() {
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
