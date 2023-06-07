#ifndef GYRO_H
#define GYRO_H

#include <Wire.h>

class Gyro {
  public:
    Gyro(uint8_t MPD, TwoWire &w);
    void writeRegister(uint8_t reg, uint8_t data);
    void begin();
    void calibrate();
    void update();
    void getAngleAccel(int16_t Ax, int16_t Ay, int16_t Az);
    void getAngleGyro(int16_t Gx, int16_t Gy, int16_t Gz);
    void updateFusedData();
    void updatePR();
    double getFusedPitch();
    double getFusedRoll();
    double getFusedYaw();
    double getPitch();
    double getRoll();
    double getPitchGyro();
    double getRollGyro();
    double getTempC();
    double getTempF();
    int16_t getAcX();
    int16_t getAcY();
    int16_t getAcZ();
    int16_t getGyX();
    int16_t getGyY();
    int16_t getGyZ();
    float getAcX_g();
    float getAcY_g();
    float getAcZ_g();
    float getAcX_noG();
    float getAcY_noG();
    float getAcZ_noG();
    float getGyX_dg();
    float getGyY_dg();
    float getGyZ_dg();
    float getGyX_rad();
    float getGyY_rad();
    float getGyZ_rad();
    double fAll[6];
    float fPitch, fRoll, fYaw;

  private:
    uint8_t _MPD;
    TwoWire *_wire;
    int16_t Tmp;
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
    float AcX_g, AcY_g, AcZ_g;
    float AcX_noG, AcY_noG, AcZ_noG;
    float GyX_dg, GyY_dg, GyZ_dg;
    float GyX_rad, GyY_rad, GyZ_rad;
    double t, tx, tf, pitch, roll, pitchGyro, rollGyro;
    int16_t AcXcal, AcYcal, AcZcal, Tmpcal, GyXcal, GyYcal, GyZcal;
    float pitchCal, rollCal, yawCal;
    float fgx, fgy, fgz, fax, fay, faz;
    
    float deltat;
    
};

#endif
