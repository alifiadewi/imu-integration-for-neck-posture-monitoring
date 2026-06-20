#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <Arduino.h>
#include <Wire.h>

// Sensor variables
extern float RateRoll, RatePitch, RateYaw;
extern float AccX, AccY, AccZ;
extern float AngleRoll, AnglePitch;
extern float Kalman1DOutput[];

// Function
void gyro_signals(void);
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
String classify_pitch(float pitch);
String classify_roll(float roll);

#endif
