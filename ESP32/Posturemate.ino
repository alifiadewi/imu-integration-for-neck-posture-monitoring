#define BLYNK_TEMPLATE_ID "TMPL6GowfxVjY"
#define BLYNK_TEMPLATE_NAME "POSTUREMATE"
#define BLYNK_AUTH_TOKEN "5jBrz1NWNbvfwFuajO-OjMzULHhbnas5"

#include <Wire.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "IMU_Filter.h"

char ssid[] = "...";
char pass[] = "...";

// Variables Instantiation
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4.0;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4.0;
float Kalman1DOutput[] = {0, 0};

const int vibrator1 = 5;
const int vibrator2 = 18;
const int vibrator3 = 19;

#define VPITCH_ANGLE V0
#define VROLL_ANGLE V1
#define VSTATUS_PITCH V2
#define VSTATUS_ROLL V3

void setup() {
  Serial.begin(57600);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  pinMode(vibrator1, OUTPUT);
  pinMode(vibrator2, OUTPUT);
  pinMode(vibrator3, OUTPUT);
  
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  
  LoopTimer = micros();
}

void loop() {
  Blynk.run();
  gyro_signals();
  
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
  
  String pitchStatus = classify_pitch(KalmanAnglePitch);
  String rollStatus = classify_roll(KalmanAngleRoll);
  
  // Data streaming to Blynk
  Blynk.virtualWrite(VPITCH_ANGLE, KalmanAnglePitch);
  Blynk.virtualWrite(VROLL_ANGLE, KalmanAngleRoll);
  Blynk.virtualWrite(VSTATUS_PITCH, pitchStatus);
  Blynk.virtualWrite(VSTATUS_ROLL, rollStatus);
  
  // Haptic execution feedback
  if (KalmanAnglePitch > 15 && KalmanAnglePitch <= 90) {
    digitalWrite(vibrator1, HIGH);
    delay((KalmanAnglePitch <= 30) ? 1000 : 500);
    digitalWrite(vibrator1, LOW);
  }
  if (KalmanAngleRoll > -45 && KalmanAngleRoll < -5) {
    digitalWrite(vibrator3, HIGH);
    delay((KalmanAngleRoll >= -10) ? 1000 : 500);
    digitalWrite(vibrator3, LOW);
  }
  if (KalmanAngleRoll > 5 && KalmanAngleRoll < 45) {
    digitalWrite(vibrator2, HIGH);
    delay((KalmanAngleRoll <= 10) ? 1000 : 500);
    digitalWrite(vibrator2, LOW);
  }
  
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
