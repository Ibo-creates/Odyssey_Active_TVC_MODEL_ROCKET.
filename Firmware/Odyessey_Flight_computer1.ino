#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <math.h>
#include <Adafruit_BMP280.h>

const int SD_CS       = 10;
const int Servo_X     = 5;
const int Servo_Y     = 6;
const int Servo_CHUTE = 9;

const int BUZZER_PIN  = 3;
const int LED_PIN     = 2;

const int PARACHUTE_LOCKED   = 0;    
const int PARACHUTE_UNLOCKED = 180;  

Servo servoX;
Servo servoY;
Servo servochute;

const float SERVO_X_CENTER   = 90.0f;
const float SERVO_Y_CENTER   = 90.0f;
const float MAX_GIMBAL_ANGLE = 15.0f;

const float SERVO_X_DIRECTION_SIGN = -1.0f; 

Adafruit_BMP280 bmp;

float currentAltitude = 0.0f;
float maxAltitude     = 0.0f;
float groundAltitude  = 0.0f;

const uint8_t MPU_ADDR = 0x68;

float accX = 0.0f, accY = 0.0f, accZ = 0.0f;
float totalGForce = 1.0f;
float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;
float gyroX_offset = 0.0f, gyroY_offset = 0.0f, gyroZ_offset = 0.0f;

const float GYRO_FILTER_ALPHA = 0.20f;
float filteredGyroX = 0.0f, filteredGyroY = 0.0f, filteredGyroZ = 0.0f;

float pitch = 0.0f;
float yaw   = 0.0f;
const float YAW_DIRECTION_SIGN = 1.0f;

float targetPitch = 0.0f;
float targetYaw   = 0.0f;
const float YAW_FOLLOW_ALPHA = 0.05f; 

float Kp_pitch = 1.2f, Ki_pitch = 0.0f, Kd_pitch = 0.05f;
float Kp_yaw   = 2.0f, Ki_yaw   = 0.0f, Kd_yaw   = 0.10f;

float pitchIntegral = 0.0f, yawIntegral = 0.0f;
float previousPitchError = 0.0f, previousYawError = 0.0f;

float pitchOutput = 0.0f;
float yawOutput   = 0.0f;

enum Flightstate {
  PAD,
  IGNITION,
  COAST,
  APOGEE,
  LANDED
};

Flightstate currentstate = PAD;

uint32_t launchTime      = 0;
uint32_t burnoutTime     = 0;
uint32_t lastLoopTime    = 0;
const uint32_t LOOP_INTERVAL_US = 10000;

const uint32_t MIN_IGNITION_TIME_MS = 1500; 

File logFile;
bool sdInitialized = false;

void initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00); 
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00); 
  Wire.endTransmission();
}

bool readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;

  int16_t rawAccX  = (Wire.read() << 8) | Wire.read();
  int16_t rawAccY  = (Wire.read() << 8) | Wire.read();
  int16_t rawAccZ  = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t rawGyroX = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroY = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroZ = (Wire.read() << 8) | Wire.read();

  accX = rawAccX / 16384.0f;
  accY = rawAccY / 16384.0f;
  accZ = rawAccZ / 16384.0f;

  totalGForce = sqrt(accX * accX + accY * accY + accZ * accZ);

  gyroX = (rawGyroX / 131.0f) - gyroX_offset;
  gyroY = (rawGyroY / 131.0f) - gyroY_offset;
  gyroZ = (rawGyroZ / 131.0f) - gyroZ_offset;

  filteredGyroX += GYRO_FILTER_ALPHA * (gyroX - filteredGyroX);
  filteredGyroY += GYRO_FILTER_ALPHA * (gyroY - filteredGyroY);
  filteredGyroZ += GYRO_FILTER_ALPHA * (gyroZ - filteredGyroZ);

  gyroX = filteredGyroX;
  gyroY = filteredGyroY;
  gyroZ = filteredGyroZ;

  return true;
}

void calibrateGyro() {
  Serial.println(F("CALIBRATING GYRO... KEEP STILL"));
  delay(1000);
  float sumX = 0, sumY = 0, sumZ = 0;
  int valid = 0;

  for (int i = 0; i < 500; i++) {
    if (readMPU6050()) {
      sumX += gyroX; sumY += gyroY; sumZ += gyroZ;
      valid++;
    }
    delay(3);
  }

  if (valid > 0) {
    gyroX_offset = sumX / valid;
    gyroY_offset = sumY / valid;
    gyroZ_offset = sumZ / valid;
  }
}

void updateAttitude(float dt) {
  float accelPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0f / M_PI;
  pitch = 0.98f * (pitch + gyroX * dt) + 0.02f * accelPitch;

  yaw += (gyroZ * YAW_DIRECTION_SIGN) * dt;

  targetYaw += YAW_FOLLOW_ALPHA * (yaw - targetYaw);

  if (yaw > 180.0f)  yaw -= 360.0f;
  if (yaw < -180.0f) yaw += 360.0f;
}

void updateGimbal(float dt) {
  float pitchError = targetPitch - pitch;
  pitchOutput = (Kp_pitch * pitchError) + (Kd_pitch * (pitchError - previousPitchError) / dt);
  previousPitchError = pitchError;

  float yawError = targetYaw - yaw;
  if (yawError > 180.0f)  yawError -= 360.0f;
  if (yawError < -180.0f) yawError += 360.0f;

  yawOutput = (Kp_yaw * yawError) + (Kd_yaw * (yawError - previousYawError) / dt);
  previousYawError = yawError;

  float rawServoX = SERVO_X_CENTER + (SERVO_X_DIRECTION_SIGN * pitchOutput);
  float servoXAngle = constrain(rawServoX, SERVO_X_CENTER - MAX_GIMBAL_ANGLE, SERVO_X_CENTER + MAX_GIMBAL_ANGLE);

  float rawServoY = SERVO_Y_CENTER + yawOutput;
  float servoYAngle = constrain(rawServoY, SERVO_Y_CENTER - MAX_GIMBAL_ANGLE, SERVO_Y_CENTER + MAX_GIMBAL_ANGLE);

  servoX.write((int)servoXAngle);
  servoY.write((int)servoYAngle);
}

void initSDCard() {
  Serial.print(F("Initializing SD card..."));
  
  if (!SD.begin(SD_CS)) {
    Serial.println(F("SD Initialization Failed!"));
    sdInitialized = false;
    return;
  }

  sdInitialized = true;
  Serial.println(F("SD Card Ready."));

  logFile = SD.open("log.csv", FILE_WRITE);
  if (logFile) {
    logFile.println(F("Time_ms,State,Pitch,Yaw,Altitude,GForce,OutputPitch,OutputYaw"));
    logFile.flush();
  }
}

void logTelemetry() {
  if (!sdInitialized || !logFile) return;

  logFile.print(millis());
  logFile.print(F(","));
  logFile.print(currentstate);
  logFile.print(F(","));
  logFile.print(pitch);
  logFile.print(F(","));
  logFile.print(yaw);
  logFile.print(F(","));
  logFile.print(currentAltitude);
  logFile.print(F(","));
  logFile.print(totalGForce);
  logFile.print(F(","));
  logFile.print(pitchOutput);
  logFile.print(F(","));
  logFile.println(yawOutput);

  static uint32_t lastFlush = 0;
  if (millis() - lastFlush > 1000) {
    lastFlush = millis();
    logFile.flush();
  }
}

void updateIndicators() {
  switch (currentstate) {
    case PAD:
      digitalWrite(LED_PIN, (millis() / 500) % 2);
      noTone(BUZZER_PIN);
      break;

    case IGNITION:
      digitalWrite(LED_PIN, (millis() / 100) % 2);
      tone(BUZZER_PIN, 2400); 
      break;

    case COAST:
      digitalWrite(LED_PIN, (millis() / 100) % 2);
      if ((millis() / 150) % 2) {
        tone(BUZZER_PIN, 1500);
      } else {
        noTone(BUZZER_PIN);
      }
      break;

    case APOGEE:
      digitalWrite(LED_PIN, HIGH);
      tone(BUZZER_PIN, 2200); 
      break;

    case LANDED:
      digitalWrite(LED_PIN, HIGH);
      if ((millis() / 1000) % 2) {
        tone(BUZZER_PIN, 1000);
      } else {
        noTone(BUZZER_PIN);
      }
      break;
  }
}

void updateStateMachine() {
  float rawAlt = bmp.readAltitude(1013.25) - groundAltitude;
  
  if (currentstate == PAD && rawAlt < 0.10f) {
    currentAltitude = 0.0f;
  } else {
    currentAltitude = rawAlt;
  }

  if (currentAltitude > maxAltitude) {
    maxAltitude = currentAltitude;
  }

  switch (currentstate) {
    case PAD:
      servochute.write(PARACHUTE_LOCKED);
      if (totalGForce > 2.0f) {
        currentstate = IGNITION;
        launchTime = millis();
        Serial.println(F("IGNITION DETECTED, LIFTOFF!"));
      }
      break;

    case IGNITION:
      servochute.write(PARACHUTE_LOCKED);
      if (totalGForce < 0.8f && (millis() - launchTime > MIN_IGNITION_TIME_MS)) {
        currentstate = COAST;
        burnoutTime = millis();
        Serial.println(F("BURNOUT DETECTED, COASTING..."));
      }
      break;

    case COAST:
      servochute.write(PARACHUTE_LOCKED);
      {
        bool altitudeApogee = (maxAltitude > 0.5f) && (currentAltitude <= (maxAltitude - 0.3f));
        bool timerApogee    = (millis() - burnoutTime > 2500);

        if (altitudeApogee || timerApogee) {
          currentstate = APOGEE;
          Serial.println(F("SINGLE APOGEE DETECTED -> DEPLOYING PARACHUTE IMMEDIATELY!"));
          Serial.print(F("MAX ALTITUDE: "));
          Serial.println(maxAltitude);
        }
      }
      break;

    case APOGEE:
      servochute.write(PARACHUTE_UNLOCKED);

      static uint32_t chuteDeployTime = 0;
      if (chuteDeployTime == 0) chuteDeployTime = millis();

      if (millis() - chuteDeployTime > 2000) {
        currentstate = LANDED;
        Serial.println(F("ROCKET LANDED"));
      }
      break;

    case LANDED:
      servochute.write(PARACHUTE_UNLOCKED);
      break;
  }
}

void processParachuteOutput() {
  if (currentstate == APOGEE || currentstate == LANDED) {
    servochute.write(PARACHUTE_UNLOCKED);
  } else {
    servochute.write(PARACHUTE_LOCKED);
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin();

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  if (bmp.begin(0x76) || bmp.begin(0x77)) {
    Serial.println(F("BMP280 Initialized."));
  } else {
    Serial.println(F("BMP280 Init Failed!"));
  }

  delay(200);
  float altSum = 0;
  for (int i = 0; i < 50; i++) {
    altSum += bmp.readAltitude(1013.25);
    delay(20);
  }
  groundAltitude = altSum / 50.0f;
  currentAltitude = 0.0f;
  maxAltitude = 0.0f;

  initMPU6050();
  calibrateGyro();
  initSDCard();

  servoX.attach(Servo_X);
  servoY.attach(Servo_Y);
  servochute.attach(Servo_CHUTE);

  servoX.write((int)SERVO_X_CENTER);
  servoY.write((int)SERVO_Y_CENTER);
  servochute.write(PARACHUTE_LOCKED);

  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 2000);
    digitalWrite(LED_PIN, HIGH);
    delay(80);
    noTone(BUZZER_PIN);
    digitalWrite(LED_PIN, LOW);
    delay(80);
  }

  lastLoopTime = micros();
}

void loop() {
  uint32_t now = micros();
  uint32_t elapsed = now - lastLoopTime;

  if (elapsed >= LOOP_INTERVAL_US) {
    lastLoopTime = now;
    float dt = elapsed / 1000000.0f;

    if (dt > 0.0f && dt < 0.1f && readMPU6050()) {
      updateAttitude(dt);
      updateGimbal(dt);
      updateStateMachine();
      processParachuteOutput();
      updateIndicators();
      logTelemetry();
    }

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      Serial.print(F("State: ")); Serial.print(currentstate);
      Serial.print(F(" | Pitch: ")); Serial.print(pitch);
      Serial.print(F(" | Yaw: ")); Serial.print(yaw);
      Serial.print(F(" | G-Force: ")); Serial.print(totalGForce);
      Serial.print(F(" | Alt: ")); Serial.println(currentAltitude);
    }
  }
}