/*
  https://github.com/mahowik/BalancingWii/tree/master

*/

#include <Wire.h>

#define MPU_ADDR 0x68

// Motor pins
#define DIR_LEFT  4
#define STEP_LEFT 5
#define DIR_RIGHT 19
#define STEP_RIGHT 18

#define INTERVAL 4000

#define KP 10.0   // proporcional
#define KI 0.0    // integral
#define KD 2.0    // derivativo

#define SETPOINT 0 // target angle (upright)
#define INTEGRAL_MAX 1000
#define DEAD_ZONE 3.5  // zona morta

double angle, gyroMicros;

double lastError = 0;
double integral = 0;
double pidOutput = 0;

unsigned long previousMicros;
unsigned long currentMicros;

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  double xAcc = (int16_t(Wire.read() << 8 | Wire.read())) / 16384.0;
  double yAcc = (int16_t(Wire.read() << 8 | Wire.read())) / 16384.0;
  double zAcc = (int16_t(Wire.read() << 8 | Wire.read())) / 16384.0;

  double deltaGyroTime = (double)(micros() - gyroMicros) / 1000000;
  gyroMicros = micros();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  double xGyro = (int16_t(Wire.read() << 8 | Wire.read())) / 131.0;
  double yGyro = (int16_t(Wire.read() << 8 | Wire.read())) / 131.0;
  double zGyro = (int16_t(Wire.read() << 8 | Wire.read())) / 131.0;

  double angleGyro = yGyro * deltaGyroTime;
  double angleAcc = atan2(-yAcc, zAcc) * RAD_TO_DEG;

  angle = (0.9 * (angle + angleGyro) + 0.1 * (angleAcc)) - SETPOINT;

}

void calculate_PID() {
  double error = -angle;

  if (abs(error) < DEAD_ZONE) {
    pidOutput = 0;
    integral = 0;
    return;
  }

  integral += error;
  if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
  if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

  double derivative = error - lastError;
  lastError = error;

  pidOutput = KP * error + KI * integral + KD * derivative;
  pidOutput *= -0.1;

  /*
  Serial.print("angle = ");
  Serial.print(angle);
  Serial.print(" pidOutput = ");
  Serial.println(pidOutput);
  */
}

void setup() {
  Serial.begin(115200);

  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  pinMode(DIR_LEFT, OUTPUT);
  pinMode(STEP_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  pinMode(STEP_RIGHT, OUTPUT);
}

void loop() {
  readMPU();
  currentMicros = micros();

  if ((currentMicros - previousMicros) >= INTERVAL) {
    previousMicros = currentMicros;

    calculate_PID();

    int motorSpeed = constrain(abs(pidOutput), 0, 800);
    bool direction = (pidOutput > 0);

    // Set direction
    digitalWrite(DIR_RIGHT, direction ? LOW : HIGH);
    digitalWrite(DIR_LEFT, direction ? HIGH : LOW);

    if (motorSpeed > 0) {
      // INVERTED MAP: higher motorSpeed → smaller delay → faster pulses
      int pulseDelay = map(motorSpeed, 0, 800, 870, 200);  // adjust as needed

      // Minimum delay cap (avoid too fast)
      //if (pulseDelay < 200) pulseDelay = 200;

      digitalWrite(STEP_LEFT, HIGH);
      digitalWrite(STEP_RIGHT, HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(STEP_LEFT, LOW);
      digitalWrite(STEP_RIGHT, LOW);
      delayMicroseconds(pulseDelay);
    }
  }
}


