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

#define INTERVAL 3800

// PID 1 (Angle control)
#define KP_ANGLE 10.0
#define KI_ANGLE 0.0
#define KD_ANGLE 2.5

// PID 2 (Motor control smoothing)
#define KP_MOTOR 0.12  // suavização ainda mais lenta
#define KI_MOTOR 0.0
#define KD_MOTOR 0.0

#define SETPOINT 0
#define INTEGRAL_MAX 1000
#define SPEED_STEPS 2450

double angle, gyroMicros;
double lastErrorAngle = 0;
double integralAngle = 0;
double pidAngleOutput = 0;

double lastErrorMotor = 0;
double integralMotor = 0;
double pidMotorOutput = 0;

double smoothedMotorCommandOne = 0;
double smoothedMotorCommandTwo = 0;

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

  // Filtro suavizado
  angle = 0.95 * (angle + angleGyro) + 0.05 * (angleAcc) - SETPOINT;
}

double calculateAnglePID(double input) {
  double error = -input;

  integralAngle += error;
  if (integralAngle > INTEGRAL_MAX) integralAngle = INTEGRAL_MAX;
  if (integralAngle < -INTEGRAL_MAX) integralAngle = -INTEGRAL_MAX;

  double derivative = error - lastErrorAngle;
  lastErrorAngle = error;

  double output = KP_ANGLE * error + KI_ANGLE * integralAngle + KD_ANGLE * derivative;
  output *= -0.1;

  // Limitar saída bruta do primeiro PID
  output = constrain(output, -500, 500);

  return output;
}

double calculateMotorPID(double target, double current) {
  double error = target - current;

  integralMotor += error;
  if (integralMotor > INTEGRAL_MAX) integralMotor = INTEGRAL_MAX;
  if (integralMotor < -INTEGRAL_MAX) integralMotor = -INTEGRAL_MAX;

  double derivative = error - lastErrorMotor;
  lastErrorMotor = error;

  double output = KP_MOTOR * error + KI_MOTOR * integralMotor + KD_MOTOR * derivative;

  return output;
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

    pidAngleOutput = calculateAnglePID(angle);

    // PID motor suavizado em cascata
    double nextCommand = smoothedMotorCommandOne + calculateMotorPID(pidAngleOutput, smoothedMotorCommandOne);

    // Rampagem: limitar mudança brusca
    double maxChange = 10;
    double delta = nextCommand - smoothedMotorCommandTwo;
    if (delta > maxChange) delta = maxChange;
    if (delta < -maxChange) delta = -maxChange;

    smoothedMotorCommandTwo += delta;

    smoothedMotorCommandTwo = constrain(smoothedMotorCommandTwo, -800, 800);

    int motorSpeed = constrain(abs(smoothedMotorCommandTwo), 0, 800);
    bool direction = (smoothedMotorCommandTwo > 0);

    digitalWrite(DIR_RIGHT, direction ? LOW : HIGH);
    digitalWrite(DIR_LEFT, direction ? HIGH : LOW);

    if (motorSpeed > 0) {
      int numSteps = map(motorSpeed, 0, 800, 1, 20);

      for (int i = 0; i < numSteps; i++) {
        digitalWrite(STEP_LEFT, HIGH);
        digitalWrite(STEP_RIGHT, HIGH);
        delayMicroseconds(SPEED_STEPS);
        digitalWrite(STEP_LEFT, LOW);
        digitalWrite(STEP_RIGHT, LOW);
        delayMicroseconds(SPEED_STEPS);
      }
    }
  }
}

