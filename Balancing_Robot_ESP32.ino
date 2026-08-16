/*
  Balancing Robot ESP32 - V22 PD/PID Simples + REST + Drive
  MPU6050 -> Kalman simples -> PD/PID -> PWM -> BTS7960

  Defaults:
    setpoint = 3.00
    Kp = 15.0
    Ki = 0.0
    Kd = 0.12
    PWM_MAX = 180
    FALL_ANGLE_DEG = 35.0
    INTEGRAL_LIMIT = 100.0
    CONTROL_PERIOD_US = 5000

  REST:
    GET  /api/state
    POST /api/config
    POST /api/reset
    GET  /api/health

  Troque WIFI_SSID e WIFI_PASSWORD.
  Fallback AP:
    SSID: STARK_BALANCER
    senha: starkrobot
    IP: 192.168.4.1
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <MPU6050_light.h>
#include <math.h>

const char* WIFI_SSID = "Star_Gate";
const char* WIFI_PASSWORD = "STARgate$321";
const char* AP_SSID = "STARK_BALANCER";
const char* AP_PASSWORD = "starkrobot";

WebServer server(80);

#define MPU_SDA 21
#define MPU_SCL 22
MPU6050 mpu(Wire);

constexpr double ACC_ANGLE_DIRECTION = 1.0;
constexpr double GYRO_DIRECTION = 1.0;
constexpr double ACC_ANGLE_OFFSET_DEG = 0.0;

class KalmanAngle {
public:
  void setAngle(const double initialAngle) { angle = initialAngle; }

  double update(const double measuredAngle, const double measuredRate, const double dt) {
    const double rate = measuredRate - bias;
    angle += dt * rate;

    p00 += dt * (dt * p11 - p01 - p10 + qAngle);
    p01 -= dt * p11;
    p10 -= dt * p11;
    p11 += qBias * dt;

    const double innovation = measuredAngle - angle;
    const double s = p00 + rMeasure;
    const double k0 = p00 / s;
    const double k1 = p10 / s;

    angle += k0 * innovation;
    bias += k1 * innovation;

    const double oldP00 = p00;
    const double oldP01 = p01;

    p00 -= k0 * oldP00;
    p01 -= k0 * oldP01;
    p10 -= k1 * oldP00;
    p11 -= k1 * oldP01;

    return angle;
  }

private:
  double qAngle = 0.001;
  double qBias = 0.003;
  double rMeasure = 0.03;
  double angle = 0.0;
  double bias = 0.0;
  double p00 = 0.0, p01 = 0.0, p10 = 0.0, p11 = 0.0;
};

KalmanAngle kalman;

#define LEFT_RPWM 25
#define LEFT_LPWM 26
#define LEFT_REN  27
#define LEFT_LEN  14
#define RIGHT_RPWM 32
#define RIGHT_LPWM 33
#define RIGHT_REN  16
#define RIGHT_LEN  17

constexpr bool INVERT_LEFT_MOTOR = false;
constexpr bool INVERT_RIGHT_MOTOR = true;

constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION = 8;
constexpr int PWM_ZERO_BAND = 3;

constexpr int PWM_MIN_LEFT_FORWARD  = 18;
constexpr int PWM_MIN_LEFT_REVERSE  = 18;
constexpr int PWM_MIN_RIGHT_FORWARD = 18;
constexpr int PWM_MIN_RIGHT_REVERSE = 18;

constexpr double DEFAULT_SETPOINT = -1.50;
constexpr double DEFAULT_KP = 15.0;
constexpr double DEFAULT_KI = 0.0;
constexpr double DEFAULT_KD = 0.13;
constexpr int DEFAULT_PWM_MAX = 180;
constexpr double DEFAULT_FALL_ANGLE_DEG = 35.0;
constexpr double DEFAULT_INTEGRAL_LIMIT = 100.0;
constexpr uint32_t DEFAULT_CONTROL_PERIOD_US = 5000;

double setpoint = DEFAULT_SETPOINT;
double Kp = DEFAULT_KP;
double Ki = DEFAULT_KI;
double Kd = DEFAULT_KD;
int PWM_MAX = DEFAULT_PWM_MAX;
double FALL_ANGLE_DEG = DEFAULT_FALL_ANGLE_DEG;
double INTEGRAL_LIMIT = DEFAULT_INTEGRAL_LIMIT;
uint32_t CONTROL_PERIOD_US = DEFAULT_CONTROL_PERIOD_US;

constexpr double MIN_SETPOINT = -15.0, MAX_SETPOINT = 15.0;
constexpr double MIN_KP = 0.0, MAX_KP = 100.0;
constexpr double MIN_KI = 0.0, MAX_KI = 20.0;
constexpr double MIN_KD = 0.0, MAX_KD = 5.0;
constexpr int MIN_PWM_MAX = 0, MAX_PWM_MAX = 255;
constexpr double MIN_FALL_ANGLE = 5.0, MAX_FALL_ANGLE = 80.0;
constexpr double MIN_INTEGRAL_LIMIT = 0.0, MAX_INTEGRAL_LIMIT = 1000.0;
constexpr uint32_t MIN_CONTROL_PERIOD_US = 2000;
constexpr uint32_t MAX_CONTROL_PERIOD_US = 20000;

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

double telemetryAccAngle = 0.0;
double telemetryAngle = 0.0;
double telemetryGyro = 0.0;
double telemetryError = 0.0;
double telemetryPTerm = 0.0;
double telemetryITerm = 0.0;
double telemetryDTerm = 0.0;
double telemetryPidOutput = 0.0;
double telemetryLoopHz = 0.0;
int telemetryPwm = 0;
bool telemetryFallen = false;

double integral = 0.0;
TaskHandle_t controlTaskHandle = nullptr;

enum DriveCommand { DRIVE_STOP=0, DRIVE_FORWARD, DRIVE_BACKWARD, DRIVE_LEFT, DRIVE_RIGHT };
volatile DriveCommand driveCommand = DRIVE_STOP;
volatile uint32_t lastDriveCommandMs = 0;
double MOVE_OFFSET_DEG = 0.30;
int TURN_PWM = 25;
constexpr uint32_t DRIVE_TIMEOUT_MS = 300;

double calculateAccelerometerAngle(const double ax, const double ay, const double az) {
  (void)ax;
  return atan2(ay, az) * RAD_TO_DEG * ACC_ANGLE_DIRECTION + ACC_ANGLE_OFFSET_DEG;
}

int compensateMotorDeadZone(int command, int minForward, int minReverse, int pwmMax) {
  command = constrain(command, -pwmMax, pwmMax);
  const int magnitude = abs(command);
  if (magnitude <= PWM_ZERO_BAND) return 0;
  if (command > 0 && magnitude < minForward) return minForward;
  if (command < 0 && magnitude < minReverse) return -minReverse;
  return command;
}

void setMotor(int rpwmPin, int lpwmPin, int command, bool invert, int minForward, int minReverse, int pwmMax) {
  command = constrain(command, -pwmMax, pwmMax);
  if (invert) command = -command;
  command = compensateMotorDeadZone(command, minForward, minReverse, pwmMax);

  if (command > 0) {
    ledcWrite(rpwmPin, command);
    ledcWrite(lpwmPin, 0);
  } else if (command < 0) {
    ledcWrite(rpwmPin, 0);
    ledcWrite(lpwmPin, -command);
  } else {
    ledcWrite(rpwmPin, 0);
    ledcWrite(lpwmPin, 0);
  }
}

void stopMotors() {
  ledcWrite(LEFT_RPWM, 0);
  ledcWrite(LEFT_LPWM, 0);
  ledcWrite(RIGHT_RPWM, 0);
  ledcWrite(RIGHT_LPWM, 0);
}

void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void sendJson(int code, const String& body) {
  addCorsHeaders();
  server.send(code, "application/json", body);
}

void handleOptions() {
  addCorsHeaders();
  server.send(204);
}

void handleState() {
  double accAngle, angle, gyro, err, p, i, d, pid, loopHz;
  int pwm;
  bool fallen;
  double sp, kp, ki, kd, fallAngle, integralLimit;
  int pwmMax;
  uint32_t periodUs;

  portENTER_CRITICAL(&stateMux);
  accAngle = telemetryAccAngle;
  angle = telemetryAngle;
  gyro = telemetryGyro;
  err = telemetryError;
  p = telemetryPTerm;
  i = telemetryITerm;
  d = telemetryDTerm;
  pid = telemetryPidOutput;
  pwm = telemetryPwm;
  fallen = telemetryFallen;
  loopHz = telemetryLoopHz;

  sp = setpoint; kp = Kp; ki = Ki; kd = Kd;
  pwmMax = PWM_MAX;
  fallAngle = FALL_ANGLE_DEG;
  integralLimit = INTEGRAL_LIMIT;
  periodUs = CONTROL_PERIOD_US;
  portEXIT_CRITICAL(&stateMux);

  String json = "{";
  json += "\"telemetry\":{";
  json += "\"accAngle\":" + String(accAngle,4) + ",";
  json += "\"angle\":" + String(angle,4) + ",";
  json += "\"gyro\":" + String(gyro,4) + ",";
  json += "\"error\":" + String(err,4) + ",";
  json += "\"p\":" + String(p,4) + ",";
  json += "\"i\":" + String(i,4) + ",";
  json += "\"d\":" + String(d,4) + ",";
  json += "\"pid\":" + String(pid,4) + ",";
  json += "\"pwm\":" + String(pwm) + ",";
  json += "\"fallen\":" + String(fallen ? "true" : "false") + ",";
  json += "\"loopHz\":" + String(loopHz,1);
  json += "},\"config\":{";
  json += "\"setpoint\":" + String(sp,4) + ",";
  json += "\"kp\":" + String(kp,4) + ",";
  json += "\"ki\":" + String(ki,4) + ",";
  json += "\"kd\":" + String(kd,4) + ",";
  json += "\"pwmMax\":" + String(pwmMax) + ",";
  json += "\"fallAngle\":" + String(fallAngle,4) + ",";
  json += "\"integralLimit\":" + String(integralLimit,4) + ",";
  json += "\"controlPeriodUs\":" + String(periodUs) + ",";
  json += "\"moveOffset\":" + String(MOVE_OFFSET_DEG,4) + ",";
  json += "\"turnPwm\":" + String(TURN_PWM);
  json += "}}";

  sendJson(200, json);
}

void handleConfig() {
  // Parseia antes da secao critica.
  const bool hasSetpoint = server.hasArg("setpoint");
  const bool hasKp = server.hasArg("kp");
  const bool hasKi = server.hasArg("ki");
  const bool hasKd = server.hasArg("kd");
  const bool hasPwmMax = server.hasArg("pwmMax");
  const bool hasFallAngle = server.hasArg("fallAngle");
  const bool hasIntegralLimit = server.hasArg("integralLimit");
  const bool hasPeriod = server.hasArg("controlPeriodUs");

  const double newSetpoint = hasSetpoint ? constrain(server.arg("setpoint").toDouble(), MIN_SETPOINT, MAX_SETPOINT) : 0;
  const double newKp = hasKp ? constrain(server.arg("kp").toDouble(), MIN_KP, MAX_KP) : 0;
  const double newKi = hasKi ? constrain(server.arg("ki").toDouble(), MIN_KI, MAX_KI) : 0;
  const double newKd = hasKd ? constrain(server.arg("kd").toDouble(), MIN_KD, MAX_KD) : 0;
  const int newPwmMax = hasPwmMax ? constrain(server.arg("pwmMax").toInt(), MIN_PWM_MAX, MAX_PWM_MAX) : 0;
  const double newFallAngle = hasFallAngle ? constrain(server.arg("fallAngle").toDouble(), MIN_FALL_ANGLE, MAX_FALL_ANGLE) : 0;
  const double newIntegralLimit = hasIntegralLimit ? constrain(server.arg("integralLimit").toDouble(), MIN_INTEGRAL_LIMIT, MAX_INTEGRAL_LIMIT) : 0;
  const uint32_t newPeriod = hasPeriod
    ? (uint32_t)constrain(server.arg("controlPeriodUs").toInt(), (int)MIN_CONTROL_PERIOD_US, (int)MAX_CONTROL_PERIOD_US)
    : 0;

  portENTER_CRITICAL(&stateMux);
  if (hasSetpoint) setpoint = newSetpoint;
  if (hasKp) Kp = newKp;
  if (hasKi) Ki = newKi;
  if (hasKd) Kd = newKd;
  if (hasPwmMax) PWM_MAX = newPwmMax;
  if (hasFallAngle) FALL_ANGLE_DEG = newFallAngle;
  if (hasIntegralLimit) INTEGRAL_LIMIT = newIntegralLimit;
  if (hasPeriod) CONTROL_PERIOD_US = newPeriod;
  portEXIT_CRITICAL(&stateMux);

  sendJson(200, "{\"ok\":true}");
}


void handleDrive() {
  if (!server.hasArg("command")) { sendJson(400, "{\"ok\":false}"); return; }
  String c=server.arg("command"); c.toLowerCase();
  DriveCommand cmd=DRIVE_STOP;
  if(c=="forward") cmd=DRIVE_FORWARD; else if(c=="backward") cmd=DRIVE_BACKWARD;
  else if(c=="left") cmd=DRIVE_LEFT; else if(c=="right") cmd=DRIVE_RIGHT;
  else if(c!="stop") { sendJson(400, "{\"ok\":false}"); return; }
  double mo = server.hasArg("moveOffset") ? constrain(server.arg("moveOffset").toDouble(),0.0,5.0) : MOVE_OFFSET_DEG;
  int tp = server.hasArg("turnPwm") ? constrain(server.arg("turnPwm").toInt(),0,100) : TURN_PWM;
  portENTER_CRITICAL(&stateMux); driveCommand=cmd; lastDriveCommandMs=millis(); MOVE_OFFSET_DEG=mo; TURN_PWM=tp; portEXIT_CRITICAL(&stateMux);
  sendJson(200, "{\"ok\":true}");
}

void handleReset() {
  portENTER_CRITICAL(&stateMux);
  setpoint = DEFAULT_SETPOINT;
  Kp = DEFAULT_KP;
  Ki = DEFAULT_KI;
  Kd = DEFAULT_KD;
  PWM_MAX = DEFAULT_PWM_MAX;
  FALL_ANGLE_DEG = DEFAULT_FALL_ANGLE_DEG;
  INTEGRAL_LIMIT = DEFAULT_INTEGRAL_LIMIT;
  CONTROL_PERIOD_US = DEFAULT_CONTROL_PERIOD_US;
  MOVE_OFFSET_DEG = 0.30; TURN_PWM = 25; driveCommand = DRIVE_STOP;
  integral = 0.0;
  portEXIT_CRITICAL(&stateMux);

  sendJson(200, "{\"ok\":true,\"reset\":true}");
}

void handleHealth() {
  sendJson(200, "{\"ok\":true,\"name\":\"STARK_BALANCER\"}");
}

void startNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Conectando ao Wi-Fi");
  const uint32_t start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < 12000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP do ESP32: http://");
    Serial.println(WiFi.localIP());
    return;
  }

  Serial.println("Wi-Fi falhou. Iniciando AP...");
  WiFi.disconnect(true);
  delay(200);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("IP do ESP32: http://");
  Serial.println(WiFi.softAPIP());
}

void controlTask(void* parameter) {
  uint32_t previousMicros = micros();
  uint32_t nextCycle = previousMicros;

  while (true) {
    double localSetpoint, localKp, localKi, localKd, localFallAngle, localIntegralLimit;
    int localPwmMax;
    uint32_t localPeriodUs;

    portENTER_CRITICAL(&stateMux);
    localSetpoint = setpoint;
    localKp = Kp;
    localKi = Ki;
    localKd = Kd;
    localPwmMax = PWM_MAX;
    localFallAngle = FALL_ANGLE_DEG;
    localIntegralLimit = INTEGRAL_LIMIT;
    localPeriodUs = CONTROL_PERIOD_US;
    portEXIT_CRITICAL(&stateMux);

    const uint32_t now = micros();
    if ((int32_t)(now - nextCycle) < 0) {
      delayMicroseconds(100);
      continue;
    }

    nextCycle = now + localPeriodUs;

    const double dt = constrain((now - previousMicros) * 0.000001, 0.001, 0.050);
    previousMicros = now;

    mpu.update();

    const double ax = mpu.getAccX();
    const double ay = mpu.getAccY();
    const double az = mpu.getAccZ();
    const double accAngle = calculateAccelerometerAngle(ax, ay, az);
    const double gyroRate = mpu.getGyroX() * GYRO_DIRECTION;
    const double angle = kalman.update(accAngle, gyroRate, dt);

    DriveCommand localDrive; uint32_t localLast; double localMove; int localTurn;
    portENTER_CRITICAL(&stateMux); localDrive=driveCommand; localLast=lastDriveCommandMs; localMove=MOVE_OFFSET_DEG; localTurn=TURN_PWM; portEXIT_CRITICAL(&stateMux);
    if(localDrive!=DRIVE_STOP && millis()-localLast>DRIVE_TIMEOUT_MS){ localDrive=DRIVE_STOP; portENTER_CRITICAL(&stateMux); driveCommand=DRIVE_STOP; portEXIT_CRITICAL(&stateMux); }
    double commandedSetpoint=localSetpoint; int turnMix=0;
    if(localDrive==DRIVE_FORWARD) commandedSetpoint=localSetpoint+localMove;
    else if(localDrive==DRIVE_BACKWARD) commandedSetpoint=localSetpoint-localMove;
    else if(localDrive==DRIVE_LEFT) turnMix=-localTurn;
    else if(localDrive==DRIVE_RIGHT) turnMix=localTurn;

    double error = 0, pTerm = 0, iTerm = 0, dTerm = 0, pidOutput = 0;
    int pwm = 0;
    bool fallen = false;

    if (fabs(angle - commandedSetpoint) > localFallAngle) {
      fallen = true;
      integral = 0.0;
      stopMotors();
    } else {
      error = commandedSetpoint - angle;
      pTerm = localKp * error;

      integral += error * dt;
      integral = constrain(integral, -localIntegralLimit, localIntegralLimit);
      iTerm = localKi * integral;

      dTerm = -localKd * gyroRate;
      pidOutput = constrain(pTerm + iTerm + dTerm, -(double)localPwmMax, (double)localPwmMax);
      pwm = (int)lround(pidOutput);

      const int leftCommand=constrain(pwm-turnMix,-localPwmMax,localPwmMax);
      const int rightCommand=constrain(pwm+turnMix,-localPwmMax,localPwmMax);
      setMotor(LEFT_RPWM, LEFT_LPWM, leftCommand, INVERT_LEFT_MOTOR, PWM_MIN_LEFT_FORWARD, PWM_MIN_LEFT_REVERSE, localPwmMax);
      setMotor(RIGHT_RPWM, RIGHT_LPWM, rightCommand, INVERT_RIGHT_MOTOR, PWM_MIN_RIGHT_FORWARD, PWM_MIN_RIGHT_REVERSE, localPwmMax);
    }

    const double loopHz = dt > 0 ? 1.0 / dt : 0.0;

    portENTER_CRITICAL(&stateMux);
    telemetryAccAngle = accAngle;
    telemetryAngle = angle;
    telemetryGyro = gyroRate;
    telemetryError = error;
    telemetryPTerm = pTerm;
    telemetryITerm = iTerm;
    telemetryDTerm = dTerm;
    telemetryPidOutput = pidOutput;
    telemetryPwm = pwm;
    telemetryFallen = fallen;
    telemetryLoopHz = loopHz;
    portEXIT_CRITICAL(&stateMux);
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);

  const byte status = mpu.begin();
  if (status != 0) {
    Serial.print("Falha MPU6050. Codigo: ");
    Serial.println(status);
    while (true) delay(1000);
  }

  Serial.println("V22 - PD/PID SIMPLES + REST + HOLD-TO-DRIVE");
  Serial.println("Nao mova o robo durante a calibracao.");

  delay(1000);
  mpu.calcOffsets(true, true);

  for (int i = 0; i < 100; i++) {
    mpu.update();
    delay(5);
  }

  const double initialAngle =
      calculateAccelerometerAngle(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ());
  kalman.setAngle(initialAngle);

  pinMode(LEFT_REN, OUTPUT);
  pinMode(LEFT_LEN, OUTPUT);
  pinMode(RIGHT_REN, OUTPUT);
  pinMode(RIGHT_LEN, OUTPUT);

  digitalWrite(LEFT_REN, HIGH);
  digitalWrite(LEFT_LEN, HIGH);
  digitalWrite(RIGHT_REN, HIGH);
  digitalWrite(RIGHT_LEN, HIGH);

  const bool pwmOk =
      ledcAttach(LEFT_RPWM, PWM_FREQUENCY, PWM_RESOLUTION) &&
      ledcAttach(LEFT_LPWM, PWM_FREQUENCY, PWM_RESOLUTION) &&
      ledcAttach(RIGHT_RPWM, PWM_FREQUENCY, PWM_RESOLUTION) &&
      ledcAttach(RIGHT_LPWM, PWM_FREQUENCY, PWM_RESOLUTION);

  if (!pwmOk) {
    Serial.println("Erro ao configurar PWM.");
    while (true) delay(1000);
  }

  stopMotors();
  startNetwork();

  server.on("/api/health", HTTP_GET, handleHealth);
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.on("/api/reset", HTTP_POST, handleReset);
  server.on("/api/drive", HTTP_POST, handleDrive);

  server.on("/api/health", HTTP_OPTIONS, handleOptions);
  server.on("/api/state", HTTP_OPTIONS, handleOptions);
  server.on("/api/config", HTTP_OPTIONS, handleOptions);
  server.on("/api/reset", HTTP_OPTIONS, handleOptions);
  server.on("/api/drive", HTTP_OPTIONS, handleOptions);

  server.begin();

  const BaseType_t created = xTaskCreatePinnedToCore(
    controlTask, "BalanceControl", 8192, nullptr, 3, &controlTaskHandle, 0
  );

  if (created != pdPASS) {
    Serial.println("Erro ao criar task de controle.");
    while (true) {
      stopMotors();
      delay(1000);
    }
  }

  Serial.println("REST pronta.");
  Serial.println("Defaults: SP=-1.50 KP=15 KI=0 KD=0.13 PWM_MAX=180 FALL=35 LIMIT=100 PERIOD=5000");
}

void loop() {
  server.handleClient();
  delay(1);
}

