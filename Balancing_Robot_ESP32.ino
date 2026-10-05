/*
  Balancing Robot ESP32 - V36 PD/PID Simples + REST + Drive + Encoders
  MPU6050 -> Kalman simples -> PD/PID -> PWM -> BTS7960

  Defaults:
    setpoint base = -1.50 (somente interno; painel mostra leitura em tempo real)
    Kp = 15.0
    Ki = 0.0
    Kd = 0.12
    PWM_MAX = 210
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
#include <Preferences.h>

const char* WIFI_SSID = "Star_Gate";
const char* WIFI_PASSWORD = "STARgate$321";
const char* AP_SSID = "STARK_BALANCER";
const char* AP_PASSWORD = "starkrobot";

WebServer server(80);
Preferences preferences;

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

// Encoders Hall em quadratura
// Esquerdo: verde=A -> GPIO18 | amarelo=B -> GPIO19
// Direito : verde=A -> GPIO34 | amarelo=B -> GPIO35
#define LEFT_ENC_A   18
#define LEFT_ENC_B   19
#define RIGHT_ENC_A  34
#define RIGHT_ENC_B  35

// GPIO34/35 sao somente entrada e nao possuem pull-up interno.
// O codigo usa INPUT (nao INPUT_PULLUP).
volatile int32_t leftEncoderTicks = 0;
volatile int32_t rightEncoderTicks = 0;

volatile uint8_t leftEncoderState = 0;
volatile uint8_t rightEncoderState = 0;

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// Tabela de decodificacao quadratura x4.
// Indice = estado_anterior(2 bits) << 2 | estado_atual(2 bits)
static const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR handleLeftEncoder() {
  const uint8_t current = (digitalRead(LEFT_ENC_A) << 1) | digitalRead(LEFT_ENC_B);
  const uint8_t index = (leftEncoderState << 2) | current;
  portENTER_CRITICAL_ISR(&encoderMux);
  leftEncoderTicks += QUAD_TABLE[index];
  leftEncoderState = current;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR handleRightEncoder() {
  const uint8_t current = (digitalRead(RIGHT_ENC_A) << 1) | digitalRead(RIGHT_ENC_B);
  const uint8_t index = (rightEncoderState << 2) | current;
  portENTER_CRITICAL_ISR(&encoderMux);
  rightEncoderTicks += QUAD_TABLE[index];
  rightEncoderState = current;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

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
constexpr double DEFAULT_KP = 16.0;
constexpr double DEFAULT_KI = 0.0;
constexpr double DEFAULT_KD = 0.22;
constexpr int DEFAULT_PWM_MAX = 210;

// V36 - RECOVERY BOOST
// V26 permanece identica ate 2 graus de erro.
// Acima disso, os ganhos e o limite de PWM crescem progressivamente.
constexpr double RECOVERY_START_ERROR_DEG = 2.0;
constexpr double RECOVERY_FULL_ERROR_DEG = 7.0;
constexpr double RECOVERY_KP_MULT_MAX = 1.55;
constexpr double RECOVERY_KD_MULT_MAX = 1.20;
constexpr int RECOVERY_PWM_MAX = 235;

// V36 - antecipacao pelo giroscopio.
// O gyro detecta o empurrao antes de o angulo fugir muito.
constexpr double RECOVERY_START_ERROR_DEG_V36 = 1.35;
constexpr double GYRO_RECOVERY_START_DPS = 8.0;
constexpr double GYRO_RECOVERY_FULL_DPS = 45.0;
constexpr double RECOVERY_KP_MULT_MAX_V36 = 1.75;
constexpr double RECOVERY_KD_MULT_MAX_V36 = 1.32;
constexpr int RECOVERY_PWM_MAX_V36 = 245;

// V36 - ADAPTIVE NEUTRAL + FAST REARM
double neutralTrim = 0.0;
constexpr double NEUTRAL_TRIM_LIMIT_DEG = 1.20;
constexpr double PENDULUM_ACTIVE_WINDOW_DEG = 4.0;
constexpr double PENDULUM_MAX_GYRO_DPS = 45.0;
constexpr double PENDULUM_ZERO_SPEED_TPS = 55.0;
constexpr double PENDULUM_MIN_PEAK_TPS = 110.0;
constexpr double PENDULUM_MAX_PEAK_TPS = 1800.0;
constexpr double PENDULUM_MAX_STEP_DEG = 0.045;
constexpr double PENDULUM_ASYMMETRY_GAIN = 0.000030;
double pendulumPositivePeak=0.0, pendulumNegativePeak=0.0;
bool pendulumHavePositivePeak=false, pendulumHaveNegativePeak=false;
uint32_t pendulumCycles=0;

// V36 - STABLE SETPOINT LEARNING
//
// Regra principal:
// 1) SEARCHING: Position Hold + Pendulum Search podem procurar o centro.
// 2) Se o robo ficar REALMENTE parado e em pe por alguns segundos,
//    o Setpoint Atual e aceito como o setpoint ideal.
// 3) O valor e travado e salvo na NVS.
// 4) Um empurrao nao muda o setpoint.
// 5) Somente deriva persistente de posicao + velocidade reabre a busca.
enum NeutralState : uint8_t { NEUTRAL_SEARCHING=0, NEUTRAL_CONFIRMING=1, NEUTRAL_LOCKED=2 };
NeutralState neutralState = NEUTRAL_SEARCHING;

// Janela para declarar "estou parado em pe".
constexpr uint32_t STABLE_LOCK_TIME_MS = 3000;
constexpr double STABLE_MAX_SPEED_TPS = 55.0;
constexpr double STABLE_MAX_GYRO_DPS = 2.0;
constexpr double STABLE_MAX_ANGLE_ERROR_DEG = 0.30;
constexpr int STABLE_MAX_PWM = 18;

// V36: confiança do setpoint por instabilidade acumulada.
constexpr double INSTABILITY_SPEED_TPS = 65.0;
constexpr double INSTABILITY_POSITION_TICKS = 75.0;
constexpr int INSTABILITY_PWM = 22;
constexpr uint32_t INSTABILITY_SAMPLE_MS = 100;
constexpr uint32_t INSTABILITY_UNLOCK_SCORE_MS = 5500;
constexpr uint32_t INSTABILITY_RECOVERY_MS = 180;

uint32_t stableStartMs = 0;
uint32_t instabilityScoreMs = 0;
uint32_t lastInstabilitySampleMs = 0;

// Valor absoluto aprendido (ex.: -1.43 graus), e nao apenas Neutral Trim.
double learnedSetpoint = -1.50;
bool learnedSetpointValid = false;
bool learnedLoadedFromNvs = false;

constexpr double FAST_REARM_WINDOW_DEG = 5.0;
constexpr double FAST_REARM_MAX_GYRO_DPS = 35.0;
bool wasFallen = false;
constexpr double DEFAULT_FALL_ANGLE_DEG = 35.0;
constexpr double DEFAULT_INTEGRAL_LIMIT = 100.0;
constexpr uint32_t DEFAULT_CONTROL_PERIOD_US = 5000;

double setpoint = DEFAULT_SETPOINT;

// V36 - Malha externa de POSICAO + VELOCIDADE por encoder.
//
// Sinais confirmados no robo:
//   frente: encoder esquerdo NEGATIVO / direito POSITIVO.
//
// A posicao normalizada e a media:
//   leftPosition  = -leftTicks
//   rightPosition = +rightTicks
//   robotPosition = media(leftPosition, rightPosition)
//
// O PD de angulo continua sendo a malha rapida (~200 Hz).
// A malha externa apenas desloca lentamente o setpoint para manter
// o robo perto da posicao onde foi armado.
double autoSetpointOffset = 0.0;

// Referencia de posicao capturada automaticamente quando o robo entra
// pela primeira vez na janela de equilibrio.
double positionTargetTicks = 0.0;
bool positionTargetArmed = false;

void clearPendulumCycle() {
  pendulumPositivePeak = 0.0;
  pendulumNegativePeak = 0.0;
  pendulumHavePositivePeak = false;
  pendulumHaveNegativePeak = false;
}

void saveLearnedSetpoint() {
  preferences.putDouble("learnedSP", learnedSetpoint);
  preferences.putBool("learnedValid", true);
}

void lockCurrentSetpoint(double currentSetpoint) {
  learnedSetpoint = currentSetpoint;
  learnedSetpointValid = true;

  // Mantem compatibilidade com a arquitetura existente:
  // localSetpoint + neutralTrim = learnedSetpoint.
  neutralTrim = learnedSetpoint - setpoint;
  neutralTrim = constrain(neutralTrim,
                          -NEUTRAL_TRIM_LIMIT_DEG,
                          NEUTRAL_TRIM_LIMIT_DEG);

  neutralState = NEUTRAL_LOCKED;
  stableStartMs = 0;
  instabilityScoreMs = 0;
  lastInstabilitySampleMs = millis();
  autoSetpointOffset = 0.0;
  clearPendulumCycle();
  saveLearnedSetpoint();
}

void unlockStableSearch() {
  neutralState = NEUTRAL_SEARCHING;
  stableStartMs = 0;
  instabilityScoreMs = 0;
  lastInstabilitySampleMs = millis();
  clearPendulumCycle();

  // O ultimo setpoint bom e o ponto inicial da nova busca.
  neutralTrim = learnedSetpoint - setpoint;
  neutralTrim = constrain(neutralTrim,
                          -NEUTRAL_TRIM_LIMIT_DEG,
                          NEUTRAL_TRIM_LIMIT_DEG);

  positionTargetArmed = false;
  autoSetpointOffset = 0.0;
}





void saveLockedNeutral() {
  preferences.putDouble("learnedSP", learnedSetpoint);
  preferences.putBool("learnedValid", true);
}

void lockNeutralNow() {
  learnedSetpoint = neutralTrim;
  neutralState = NEUTRAL_LOCKED;
  stableStartMs = 0;
  instabilityScoreMs = 0;
  lastInstabilitySampleMs = millis();
  clearPendulumCycle();
  saveLockedNeutral();
}

void unlockNeutralSearch() {
  neutralState = NEUTRAL_SEARCHING;
  stableStartMs = 0;
  instabilityScoreMs = 0;
  lastInstabilitySampleMs = millis();
  clearPendulumCycle();

  // Novo terreno = nova referencia espacial. O neutro aprendido continua
  // como ponto inicial, mas Position Hold recomeca daqui.
  positionTargetArmed = false;
  autoSetpointOffset = 0.0;
}



// Ganhos iniciais CONSERVADORES da malha externa.
// Saida em graus de correcao do setpoint.
constexpr double POSITION_KP_DEG_PER_TICK = 0.00010;
constexpr double SPEED_KD_DEG_PER_TPS = 0.00008;

constexpr double AUTO_SP_LIMIT_DEG = 0.60;

// So arma/atua perto do equilibrio. Fora daqui, o PD de angulo
// recupera o robo sem interferencia da malha externa.
constexpr double OUTER_LOOP_ARM_WINDOW_DEG = 5.00;
constexpr double OUTER_LOOP_ACTIVE_WINDOW_DEG = 8.00;

// Pequenas zonas mortas evitam cacar ruido mecanico/encoder.
constexpr double POSITION_DEADBAND_TICKS = 25.0;
constexpr double SPEED_DEADBAND_TPS = 60.0;

// Em recuperacao muito rapida, congela a malha externa.
constexpr double OUTER_LOOP_MAX_SPEED_TPS = 4000.0;
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

double telemetrySetpoint = DEFAULT_SETPOINT;
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

// Telemetria dos encoders.
// ticks/s funciona sem conhecermos ainda o CPR/PPR exato do motor.
int32_t telemetryLeftEncoderTicks = 0;
int32_t telemetryRightEncoderTicks = 0;
double telemetryLeftTicksPerSecond = 0.0;
double telemetryRightTicksPerSecond = 0.0;
double telemetryRobotSpeed = 0.0;
double telemetryRobotPosition = 0.0;
double telemetryPositionError = 0.0;
double telemetryAutoSetpointOffset = 0.0;
double telemetryNeutralTrim = 0.0;
double telemetryPendulumPositivePeak = 0.0;
double telemetryPendulumNegativePeak = 0.0;
uint32_t telemetryPendulumCycles = 0;
uint8_t telemetryNeutralState = NEUTRAL_SEARCHING;
double telemetryLockedNeutralTrim = 0.0;
uint32_t telemetryInstabilityMs = 0;
uint32_t telemetryStableMs = 0;
bool telemetryLearnedValid = false;
bool telemetryAutoSetpointLearning = false;

double integral = 0.0;
TaskHandle_t controlTaskHandle = nullptr;

enum DriveCommand { DRIVE_STOP=0, DRIVE_FORWARD, DRIVE_BACKWARD, DRIVE_LEFT, DRIVE_RIGHT };
volatile DriveCommand driveCommand = DRIVE_STOP;
volatile uint32_t lastDriveCommandMs = 0;
double MOVE_OFFSET_DEG = 0.12;
int TURN_PWM = 20;

// V36 - LEAN DRIVE
constexpr double LEAN_RAMP_DEG_PER_SEC = 0.12;
constexpr double LEAN_MAX_DEG = 0.14;
double driveLeanDeg = 0.0;

// V36 - BALANCE PRIORITY STEERING
// Giro entra e sai em rampa. Quanto maior o erro angular, menor a autoridade
// de giro. Se o robo estiver realmente caindo, giro = 0 e o PID fica sozinho.
constexpr double TURN_RAMP_PWM_PER_SEC = 45.0;
constexpr double TURN_FULL_AUTH_ERROR_DEG = 0.60;
constexpr double TURN_ZERO_AUTH_ERROR_DEG = 3.00;
constexpr int TURN_SAFE_MAX_PWM = 25;
double smoothTurnMix = 0.0;
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
  double currentSetpoint, accAngle, angle, gyro, err, p, i, d, pid, loopHz;
  int pwm;
  bool fallen;
  double sp, kp, ki, kd, fallAngle, integralLimit;
  int pwmMax;
  uint32_t periodUs;
  int32_t leftTicks, rightTicks;
  double leftTicksPerSecond, rightTicksPerSecond, robotSpeed, robotPosition, positionError, autoOffset;
  bool autoLearning;

  portENTER_CRITICAL(&stateMux);
  currentSetpoint = telemetrySetpoint;
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
  leftTicks = telemetryLeftEncoderTicks;
  rightTicks = telemetryRightEncoderTicks;
  leftTicksPerSecond = telemetryLeftTicksPerSecond;
  rightTicksPerSecond = telemetryRightTicksPerSecond;
  robotSpeed = telemetryRobotSpeed;
  robotPosition = telemetryRobotPosition;
  positionError = telemetryPositionError;
  autoOffset = telemetryAutoSetpointOffset;
  autoLearning = telemetryAutoSetpointLearning;
  portEXIT_CRITICAL(&stateMux);

  String json = "{";
  json += "\"telemetry\":{";
  json += "\"setpoint\":" + String(currentSetpoint,4) + ",";
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
  json += "\"loopHz\":" + String(loopHz,1) + ",";
  json += "\"encoderLeftTicks\":" + String(leftTicks) + ",";
  json += "\"encoderRightTicks\":" + String(rightTicks) + ",";
  json += "\"encoderLeftTicksPerSecond\":" + String(leftTicksPerSecond,1) + ",";
  json += "\"encoderRightTicksPerSecond\":" + String(rightTicksPerSecond,1) + ",";
  json += "\"robotSpeed\":" + String(robotSpeed,1) + ",";
  json += "\"robotPosition\":" + String(robotPosition,1) + ",";
  json += "\"positionError\":" + String(positionError,1) + ",";
  json += "\"autoSetpointOffset\":" + String(autoOffset,4) + ",";
  json += "\"neutralTrim\":" + String(telemetryNeutralTrim,4) + ",";
  json += "\"pendulumPositivePeak\":" + String(telemetryPendulumPositivePeak,1) + ",";
  json += "\"pendulumNegativePeak\":" + String(telemetryPendulumNegativePeak,1) + ",";
  json += "\"pendulumCycles\":" + String(telemetryPendulumCycles) + ",";
  json += "\"neutralState\":" + String(telemetryNeutralState) + ",";
  json += "\"learnedSetpoint\":" + String(telemetryLockedNeutralTrim,4) + ",";
  json += "\"instabilityMs\":" + String(telemetryInstabilityMs) + ",";
  json += "\"stableMs\":" + String(telemetryStableMs) + ",";
  json += "\"learnedValid\":" + String(telemetryLearnedValid ? "true" : "false") + ",";
  json += "\"fallen\":" + String(telemetryFallen ? "true" : "false") + ",";
  json += "\"autoSetpointLearning\":" + String(autoLearning ? "true" : "false");
  json += "},\"config\":{";
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
  const bool hasKp = server.hasArg("kp");
  const bool hasKi = server.hasArg("ki");
  const bool hasKd = server.hasArg("kd");
  const bool hasPwmMax = server.hasArg("pwmMax");
  const bool hasFallAngle = server.hasArg("fallAngle");
  const bool hasIntegralLimit = server.hasArg("integralLimit");
  const bool hasPeriod = server.hasArg("controlPeriodUs");

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
  autoSetpointOffset = 0.0;
  neutralTrim = 0.0;
  learnedSetpoint = 0.0;
  neutralState = NEUTRAL_SEARCHING;
  stableStartMs = 0;
  instabilityScoreMs = 0;
  lastInstabilitySampleMs = millis();
  clearPendulumCycle();
  preferences.putBool("learnedValid", false);
  learnedSetpointValid = false;
  learnedSetpoint = setpoint;
  positionTargetTicks = 0.0;
  positionTargetArmed = false;
  Kp = DEFAULT_KP;
  Ki = DEFAULT_KI;
  Kd = DEFAULT_KD;
  PWM_MAX = DEFAULT_PWM_MAX;
  FALL_ANGLE_DEG = DEFAULT_FALL_ANGLE_DEG;
  INTEGRAL_LIMIT = DEFAULT_INTEGRAL_LIMIT;
  CONTROL_PERIOD_US = DEFAULT_CONTROL_PERIOD_US;
  MOVE_OFFSET_DEG = 0.12; TURN_PWM = 20; driveCommand = DRIVE_STOP;
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

  int32_t previousLeftTicks = 0;
  int32_t previousRightTicks = 0;
  uint32_t previousEncoderSampleMs = millis();

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
    double commandedSetpoint=localSetpoint + neutralTrim + autoSetpointOffset;

    // V36: LEAN DRIVE - pequena queda controlada.
    // O PID de equilibrio move as rodas para buscar o corpo.
    double targetLeanDeg = 0.0;
    const double requestedLean = min(fabs(localMove), LEAN_MAX_DEG);
    if(localDrive==DRIVE_FORWARD) targetLeanDeg = requestedLean;
    else if(localDrive==DRIVE_BACKWARD) targetLeanDeg = -requestedLean;

    const double maxLeanStep = LEAN_RAMP_DEG_PER_SEC * dt;
    if(driveLeanDeg < targetLeanDeg)
      driveLeanDeg = min(driveLeanDeg + maxLeanStep, targetLeanDeg);
    else if(driveLeanDeg > targetLeanDeg)
      driveLeanDeg = max(driveLeanDeg - maxLeanStep, targetLeanDeg);

    // O erro angular pequeno e proposital: nao o anulamos.
    commandedSetpoint += driveLeanDeg;

    // Steering V34 preservado.
    double targetTurnMix = 0.0;
    const int safeTurnPwm = min(localTurn, TURN_SAFE_MAX_PWM);
    if(localDrive==DRIVE_LEFT) targetTurnMix=-safeTurnPwm;
    else if(localDrive==DRIVE_RIGHT) targetTurnMix=safeTurnPwm;

    // Slew-rate: evita o degrau brutal de torque entre as rodas.
    const double maxTurnStep = TURN_RAMP_PWM_PER_SEC * dt;
    if(smoothTurnMix < targetTurnMix)
      smoothTurnMix = min(smoothTurnMix + maxTurnStep, targetTurnMix);
    else if(smoothTurnMix > targetTurnMix)
      smoothTurnMix = max(smoothTurnMix - maxTurnStep, targetTurnMix);

    double error = 0, pTerm = 0, iTerm = 0, dTerm = 0, pidOutput = 0;
    int pwm = 0;
    bool fallen = false;

    if (fabs(angle - commandedSetpoint) > localFallAngle) {
      fallen = true;
      integral = 0.0;
      smoothTurnMix = 0.0;
      driveLeanDeg = 0.0;
      // Queda: a referencia de posicao anterior deixa de ser valida.
      positionTargetArmed = false;
      autoSetpointOffset = 0.0;
      wasFallen = true;
      clearPendulumCycle();
      stableStartMs = 0;
      instabilityScoreMs = 0;
  lastInstabilitySampleMs = millis();
      stopMotors();
    } else {
      error = commandedSetpoint - angle;

      // V36: recovery antecipado por ANGULO + GYRO.
      // O maior dos dois sinais decide a intensidade do boost.
      const double absError = fabs(error);
      const double absGyroRate = fabs(gyroRate);

      double angleBlend = 0.0;
      if (absError > RECOVERY_START_ERROR_DEG_V36) {
        angleBlend =
            (absError - RECOVERY_START_ERROR_DEG_V36) /
            (RECOVERY_FULL_ERROR_DEG - RECOVERY_START_ERROR_DEG_V36);
        angleBlend = constrain(angleBlend, 0.0, 1.0);
      }

      double gyroBlend = 0.0;
      if (absGyroRate > GYRO_RECOVERY_START_DPS) {
        gyroBlend =
            (absGyroRate - GYRO_RECOVERY_START_DPS) /
            (GYRO_RECOVERY_FULL_DPS - GYRO_RECOVERY_START_DPS);
        gyroBlend = constrain(gyroBlend, 0.0, 1.0);
      }

      const double recoveryBlend = max(angleBlend, gyroBlend);

      const double recoveryKp =
          localKp * (1.0 + recoveryBlend * (RECOVERY_KP_MULT_MAX_V36 - 1.0));
      const double recoveryKd =
          localKd * (1.0 + recoveryBlend * (RECOVERY_KD_MULT_MAX_V36 - 1.0));

      int effectivePwmMax = localPwmMax;
      if (recoveryBlend > 0.0) {
        effectivePwmMax = (int)lround(
            localPwmMax +
            recoveryBlend * (RECOVERY_PWM_MAX_V36 - localPwmMax));
      }

      pTerm = recoveryKp * error;

      integral += error * dt;
      integral = constrain(integral, -localIntegralLimit, localIntegralLimit);
      iTerm = localKi * integral;

      dTerm = -recoveryKd * gyroRate;
      pidOutput = constrain(pTerm + iTerm + dTerm, -(double)effectivePwmMax, (double)effectivePwmMax);
      pwm = (int)lround(pidOutput);

      // Prioridade absoluta para o equilibrio:
      // erro <= 0.6°: 100% do giro solicitado
      // erro >= 3.0°: 0% de giro
      // entre eles: reducao progressiva.
      const double steeringError = fabs(error);
      double steeringAuthority = 1.0;
      if(steeringError >= TURN_ZERO_AUTH_ERROR_DEG) {
        steeringAuthority = 0.0;
      } else if(steeringError > TURN_FULL_AUTH_ERROR_DEG) {
        steeringAuthority =
            1.0 - ((steeringError - TURN_FULL_AUTH_ERROR_DEG) /
                   (TURN_ZERO_AUTH_ERROR_DEG - TURN_FULL_AUTH_ERROR_DEG));
      }
      steeringAuthority = constrain(steeringAuthority, 0.0, 1.0);

      const int turnMix = lround(smoothTurnMix * steeringAuthority);
      const int leftCommand=constrain(pwm-turnMix,-effectivePwmMax,effectivePwmMax);
      const int rightCommand=constrain(pwm+turnMix,-effectivePwmMax,effectivePwmMax);
      setMotor(LEFT_RPWM, LEFT_LPWM, leftCommand, INVERT_LEFT_MOTOR, PWM_MIN_LEFT_FORWARD, PWM_MIN_LEFT_REVERSE, effectivePwmMax);
      setMotor(RIGHT_RPWM, RIGHT_LPWM, rightCommand, INVERT_RIGHT_MOTOR, PWM_MIN_RIGHT_FORWARD, PWM_MIN_RIGHT_REVERSE, effectivePwmMax);
    }

    const double loopHz = dt > 0 ? 1.0 / dt : 0.0;

    // ============================================================
    // V36 - ENCODERS: MALHA EXTERNA DE POSICAO + VELOCIDADE
    // ============================================================
    // Normalizacao confirmada experimentalmente:
    //   frente -> L negativo / R positivo.
    //
    // A malha externa NAO tenta equilibrar o angulo diretamente.
    // Ela observa se o robo abandonou a posicao de referencia e gera
    // somente um pequeno offset angular. O PD interno continua sendo
    // o responsavel por manter o robo em pe.
    int32_t currentLeftTicks;
    int32_t currentRightTicks;
    portENTER_CRITICAL(&encoderMux);
    currentLeftTicks = leftEncoderTicks;
    currentRightTicks = rightEncoderTicks;
    portEXIT_CRITICAL(&encoderMux);

    double leftTicksPerSecond = telemetryLeftTicksPerSecond;
    double rightTicksPerSecond = telemetryRightTicksPerSecond;
    double robotSpeed = telemetryRobotSpeed;
    double robotPosition = telemetryRobotPosition;
    double positionError = telemetryPositionError;
    bool outerLoopActive = false;

    const uint32_t encoderNowMs = millis();
    const uint32_t encoderElapsedMs = encoderNowMs - previousEncoderSampleMs;

    if (encoderElapsedMs >= 100) {
      leftTicksPerSecond =
          (currentLeftTicks - previousLeftTicks) * 1000.0 / encoderElapsedMs;
      rightTicksPerSecond =
          (currentRightTicks - previousRightTicks) * 1000.0 / encoderElapsedMs;

      // Normalizacao: frente positivo nos dois lados.
      const double normalizedLeftSpeed = -leftTicksPerSecond;
      const double normalizedRightSpeed = rightTicksPerSecond;
      robotSpeed = (normalizedLeftSpeed + normalizedRightSpeed) * 0.5;

      const double normalizedLeftPosition =
          -static_cast<double>(currentLeftTicks);
      const double normalizedRightPosition =
          static_cast<double>(currentRightTicks);
      robotPosition =
          (normalizedLeftPosition + normalizedRightPosition) * 0.5;

      const double learnedNeutralSetpoint = localSetpoint + neutralTrim;
      const double baseBalanceError = learnedNeutralSetpoint - angle;

      // Movimento manual nao e evidencia de mudanca de terreno.
      if(localDrive != DRIVE_STOP) {
        instabilityScoreMs = 0;
        lastInstabilitySampleMs = millis();
        clearPendulumCycle();
        stableStartMs = 0;
      }

      // FAST REARM: depois de uma queda, ao voltar para uma zona segura,
      // a posicao atual vira imediatamente o novo zero.
      const bool fastRearmReady =
          wasFallen &&
          localDrive == DRIVE_STOP &&
          !fallen &&
          fabs(baseBalanceError) <= FAST_REARM_WINDOW_DEG &&
          fabs(gyroRate) <= FAST_REARM_MAX_GYRO_DPS;

      const bool normalArmReady =
          !positionTargetArmed &&
          localDrive == DRIVE_STOP &&
          !fallen &&
          fabs(baseBalanceError) <= OUTER_LOOP_ARM_WINDOW_DEG &&
          fabs(robotSpeed) <= OUTER_LOOP_MAX_SPEED_TPS;

      if (!positionTargetArmed && (fastRearmReady || normalArmReady)) {
        positionTargetTicks = robotPosition;
        positionTargetArmed = true;
        autoSetpointOffset = 0.0;
        wasFallen = false;
      }

      if (positionTargetArmed) {
        positionError = robotPosition - positionTargetTicks;

        const bool canRunOuterLoop =
            localDrive == DRIVE_STOP &&
            !fallen &&
            fabs(baseBalanceError) <= OUTER_LOOP_ACTIVE_WINDOW_DEG &&
            fabs(robotSpeed) <= OUTER_LOOP_MAX_SPEED_TPS;

        if (canRunOuterLoop) {
          double effectivePositionError = positionError;
          double effectiveSpeed = robotSpeed;

          if (fabs(effectivePositionError) < POSITION_DEADBAND_TICKS) {
            effectivePositionError = 0.0;
          }
          if (fabs(effectiveSpeed) < SPEED_DEADBAND_TPS) {
            effectiveSpeed = 0.0;
          }

          // Para esta montagem, o teste real mostrou:
          //   angle positivo (queda para frente) -> PID negativo -> rodas para frente.
          //
          // Portanto, se posicao/velocidade sao POSITIVAS (robo fugindo para frente),
          // o setpoint precisa ficar MAIS POSITIVO para reduzir/inverter o comando
          // que o empurra para frente. No V25 este sinal estava invertido.
          autoSetpointOffset =
              (POSITION_KP_DEG_PER_TICK * effectivePositionError) +
              (SPEED_KD_DEG_PER_TPS * effectiveSpeed);

          autoSetpointOffset =
              constrain(autoSetpointOffset,
                        -AUTO_SP_LIMIT_DEG,
                        AUTO_SP_LIMIT_DEG);

          // V36 - STABLE SETPOINT LEARNING
          const uint32_t nowMs = millis();

          if (neutralState == NEUTRAL_LOCKED) {
            // Setpoint aprendido fica ABSOLUTAMENTE congelado.
            neutralTrim = learnedSetpoint - localSetpoint;
            neutralTrim = constrain(neutralTrim,
                                    -NEUTRAL_TRIM_LIMIT_DEG,
                                    NEUTRAL_TRIM_LIMIT_DEG);
            autoSetpointOffset = 0.0;

            // Piso irregular pode alternar frente/tras. Em vez de exigir
            // deriva continua, acumulamos evidencia de instabilidade.
            if (lastInstabilitySampleMs == 0) lastInstabilitySampleMs = nowMs;

            if (nowMs - lastInstabilitySampleMs >= INSTABILITY_SAMPLE_MS) {
              const uint32_t elapsed = nowMs - lastInstabilitySampleMs;
              lastInstabilitySampleMs = nowMs;

              const bool movingTooMuch = fabs(robotSpeed) >= INSTABILITY_SPEED_TPS;
              const bool displacedTooMuch =
                  fabs(positionError) >= INSTABILITY_POSITION_TICKS;
              const bool workingTooHard = abs(pwm) >= INSTABILITY_PWM;
              const bool unstableNow =
                  movingTooMuch && (displacedTooMuch || workingTooHard);

              if (unstableNow) {
                instabilityScoreMs = min(instabilityScoreMs + elapsed,
                                         INSTABILITY_UNLOCK_SCORE_MS);
              } else {
                if (instabilityScoreMs > INSTABILITY_RECOVERY_MS)
                  instabilityScoreMs -= INSTABILITY_RECOVERY_MS;
                else
                  instabilityScoreMs = 0;
              }

              if (instabilityScoreMs >= INSTABILITY_UNLOCK_SCORE_MS) {
                unlockStableSearch();
              }
            }
          }
          else {
            // Enquanto procura, a malha externa e o pendulo continuam podendo
            // aproximar o neutro. A palavra final, porem, e ESTABILIDADE REAL.
            const bool pendulumCanObserve =
                fabs(baseBalanceError) <= PENDULUM_ACTIVE_WINDOW_DEG &&
                fabs(gyroRate) <= PENDULUM_MAX_GYRO_DPS &&
                fabs(robotSpeed) <= PENDULUM_MAX_PEAK_TPS;

            if (pendulumCanObserve && neutralState == NEUTRAL_SEARCHING) {
              if (robotSpeed > PENDULUM_ZERO_SPEED_TPS) {
                pendulumPositivePeak = max(pendulumPositivePeak, robotSpeed);
                pendulumHavePositivePeak = true;
              } else if (robotSpeed < -PENDULUM_ZERO_SPEED_TPS) {
                pendulumNegativePeak = min(pendulumNegativePeak, robotSpeed);
                pendulumHaveNegativePeak = true;
              }

              if (pendulumHavePositivePeak && pendulumHaveNegativePeak) {
                const double pos = fabs(pendulumPositivePeak);
                const double neg = fabs(pendulumNegativePeak);

                if (pos >= PENDULUM_MIN_PEAK_TPS && neg >= PENDULUM_MIN_PEAK_TPS) {
                  const double asymmetry = pos - neg;
                  double trimStep = constrain(asymmetry * PENDULUM_ASYMMETRY_GAIN,
                                              -PENDULUM_MAX_STEP_DEG,
                                              PENDULUM_MAX_STEP_DEG);
                  neutralTrim += trimStep;
                  neutralTrim = constrain(neutralTrim,
                                          -NEUTRAL_TRIM_LIMIT_DEG,
                                          NEUTRAL_TRIM_LIMIT_DEG);
                  pendulumCycles++;
                  clearPendulumCycle();
                }
              }
            }

            // O criterio decisivo: ficou parado, vertical e praticamente sem
            // esforco por 3 s continuos? Esse Setpoint Atual FUNCIONA.
            const double candidateSetpoint =
                localSetpoint + neutralTrim + autoSetpointOffset;
            const double candidateError = candidateSetpoint - angle;

            const bool reallyStable =
                localDrive == DRIVE_STOP &&
                !fallen &&
                fabs(robotSpeed) <= STABLE_MAX_SPEED_TPS &&
                fabs(gyroRate) <= STABLE_MAX_GYRO_DPS &&
                fabs(candidateError) <= STABLE_MAX_ANGLE_ERROR_DEG &&
                abs(pwm) <= STABLE_MAX_PWM;

            if (reallyStable) {
              if (stableStartMs == 0) {
                stableStartMs = nowMs;
                neutralState = NEUTRAL_CONFIRMING;
              }

              if (nowMs - stableStartMs >= STABLE_LOCK_TIME_MS) {
                lockCurrentSetpoint(candidateSetpoint);
              }
            } else {
              stableStartMs = 0;
              neutralState = NEUTRAL_SEARCHING;
            }
          }

          outerLoopActive = true;
        }
        // Fora da janela, congelamos o ultimo offset. Nao "aprendemos"
        // durante uma recuperacao forte.
      } else {
        positionError = 0.0;
        autoSetpointOffset = 0.0;
      }

      previousLeftTicks = currentLeftTicks;
      previousRightTicks = currentRightTicks;
      previousEncoderSampleMs = encoderNowMs;
    }

    // Setpoint efetivo mostrado no painel. O novo offset entra na malha
    // interna no ciclo seguinte (~5 ms), o que e irrelevante frente aos
    // ~100 ms da malha externa.
    double liveSetpoint = localSetpoint + neutralTrim + autoSetpointOffset;
    if(localDrive==DRIVE_FORWARD) liveSetpoint += localMove;
    else if(localDrive==DRIVE_BACKWARD) liveSetpoint -= localMove;

    // O offset calculado pelos encoders passa a valer imediatamente.
    // Recalcula o erro/PID no proximo ciclo de 5 ms; a telemetria ja mostra
    // neste ciclo o setpoint efetivo atualizado.

    portENTER_CRITICAL(&stateMux);
    telemetrySetpoint = liveSetpoint;
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
    telemetryLeftEncoderTicks = currentLeftTicks;
    telemetryRightEncoderTicks = currentRightTicks;
    telemetryLeftTicksPerSecond = leftTicksPerSecond;
    telemetryRightTicksPerSecond = rightTicksPerSecond;
    telemetryRobotSpeed = robotSpeed;
    telemetryRobotPosition = robotPosition;
    telemetryPositionError = positionError;
    telemetryAutoSetpointOffset = autoSetpointOffset;
    telemetryNeutralTrim = neutralTrim;
    telemetryPendulumPositivePeak = pendulumPositivePeak;
    telemetryPendulumNegativePeak = pendulumNegativePeak;
    telemetryPendulumCycles = pendulumCycles;
    telemetryNeutralState = (uint8_t)neutralState;
    telemetryLockedNeutralTrim = learnedSetpoint;
    telemetryInstabilityMs = instabilityScoreMs;
    telemetryAutoSetpointLearning = outerLoopActive;
    portEXIT_CRITICAL(&stateMux);
  }
}

void setup() {
  Serial.begin(115200);
  preferences.begin("starkbal", false);
  if (preferences.getBool("learnedValid", false)) {
    learnedSetpoint = preferences.getDouble("learnedSP", setpoint);
    learnedSetpoint = constrain(learnedSetpoint,
                                setpoint - NEUTRAL_TRIM_LIMIT_DEG,
                                setpoint + NEUTRAL_TRIM_LIMIT_DEG);
    learnedSetpointValid = true;
    learnedLoadedFromNvs = true;
    neutralTrim = learnedSetpoint - setpoint;
    neutralState = NEUTRAL_LOCKED;
  } else {
    learnedSetpoint = setpoint;
    learnedSetpointValid = false;
    neutralState = NEUTRAL_SEARCHING;
  }


  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);

  const byte status = mpu.begin();
  if (status != 0) {
    Serial.print("Falha MPU6050. Codigo: ");
    Serial.println(status);
    while (true) delay(1000);
  }

  Serial.println("V36 - PD/PID SIMPLES + REST + HOLD-TO-DRIVE + ENCODERS");
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

  // Encoders Hall
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  leftEncoderState = (digitalRead(LEFT_ENC_A) << 1) | digitalRead(LEFT_ENC_B);
  rightEncoderState = (digitalRead(RIGHT_ENC_A) << 1) | digitalRead(RIGHT_ENC_B);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), handleLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), handleLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), handleRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), handleRightEncoder, CHANGE);

  Serial.println("Encoders: L=A18/B19 | R=A34/B35");

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

