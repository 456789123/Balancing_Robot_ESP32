/*
  Balancing Robot ESP32 - V18 Teste Final - Setpoint 0.285 + Self Balance Limitado

  Mudanca principal:
  - Substitui mpu.getAngleX() por um filtro de Kalman proprio.
  - O filtro combina:
      * acelerometro: referencia absoluta de inclinacao;
      * giroscopio: resposta rapida.
  - Quando o modulo da aceleracao se afasta de 1 g, o filtro reduz
    automaticamente a confianca no acelerometro. Isso ajuda quando
    o MPU6050 esta montado longe do eixo das rodas.

  IMPORTANTE:
  - Isto nao elimina completamente o problema mecanico causado pelo
    sensor no topo da estrutura.
  - A melhor posicao continua sendo perto do eixo das rodas.
  - Ajuste ACC_ANGLE_DIRECTION e GYRO_DIRECTION caso o sentido esteja errado.
  - O MPU6050 deve ficar completamente imovel durante a calibracao.
*/

#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>

// ============================================================
// MPU6050
// ============================================================

#define MPU_SDA 21
#define MPU_SCL 22

MPU6050 mpu(Wire);

// Inverta um destes sinais caso angulo e giro estejam ao contrario.
constexpr double ACC_ANGLE_DIRECTION = 1.0;
constexpr double GYRO_DIRECTION = 1.0;

// Pequeno ajuste mecanico do angulo calculado pelo acelerometro.
constexpr double ACC_ANGLE_OFFSET_DEG = 0.0;

// ============================================================
// Filtro de Kalman 1D
// Estado estimado:
//   angle = angulo
//   bias  = erro constante do giroscopio
// ============================================================

class KalmanAngle {
public:
  void setAngle(const double initialAngle) {
    angle = initialAngle;
  }

  void setProcessNoise(
    const double newQAngle,
    const double newQBias
  ) {
    qAngle = newQAngle;
    qBias = newQBias;
  }

  void setMeasurementNoise(const double newRMeasure) {
    rMeasure = newRMeasure;
  }

  double update(
    const double measuredAngle,
    const double measuredRate,
    const double dt
  ) {
    // Predicao.
    const double unbiasedRate = measuredRate - bias;
    angle += dt * unbiasedRate;

    // Atualizacao da matriz de covariancia.
    p00 += dt * (
      dt * p11 -
      p01 -
      p10 +
      qAngle
    );

    p01 -= dt * p11;
    p10 -= dt * p11;
    p11 += qBias * dt;

    // Inovacao: diferenca entre acelerometro e predicao.
    const double innovation =
        measuredAngle - angle;

    const double innovationCovariance =
        p00 + rMeasure;

    const double k0 =
        p00 / innovationCovariance;

    const double k1 =
        p10 / innovationCovariance;

    // Correcao do estado.
    angle += k0 * innovation;
    bias += k1 * innovation;

    // Correcao da covariancia.
    const double oldP00 = p00;
    const double oldP01 = p01;

    p00 -= k0 * oldP00;
    p01 -= k0 * oldP01;
    p10 -= k1 * oldP00;
    p11 -= k1 * oldP01;

    return angle;
  }

  double getBias() const {
    return bias;
  }

private:
  double qAngle = 0.001;
  double qBias = 0.003;
  double rMeasure = 0.03;

  double angle = 0.0;
  double bias = 0.0;

  double p00 = 0.0;
  double p01 = 0.0;
  double p10 = 0.0;
  double p11 = 0.0;
};

KalmanAngle angleKalman;

// ============================================================
// Kalman adaptativo
// ============================================================

// Ruido normal de medicao quando o acelerometro esta proximo de 1 g.
constexpr double KALMAN_R_MEASURE_NORMAL = 0.03;

// Ruido maximo durante forte aceleracao linear/centripeta.
// Quanto maior, menos o acelerometro influencia.
constexpr double KALMAN_R_MEASURE_MOVING = 2.50;

// Comeca a reduzir a confianca quando |modulo - 1g| passa deste valor.
constexpr double ACCEL_DEVIATION_START_G = 0.04;

// A partir deste desvio usa praticamente apenas o giroscopio.
constexpr double ACCEL_DEVIATION_FULL_G = 0.30;

// Filtro leve do modulo de aceleracao para evitar mudancas bruscas.
constexpr double ACCEL_MAG_FILTER_ALPHA = 0.15;

// ============================================================
// IBT-2 / BTS7960
// ============================================================

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

// ============================================================
// PWM
// ============================================================

constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION = 8;

constexpr int PWM_ZERO_BAND = 3;

constexpr int PWM_MIN_LEFT_FORWARD  = 18;
constexpr int PWM_MIN_LEFT_REVERSE  = 18;
constexpr int PWM_MIN_RIGHT_FORWARD = 18;
constexpr int PWM_MIN_RIGHT_REVERSE = 18;

// ============================================================
// Controle
// ============================================================

// Ponto mecanico base que funcionava melhor na V10.
// O MPU6050 esta DEITADO/HORIZONTAL sob o ESP32.
// Para esta montagem usamos pitch em torno de X:
//   Acc angle = atan2(AY, AZ)
//   Gyro rate = GX
double setpoint = -1.20;

// Correcao dinamica aprendida enquanto o robo esta quase em pe,
// mas ainda insiste em andar para um dos lados.
double selfBalanceOffset = 0.0;

// Janela onde o auto-balance pode aprender.
// Fora dela, a prioridade e recuperar o angulo, nao "aprender".
constexpr double SELF_BALANCE_LEARN_WINDOW_DEG = 3.0;

// So aprende quando existe comando real, mas sem saturacao.
constexpr double SELF_BALANCE_MIN_OUTPUT = 10.0;
constexpr double SELF_BALANCE_MAX_OUTPUT = 180.0;

// Passo por ciclo de 5 ms. Comecamos conservadores.
// Ajuste fino: aprende bem devagar para nao "inventar" um novo
// ponto de equilibrio por causa de atrito externo (ex.: fio na roda).
constexpr double SELF_BALANCE_STEP = 0.0002;

// Limite de seguranca para o ajuste automatico.
// O SelfSP agora e apenas TRIM fino. Nunca pode compensar varios graus.
constexpr double SELF_BALANCE_LIMIT_DEG = 0.30;

// So deixa o auto-balance aprender quando o acelerometro
// ainda possui alguma confianca no Kalman adaptativo.
constexpr double SELF_BALANCE_MIN_ACC_TRUST = 0.20;

constexpr double Ki = 0.0;
constexpr double FALL_ANGLE_DEG = 35.0;
constexpr double GAIN_TRANSITION_END_DEG = 5.0;

constexpr double KP_CENTER = 10.0;
constexpr double KD_CENTER = 0.18;
constexpr int PWM_MAX_CENTER = 140;

constexpr double KP_RECOVERY = 16.0;
constexpr double KD_RECOVERY = 0.20;
constexpr int PWM_MAX_RECOVERY = 220;

constexpr double GYRO_FILTER_ALPHA = 0.15;

constexpr double OUTPUT_ALPHA_CENTER = 0.18;
constexpr double OUTPUT_ALPHA_RECOVERY = 0.62;

constexpr int PWM_STEP_CENTER = 4;
constexpr int PWM_STEP_RECOVERY = 18;
constexpr int PWM_REVERSAL_STEP = 24;

constexpr double CENTER_ZONE_DEG = 1.5;
constexpr double ANGLE_ERROR_DEADBAND_DEG = 0.04;

// ============================================================
// Estado compartilhado
// ============================================================

volatile int motorCommand = 0;
volatile bool emergencyStop = false;

int appliedMotorCommand = 0;

double input = 0.0;
double accelerometerAngle = 0.0;
double angularVelocity = 0.0;
double error = 0.0;
double output = 0.0;
double effectiveSetpoint = 0.285;

double activeKp = KP_CENTER;
double activeKd = KD_CENTER;
int activePwmMax = PWM_MAX_CENTER;

double filteredGyro = 0.0;
double filteredOutput = 0.0;
double filteredAccelerationMagnitude = 1.0;
double accelerometerTrust = 1.0;
double activeKalmanR = KALMAN_R_MEASURE_NORMAL;

TaskHandle_t pidTaskHandle = nullptr;

// ============================================================
// Prototipos
// ============================================================

void pidLoop(void *parameter);
void motorControl();

double calculateAccelerometerAngleX(
  double ax,
  double ay,
  double az
);

double updateAdaptiveMeasurementNoise(
  double accelerationMagnitude
);

void updateControlParameters(double absoluteError);

void setMotor(
  int rpwmPin,
  int lpwmPin,
  int command,
  bool invert,
  int minForward,
  int minReverse
);

void stopMotors();

int compensateMotorDeadZone(
  int command,
  int minForward,
  int minReverse
);

int moveToward(int current, int target, int step);
int commandSign(int value);

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);

  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);

  const byte status = mpu.begin();

  if (status != 0) {
    Serial.print("Falha ao conectar no MPU6050. Codigo: ");
    Serial.println(status);

    while (true) {
      delay(1000);
    }
  }

  Serial.println();
  Serial.println("MPU6050 inicializado.");
  Serial.println("Mantenha o robo totalmente imovel durante a calibracao.");

  delay(1000);
  mpu.calcOffsets(true, true);

  // Faz uma primeira leitura para inicializar o Kalman sem salto.
  for (int i = 0; i < 50; i++) {
    mpu.update();
    delay(5);
  }

  const double initialAx = mpu.getAccX();
  const double initialAy = mpu.getAccY();
  const double initialAz = mpu.getAccZ();

  const double initialAngle =
      calculateAccelerometerAngleX(
        initialAx,
        initialAy,
        initialAz
      );

  angleKalman.setProcessNoise(0.001, 0.003);
  angleKalman.setMeasurementNoise(KALMAN_R_MEASURE_NORMAL);
  angleKalman.setAngle(initialAngle);

  input = initialAngle;

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
    Serial.println("Erro ao configurar os canais PWM.");

    while (true) {
      delay(1000);
    }
  }

  stopMotors();

  const BaseType_t taskCreated = xTaskCreatePinnedToCore(
    pidLoop,
    "Balance Kalman PD",
    10000,
    nullptr,
    2,
    &pidTaskHandle,
    0
  );

  if (taskCreated != pdPASS) {
    Serial.println("Erro ao criar a tarefa de controle.");

    while (true) {
      stopMotors();
      delay(1000);
    }
  }

  Serial.println("V18 - SP 0.285 | KD_CENTER 0.20 | SelfSP +/-0.30 | Step 0.0002");
}

// ============================================================
// Loop principal
// ============================================================

void loop() {
  motorControl();
  delay(1);
}

// ============================================================
// Tarefa de equilibrio - 200 Hz
// ============================================================

void pidLoop(void *parameter) {
  constexpr TickType_t PID_PERIOD_TICKS = pdMS_TO_TICKS(5);

  TickType_t lastWakeTime = xTaskGetTickCount();
  uint32_t previousMicros = micros();
  uint32_t lastDebugMs = 0;

  while (true) {
    vTaskDelayUntil(&lastWakeTime, PID_PERIOD_TICKS);

    const uint32_t nowMicros = micros();

    double dt =
        (nowMicros - previousMicros) * 0.000001;

    previousMicros = nowMicros;

    dt = constrain(dt, 0.003, 0.010);

    mpu.update();

    const double ax = mpu.getAccX();
    const double ay = mpu.getAccY();
    const double az = mpu.getAccZ();

    accelerometerAngle =
        calculateAccelerometerAngleX(ax, ay, az);

    const double rawAccelerationMagnitude =
        sqrt(ax * ax + ay * ay + az * az);

    filteredAccelerationMagnitude +=
        ACCEL_MAG_FILTER_ALPHA *
        (
          rawAccelerationMagnitude -
          filteredAccelerationMagnitude
        );

    activeKalmanR =
        updateAdaptiveMeasurementNoise(
          filteredAccelerationMagnitude
        );

    angleKalman.setMeasurementNoise(activeKalmanR);

    const double rawGyro =
        mpu.getGyroX() * GYRO_DIRECTION;

    filteredGyro +=
        GYRO_FILTER_ALPHA *
        (rawGyro - filteredGyro);

    angularVelocity = filteredGyro;

    input = angleKalman.update(
      accelerometerAngle,
      rawGyro,
      dt
    );

    if (fabs(input) > FALL_ANGLE_DEG) {
      error = 0.0;
      output = 0.0;
      filteredOutput = 0.0;

      motorCommand = 0;
      emergencyStop = true;

      // Ao cair, descartamos a correcao aprendida nesta tentativa.
      // Isso evita carregar um offset incorreto para o proximo teste.
      selfBalanceOffset = 0.0;
      effectiveSetpoint = setpoint;
    } else {
      emergencyStop = false;

      effectiveSetpoint =
          setpoint + selfBalanceOffset;

      error =
          effectiveSetpoint - input;

      double controlError = error;

      if (
        fabs(controlError) <
        ANGLE_ERROR_DEADBAND_DEG
      ) {
        controlError = 0.0;
      }

      const double absoluteError =
          fabs(controlError);

      updateControlParameters(absoluteError);

      const double proportionalTerm =
          activeKp * controlError;

      const double dampingTerm =
          -activeKd * angularVelocity;

      const double integralTerm =
          Ki * 0.0;

      const double rawOutput =
          proportionalTerm +
          integralTerm +
          dampingTerm;

      const double limitedOutput =
          constrain(
            rawOutput,
            -static_cast<double>(activePwmMax),
            static_cast<double>(activePwmMax)
          );

      const double transitionFactor =
          constrain(
            absoluteError /
            GAIN_TRANSITION_END_DEG,
            0.0,
            1.0
          );

      const double outputAlpha =
          OUTPUT_ALPHA_CENTER +
          (
            OUTPUT_ALPHA_RECOVERY -
            OUTPUT_ALPHA_CENTER
          ) *
          transitionFactor;

      filteredOutput +=
          outputAlpha *
          (limitedOutput - filteredOutput);

      output =
          constrain(
            filteredOutput,
            -static_cast<double>(activePwmMax),
            static_cast<double>(activePwmMax)
          );

      motorCommand =
          static_cast<int>(lround(output));

      // ========================================================
      // SELF BALANCE - "pulo do gato"
      //
      // Se o robo esta quase vertical, mas o controlador continua
      // pedindo movimento consistente, deslocamos lentamente o
      // ponto neutro para faze-lo frear e procurar o equilibrio.
      //
      // IMPORTANTE:
      // - nao aprende durante queda grande;
      // - nao aprende com saida quase zero;
      // - nao aprende com saida saturada;
      // - nao aprende quando o acelerometro esta muito contaminado.
      // ========================================================

      const double absControlError =
          fabs(controlError);

      const double absOutput =
          fabs(output);

      const bool canLearnSelfBalance =
          absControlError <= SELF_BALANCE_LEARN_WINDOW_DEG &&
          absOutput >= SELF_BALANCE_MIN_OUTPUT &&
          absOutput <= SELF_BALANCE_MAX_OUTPUT &&
          accelerometerTrust >= SELF_BALANCE_MIN_ACC_TRUST;

      if (canLearnSelfBalance) {
        // Mantemos o mesmo sentido da ideia do software encontrado.
        // Se durante o teste ele "aprende para o lado errado",
        // basta inverter estes dois sinais.
        if (output < 0.0) {
          selfBalanceOffset += SELF_BALANCE_STEP;
        } else if (output > 0.0) {
          selfBalanceOffset -= SELF_BALANCE_STEP;
        }

        selfBalanceOffset =
            constrain(
              selfBalanceOffset,
              -SELF_BALANCE_LIMIT_DEG,
              SELF_BALANCE_LIMIT_DEG
            );
      }
    }

    if (millis() - lastDebugMs >= 100) {
      lastDebugMs = millis();

      Serial.print("AccAngle: ");
      Serial.print(accelerometerAngle, 2);

      Serial.print(" | Kalman: ");
      Serial.print(input, 2);

      Serial.print(" | Giro: ");
      Serial.print(angularVelocity, 2);

      Serial.print(" | |A|: ");
      Serial.print(filteredAccelerationMagnitude, 3);

      Serial.print(" g | ConfiancaAcc: ");
      Serial.print(accelerometerTrust * 100.0, 0);
      Serial.print("%");

      Serial.print(" | R: ");
      Serial.print(activeKalmanR, 3);

      Serial.print(" | BaseSP: ");
      Serial.print(setpoint, 3);

      Serial.print(" | SelfSP: ");
      Serial.print(selfBalanceOffset, 3);

      Serial.print(" | EffSP: ");
      Serial.print(effectiveSetpoint, 3);

      Serial.print(" | Erro: ");
      Serial.print(error, 2);

      Serial.print(" | Saida: ");
      Serial.print(output, 1);

      Serial.print(" | PWM: ");
      Serial.println(appliedMotorCommand);
    }
  }
}

// ============================================================
// Angulo absoluto calculado pelo acelerometro
// Para inclinacao ao redor do eixo X:
// atan2(Y, Z)
// ============================================================

double calculateAccelerometerAngleX(
  const double ax,
  const double ay,
  const double az
) {
  (void)ax;

  const double angle =
      atan2(ay, az) * RAD_TO_DEG;

  return
      angle * ACC_ANGLE_DIRECTION +
      ACC_ANGLE_OFFSET_DEG;
}

// ============================================================
// Ajusta dinamicamente a confianca no acelerometro
// ============================================================

double updateAdaptiveMeasurementNoise(
  const double accelerationMagnitude
) {
  const double deviation =
      fabs(accelerationMagnitude - 1.0);

  const double denominator =
      ACCEL_DEVIATION_FULL_G -
      ACCEL_DEVIATION_START_G;

  double movementFactor =
      (
        deviation -
        ACCEL_DEVIATION_START_G
      ) /
      denominator;

  movementFactor =
      constrain(movementFactor, 0.0, 1.0);

  // Curva quadratica:
  // preserva boa confianca perto de 1 g e reduz rapidamente
  // quando aparece aceleracao linear/centripeta.
  const double curvedFactor =
      movementFactor * movementFactor;

  accelerometerTrust =
      1.0 - curvedFactor;

  return
      KALMAN_R_MEASURE_NORMAL +
      (
        KALMAN_R_MEASURE_MOVING -
        KALMAN_R_MEASURE_NORMAL
      ) *
      curvedFactor;
}

// ============================================================
// Ganhos continuos
// ============================================================

void updateControlParameters(
  const double absoluteError
) {
  const double factor =
      constrain(
        absoluteError /
        GAIN_TRANSITION_END_DEG,
        0.0,
        1.0
      );

  activeKp =
      KP_CENTER +
      (KP_RECOVERY - KP_CENTER) * factor;

  activeKd =
      KD_CENTER +
      (KD_RECOVERY - KD_CENTER) * factor;

  activePwmMax =
      static_cast<int>(
        PWM_MAX_CENTER +
        (
          PWM_MAX_RECOVERY -
          PWM_MAX_CENTER
        ) *
        factor
      );
}

// ============================================================
// Aplicacao do PWM
// ============================================================

void motorControl() {
  if (emergencyStop) {
    appliedMotorCommand = 0;
    stopMotors();
    return;
  }

  const int targetCommand =
      constrain(
        static_cast<int>(motorCommand),
        -PWM_MAX_RECOVERY,
        PWM_MAX_RECOVERY
      );

  const int currentSign =
      commandSign(appliedMotorCommand);

  const int targetSign =
      commandSign(targetCommand);

  if (
    currentSign != 0 &&
    targetSign != 0 &&
    currentSign != targetSign
  ) {
    appliedMotorCommand =
        moveToward(
          appliedMotorCommand,
          0,
          PWM_REVERSAL_STEP
        );
  } else {
    const double absoluteError =
        fabs(error);

    const double factor =
        constrain(
          absoluteError /
          GAIN_TRANSITION_END_DEG,
          0.0,
          1.0
        );

    const int responseStep =
        static_cast<int>(
          PWM_STEP_CENTER +
          (
            PWM_STEP_RECOVERY -
            PWM_STEP_CENTER
          ) *
          factor
        );

    appliedMotorCommand =
        moveToward(
          appliedMotorCommand,
          targetCommand,
          responseStep
        );
  }

  setMotor(
    LEFT_RPWM,
    LEFT_LPWM,
    appliedMotorCommand,
    INVERT_LEFT_MOTOR,
    PWM_MIN_LEFT_FORWARD,
    PWM_MIN_LEFT_REVERSE
  );

  setMotor(
    RIGHT_RPWM,
    RIGHT_LPWM,
    appliedMotorCommand,
    INVERT_RIGHT_MOTOR,
    PWM_MIN_RIGHT_FORWARD,
    PWM_MIN_RIGHT_REVERSE
  );
}

// ============================================================
// Motor individual
// ============================================================

void setMotor(
  const int rpwmPin,
  const int lpwmPin,
  int command,
  const bool invert,
  const int minForward,
  const int minReverse
) {
  command =
      constrain(
        command,
        -PWM_MAX_RECOVERY,
        PWM_MAX_RECOVERY
      );

  if (invert) {
    command = -command;
  }

  command =
      compensateMotorDeadZone(
        command,
        minForward,
        minReverse
      );

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

// ============================================================
// Compensacao da zona morta
// ============================================================

int compensateMotorDeadZone(
  const int command,
  const int minForward,
  const int minReverse
) {
  const int magnitude = abs(command);

  if (magnitude <= PWM_ZERO_BAND) {
    return 0;
  }

  if (
    command > 0 &&
    magnitude < minForward
  ) {
    return minForward;
  }

  if (
    command < 0 &&
    magnitude < minReverse
  ) {
    return -minReverse;
  }

  return command;
}

// ============================================================
// Utilitarios
// ============================================================

int moveToward(
  const int current,
  const int target,
  const int step
) {
  if (current < target) {
    return min(current + step, target);
  }

  if (current > target) {
    return max(current - step, target);
  }

  return current;
}

int commandSign(const int value) {
  if (value > PWM_ZERO_BAND) {
    return 1;
  }

  if (value < -PWM_ZERO_BAND) {
    return -1;
  }

  return 0;
}

void stopMotors() {
  ledcWrite(LEFT_RPWM, 0);
  ledcWrite(LEFT_LPWM, 0);
  ledcWrite(RIGHT_RPWM, 0);
  ledcWrite(RIGHT_LPWM, 0);
}

