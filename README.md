# 🤖 Balancing Robot ESP32

> **O SIMPLES FUNCIONA E MUITO BEM!!!**

Um robô de duas rodas autoequilibrado construído com **ESP32**,
**MPU6050**, motores DC com **BTS7960** e uma interface Web para
telemetria, ajuste do controlador em tempo real e controle remoto.

Depois de várias versões experimentais, a grande virada do projeto foi
simplificar o controle: **IMU → filtro de ângulo → PD/PID → PWM →
motores**. Essa versão finalmente mantém o robô em pé, recupera o
equilíbrio após pequenos empurrões e permite ajustar os parâmetros sem
recompilar o firmware.

------------------------------------------------------------------------

## 🎯 Objetivo

Construir uma plataforma de robótica móvel que consiga:

-   manter-se equilibrada sobre duas rodas;
-   recuperar-se de pequenas perturbações;
-   permitir tuning do controlador em tempo real;
-   receber comandos de movimento pela rede;
-   evoluir para controle fechado de velocidade e posição usando
    encoders;
-   servir futuramente como base para sensores, câmera, visão
    computacional e outras experiências.

------------------------------------------------------------------------

## 🧠 Como funciona

A versão atual utiliza uma malha rápida de equilíbrio:

``` text
        MPU6050
           │
           ▼
 Acelerômetro + Giroscópio
           │
           ▼
      Filtro Kalman
           │
           ▼
       Ângulo atual
           │
           ▼
        PD / PID
           │
           ▼
         PWM base
        /        \
       ▼          ▼
  BTS7960 L    BTS7960 R
       │          │
       ▼          ▼
   Motor L      Motor R
```

O **MPU6050** fornece acelerômetro e giroscópio. O ângulo estimado
alimenta o controlador de equilíbrio, que calcula o PWM necessário para
manter o corpo próximo ao setpoint.

O loop de controle roda aproximadamente a **200 Hz** (`5000 µs`) e é
separado do servidor HTTP para que a interface Web não bloqueie a malha
de equilíbrio.

------------------------------------------------------------------------

## ⚙️ Baseline funcional

Estes são os valores que marcaram a primeira versão estável do robô:

``` cpp
setpoint = -1.50;
Kp = 15.0;
Ki = 0.0;
Kd = 0.13;
PWM_MAX = 180;

FALL_ANGLE_DEG = 35.0;
INTEGRAL_LIMIT = 100.0;
CONTROL_PERIOD_US = 5000;
```

Eles também são os valores restaurados pelo botão **RESET DEFAULTS** do
painel.

> O setpoint depende da montagem física, distribuição de peso e
> inclinação da IMU. Portanto, ele pode precisar de pequenos ajustes
> após alterações mecânicas.

------------------------------------------------------------------------

## 🌐 Interface Web

Uma das partes mais importantes do projeto é o painel Web de tuning.

O ESP32 expõe uma API REST e o painel roda no navegador do computador ou
celular conectado à mesma rede.

### Telemetria

O painel acompanha em tempo real:

-   ângulo filtrado;
-   ângulo do acelerômetro;
-   velocidade angular do giroscópio;
-   erro do controlador;
-   termos P, I e D;
-   saída do PID;
-   PWM;
-   frequência do loop;
-   estado de queda.

### Tuning em tempo real

Sem recompilar ou reenviar o firmware, é possível alterar:

-   `Setpoint`;
-   `Kp`;
-   `Ki`;
-   `Kd`;
-   `PWM_MAX`;
-   ângulo máximo de queda;
-   limite da integral;
-   período do loop de controle.

O setpoint possui ajustes rápidos de `±0.01°` e `±0.10°`, o que tornou
possível encontrar o ponto de equilíbrio experimentalmente em poucos
segundos.

------------------------------------------------------------------------

## 🎮 Controle remoto

A interface também funciona como um pequeno controle remoto:

``` text
                 ▲ FRENTE

       ◀ ESQUERDA   ■ PARAR   DIREITA ▶

                  ▼ TRÁS
```

Os comandos são **momentâneos**:

-   pressionou → envia o comando;
-   continua pressionando → renova o comando periodicamente;
-   soltou → envia `STOP`;
-   perdeu o foco da janela → `STOP`;
-   mudou de aba → `STOP`;
-   perdeu comunicação → o ESP32 cancela o movimento por timeout.

O botão pressionado fica **azul** para indicar visualmente o comando
ativo.

Esse comportamento foi escolhido porque os motores possuem bastante
torque e não é desejável que o robô continue se movimentando após a
perda do comando.

### Movimento atual

Na implementação atual:

-   **frente/trás** deslocam temporariamente o setpoint;
-   **esquerda/direita** aplicam um diferencial de PWM entre os motores.

Essa solução funciona para experimentação, mas revelou uma limitação
importante: somente a IMU não informa ao controlador quanto as rodas
realmente se deslocaram.

E foi justamente essa observação que definiu a próxima grande evolução
do projeto.

------------------------------------------------------------------------

## 🔌 API REST

Principais endpoints:

  Método   Endpoint        Função
  -------- --------------- -----------------------------------------
  `GET`    `/api/health`   Verifica se o ESP32 está respondendo
  `GET`    `/api/state`    Retorna telemetria e configuração atual
  `POST`   `/api/config`   Altera os parâmetros do controlador
  `POST`   `/api/reset`    Restaura os valores de referência
  `POST`   `/api/drive`    Envia comandos de movimento

Exemplo de teste:

``` text
http://IP_DO_ESP32/api/health
```

Resposta esperada:

``` json
{
  "ok": true,
  "name": "STARK_BALANCER"
}
```

------------------------------------------------------------------------

## 📡 Rede

O firmware tenta conectar o ESP32 à rede Wi-Fi configurada.

``` cpp
const char* WIFI_SSID = "SEU_WIFI";
const char* WIFI_PASSWORD = "SUA_SENHA";
```

Se não conseguir, o ESP32 cria seu próprio Access Point:

``` text
SSID: STARK_BALANCER
Senha: starkrobot
IP: 192.168.4.1
```

O endereço obtido na rede local também é exibido no Serial Monitor a
**115200 baud**.

------------------------------------------------------------------------

## 🧰 Hardware

### Atual

-   ESP32 clássico / WROOM;
-   MPU6050;
-   2 × motores DC com caixa de redução;
-   2 × drivers BTS7960 / IBT-2;
-   estrutura mecânica de duas rodas;
-   alimentação externa durante desenvolvimento.

### Próximo upgrade

Foram escolhidos **dois motores GA25-370 / JT-GA25-370 12 V, 130 RPM,
com encoder integrado**.

Os encoders fornecerão ao ESP32 informações que a IMU não possui:

-   sentido de rotação;
-   quantidade de pulsos;
-   RPM;
-   velocidade de cada roda;
-   deslocamento;
-   posição relativa.

------------------------------------------------------------------------

## 🚀 Próxima versão --- Encoders

Hoje o robô sabe responder muito bem a:

> **"Estou caindo?"**

A IMU responde isso.

Com os encoders, ele também poderá responder:

> **"Estou andando quando deveria estar parado?"**

A próxima arquitetura será um **controle em cascata**:

``` text
                  COMANDO
                     │
                     ▼
             Velocidade desejada
                     │
                     ▼
          ┌─────────────────────┐
          │ Controle de         │
          │ velocidade/posição  │
          └──────────┬──────────┘
                     │
              Setpoint dinâmico
                     │
                     ▼
 MPU6050 ──► Kalman ──► PD de equilíbrio
                     │
                     ▼
                  PWM base
                 /        \
                ▼          ▼
             Motor L    Motor R
                ▲          ▲
             Encoder L  Encoder R
                 \        /
                  \______/
               realimentação
```

A regra será simples:

``` text
Target Speed = 0
```

Se o robô começar a escapar para frente, os encoders detectam a
velocidade e a malha externa modifica suavemente o setpoint para
freá-lo.

Se escapar para trás, acontece o contrário.

Assim, o setpoint deixa de ser apenas um número fixo e passa a ser um
valor **dinâmico calculado pelo controle de movimento**.

------------------------------------------------------------------------

## 🛣️ Roadmap

### V1 --- Equilíbrio básico ✅

-   ESP32;
-   MPU6050;
-   motores DC;
-   controle de PWM;
-   robô mantendo-se em pé.

### V2 --- Tuning Web ✅

-   API REST;
-   telemetria em tempo real;
-   alteração de `Kp`, `Ki`, `Kd`, setpoint e limites;
-   reset dos valores de referência.

### V3 --- Controle remoto ✅

-   frente;
-   trás;
-   esquerda;
-   direita;
-   comandos hold-to-drive;
-   timeout de segurança.

### V4 --- Encoders 🔜

-   leitura A/B independente;
-   ticks;
-   direção;
-   RPM;
-   velocidade;
-   telemetria das rodas no painel.

### V5 --- Controle em cascata 🔜

-   target speed;
-   PID/controle externo de velocidade;
-   setpoint dinâmico;
-   frenagem automática;
-   manutenção de posição;
-   melhor recuperação após perturbações.

### V6 --- Plataforma autônoma 💡

Depois que a base de locomoção estiver confiável:

-   bateria embarcada;
-   monitoramento da bateria;
-   câmera;
-   sensores adicionais;
-   controle pelo celular;
-   visão computacional;
-   reconhecimento e tracking;
-   experimentos com IA.

A regra continua sendo: **uma camada por vez, somente depois que a
anterior estiver funcionando.**

------------------------------------------------------------------------

## 🧪 Filosofia do projeto

Este projeto passou por várias tentativas com filtros, compensações,
autoajustes e estratégias mais sofisticadas.

A versão que finalmente funcionou nasceu quando o sistema foi reduzido
ao essencial:

``` text
Sensor
  ↓
Ângulo
  ↓
PD
  ↓
PWM
  ↓
Motor
```

A telemetria e o tuning em tempo real passaram a orientar as decisões
com base no comportamento observado, em vez de adicionar complexidade
por tentativa.

Por isso, a principal lição do projeto virou sua regra de engenharia:

> ## **O SIMPLES FUNCIONA E MUITO BEM!!!**

Complexidade só entra quando existe um problema concreto que justifique
sua existência.

------------------------------------------------------------------------

## 🏆 O commit

Depois de anos de desenvolvimento, a primeira versão realmente funcional
ganhou o commit que merecia:

``` text
b44c1cf feat: balancing robot finally fucking works
```

Não será feito squash. 😎

------------------------------------------------------------------------

## ⚠️ Segurança

Este robô utiliza motores com torque significativo.

Durante desenvolvimento:

-   teste novas lógicas inicialmente com as rodas suspensas;
-   mantenha uma forma rápida de cortar a alimentação;
-   use limites de PWM;
-   preserve o detector de queda;
-   mantenha timeout nos comandos remotos;
-   não teste próximo a escadas, animais, crianças ou objetos frágeis;
-   após instalar bateria, utilize proteção elétrica adequada.

------------------------------------------------------------------------

## ❤️ Status

**Ele finalmente fica em pé.**

Pequenos empurrões são compensados automaticamente pela malha de
equilíbrio. A interface Web permite observar o comportamento físico do
robô e ajustar os parâmetros em tempo real.

Agora o objetivo não é mais descobrir como fazê-lo equilibrar.

O objetivo é transformá-lo em uma **plataforma móvel autoequilibrada
completa**.

------------------------------------------------------------------------

**Balancing Robot ESP32**\
*Seis anos depois, o miserável finalmente ficou em pé.* 🤖🔥
