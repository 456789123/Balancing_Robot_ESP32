# 🤖 Balancing Robot ESP32

> ## **O SIMPLES FUNCIONA E MUITO BEM!!!**

Robô autoequilibrado de duas rodas desenvolvido com **ESP32**,
**MPU6050**, dois motores DC com **encoders Hall**, drivers **BTS7960 /
IBT-2** e uma **interface Web de telemetria e controle em tempo real**.

O projeto nasceu como um experimento de equilíbrio e, depois de anos de
testes, tornou-se uma plataforma de controle em cascata capaz de
observar não apenas **se o robô está caindo**, mas também **se as rodas
estão se deslocando quando deveriam estar paradas**.

A grande virada aconteceu quando o controle foi simplificado e cada
sensor passou a ter uma responsabilidade clara:

``` text
MPU6050  →  ângulo / queda  →  controle rápido de equilíbrio
Encoders →  movimento       →  posição, velocidade e ajuste do neutro
```

A interface Web tornou-se parte essencial do desenvolvimento. Sem ela,
grande parte das decisões seria baseada apenas na observação visual do
robô. Hoje é possível enxergar, em tempo real, ângulo, giroscópio, erro,
PID, PWM, velocidade das rodas, posição, setpoint efetivo, estado do
aprendizado e vários outros sinais internos do controlador.

------------------------------------------------------------------------

## 🎯 Objetivo

Construir uma plataforma robótica móvel capaz de:

-   manter-se equilibrada sobre duas rodas;
-   recuperar rapidamente o equilíbrio após perturbações;
-   medir velocidade, direção e deslocamento das duas rodas;
-   detectar quando está se deslocando mesmo estando próximo do ângulo
    de equilíbrio;
-   ajustar automaticamente seu ponto de equilíbrio;
-   aprender e preservar um setpoint estável;
-   adaptar-se a pequenas diferenças do piso;
-   disponibilizar toda a telemetria importante em uma interface Web;
-   permitir tuning do controlador em tempo real;
-   receber comandos remotos com mecanismos de segurança;
-   futuramente operar de forma totalmente embarcada usando bateria.

O objetivo final não é apenas **ficar em pé**.

É transformar o robô em uma **plataforma móvel autoequilibrada
confiável**.

------------------------------------------------------------------------

# 🧠 Arquitetura atual

O sistema possui duas responsabilidades principais.

## 1. Malha rápida de equilíbrio

A IMU responde à pergunta:

> **"Estou caindo?"**

``` text
        MPU6050
           │
           ▼
 Acelerômetro + Gyro
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
   Recovery Controller
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

O loop principal trabalha aproximadamente a **200 Hz**, usando:

``` cpp
CONTROL_PERIOD_US = 5000;
```

Essa malha precisa ser rápida, previsível e independente da interface
Web.

------------------------------------------------------------------------

## 2. Malha externa baseada nos encoders

Os encoders respondem a outra pergunta:

> **"Mesmo estando próximo do ângulo correto, estou andando quando
> deveria estar parado?"**

``` text
 Encoder L ──┐
             ├──► velocidade / posição ──► malha externa
 Encoder R ──┘                              │
                                           ▼
                              correção do ponto neutro
                                           │
                                           ▼
                                  setpoint efetivo
                                           │
                                           ▼
                               PID de equilíbrio
```

Essa separação foi fundamental.

A IMU não precisa descobrir se o robô atravessou o chão.

Os encoders não precisam descobrir se o robô está tombando.

Cada sensor resolve o problema que consegue observar melhor.

------------------------------------------------------------------------

# ⚙️ Controlador de equilíbrio atual

A referência atual de funcionamento é:

``` cpp
Kp = 16.0;
Ki = 0.0;
Kd = 0.22;

PWM_MAX = 210;

FALL_ANGLE_DEG = 35.0;
INTEGRAL_LIMIT = 100.0;
CONTROL_PERIOD_US = 5000;
```

O setpoint físico não é tratado como um número universal.

Ele depende de:

-   posição do MPU6050;
-   distribuição de peso;
-   estrutura mecânica;
-   posição futura da bateria;
-   irregularidade e inclinação do piso.

Por isso o projeto evoluiu para trabalhar com um **setpoint efetivo
adaptativo**.

------------------------------------------------------------------------

# 📐 Setpoint adaptativo

O robô possui uma referência mecânica de equilíbrio, mas os encoders
permitem encontrar pequenas correções necessárias para que ele realmente
permaneça parado.

Conceitualmente:

``` text
Setpoint base
     +
Neutral Trim aprendido
     +
Correção temporária da malha externa
     =
Setpoint efetivo
```

O objetivo não é fazer o setpoint mudar constantemente.

A estratégia atual procura encontrar um valor que funcione e depois
**parar de mexer nele**.

------------------------------------------------------------------------

# 🔒 Aprendizado do setpoint estável

A evolução do controle mostrou uma regra muito simples:

> **Se o robô consegue permanecer realmente parado e equilibrado durante
> algum tempo, o setpoint atual é um bom setpoint.**

O controlador possui estados semelhantes a:

``` text
🔓 PROCURANDO
      │
      ▼
⏳ CONFIRMANDO
      │
      ▼
🔒 TRAVADO
```

Para confirmar um setpoint, o sistema observa simultaneamente:

-   velocidade das rodas;
-   velocidade angular do giroscópio;
-   erro angular;
-   esforço de PWM;
-   tempo contínuo de estabilidade.

A referência atual utiliza aproximadamente **3 segundos de estabilidade
contínua** para confirmar o ponto encontrado.

Quando isso acontece:

1.  o setpoint efetivo é considerado válido;
2.  o valor é travado;
3.  o setpoint aprendido é armazenado;
4.  o ESP32 salva a referência em **NVS / Preferences**.

Assim, uma reinicialização não significa necessariamente começar o
aprendizado do zero.

------------------------------------------------------------------------

# 🧭 Detecção de mudança de terreno

Um simples empurrão não deve fazer o robô esquecer o que aprendeu.

Por outro lado, um piso diferente pode exigir outro ponto de equilíbrio.

O sistema utiliza uma medida de **instabilidade acumulada**.

Em vez de exigir que o robô derive continuamente para uma única direção,
ele observa se precisa trabalhar repetidamente para permanecer parado.

Isso é especialmente importante em pisos irregulares, onde o
comportamento pode ser:

``` text
frente → freia → trás → freia → frente → freia...
```

Mesmo que a posição líquida não mude muito, existe evidência de que o
ponto atual pode não ser ideal.

Quando a instabilidade acumulada ultrapassa o limite configurado, o
controlador pode liberar novamente a busca pelo neutro.

Períodos de estabilidade reduzem essa evidência, evitando que um
empurrão curto provoque reaprendizado desnecessário.

------------------------------------------------------------------------

# 💥 Recuperação e detecção de queda

O controlador diferencia duas coisas:

-   **o robô caiu**;
-   **o setpoint aprendido deixou de existir**.

Uma queda não significa automaticamente que o ponto de equilíbrio
aprendido estava errado.

Por isso, ao detectar uma queda:

-   o PWM é interrompido;
-   referências temporárias de movimento são descartadas;
-   a posição anterior deixa de ser usada como referência;
-   o setpoint aprendido pode ser preservado.

Na interface, isso pode ser apresentado como:

``` text
💥 CAÍDO / SP PRESERVADO
```

Quando o robô volta à região válida de equilíbrio, o controlador pode
recuperar rapidamente a operação.

------------------------------------------------------------------------

# 🚀 Recovery Controller

O PID básico continua sendo o núcleo do sistema, mas a recuperação
recebeu uma camada adicional.

Quando o erro angular ou a velocidade angular aumentam, o controlador
pode temporariamente ganhar mais autoridade.

A recuperação utiliza:

-   magnitude do erro angular;
-   velocidade angular do giroscópio;
-   ganho progressivo;
-   limite de PWM aumentado durante recuperação.

O objetivo é não deixar o robô excessivamente agressivo quando está
praticamente parado, mas fornecer mais força quando realmente precisa
voltar para a região de equilíbrio.

Esse mecanismo melhorou significativamente a capacidade de recuperar o
robô ao colocá-lo em pé ou após pequenas perturbações.

------------------------------------------------------------------------

# ⚙️ Encoders

Os motores atuais possuem encoders Hall em quadratura.

## Pinagem utilizada

### Encoder esquerdo

``` text
Hall A → GPIO 18
Hall B → GPIO 19
```

### Encoder direito

``` text
Hall A → GPIO 34
Hall B → GPIO 35
```

> GPIO 34 e GPIO 35 do ESP32 não possuem pull-up interno.

A direção dos encoders é normalizada em software para que o sistema
trabalhe com uma convenção única:

``` text
robotSpeed > 0  → movimento para frente
robotSpeed < 0  → movimento para trás
robotSpeed ≈ 0  → parado
```

Os encoders fornecem atualmente:

-   ticks individuais;
-   direção;
-   velocidade da roda esquerda;
-   velocidade da roda direita;
-   velocidade média do robô;
-   posição relativa;
-   erro de posição;
-   informação para a malha externa;
-   evidência de estabilidade ou instabilidade.

------------------------------------------------------------------------

# 🌐 Interface Web

A interface Web deixou de ser apenas um painel de tuning.

Ela virou um **instrumento de engenharia do projeto**.

Sem a interface, seria extremamente difícil compreender o que estava
acontecendo dentro do controlador em tempo real.

Visualmente, duas situações podem parecer iguais:

``` text
robô quase parado
```

Mas internamente uma delas pode ter:

``` text
gyro baixo
PWM baixo
encoders parados
erro pequeno
```

e outra:

``` text
gyro oscilando
PWM corrigindo continuamente
rodas indo e voltando
posição variando
```

A interface tornou essa diferença visível.

------------------------------------------------------------------------

## 📊 Telemetria em tempo real

O painel acompanha informações como:

### Equilíbrio

-   ângulo filtrado;
-   Acc Angle;
-   Gyro;
-   erro angular;
-   termo P;
-   termo I;
-   termo D;
-   saída do PID;
-   PWM;
-   frequência do loop;
-   estado de queda.

### Encoders

-   Encoder L;
-   Encoder R;
-   velocidade L;
-   velocidade R;
-   velocidade média do robô;
-   posição relativa;
-   erro de posição.

### Controle adaptativo

-   Setpoint Atual;
-   Auto SP Offset;
-   Neutral Trim;
-   Pico Frente;
-   Pico Trás;
-   Ciclos de pêndulo;
-   Estado Neutro;
-   Setpoint Aprendido;
-   Instabilidade Acumulada;
-   Tempo Estável;
-   estado da malha externa.

Esses dados permitiram abandonar boa parte do ajuste por tentativa e
erro.

Hoje é possível observar **o motivo** de determinado comportamento.

------------------------------------------------------------------------

# 🎛️ Tuning em tempo real

O painel também permite alterar parâmetros sem recompilar o firmware.

Entre os parâmetros disponíveis estão:

-   `Kp`;
-   `Ki`;
-   `Kd`;
-   `PWM_MAX`;
-   Fall Angle;
-   Integral Limit;
-   Control Period;
-   parâmetros auxiliares de movimento.

O setpoint deixou de ser tratado principalmente como um ajuste manual do
usuário.

A direção atual do projeto é permitir que o próprio robô determine
pequenas correções necessárias usando IMU + encoders.

------------------------------------------------------------------------

# 🎮 Controle remoto

A interface possui comandos momentâneos:

``` text
                 ▲ FRENTE

       ◀ ESQUERDA   ■ PARAR   DIREITA ▶

                  ▼ TRÁS
```

Os comandos seguem uma filosofia de segurança:

``` text
pressionou          → comando ativo
continua segurando  → comando renovado
soltou              → STOP
perdeu foco         → STOP
mudou de aba        → STOP
timeout             → STOP
```

Isso evita que uma perda de comunicação deixe os motores executando
indefinidamente um comando anterior.

------------------------------------------------------------------------

## ↔️ Giro

O giro utiliza diferencial entre os motores, mas passou a respeitar a
prioridade do equilíbrio.

A estratégia atual inclui:

-   entrada progressiva do comando de giro;
-   limite de autoridade;
-   redução automática do giro quando o erro angular aumenta;
-   prioridade absoluta para recuperar o equilíbrio.

Conceitualmente:

``` text
comando DIREITA/ESQUERDA
          │
          ▼
     rampa de giro
          │
          ▼
   erro angular pequeno?
      │           │
     sim         não
      │           │
      ▼           ▼
  permite      reduz giro
    giro           │
                   ▼
              PID recupera
```

O comportamento melhorou significativamente, embora o refinamento de
curvas ainda esteja em desenvolvimento.

------------------------------------------------------------------------

## ↕️ Frente / trás

A locomoção para frente e para trás **ainda está em desenvolvimento**.

Foram experimentadas estratégias de deslocamento suave do setpoint e
inclinação controlada, mas ainda não foi encontrada uma solução
considerada confiável.

Essa parte será retomada após a instalação da bateria, quando o robô
estiver com:

-   alimentação definitiva;
-   distribuição real de peso;
-   centro de gravidade definitivo;
-   ausência do cabo da fonte externa.

O projeto não considera frente/trás concluído neste momento.

------------------------------------------------------------------------

# 🔌 API REST

O ESP32 fornece uma API HTTP utilizada pela interface Web.

Principais endpoints:

  Método   Endpoint        Função
  -------- --------------- --------------------------------------------
  `GET`    `/api/health`   Verifica se o controlador está respondendo
  `GET`    `/api/state`    Retorna telemetria e estado atual
  `POST`   `/api/config`   Atualiza parâmetros configuráveis
  `POST`   `/api/reset`    Restaura parâmetros de referência
  `POST`   `/api/drive`    Envia comandos momentâneos de movimento

A interface consulta `/api/state` continuamente para atualizar a
telemetria.

------------------------------------------------------------------------

# 📡 Rede

O firmware tenta conectar o ESP32 à rede Wi-Fi configurada.

Se a conexão não estiver disponível, o projeto também pode utilizar um
Access Point próprio do ESP32.

O endereço IP ativo é informado pelo Serial Monitor.

Isso permite abrir o painel em:

-   notebook;
-   desktop;
-   celular;
-   tablet conectado à mesma rede.

------------------------------------------------------------------------

# 🧰 Hardware atual

-   ESP32 clássico / WROOM;
-   MPU6050;
-   2 × motores GA25/JGA25-370 12 V com encoder Hall;
-   caixa de redução aproximadamente 130 RPM;
-   2 × drivers BTS7960 / IBT-2;
-   duas rodas;
-   estrutura mecânica autoequilibrada;
-   interface Web;
-   alimentação externa durante a fase atual de desenvolvimento.

------------------------------------------------------------------------

# 🔋 Próxima alteração física: bateria embarcada

O próximo passo físico é retirar a fonte externa e instalar a bateria no
próprio robô.

Essa mudança é importante porque elimina:

-   força mecânica do cabo;
-   variação causada pela posição do cabo;
-   restrição física durante giros;
-   uma condição de teste diferente da configuração final.

Ao mesmo tempo, a bateria modifica:

-   massa total;
-   distribuição de peso;
-   centro de gravidade;
-   resposta dinâmica.

Por isso, novos ajustes de locomoção serão realizados **depois** da
instalação da bateria.

------------------------------------------------------------------------

# 🛣️ Evolução do projeto

## Equilíbrio básico ✅

-   leitura do MPU6050;
-   filtro de ângulo;
-   PD/PID;
-   PWM;
-   detector de queda;
-   robô permanecendo em pé.

## Interface Web e telemetria ✅

-   API REST;
-   painel em tempo real;
-   tuning sem recompilação;
-   visualização do PID;
-   visualização dos sensores;
-   diagnóstico do comportamento físico.

## Encoders ✅

-   quadratura A/B;
-   ticks;
-   direção;
-   velocidade individual;
-   velocidade média;
-   posição relativa;
-   telemetria Web.

## Controle externo / Position Hold ✅

-   posição de referência;
-   velocidade do robô;
-   correção de deriva;
-   Auto Setpoint Offset;
-   integração entre encoders e equilíbrio.

## Recovery Controller ✅

-   recuperação progressiva;
-   uso do erro angular;
-   uso do gyro;
-   aumento temporário de autoridade;
-   retorno mais rápido ao equilíbrio.

## Adaptive Neutral / Stable Setpoint Learning ✅

-   busca do neutro;
-   observação do movimento;
-   confirmação por estabilidade;
-   setpoint aprendido;
-   lock do setpoint;
-   persistência em NVS.

## Confidence / Instability Detection ✅

-   distinção entre empurrão e instabilidade persistente;
-   acúmulo de evidência;
-   recuperação de confiança durante estabilidade;
-   reabertura automática da busca quando necessário.

## Balance Priority Steering 🧪

-   giro em rampa;
-   autoridade variável;
-   prioridade para o equilíbrio;
-   comportamento significativamente melhor;
-   ainda necessita refinamento.

## Frente / trás 🧪

-   em desenvolvimento;
-   será retomado após instalação da bateria.

## Bateria embarcada 🔜

-   retirar alimentação externa;
-   definir posição definitiva da bateria;
-   reavaliar centro de gravidade;
-   testar novamente setpoint adaptativo;
-   refinar locomoção.

## Plataforma autônoma 💡

Depois que a base móvel estiver confiável:

-   monitoramento da bateria;
-   sensores adicionais;
-   câmera;
-   controle pelo celular;
-   visão computacional;
-   tracking;
-   navegação;
-   experimentos com IA.

------------------------------------------------------------------------

# 🧪 Filosofia do projeto

Durante vários anos foram testadas estratégias cada vez mais
sofisticadas.

A maior evolução aconteceu quando o sistema voltou ao essencial:

``` text
Sensor
  ↓
Ângulo
  ↓
PID
  ↓
PWM
  ↓
Motor
```

Depois disso, novas camadas passaram a ser adicionadas somente quando
havia um problema claramente observado.

Os encoders não foram adicionados para substituir a IMU.

Foram adicionados porque apareceu uma pergunta que a IMU não conseguia
responder:

> **"Estou andando mesmo quando pareço equilibrado?"**

O Recovery Controller apareceu porque havia outro problema concreto:

> **"Consigo equilibrar, mas consigo recuperar rápido quando sou
> perturbado?"**

O aprendizado de setpoint apareceu por outro:

> **"Como encontrar o neutro sem ficar ajustando manualmente para cada
> condição?"**

E a interface Web mostrou-se indispensável porque havia uma pergunta
ainda mais básica:

> **"O que realmente está acontecendo dentro do controlador neste exato
> momento?"**

A resposta passou a estar na tela.

------------------------------------------------------------------------

# 👁️ A importância da telemetria

Este projeto reforçou uma lição importante:

> **Não basta observar o robô. É necessário observar o controlador.**

A interface Web permitiu correlacionar o comportamento físico com:

``` text
ângulo
+
gyro
+
erro
+
P / I / D
+
PWM
+
velocidade das rodas
+
posição
+
setpoint
+
estado do aprendizado
```

Foi isso que tornou possível perceber quando uma hipótese estava errada,
quando uma correção estava brigando com outra e quando o robô realmente
havia encontrado uma condição estável.

Sem essa visibilidade, muitos comportamentos pareciam aleatórios.

Com telemetria, passaram a ser **dados**.

------------------------------------------------------------------------

# 🏆 O commit histórico

Depois de anos de desenvolvimento, a primeira versão que realmente
permaneceu em pé recebeu o commit:

``` text
b44c1cf feat: balancing robot finally fucking works
```

Não será feito squash. 😎

Esse commit faz parte da história do projeto.

------------------------------------------------------------------------

# ⚠️ Segurança

Este robô utiliza motores com torque significativo e pode reagir
rapidamente.

Durante desenvolvimento:

-   mantenha uma forma rápida de cortar a alimentação;
-   segure o robô nos primeiros testes de uma nova lógica;
-   preserve limites de PWM;
-   preserve o detector de queda;
-   mantenha timeout nos comandos remotos;
-   evite testes próximos a escadas;
-   mantenha distância de animais, crianças e objetos frágeis;
-   após instalar a bateria, utilize proteção elétrica adequada;
-   confirme polaridade e tensão antes da primeira energização.

------------------------------------------------------------------------

# ❤️ Estado atual

**O robô fica em pé.**

Mais importante: ele não apenas permanece equilibrado em uma condição
perfeita.

Ele consegue:

-   recuperar o equilíbrio rapidamente;
-   reagir a pequenos empurrões;
-   medir o movimento das duas rodas;
-   detectar deriva;
-   ajustar o ponto neutro;
-   reconhecer períodos reais de estabilidade;
-   aprender um setpoint;
-   preservar o setpoint aprendido;
-   observar instabilidade persistente;
-   reabrir a busca quando necessário;
-   disponibilizar todo esse processo em tempo real pela interface Web.

O giro já apresentou uma melhora muito grande, embora ainda esteja sendo
refinado.

Frente e trás continuam pendentes e serão retomados após a instalação da
bateria.

O próximo grande checkpoint será testar o sistema completamente
embarcado, sem a fonte externa e sem o cabo interferindo mecanicamente
no robô.

------------------------------------------------------------------------

## **O SIMPLES FUNCIONA E MUITO BEM!!!**

Complexidade só entra quando existe um problema concreto que justifique
sua existência.

------------------------------------------------------------------------

**Balancing Robot ESP32**

*Seis anos depois, o miserável finalmente ficou em pé.* 🤖🔥
