#include <Servo.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCBundle.h>
#include <TimerOne.h>
#define B_Max 11
#define B_Min 12
#define B_selec_1 2
#define B_selec_2 3
#define B_selec_3 4
#define LED_3 22  // MO 1
#define LED_4 26  // MO 2
#define LED_5 30  // MO 3
#define LED_6 34  // MO 4
#define LED_7 38  // Ethernet
#define LED_8 42  // Serial

// Declaração dos Flex
const int FLEX_1 = A0;         // Pin connected to voltage divider output
const int FLEX_2 = A1;         // Pin connected to voltage divider output
const int FLEX_3 = A2;         // Pin connected to voltage divider output
const int FLEX_4 = A3;         // Pin connected to voltage divider output
const int FLEX_5 = A4;         // Pin connected to voltage divider output
const int POTENCIOMETRO = A5;  // Pin connected to voltage divider output

// Declaração dos Servos
Servo SERVO_1;
Servo SERVO_2;
Servo SERVO_3;
Servo SERVO_4;
Servo SERVO_5;

// Variáveis de comuniação
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 0, 154);
int serverPort = 8500;  // Emotiv BCI out port
EthernetUDP Udp;        //Create UDP message object

// Variáveis dos Servos
const int numServos = 5;
const int fator_motor[numServos] = { -1, 1, -1, 1, 1 };
const int LowerLimit = 10;
const int UpperLimit = 95;
const int SafeLimit = (LowerLimit + UpperLimit) / 2;

// Variáveis dos Flex
const double VCC = 5.16;  // Measured voltage of Ardunio 5V line

// Flex_1
const double R_DIV_1 = 45600.0;                // Measured resistance of 47k resistor
const double STRAIGHT_RESISTANCE_1 = 17300.0;  // resistência com a mão aberta
const double BEND_RESISTANCE_1 = 30100.0;      // resistência com a mão fechada

// Flex_2
const double R_DIV_2 = 45200.0;                // Measured resistance of 47k resistor
const double STRAIGHT_RESISTANCE_2 = 13000.0;  // resistência com a mão aberta
const double BEND_RESISTANCE_2 = 27000.0;      // resistência com a mão fechada

// Flex_3
const double R_DIV_3 = 45900.0;                // Measured resistance of 47k resistor
const double STRAIGHT_RESISTANCE_3 = 10100.0;  // resistência com a mão aberta
const double BEND_RESISTANCE_3 = 19000.0;      // resistência com a mão fechada

// Flex_4
const double R_DIV_4 = 46100.0;                // Measured resistance of 47k resistor
const double STRAIGHT_RESISTANCE_4 = 13100.0;  // resistência com a mão aberta
const double BEND_RESISTANCE_4 = 22450.0;      // resistência com a mão fechada

// Flex_5
const double R_DIV_5 = 46700.0;                // Measured resistance of 47k resistor
const double STRAIGHT_RESISTANCE_5 = 12118.0;  // resistência com a mão aberta
const double BEND_RESISTANCE_5 = 15000.0;      // resistência com a mão fechada

// Arrays Flex Sensor
const int FLEX_PIN[5] = { FLEX_1, FLEX_2, FLEX_3, FLEX_4, FLEX_5 };
const double R_DIV[5] = { R_DIV_1, R_DIV_2, R_DIV_3, R_DIV_4, R_DIV_5 };
const double STRAIGHT_RESISTANCE[5] = { STRAIGHT_RESISTANCE_1, STRAIGHT_RESISTANCE_2, STRAIGHT_RESISTANCE_3, STRAIGHT_RESISTANCE_4, STRAIGHT_RESISTANCE_5 };
const double BEND_RESISTANCE[5] = { BEND_RESISTANCE_1, BEND_RESISTANCE_2, BEND_RESISTANCE_3, BEND_RESISTANCE_4, BEND_RESISTANCE_5 };

// Strings
String OSC_cmd;
String OSC_cmd_ant = "";

// Floats
float OSC_value;
float OSC_value_ant = 0.0;

// Booleans
bool flag_erro[numServos] = { false, false, false, false, false };
bool flag_OSC = false;
bool flag_Run = false;
bool flag_Realimentacao = false;
bool flag_Calibrar = false;

// Integers
int flag_min = 0;
int flag_max = 0;
int servoIndex;
int botoes = 0;
int input[numServos];
int setPoint[numServos];
int angle[numServos];
int aux_delay[5];
int cmd[50][5];
int count_erro = 0;
int n;
int rotina = 0;
int iter = 0;

// Doubles
double setPoint_flex[numServos];
double erro[numServos];
double flexR[numServos];

// Arrays of Integers
int ClearanceMin[5] = { LowerLimit, LowerLimit, LowerLimit, LowerLimit, LowerLimit };  // Mão fechada
int ClearanceMax[5] = { UpperLimit, UpperLimit, UpperLimit, UpperLimit, UpperLimit };  // Mão aberta
int MinBend[5] = { LowerLimit, LowerLimit, LowerLimit, LowerLimit, LowerLimit };       // Mão fechada
int MedBend[5] = { SafeLimit, SafeLimit, SafeLimit, SafeLimit, SafeLimit };
int MaxBend[5] = { UpperLimit, UpperLimit, UpperLimit, UpperLimit, UpperLimit };  // Mão aberta
int ClearanceMed[5] = { SafeLimit, SafeLimit, SafeLimit, SafeLimit, SafeLimit };
int SafeClearance[5] = { UpperLimit, UpperLimit, UpperLimit, UpperLimit, UpperLimit };
int Servo_UpperL[5] = { UpperLimit, 180 - UpperLimit, UpperLimit, 180 - UpperLimit, 180 - UpperLimit };
int Servo_LowerL[5] = { UpperLimit, 180 - UpperLimit, UpperLimit, 180 - UpperLimit, 180 - UpperLimit };


// Funções protótipo
void OSCMsgReceive();
void processMC(OSCMessage &msg, int addrOffset);
void Reset();
void Calibrar();
void Realimentacao();
void Run();
void iniciarComunicacao();
void encerrarComunicacao();
void DefineRotina(String cmd_aux);
void DefineMatrizCMD(int rotina);
void AtualizaSetPoint();
Servo getServo(int servoIndex);
int getServoPosition(int servoIndex);
double ValorFlex(int servoIndex);
double calcularErro(int servoIndex);
void mexeMotor();

// Setup
void setup() {
  Serial.begin(9600);  //9600 for a "normal" Arduino board (Uno for example). 115200 for a Teensy ++2
  iniciarComunicacao();
  pinMode(FLEX_1, INPUT);
  pinMode(FLEX_2, INPUT);
  pinMode(FLEX_3, INPUT);
  pinMode(FLEX_4, INPUT);
  pinMode(FLEX_5, INPUT);
  pinMode(POTENCIOMETRO, INPUT);
  pinMode(B_Min, INPUT_PULLUP);
  pinMode(B_Max, INPUT_PULLUP);
  pinMode(B_selec_1, INPUT);
  pinMode(B_selec_2, INPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);
  pinMode(LED_6, OUTPUT);
  pinMode(LED_7, OUTPUT);
  pinMode(LED_8, OUTPUT);
  SERVO_1.attach(5);
  SERVO_2.attach(6);
  SERVO_3.attach(7);
  SERVO_4.attach(8);
  SERVO_5.attach(9);
}
// Programa Central
void loop() {
  if (digitalRead(B_selec_3) == HIGH) {
    digitalWrite(LED_7, HIGH);
    digitalWrite(LED_8, LOW);
  } else {
    digitalWrite(LED_7, LOW);
    digitalWrite(LED_8, HIGH);
  }
  if (digitalRead(B_selec_1) == LOW && digitalRead(B_selec_2) == LOW) {
    Timer1.stop();
    digitalWrite(LED_3, HIGH);
    digitalWrite(LED_4, LOW);
    digitalWrite(LED_5, LOW);
    digitalWrite(LED_6, LOW);
    Serial.println("Reset");
    Reset();
  } else if (digitalRead(B_selec_1) == HIGH && digitalRead(B_selec_2) == LOW) {
    Timer1.stop();
    digitalWrite(LED_3, HIGH);
    digitalWrite(LED_4, HIGH);
    digitalWrite(LED_5, LOW);
    digitalWrite(LED_6, LOW);
    // Serial.println("Calibrar");
    Calibrar();
  } else if (digitalRead(B_selec_1) == LOW && digitalRead(B_selec_2) == HIGH) {
    digitalWrite(LED_3, HIGH);
    digitalWrite(LED_4, HIGH);
    digitalWrite(LED_5, HIGH);
    digitalWrite(LED_6, LOW);
    if (OSC_cmd != "Neutral" && digitalRead(B_selec_3) == HIGH) {
      while (OSC_cmd == "" && OSC_value == 0.0) {
        Serial.print("Leitura Emotiv - ");
        OSCMsgReceive();
        if (OSC_cmd == OSC_cmd_ant && OSC_value == OSC_value_ant) {
          OSC_cmd = "";
          OSC_value = 0.0;
        }
      }
      flag_Run = false;
      if (OSC_value >= 0.7) {
        Serial.println("Realimentacao");
        // Realimentacao();
        OSC_cmd = "";
        OSC_value = 0.0;
        flag_OSC = false;
      }
      if (flag_Run != true) {
        OSC_cmd_ant = OSC_cmd;
        OSC_value_ant = OSC_value;
        OSC_cmd = "";
        OSC_value = 0.0;
      }
    } else if (digitalRead(B_selec_3) == HIGH) {
      Serial.println("Reset");
      Reset();
      OSCMsgReceive();
    } else if (digitalRead(B_selec_3) == LOW) {
      Serial.println("Realimentacao");
      // Realimentacao();
    }
  } else if (digitalRead(B_selec_1) == HIGH && digitalRead(B_selec_2) == HIGH) {
    Timer1.stop();
    digitalWrite(LED_3, HIGH);
    digitalWrite(LED_4, HIGH);
    digitalWrite(LED_5, HIGH);
    digitalWrite(LED_6, HIGH);
    if (OSC_cmd != "Neutral" && digitalRead(B_selec_3) == HIGH) {
      while (OSC_cmd == "" && OSC_value == 0.0) {
        Serial.print("Leitura Emotiv - ");
        OSCMsgReceive();
        if (OSC_cmd == OSC_cmd_ant && ((OSC_value_ant - 0.03) <= OSC_value <= (OSC_value_ant + 0.05))) {
          OSC_cmd = "";
          OSC_value = 0.0;
        }
      }
      flag_Run = false;
      if (OSC_value >= 0.95) {
        Serial.println("Run");
        Run();
        OSC_cmd = "";
        OSC_value = 0.0;
        flag_OSC = false;
      }
      if (flag_Run != true) {
        OSC_cmd_ant = OSC_cmd;
        OSC_value_ant = OSC_value;
        OSC_cmd = "";
        OSC_value = 0.0;
      }
    } else if (digitalRead(B_selec_3) == HIGH) {
      Serial.println("Reset");
      Reset();
      OSCMsgReceive();
    } else if (digitalRead(B_selec_3) == LOW) {
      Serial.println("Run");
      Run();
    }
  } else {
    Serial.println("Reset");
    Reset();
  }
}
// Recebe a mensagem OSC e aciona a função de processamento
void OSCMsgReceive() {
  OSC_cmd = "";
  OSC_value = 0.0;
  flag_OSC = false;
  while (flag_OSC == false) {
    int size = Udp.parsePacket();
    if (size > 0) {
      OSCBundle bundleIN;
      while (size--)
        bundleIN.fill(Udp.read());
      if (!bundleIN.hasError()) {
        bundleIN.route("/com", processMC);  // Mental_Commands
      } else {
        count_erro++;
        if (count_erro >= 5) {
          Serial.print("Erro: ");
          Serial.println(bundleIN.getError());
          encerrarComunicacao();
          delay(500);
          iniciarComunicacao();
          count_erro = 0;
        }
      }
    }
  }
}
// Processa a mensagem OSC. define as variáveis de comando OSC_cmd e OSC_value
void processMC(OSCMessage &msg, int addrOffset) {
  if (msg.match("/neutral", addrOffset)) {
    OSC_cmd = "Neutral";
  } else if (msg.match("/push", addrOffset)) {
    OSC_cmd = "Push";
  } else if (msg.match("/pull", addrOffset)) {
    OSC_cmd = "Pull";
  } else if (msg.match("/left", addrOffset)) {
    OSC_cmd = "Left";
  } else if (msg.match("/right", addrOffset)) {
    OSC_cmd = "Right";
  } else if (msg.match("/lift", addrOffset)) {
    OSC_cmd = "Lift";
  } else if (msg.match("/drop", addrOffset)) {
    OSC_cmd = "Drop";
  } else if (msg.match("/rotateLeft", addrOffset)) {
    OSC_cmd = "rotateLeft";
  } else if (msg.match("/rotateRight", addrOffset)) {
    OSC_cmd = "rotateRight";
  } else if (msg.match("/rotateClockwise", addrOffset)) {
    OSC_cmd = "rotateClockwise";
  } else if (msg.match("/rotateCounterClockwise", addrOffset)) {
    OSC_cmd = "rotateCounterClockwise";
  } else if (msg.match("/rotateForwards", addrOffset)) {
    OSC_cmd = "rotateForwards";
  } else if (msg.match("/rotateReverse", addrOffset)) {
    OSC_cmd = "rotateReverse";
  } else if (msg.match("/disappear", addrOffset)) {
    OSC_cmd = "disappear";
  }

  if (msg.isFloat(0)) {
    OSC_value = msg.getFloat(0);
  }
  Serial.println(OSC_cmd + ": " + OSC_value);
  flag_OSC = true;
}
// Modo de Operação 1 - Move todos os motores para a posição inicial
void Reset() {
  SERVO_1.write(90);
  SERVO_2.write(90);
  SERVO_3.write(90);
  SERVO_4.write(90);
  SERVO_5.write(90);
}
// Modo de Operação 2 - Controle da posição dos motores a partir de 1 potenciômetro, define pontos de Máximo e Mínimo do paciente
void Calibrar() {
  // Mede Potenciometro
  int Pot_Calib = analogRead(POTENCIOMETRO);
  int pot_angle = map(Pot_Calib, 0, 1023, LowerLimit, UpperLimit);
  // Serial.print(Pot_Calib);
  // Serial.print(" - ");
  // Serial.println(pot_angle);

  // Acionamento dos motores
  SERVO_1.write(pot_angle);
  SERVO_2.write(180 - pot_angle);
  SERVO_3.write(pot_angle);
  SERVO_4.write(180 - pot_angle);
  SERVO_5.write(180 - pot_angle + 10);
  delay(15);

  if (digitalRead(B_Min) == 0 && digitalRead(B_Max) == 0) {
    botoes = 0;
  }
  if (botoes == 0) {
    // Define Mínimo do paciente
    if (digitalRead(B_Min) == 1) {
      for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
        MinBend[servoIndex] = ValorFlex(servoIndex);
        ClearanceMin[servoIndex] = getServoPosition(servoIndex);
        Serial.print("MinBend:" + String(ClearanceMin[servoIndex]) + " " + String(MinBend[servoIndex]) + " ; ");
      }
      flag_min = 1;
      botoes = 1;
      Serial.println();
    }
    // Define Máximo do paciente
    else if (digitalRead(B_Max) == 1) {
      for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
        MaxBend[servoIndex] = ValorFlex(servoIndex);
        ClearanceMax[servoIndex] = getServoPosition(servoIndex);
        Serial.print("MaxBend: " + String(ClearanceMax[servoIndex]) + " " + String(MaxBend[servoIndex]) + " ; ");
      }
      flag_max = 1;
      botoes = 1;
      Serial.println();
    }
    // Define ponto Médio do paciente
    else if (flag_max == 1 && flag_min == 1) {
      for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
        MedBend[servoIndex] = (MinBend[servoIndex] + MaxBend[servoIndex]) / 2;
        ClearanceMed[servoIndex] = (ClearanceMin[servoIndex] + ClearanceMax[servoIndex]) / 2;
        Serial.print("MedBend: " + String(ClearanceMed[servoIndex]) + " " + (MedBend[servoIndex]) + " ; ");
      }
      flag_max = 0;
      flag_min = 0;
      botoes = 1;
      flag_Calibrar = true;
      Serial.println();
    }
  }
}
// Modo de Operação 3 - Modo de execução, permite o uso do protótipo completo do protótipo com feedback, NECESSITA execução de Calibrar()
void Realimentacao() {
  String serial;
  // Define Rotina
  if (digitalRead(B_selec_3) == HIGH) {
    DefineRotina(OSC_cmd);
  } else if (digitalRead(B_selec_3) == LOW) {
    Serial.println("Serial disponível");
    if (Serial.available() > 0) {
      serial = Serial.readString();
    }
    rotina = serial.toInt();
    DefineMatrizCMD(rotina);
    Serial.println("Rotina: " + String(rotina));
  }
  // Exibe matriz de comando
  Serial.println("Matriz de Comando: ");
  for (int d = 0; d < n; d++) {
    for (int j = 0; j < 5; j++) {
      Serial.print(cmd[d][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println();
  // Executa rotina
  iter = 0;
  for (int comando = 0; comando < n; comando++) {
    Serial.print("Comando: ");
    Serial.println(comando);
    // Mapeia a variável de comando para o input de cada motor
    for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
      switch (cmd[comando][servoIndex]) {
        case 0:
          input[servoIndex] = ClearanceMin[servoIndex];
          setPoint_flex[servoIndex] = MinBend[servoIndex];
          break;
        case 1:
          input[servoIndex] = (ClearanceMin[servoIndex] + ClearanceMax[servoIndex]) / 2;
          setPoint_flex[servoIndex] = (MinBend[servoIndex] + MaxBend[servoIndex]) / 2;
          break;
        case 2:
          input[servoIndex] = ClearanceMax[servoIndex];
          setPoint_flex[servoIndex] = MaxBend[servoIndex];
          break;
        default:
          input[servoIndex] = ClearanceMed[servoIndex];
          break;
      }
    }
    AtualizaSetPoint();
    for (int erro_exec = 1; erro_exec != 0;) {
      Timer1.initialize(2500);
      // Associa a função de interrupção ao temporizador
      Timer1.attachInterrupt(mexeMotor);
      // Avalia erro para finalizar a execução, caso seja detectado o critério estabelecido
      if (flag_erro[0] == true && flag_erro[1] == true && flag_erro[2] == true && flag_erro[3] == true && flag_erro[4] == true) {
        Serial.println("Erro desejado atingido");
        Timer1.stop();
        erro_exec = 0;
        for (int aux = 0; aux < 5; aux++) {
          flag_erro[aux] == false;
        }
      }
      // Limita a execução a 85 passos - Range máximo do equipamento
      if (iter >= 85) {
        Serial.println("Limte de execução atingido");
        Timer1.stop();
        erro_exec = 0;
      }
    }
    delay(aux_delay[comando]);
  }
  OSC_cmd_ant = OSC_cmd;
  OSC_value_ant = OSC_value;
  OSC_cmd = "";
  OSC_value = 0.0;
  flag_Realimentacao = true;
  delay(500);
}
// Modo de Operação 4 - Modo de execução, realiza a execução das rotinas
void Run() {
  String serial;
  // Define Rotina
  if (digitalRead(B_selec_3) == HIGH) {
    DefineRotina(OSC_cmd);
  } else if (digitalRead(B_selec_3) == LOW) {
    // Serial.println("Serial disponível");
    if (Serial.available() > 0) {
      serial = Serial.readString();
    }
    rotina = serial.toInt();
    DefineMatrizCMD(rotina);
    Serial.println("Rotina: " + String(rotina));
  }
  // Exibe matriz de comando
  for (int d = 0; d < n; d++) {
    for (int j = 0; j < 5; j++) {
      Serial.print(cmd[d][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println();
  // Executa rotina
  for (int comando = 0; comando < n; comando++) {
    Serial.print("Comando: ");
    Serial.println(comando);
    // Mapeia a variável de comando para o input de cada motor
    for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
      switch (cmd[comando][servoIndex]) {
        case 0:
          input[servoIndex] = ClearanceMin[servoIndex];
          setPoint_flex[servoIndex] = MinBend[servoIndex];
          break;
        case 1:
          input[servoIndex] = (ClearanceMin[servoIndex] + ClearanceMax[servoIndex]) / 2;
          setPoint_flex[servoIndex] = (MinBend[servoIndex] + MaxBend[servoIndex]) / 2;
          break;
        case 2:
          input[servoIndex] = ClearanceMax[servoIndex];
          setPoint_flex[servoIndex] = MaxBend[servoIndex];
          break;
        default:
          input[servoIndex] = ClearanceMin[servoIndex];
          setPoint_flex[servoIndex] = MinBend[servoIndex];
          break;
      }
    }

    AtualizaSetPoint();

    int maxSteps = 40;     // número total de etapas
    int totalTime = 2500;  // tempo total em milissegundos

    for (int step = 0; step < (maxSteps - 10); step++) {
      for (int servoIndex = 0; servoIndex < numServos; servoIndex++) {
        int from = getServoPosition(servoIndex);
        int to = setPoint[servoIndex];
        int erro = to - from;
        int pos = (from + step * erro / maxSteps);
        getServo(servoIndex).write(pos);
      }
      delay(totalTime / maxSteps);
    }
    Serial.print("Erro;");
    for (int servoIndex = 0; servoIndex < numServos; servoIndex++) {
      Serial.print(calcularErro(servoIndex));
      Serial.print(";");
    }
    Serial.println();
    delay(aux_delay[comando]);
  }
  OSC_cmd_ant = OSC_cmd;
  OSC_value_ant = OSC_value;
  OSC_cmd = "";
  OSC_value = 0.0;
  flag_Run = true;
  delay(500);
}
// Função auxiliar para iniciar a comunicação via Ethernet
void iniciarComunicacao() {
  Serial.println("Emotiv BCI OSC test");
  // start the Ethernet connection:
  Ethernet.begin(mac, ip);
  Udp.begin(serverPort);
  if (Udp.begin(serverPort) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
  }
  // print your local IP address:
  Serial.print("Arduino IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print(".");
  }
  Serial.println();
}
// Função auxiliar para encerra a comunicação via Ethernet
void encerrarComunicacao() {
  Serial.println("Encerrando Comuniccacação");
  Udp.stop();
  Ethernet.maintain();  // Liberar conexões pendentes
}
// Função auxiliar para obter a rotina a partir do comando do Emotiv ou Serial
void DefineRotina(String cmd_aux) {
  int rotina;
  if (cmd_aux == "Push") {
    rotina = 0;
  } else if (cmd_aux == "Pull") {
    rotina = 1;
  } else if (cmd_aux == "Lift") {
    rotina = 2;
  } else if (cmd_aux == "Right") {
    rotina = 3;
  } else if (cmd_aux == "Left") {
    rotina = 42;
  } else {
    rotina = -1;
  }
  Serial.println("Rotina: " + String(rotina));
  DefineMatrizCMD(rotina);
}
// Função auxiliar para definir a matriz de comanndo a partir da rotina
void DefineMatrizCMD(int rotina) {
  // Define Matriz de Comando
  switch (rotina) {
    case 0:
      n = 4;
      cmd[0][0] = 0;
      cmd[0][1] = 0;
      cmd[0][2] = 0;
      cmd[0][3] = 0;
      cmd[0][4] = 0;
      aux_delay[0] = 500;
      cmd[1][0] = 2;
      cmd[1][1] = 2;
      cmd[1][2] = 2;
      cmd[1][3] = 2;
      cmd[1][4] = 2;
      aux_delay[1] = 500;
      cmd[2][0] = 0;
      cmd[2][1] = 0;
      cmd[2][2] = 0;
      cmd[2][3] = 0;
      cmd[2][4] = 0;
      aux_delay[2] = 500;
      cmd[3][0] = 1;
      cmd[3][1] = 1;
      cmd[3][2] = 1;
      cmd[3][3] = 1;
      cmd[3][4] = 1;
      aux_delay[3] = 1000;
      break;
    case 1:
      n = 4;
      cmd[0][0] = 2;
      cmd[0][1] = 2;
      cmd[0][2] = 2;
      cmd[0][3] = 2;
      cmd[0][4] = 2;
      aux_delay[0] = 500;
      cmd[1][0] = 0;
      cmd[1][1] = 0;
      cmd[1][2] = 0;
      cmd[1][3] = 0;
      cmd[1][4] = 0;
      aux_delay[1] = 500;
      cmd[2][0] = 2;
      cmd[2][1] = 2;
      cmd[2][2] = 2;
      cmd[2][3] = 2;
      cmd[2][4] = 2;
      aux_delay[2] = 500;
      cmd[3][0] = 1;
      cmd[3][1] = 1;
      cmd[3][2] = 1;
      cmd[3][3] = 1;
      cmd[3][4] = 1;
      aux_delay[3] = 500;
      break;
    case 2:
      n = 4;
      cmd[0][0] = 2;
      cmd[0][1] = 2;
      cmd[0][2] = 2;
      cmd[0][3] = 2;
      cmd[0][4] = 2;
      aux_delay[0] = 500;
      cmd[1][0] = 1;
      cmd[1][1] = 1;
      cmd[1][2] = 1;
      cmd[1][3] = 1;
      cmd[1][4] = 1;
      aux_delay[1] = 500;
      cmd[2][0] = 0;
      cmd[2][1] = 0;
      cmd[2][2] = 0;
      cmd[2][3] = 0;
      cmd[2][4] = 0;
      aux_delay[2] = 500;
      cmd[3][0] = 2;
      cmd[3][1] = 2;
      cmd[3][2] = 2;
      cmd[3][3] = 2;
      cmd[3][4] = 2;
      aux_delay[3] = 500;
      break;
    case 42:
      n = 6;
      cmd[0][0] = 0;
      cmd[0][1] = 0;
      cmd[0][2] = 0;
      cmd[0][3] = 0;
      cmd[0][4] = 0;
      aux_delay[0] = 500;
      cmd[1][0] = 2;
      cmd[1][1] = 2;
      cmd[1][2] = 2;
      cmd[1][3] = 2;
      cmd[1][4] = 2;
      aux_delay[1] = 500;
      cmd[2][0] = 0;
      cmd[2][1] = 0;
      cmd[2][2] = 0;
      cmd[2][3] = 0;
      cmd[2][4] = 0;
      aux_delay[2] = 500;
      cmd[3][0] = 1;
      cmd[3][1] = 1;
      cmd[3][2] = 1;
      cmd[3][3] = 1;
      cmd[3][4] = 1;
      aux_delay[3] = 1000;
      cmd[4][0] = 2;
      cmd[4][1] = 2;
      cmd[4][2] = 2;
      cmd[4][3] = 2;
      cmd[4][4] = 2;
      aux_delay[4] = 500;
      cmd[5][0] = 1;
      cmd[5][1] = 1;
      cmd[5][2] = 1;
      cmd[5][3] = 1;
      cmd[5][4] = 1;
      aux_delay[5] = 500;
      break;
    case 43:
      n = 6;
      cmd[0][0] = 0;
      cmd[0][1] = 1;
      cmd[0][2] = 2;
      cmd[0][3] = 1;
      cmd[0][4] = 0;
      aux_delay[0] = 500;
      cmd[1][0] = 2;
      cmd[1][1] = 0;
      cmd[1][2] = 1;
      cmd[1][3] = 2;
      cmd[1][4] = 1;
      aux_delay[1] = 500;
      cmd[2][0] = 0;
      cmd[2][1] = 2;
      cmd[2][2] = 0;
      cmd[2][3] = 1;
      cmd[2][4] = 2;
      aux_delay[2] = 500;
      cmd[3][0] = 1;
      cmd[3][1] = 0;
      cmd[3][2] = 2;
      cmd[3][3] = 0;
      cmd[3][4] = 1;
      aux_delay[3] = 1000;
      cmd[4][0] = 2;
      cmd[4][1] = 1;
      cmd[4][2] = 0;
      cmd[4][3] = 2;
      cmd[4][4] = 0;
      aux_delay[4] = 500;
      cmd[5][0] = 1;
      cmd[5][1] = 2;
      cmd[5][2] = 1;
      cmd[5][3] = 0;
      cmd[5][4] = 2;
      aux_delay[5] = 500;
      break;
    default:
      n = 1;
      cmd[0][0] = 2;
      cmd[0][1] = 2;
      cmd[0][2] = 2;
      cmd[0][3] = 2;
      cmd[0][4] = 2;
      aux_delay[0] = 500;
      break;
  }
}
// Função auxiliar para ajustar o setPoint para cada motor
void AtualizaSetPoint() {
  // Ajusta input para cada motor
  if (flag_Calibrar == true) {
    setPoint[0] = input[0];
    setPoint[1] = input[1];
    setPoint[2] = input[2];
    setPoint[3] = input[3];
    setPoint[4] = input[4];
  } else {
    setPoint[0] = input[0];
    setPoint[1] = 180 - input[1];
    setPoint[2] = input[2];
    setPoint[3] = 180 - input[3];
    setPoint[4] = 180 - input[4] + 10;
  }
  // Exibe os setPoints atualizados
  Serial.print("setPoint 1:");
  Serial.println(setPoint[0]);
  Serial.print("setPoint 2:");
  Serial.println(setPoint[1]);
  Serial.print("setPoint 3:");
  Serial.println(setPoint[2]);
  Serial.print("setPoint 4:");
  Serial.println(setPoint[3]);
  Serial.print("setPoint 5:");
  Serial.println(setPoint[4]);
}
// Função auxiliar para obter um servo pelo índice
Servo getServo(int servoIndex) {
  switch (servoIndex) {
    case 0: return SERVO_1;
    case 1: return SERVO_2;
    case 2: return SERVO_3;
    case 3: return SERVO_4;
    case 4: return SERVO_5;
    default: return SERVO_1;
  }
}
// Função auxiliar para obter a posição atual de um servo pelo índice
int getServoPosition(int servoIndex) {
  int positions[] = { SERVO_1.read(), SERVO_2.read(), SERVO_3.read(), SERVO_4.read(), SERVO_5.read() };
  return positions[servoIndex];
}
// Função auxiliar para obter a posição atual de um dedo (Flex Sensor) pelo índice
double ValorFlex(int servoIndex) {

  double flexADC = analogRead(FLEX_PIN[servoIndex]);                       // Leitura do sensor
  double flexV = flexADC * VCC / 1023.0;                                   // Conversão para Voltagem
  flexR[servoIndex] = (R_DIV[servoIndex] * (VCC / flexV - 1.0)) / 1000;  // Conversão para Resistência
  // angle[servoIndex] = map(flexR[servoIndex], STRAIGHT_RESISTANCE[servoIndex], BEND_RESISTANCE[servoIndex], 0, 90);  // Conversão para Ângulo
  return flexR[servoIndex];
}
// Função auxiliar para o cálculo do erro de posição
double calcularErro(int servoIndex) {
  double from_flex = ValorFlex(servoIndex);
  double to_flex = setPoint_flex[servoIndex];
  erro[servoIndex] = to_flex - from_flex;
  erro[4] = 0.0;
  return erro[servoIndex];
}
// Função auxiliar para mover o motoor conforme o erro do Flex Sensor
void mexeMotor() {
  Timer1.stop();
  Serial.print("Iteração: ");
  Serial.println(iter);
  int passo[numServos] = { 0, 0, 0, 0, 0 };
  for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
    double from_flex = ValorFlex(servoIndex);
    double to_flex = setPoint_flex[servoIndex];
    erro[servoIndex] = to_flex - from_flex;
    erro[4] = 0.0;

    if (abs(erro[servoIndex]) <= setPoint_flex[servoIndex] / 5) {
      passo[servoIndex] = 0;
      flag_erro[servoIndex] = true;
    } else if (erro[servoIndex] > 0) {
      passo[servoIndex] = 1 * fator_motor[servoIndex];
      flag_erro[servoIndex] = false;
    } else if (erro[servoIndex] < 0) {
      passo[servoIndex] = -1 * fator_motor[servoIndex];
      flag_erro[servoIndex] = false;
    }

    int from = getServoPosition(servoIndex);
    int pos = from + passo[servoIndex];

    if (pos <= Servo_LowerL[servoIndex] - 5) {
      pos = Servo_LowerL[servoIndex];
    }
    if (pos >= Servo_UpperL[servoIndex] + 5) {
      pos = Servo_UpperL[servoIndex];
    }
    getServo(servoIndex).write(pos);
    if (true) {
      Serial.print(servoIndex);
      Serial.print(" - ");
      Serial.print(setPoint[servoIndex]);
      Serial.print(" - ");
      Serial.print(from);
      Serial.print(" - ");
      Serial.print(pos);
      Serial.print(" - ");
      Serial.print(erro[servoIndex]);
      Serial.print(" - ");
      Serial.print(setPoint_flex[servoIndex] / 10);
      Serial.print(" - ");
      Serial.println(passo[servoIndex]);
    }
  }
  iter++;
  delay(15);
}