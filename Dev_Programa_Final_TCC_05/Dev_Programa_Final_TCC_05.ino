#include <Servo.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCBundle.h>
#define B_Max 11
#define B_Min 12
#define B_selec_1 2
#define B_selec_2 3
#define B_selec_3 4
// #define LED_1 24 // Liga
// #define LED_2 26 // Desliga
#define LED_3 22 // MO 1
#define LED_4 26 // MO 2
#define LED_5 30 // MO 3
#define LED_6 34 // MO 4
#define LED_7 38 // Ethernet
#define LED_8 42 // Serial

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

// Variáveis dos Servos
const int numSensors = 5;
const int MinClearance = 10;
const int MaxClearance = 95;

// Variáveis dos Flex
const float VCC = 5.16;  // Measured voltage of Ardunio 5V line

// Flex_1
const float R_DIV_1 = 45600.0;                // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_1 = 8500.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_1 = 14000.0;      // resistência com a mão fechada

// Flex_2
const float R_DIV_2 = 45200.0;                // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_2 = 10000.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_2 = 21500.0;      // resistência com a mão fechada

// Flex_3
const float R_DIV_3 = 45900.0;                // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_3 = 12000.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_3 = 24500.0;      // resistência com a mão fechada

// Flex_4
const float R_DIV_4 = 46100.0;                // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_4 = 10400.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_4 = 25500.0;      // resistência com a mão fechada

// Flex_5
const float R_DIV_5 = 46700.0;               // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_5 = 10400.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_5 = 15000.0;     // resistência com a mão fechada

// Vetores Flex Sensor
const int FLEX_PIN[5] = { FLEX_1, FLEX_2, FLEX_3, FLEX_4, FLEX_5 };
const float R_DIV[5] = { R_DIV_1, R_DIV_2, R_DIV_3, R_DIV_4, R_DIV_5 };
const float STRAIGHT_RESISTANCE[5] = { STRAIGHT_RESISTANCE_1, STRAIGHT_RESISTANCE_2, STRAIGHT_RESISTANCE_3, STRAIGHT_RESISTANCE_4, STRAIGHT_RESISTANCE_5 };
const float BEND_RESISTANCE[5] = { BEND_RESISTANCE_1, BEND_RESISTANCE_2, BEND_RESISTANCE_3, BEND_RESISTANCE_4, BEND_RESISTANCE_5 };

// Declaração das Variáveis
String OSC_cmd;
float OSC_value;
String OSC_cmd_ant = "";
float OSC_value_ant = 0.0;
unsigned long inicio;  // Variável para armazenar o tempo inicial

bool flag_OSC = false;
bool flag_Eureka = false;
bool flag_Run = false;
int flag_min = 0;
int flag_max = 0;
int servoIndex;
int botoes = 0;
int input[numSensors];
int setPoint[numSensors];
int setPoint_ant[numSensors];
int erro[numSensors];
float flexR[numSensors];
int angle[numSensors];
int ClearanceMin[5] = {MaxClearance, MaxClearance, MaxClearance, MaxClearance, MaxClearance};
int ClearanceMax[5] = {MinClearance, MinClearance, MinClearance, MinClearance, MinClearance};
int MaxBend[5] = {MinClearance, MinClearance, MinClearance, MinClearance, MinClearance};
int MedBend[5] = { 45, 45, 45, 45, 45 };
int MinBend[5] = {MaxClearance, MaxClearance, MaxClearance, MaxClearance, MaxClearance};
int SafeBend[numSensors]= { 45, 45, 45, 45, 45 };
int SafeClearance[numSensors] = {MaxClearance, MaxClearance, MaxClearance, MaxClearance, MaxClearance};
int cmd[50][numSensors];
int count_erro = 0;

// you can find this written on the board of some Arduino Ethernets or shields
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
// NOTEAlternatively, you can assign a fixed IP to configure your Ethernet shield.
//       IPAddress  ip[] = { 192, 168, 0, 154 };
IPAddress ip(192, 168, 0, 154);
int serverPort = 8500; // Emotiv BCI out port
//Create UDP message object
EthernetUDP Udp;

// Funções protótipo
void OSCMsgReceive();
void processMC(OSCMessage &msg, int addrOffset);
void delay(unsigned long periodo);
void Reset(int aux_reset);
void Calibrar();
void Run();
void Eureka();
Servo getServo(int index);
int getServoPosition(int index);
int ValorFlex(int servoIndex);
int DefineRotina(String cmd);
void iniciarComunicacao();
void encerrarComunicacao();

void setup(){
  Serial.begin(9600); //9600 for a "normal" Arduino board (Uno for example). 115200 for a Teensy ++2 

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
  // pinMode(LED_1, OUTPUT);
  // pinMode(LED_2, OUTPUT);
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
  // OSCMsgReceive();
  if (digitalRead(B_selec_1) == LOW && digitalRead(B_selec_2) == LOW) {
    digitalWrite(LED_3, HIGH);
    digitalWrite(LED_4, LOW);
    digitalWrite(LED_5, LOW);
    digitalWrite(LED_6, LOW);
    Serial.println("Reset");
    Reset(1);
  } 
  else if (digitalRead(B_selec_1) == HIGH && digitalRead(B_selec_2) == LOW) {
    digitalWrite(LED_3, HIGH);
    digitalWrite(LED_4, HIGH);
    digitalWrite(LED_5, LOW);
    digitalWrite(LED_6, LOW);
    // Serial.println("Calibrar");
    Calibrar();
  } 
  else if (digitalRead(B_selec_1) == LOW && digitalRead(B_selec_2) == HIGH) {
    digitalWrite(LED_3, HIGH);
    digitalWrite(LED_4, HIGH);
    digitalWrite(LED_5, HIGH);
    digitalWrite(LED_6, LOW);
    Serial.println("Run");
    // if (OSC_cmd != "Neutral" && digitalRead(B_selec_3) == HIGH) {
    //   // digitalWrite(LED_7, HIGH);
    //   // digitalWrite(LED_8, LOW);
    //   while (OSC_cmd == "" && OSC_value == 0.0) {
    //     Serial.print("Leitura Emotiv - ");
    //     OSCMsgReceive();
    //     if (OSC_cmd == OSC_cmd_ant && OSC_value == OSC_value_ant) {
    //       OSC_cmd = "";
    //       OSC_value = 0.0;
    //     }
    //   }
    //   flag_Eureka = false;
    //   if (OSC_value >= 0.42) {
    //     Serial.println("Run");
    //     // Run();
    //     OSC_cmd = "";
    //     OSC_value = 0.0;
    //     flag_OSC = false;
    //   }
    //   if (flag_Eureka != true) {
    //     OSC_cmd_ant = OSC_cmd;
    //     OSC_value_ant = OSC_value;
    //     OSC_cmd = "";
    //     OSC_value = 0.0;
    //   }
    // } else if (OSC_cmd == "Neutral" && digitalRead(B_selec_3) == HIGH){
    //   // digitalWrite(LED_7, HIGH);
    //   // digitalWrite(LED_8, LOW);
    //   Reset(3);
    //   OSCMsgReceive();
    // } else if (digitalRead(B_selec_3) == LOW) {
    //   digitalWrite(LED_7, LOW);
    //   digitalWrite(LED_8, HIGH);
    //   Serial.println("Run");
    //   // Run();
    // }
  } 
  else if (digitalRead(B_selec_1) == HIGH && digitalRead(B_selec_2) == HIGH) {
    digitalWrite(LED_3, HIGH);
    digitalWrite(LED_4, HIGH);
    digitalWrite(LED_5, HIGH);
    digitalWrite(LED_6, HIGH);
    // Serial.println(OSC_cmd + ": " + OSC_value);
    if (OSC_cmd != "Neutral" && digitalRead(B_selec_3) == HIGH) {
      Serial.println("entrou");
      // digitalWrite(LED_7, HIGH);
      // digitalWrite(LED_8, LOW);
      while (OSC_cmd == "" && OSC_value == 0.0) {
        Serial.print("Leitura Emotiv - ");
        OSCMsgReceive();
        if (OSC_cmd == OSC_cmd_ant && OSC_value == OSC_value_ant) {
          OSC_cmd = "";
          OSC_value = 0.0;
        }
      }
      flag_Eureka = false;
      if (OSC_value >= 0.42) {
        Serial.println("Eureka");
        Eureka();
        OSC_cmd = "";
        OSC_value = 0.0;
        flag_OSC = false;
      }
      if (flag_Eureka != true) {
        OSC_cmd_ant = OSC_cmd;
        OSC_value_ant = OSC_value;
        OSC_cmd = "";
        OSC_value = 0.0;
      }
    } else if (digitalRead(B_selec_3) == HIGH){
      // digitalWrite(LED_7, HIGH);
      // digitalWrite(LED_8, LOW);
      Serial.println("Reset");
      Reset(1);
      OSCMsgReceive();
    } else if (digitalRead(B_selec_3) == LOW) {
      Serial.println("Eureka");
      Eureka();
    }
  } 
  else {
    Serial.println("Reset");
    Reset(1);
  }
}
// Inicia a comunicação via Ethernet
void iniciarComunicacao() {
  Serial.println("Emotiv BCI OSC test");
  // start the Ethernet connection:
  Ethernet.begin(mac, ip);
  Udp.begin(serverPort);
  if (Udp.begin(serverPort) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
  }
  // print your local IP address:
  Serial.print("Arduino IP address");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print(".");
  }
  Serial.println();
}
// Encerra a comunicação via Ethernet
void encerrarComunicacao() {
  Serial.println("Encerrando Comuniccacação");
  Udp.stop();
  Ethernet.maintain();  // Liberar conexões pendentes
}
// Recebe a mensagem OSC e aciona a função de processamento
void OSCMsgReceive() {
  // Serial.println("OSCMsgReceive");
  OSC_cmd = "";
  OSC_value = 0.0;
  flag_OSC = false;
  while (flag_OSC == false){
    int size = Udp.parsePacket();  
    // Serial.println(size);
    if(size > 0) {    
      OSCBundle bundleIN;
      while(size--)
        bundleIN.fill(Udp.read());
      if (!bundleIN.hasError()) {
          bundleIN.route("/com", processMC); // Mental_Commands
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
  // Serial.println("processMC");
  if(msg.match("/neutral", addrOffset)) {
    OSC_cmd = "Neutral";
  } else if(msg.match("/push", addrOffset)) {
    OSC_cmd = "Push";
  } else if(msg.match("/pull", addrOffset)) {
    OSC_cmd = "Pull";
  } else if(msg.match("/left", addrOffset)) {
    OSC_cmd = "Left";
  } else if(msg.match("/right", addrOffset)) {
    OSC_cmd = "Right";
  } else if(msg.match("/lift", addrOffset)) {
    OSC_cmd = "Lift";
  } else if(msg.match("/drop", addrOffset)) {
    OSC_cmd = "Drop";
  } else if(msg.match("/rotateLeft", addrOffset)) {
    OSC_cmd = "rotateLeft";
  } else if(msg.match("/rotateRight", addrOffset)) {
    OSC_cmd = "rotateRight";
  } else if(msg.match("/rotateClockwise", addrOffset)) {
    OSC_cmd = "rotateClockwise";
  } else if(msg.match("/rotateCounterClockwise", addrOffset)) {
    OSC_cmd = "rotateCounterClockwise";
  } else if(msg.match("/rotateForwards", addrOffset)) {
    OSC_cmd = "rotateForwards";
  } else if(msg.match("/rotateReverse", addrOffset)) {
    OSC_cmd = "rotateReverse";
  } else if(msg.match("/disappear", addrOffset)) {
    OSC_cmd = "disappear";
  }

  if(msg.isFloat(0)) {
    OSC_value = msg.getFloat(0);
  }
  Serial.println(OSC_cmd + ": " + OSC_value);
  flag_OSC = true;
}
// Define delay
void FuncadoDeEspera(unsigned long periodo) {
  inicio = millis();  // Registra o tempo inicial

  while (millis() - inicio < periodo) {
    // Não faça nada aqui, apenas aguarde
    // Outras tarefas podem continuar a ser executadas
  }
}
// Modo de Operação 1 - Move todos os motores para a posição inicial
void Reset(int aux_reset) {
    SERVO_1.write(90);
    SERVO_2.write(90);
    SERVO_3.write(90);
    SERVO_4.write(90);
    SERVO_5.write(90);
  // if (aux_reset == 1) {
  //   SERVO_1.write(90);
  //   SERVO_2.write(90);
  //   SERVO_3.write(90);
  //   SERVO_4.write(90);
  //   SERVO_5.write(90);
  // } else if (aux_reset == 2) {
  //   SERVO_1.write(SafeBend[0]);
  //   SERVO_2.write(SafeBend[1]);
  //   SERVO_3.write(SafeBend[2]);
  //   SERVO_4.write(SafeBend[3]);
  //   SERVO_5.write(SafeBend[4]);
  // } else if (aux_reset == 3) {
  //   SERVO_1.write(SafeBend[0]);
  //   SERVO_2.write(SafeBend[1]);
  //   SERVO_3.write(SafeBend[2]);
  //   SERVO_4.write(SafeBend[3]);
  //   SERVO_5.write(SafeBend[4]);
  // }
}
// Modo de Operação 2 - Controle da posição dos motores a partir de 1 potenciômetro, define pontos de Máximo e Mínimo do paciente
void Calibrar() {
  // Mede Potenciometro
  int Pot_Calib = analogRead(POTENCIOMETRO);
  int pot_angle = map(Pot_Calib, 0, 1023, MinClearance, MaxClearance);
  // Serial.print(Pot_Calib);
  // Serial.print(" - ");
  Serial.println(pot_angle);

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
      for (servoIndex = 0; servoIndex < numSensors; servoIndex++) {
        if (servoIndex == 0 || servoIndex == 2){
          MinBend[servoIndex] = ValorFlex(servoIndex);
          // ClearanceMin[servoIndex] = 180 - getServoPosition(servoIndex);
        } else {
          MinBend[servoIndex] = ValorFlex(servoIndex);
          // ClearanceMin[servoIndex] = getServoPosition(servoIndex);
        }
        Serial.print("MinBend:" + String(MinBend[servoIndex]) + " ; ");
      }
      flag_min = 1;
      botoes = 1;
      Serial.println();
      delay(500);
    }
    // Define Máximo do paciente
    else if (digitalRead(B_Max) == 1) {
      for (servoIndex = 0; servoIndex < numSensors; servoIndex++) {
        if (servoIndex == 0 || servoIndex == 2){
          MaxBend[servoIndex] = ValorFlex(servoIndex);
          // ClearanceMax[servoIndex] = 180 - getServoPosition(servoIndex);
        } else {
          MaxBend[servoIndex] = ValorFlex(servoIndex);
          // ClearanceMax[servoIndex] = getServoPosition(servoIndex);
        }
        Serial.print("MaxBend: " + String(MaxBend[servoIndex]) + " ; ");
      }
      flag_max = 1;
      botoes = 1;
      Serial.println();
      delay(500);
    }
    // Define ponto Médio do paciente
    else if (flag_max == 1 && flag_min == 1) {
      for (servoIndex = 0; servoIndex < numSensors; servoIndex++) {
        MedBend[servoIndex] = (MinBend[servoIndex] + MaxBend[servoIndex]) / 2;
        Serial.print("MedBend: " + String(MedBend[servoIndex]) + " ; ");
      }
      flag_max = 0;
      flag_min = 0;
      botoes = 1;
      Serial.println();
      delay(500);
    }
  }
  for (servoIndex = 0; servoIndex < numSensors; servoIndex++) {
    SafeBend[servoIndex] = ValorFlex(servoIndex);
    SafeClearance[servoIndex] = getServoPosition(servoIndex);
  }
}
// Modo de Operação 3 - Modo de execução, permite o uso do protótipo com feedbacl, NECESSITA execução de Calibrar()
void Run() {
  // if (Serial.available() > 0){
  //   rotina = Serial.readString();
  //   Serial.println(rotina);
  // }
  int rotina = 0;
  int n;
  int aux_delay[50];

  // Define Matriz de Comando
  switch (rotina) {
    case 0:
      n = 4;
      cmd[0][0] = 2;
      cmd[0][1] = 2;
      cmd[0][2] = 2;
      cmd[0][3] = 2;
      cmd[0][4] = 2;
      aux_delay[0] = 2000;
      cmd[1][0] = 0;
      cmd[1][1] = 0;
      cmd[1][2] = 0;
      cmd[1][3] = 0;
      cmd[1][4] = 0;
      aux_delay[1] = 2000;
      cmd[2][0] = 2;
      cmd[2][1] = 2;
      cmd[2][2] = 2;
      cmd[2][3] = 2;
      cmd[2][4] = 2;
      aux_delay[2] = 2000;
      cmd[3][0] = 0;
      cmd[3][1] = 0;
      cmd[3][2] = 0;
      cmd[3][3] = 0;
      cmd[3][4] = 0;
      break;

    case 1:
      n = 3;
      cmd[0][0] = 2;
      cmd[0][1] = 2;
      cmd[0][2] = 2;
      cmd[0][3] = 2;
      cmd[0][4] = 2;
      aux_delay[0] = 2000;
      cmd[1][0] = 1;
      cmd[1][1] = 1;
      cmd[1][2] = 1;
      cmd[1][3] = 1;
      cmd[1][4] = 1;
      aux_delay[1] = 2000;
      cmd[2][0] = 0;
      cmd[2][1] = 0;
      cmd[2][2] = 0;
      cmd[2][3] = 0;
      cmd[2][4] = 0;
      break;

    case 2:
      n = 5;
      cmd[0][0] = 0;
      cmd[0][1] = 0;
      cmd[0][2] = 0;
      cmd[0][3] = 0;
      cmd[0][4] = 0;
      aux_delay[0] = 1000;
      cmd[1][0] = 2;
      cmd[1][1] = 2;
      cmd[1][2] = 2;
      cmd[1][3] = 2;
      cmd[1][4] = 2;
      aux_delay[1] = 3000;
      cmd[2][0] = 1;
      cmd[2][1] = 2;
      cmd[2][2] = 2;
      cmd[2][3] = 2;
      cmd[2][4] = 0;
      aux_delay[2] = 2000;
      cmd[3][0] = 1;
      cmd[3][1] = 1;
      cmd[3][2] = 1;
      cmd[3][3] = 1;
      cmd[3][4] = 1;
      aux_delay[3] = 2000;
      cmd[4][0] = 0;
      cmd[4][1] = 0;
      cmd[4][2] = 0;
      cmd[4][3] = 0;
      cmd[4][4] = 0;
      break;

    default:
      Reset(1);
      break;
  }
  // Exibe matriz de comando
  for (int d = 0; d < n; d++) {
    for (int j = 0; j < 5; j++) {
      Serial.print(cmd[d][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
  // Executa rotina
  for (int comando = 0; comando < n; comando++) {
    Serial.println(comando);
    // Mapeia a variável de comando para o input de cada motor
    for (servoIndex = 0; servoIndex < numSensors; servoIndex++) {
      switch (cmd[comando][servoIndex]) {
        case 2:
          input[servoIndex] = MinBend[servoIndex];
          break;
        case 1:
          input[servoIndex] = MedBend[servoIndex];
          break;
        case 0:
          input[servoIndex] = MaxBend[servoIndex];
          break;
        default:
          input[servoIndex] = SafeBend[servoIndex];
          break;
      }
    }

    int maxSteps = 40;     // número total de etapas
    int totalTime = 2500;  // tempo total em milissegundos

    for (int step = 0; step < (maxSteps - 10); step++) {
      for (int servoIndex = 0; servoIndex < numSensors; servoIndex++) {
        int from = ValorFlex(servoIndex);
        int to = input[servoIndex];
        erro[servoIndex] = to - from;
        int pos = (from + step * erro[servoIndex] / maxSteps);
        setPoint[servoIndex] = map(pos, MinBend[servoIndex], MaxBend[servoIndex], ClearanceMin[servoIndex], ClearanceMax[servoIndex]);
        getServo(servoIndex).write(setPoint[servoIndex]);
        Serial.print(comando);
        Serial.print(servoIndex);
        Serial.print(" - ");
        Serial.print(step);
        Serial.print(" - ");
        Serial.print(from);
        Serial.print(" - ");
        Serial.print(to);
        Serial.print(" - ");
        Serial.print(setPoint[servoIndex]);
        Serial.print(" - ");
        Serial.println(erro[servoIndex]);
      }
      delay(totalTime / maxSteps); // Velocidade do movimento
    }
    delay(aux_delay[comando]); // Tempo de espera na posição
  }
  OSC_cmd_ant = OSC_cmd;
  OSC_value_ant = OSC_value;
  OSC_cmd = "";
  OSC_value = 0.0;
  flag_Eureka = true;
  delay(500);
}
// Modo de Operação 4 - Demonstra a funcionalidade e o movimento do protótipo, não utiliza feedback
void Eureka() {
  int n;
  int aux_delay[5];
  int rotina = 42;
  String serial;
  bool aux_eureka = false;
  // Define Rotina
  if (digitalRead(B_selec_3) == HIGH) {
    rotina = DefineRotina(OSC_cmd);
  } else if (digitalRead(B_selec_3) == LOW) {
    Serial.println("Serial disponível");
    // delay(250);
    if (Serial.available() > 0){
      serial = Serial.readString();
    }
    rotina = serial.toInt();
  }
  // Define Matriz de Comando
  switch (rotina) {
    case 0:
      n = 9;
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
      aux_delay[1] = 2000;
      cmd[2][0] = 0;
      cmd[2][1] = 0;
      cmd[2][2] = 0;
      cmd[2][3] = 0;
      cmd[2][4] = 0;
      aux_delay[2] = 1500;
      cmd[3][0] = 1;
      cmd[3][1] = 1;
      cmd[3][2] = 1;
      cmd[3][3] = 1;
      cmd[3][4] = 1;
      aux_delay[3] = 1500;
      cmd[4][0] = 2;
      cmd[4][1] = 2;
      cmd[4][2] = 2;
      cmd[4][3] = 2;
      cmd[4][4] = 2;
      aux_delay[4] = 1500;
      cmd[5][0] = 0;
      cmd[5][1] = 0;
      cmd[5][2] = 2;
      cmd[5][3] = 0;
      cmd[5][4] = 0;
      aux_delay[5] = 1500;
      cmd[6][0] = 0;
      cmd[6][1] = 2;
      cmd[6][2] = 0;
      cmd[6][3] = 2;
      cmd[6][4] = 0;
      aux_delay[6] = 1500;
      cmd[7][0] = 1;
      cmd[7][1] = 2;
      cmd[7][2] = 1;
      cmd[7][3] = 2;
      cmd[7][4] = 1;
      aux_delay[7] = 1500;
      cmd[8][0] = 1;
      cmd[8][1] = 2;
      cmd[8][2] = 2;
      cmd[8][3] = 2;
      cmd[8][4] = 0;
      aux_delay[8] = 1500;
      break;

    case 1:
      n = 3;
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
      aux_delay[1] = 2000;
      cmd[2][0] = 0;
      cmd[2][1] = 0;
      cmd[2][2] = 0;
      cmd[2][3] = 0;
      cmd[2][4] = 0;
      aux_delay[2] = 1500;
      break;

    case 2:
      n = 3;
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
      aux_delay[1] = 2000;
      cmd[2][0] = 1;
      cmd[2][1] = 1;
      cmd[2][2] = 1;
      cmd[2][3] = 1;
      cmd[2][4] = 1;
      aux_delay[2] = 1500;
      break;

    case 3:
      n = 3;
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
      aux_delay[1] = 2000;
      cmd[2][0] = 1;
      cmd[2][1] = 2;
      cmd[2][2] = 2;
      cmd[2][3] = 2;
      cmd[2][4] = 0;
      aux_delay[2] = 1500;
      break;

    default:
      n = 1;
      cmd[0][0] = 3;
      cmd[0][1] = 3;
      cmd[0][2] = 3;
      cmd[0][3] = 3;
      cmd[0][4] = 3;
      aux_delay[0] = 500;
      break;
  }
  // Exibe matriz de comando
  Serial.print("Rotina: ");
  Serial.println(rotina);
  Serial.println();
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
    for (servoIndex = 0; servoIndex < numSensors; servoIndex++) {
      switch (cmd[comando][servoIndex]) {
        case 2:
          input[servoIndex] = ClearanceMin[servoIndex];
          break;
        case 1:
          input[servoIndex] = (ClearanceMin[servoIndex] + ClearanceMax[servoIndex]) / 2;
          break;
        case 0:
          input[servoIndex] = ClearanceMax[servoIndex];
          break;
        default:
          input[servoIndex] = SafeClearance[servoIndex];
          break;
      }
    }

    setPoint[0] = input[0];
    setPoint[1] = 180 - input[1];
    setPoint[2] = input[2];
    setPoint[3] = 180 - input[3];
    setPoint[4] = 180 - input[4] + 10;

    // Serial.print("setPoint 1:");
    // Serial.println(setPoint[0]);
    // Serial.print("setPoint 2:");
    // Serial.println(setPoint[1]);
    // Serial.print("setPoint 3:");
    // Serial.println(setPoint[2]);
    // Serial.print("setPoint 4:");
    // Serial.println(setPoint[3]);
    // Serial.print("setPoint 5:");
    // Serial.println(setPoint[4]);

    int maxSteps = 40;     // número total de etapas
    int totalTime = 2500;  // tempo total em milissegundos

    for (int step = 0; step < (maxSteps - 10); step++) {
      for (int servoIndex = 0; servoIndex < numSensors; servoIndex++) {
        int from = getServoPosition(servoIndex);
        int to = setPoint[servoIndex];
        int erro = to - from;
        int pos = (from + step * erro / maxSteps);
        getServo(servoIndex).write(pos);
        // Serial.print(comando);
        // Serial.print(servoIndex);
        // Serial.print(" - ");
        // Serial.print(step);
        // Serial.print(" - ");
        // Serial.print(pos);
        // Serial.print(" - ");
        // Serial.println(erro);
      }
      delay(totalTime / maxSteps);
    }
    delay(aux_delay[comando]);
  }
  OSC_cmd_ant = OSC_cmd;
  OSC_value_ant = OSC_value;
  OSC_cmd = "";
  OSC_value = 0.0;
  flag_Eureka = true;
  delay(500);
}
// Função auxiliar para obter um servo pelo índice
Servo getServo(int index) {
  switch (index) {
    case 0: return SERVO_1;
    case 1: return SERVO_2;
    case 2: return SERVO_3;
    case 3: return SERVO_4;
    case 4: return SERVO_5;
    default: return SERVO_1;
  }
}
// Função auxiliar para obter a posição atual de um servo pelo índice
int getServoPosition(int index) {
  int positions[] = { SERVO_1.read(), SERVO_2.read(), SERVO_3.read(), SERVO_4.read(), SERVO_5.read() };
  return positions[index];
}
// Função auxiliar para obter a posição atual de um dedo (Flex Sensor) pelo índice
int ValorFlex(int servoIndex) {
  // Leitura do sensor
  int flexADC = analogRead(FLEX_PIN[servoIndex]);
  float flexV = flexADC * VCC / 1023.0;
  flexR[servoIndex] = R_DIV[servoIndex] * (VCC / flexV - 1.0);                                                                           // Conversão para Resistência
  angle[servoIndex] = map(flexR[servoIndex], STRAIGHT_RESISTANCE[servoIndex], BEND_RESISTANCE[servoIndex], MinClearance, MaxClearance);  // Connversão para Ângulo
  return angle[servoIndex];
}
// Função auxiliar para obter a rotina a partir do comando do Emotiv ou Serial
int DefineRotina(String cmd_aux) {
  int rotina;
  if (cmd_aux == "Push") {
    rotina = 0;
    // Serial.print("rotina: ");
    // Serial.println(rotina);
  } else if (cmd_aux == "Lift") {
    rotina = 1;
    // Serial.print("rotina: ");
    // Serial.println(rotina);
  } else if (cmd_aux == "Right") {
    rotina = 2;
    // Serial.print("rotina: ");
    // Serial.println(rotina);
  } 
  // else if (cmd_aux == "Right") {
  //   rotina = 3;
  //   Serial.print("rotina: ");
  //   Serial.println(rotina);
  // } else if (cmd_aux == "Pull") {
  //   rotina = 4;
  //   Serial.print("rotina: ");
  //   Serial.println(rotina);
  // } 
  else {
    rotina = -1;
    // Serial.print("rotina: ");
    // Serial.println(rotina);
  }
  return rotina;
}