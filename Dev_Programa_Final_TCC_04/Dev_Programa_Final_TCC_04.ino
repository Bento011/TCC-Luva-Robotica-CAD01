#include <Servo.h>
#define B_Max 13
#define B_Min 12
#define B_selec_1 2
#define B_selec_2 3

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
const int MinClearance = 0;
const int MaxClearance = 130;

// Variáveis dos Flex
const float VCC = 5.16;  // Measured voltage of Ardunio 5V line

// Flex_1
const float R_DIV_1 = 45600.0;                // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_1 = 10000.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_1 = 14000.0;      // resistência com a mão fechada

// Flex_2
const float R_DIV_2 = 45200.0;                // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_2 = 11000.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_2 = 21500.0;      // resistência com a mão fechada

// Flex_3
const float R_DIV_3 = 45900.0;                // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_3 = 12000.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_3 = 24500.0;      // resistência com a mão fechada

// Flex_4
const float R_DIV_4 = 46100.0;                // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_4 = 10000.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_4 = 25500.0;      // resistência com a mão fechada

// Flex_5
const float R_DIV_5 = 46700.0;               // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_5 = 9000.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_5 = 15000.0;     // resistência com a mão fechada

// Vetores Flex Sensor
const int FLEX_PIN[numSensors] = { FLEX_1, FLEX_2, FLEX_3, FLEX_4, FLEX_5 };
const float R_DIV[numSensors] = { R_DIV_1, R_DIV_2, R_DIV_3, R_DIV_4, R_DIV_5 };
const float STRAIGHT_RESISTANCE[numSensors] = { STRAIGHT_RESISTANCE_1, STRAIGHT_RESISTANCE_2, STRAIGHT_RESISTANCE_3, STRAIGHT_RESISTANCE_4, STRAIGHT_RESISTANCE_5 };
const float BEND_RESISTANCE[numSensors] = { BEND_RESISTANCE_1, BEND_RESISTANCE_2, BEND_RESISTANCE_3, BEND_RESISTANCE_4, BEND_RESISTANCE_5 };

// Declaração das Variáveis
String ValString;
String ValString_Res;
String ValString_Ang;
String rotina;
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
int ClearanceMin[numSensors] = { 180, 180, 180, 180, 180 };
int ClearanceMax[numSensors] = { 0, 0, 0, 0, 0 };
int MaxBend[numSensors] = { 0, 0, 0, 0, 0 };
int MedBend[numSensors] = { 90, 90, 90, 90, 90 };
int MinBend[numSensors] = { 180, 180, 180, 180, 180 };
int SafeBend[numSensors];
int cmd[50][numSensors];

void setup() {
  Serial.begin(9600);
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
  SERVO_1.attach(5);
  SERVO_2.attach(6);
  SERVO_3.attach(9);
  SERVO_4.attach(10);
  SERVO_5.attach(11);
}
// Modo de Operação 1 - Move todos os motores para a posição inicial
void Reset(int aux_reset) {
  if (aux_reset == 1) {
  SERVO_1.write(90);
  SERVO_2.write(90);
  SERVO_3.write(90);
  SERVO_4.write(90);
  SERVO_5.write(90);
  } else if (aux_reset == 2) {
  SERVO_1.write(SafeBend[0]);
  SERVO_2.write(SafeBend[1]);
  SERVO_3.write(SafeBend[2]);
  SERVO_4.write(SafeBend[3]);
  SERVO_5.write(SafeBend[4]);
  }
}
// Programa Central
void loop() {
  if (digitalRead(B_selec_1) == LOW && digitalRead(B_selec_2) == LOW) {
    Serial.println("Reset");
    Reset(1);
  } else if (digitalRead(B_selec_1) == HIGH && digitalRead(B_selec_2) == LOW) {
    Serial.println("Calibrar");
    Calibrar();
  } else if (digitalRead(B_selec_1) == LOW && digitalRead(B_selec_2) == HIGH) {
    Serial.println("Run");
    Run();
  } else if (digitalRead(B_selec_1) == HIGH && digitalRead(B_selec_2) == HIGH) {
    Serial.println("Show");
    delay(500);
    Eureka();
  } else {
    Serial.println("Reset");
    Reset(1);
  }
}

// Modo de Operação 2 - Controle da posição dos motores a partir de 1 potenciômetro, define pontos de Máximo e Mínimo do paciente
void Calibrar() {
  // Mede Potenciometro
  int Pot_Calib = analogRead(POTENCIOMETRO);
  int pot_angle = map(Pot_Calib, 0, 1023, MinClearance, MaxClearance);
  Serial.println(pot_angle);

  // Acionamento dos motores
  SERVO_1.write(pot_angle);
  SERVO_2.write(180 - pot_angle);
  SERVO_3.write(pot_angle);
  SERVO_4.write(180 - pot_angle);
  SERVO_5.write(pot_angle);
  delay(15);

  if (digitalRead(B_Min) == 0 && digitalRead(B_Max) == 0) {
    botoes = 0;
  }
  if (botoes == 0) {
    // Define Mínimo do paciente
    if (digitalRead(B_Min) == 1) {
      for (servoIndex = 0; servoIndex < numSensors; servoIndex++) {
        MinBend[servoIndex] = ValorFlex(servoIndex);
        ClearanceMin[servoIndex] = getServoPosition[servoIndex];
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
        MaxBend[servoIndex] = ValorFlex(servoIndex);
        ClearanceMax[servoIndex] = getServoPosition[servoIndex];
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
  }
}
// Modo de Operação 3 - Modo de execução, permite o uso do protótipo com feedbacl, NECESSITA execução de Calibrar()
void Run() {
  // if (Serial.available() > 0){
  //   rotina = Serial.readString();
  //   Serial.println(rotina);
  // }
  rotina = "0";
  int n;
  int aux_delay[50];

  // Define Matriz de Comando
  int rotinaValor = rotina.toInt();  // Converte o valor da string rotina para inteiro
  switch (rotinaValor) {
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
        Serial.println(setPoint[servoIndex]);
      }
      delay(totalTime / maxSteps); // Velocidade do movimento
    }
    delay(aux_delay[comando]); // Tempo de espera na posição
  }
  delay(2000);
  Reset(2);
  delay(2000);
}
// Modo de Operação 4 - Demonstra a funcionalidade e o movimento do protótipo, não utiliza feedback
void Eureka() {
  // if (Serial.available() > 0){
  //   rotina = Serial.readString();
  //   Serial.print("Rotina: ");
  //   Serial.println(rotina);
  // }
  rotina = "0";
  int n;
  int aux_delay[5];
  // Define Matriz de Comando
  int rotinaValor = rotina.toInt();  // Converte o valor da string rotina para inteiro
  switch (rotinaValor) {
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
  for (int i = 0; i < n; i++) {
    Serial.println(i);
    // Mapeia a variável de comando para o input de cada motor
    for (servoIndex = 0; servoIndex < numSensors; servoIndex++) {
      switch (cmd[i][servoIndex]) {
        case 2:
          input[servoIndex] = MinClearance;
          break;
        case 1:
          input[servoIndex] = (MinClearance + MaxClearance) / 2;
          break;
        case 0:
          input[servoIndex] = MaxClearance;
          break;
        default:
          input[servoIndex] = 90;
          break;
      }
    }

    setPoint[0] = input[0];
    setPoint[1] = 180 - input[1] + 15;
    setPoint[2] = input[2];
    setPoint[3] = 180 - input[3];
    setPoint[4] = input[4];

    Serial.println(setPoint[0]);
    Serial.println(setPoint[1]);
    Serial.println(setPoint[2]);
    Serial.println(setPoint[3]);
    Serial.println(setPoint[4]);

    int maxSteps = 40;     // número total de etapas
    int totalTime = 2500;  // tempo total em milissegundos

    for (int step = 0; step < (maxSteps - 10); step++) {
      for (int i = 0; i < 5; i++) {
        int from = getServoPosition(i);
        int to = setPoint[i];
        int erro = to - from;
        int pos = (from + step * erro / maxSteps);
        getServo(i).write(pos);
        // Serial.print(step);
        // Serial.print(" - ");
        // Serial.print(i);
        // Serial.print(" - ");
        // Serial.println(pos);
      }
      delay(totalTime / maxSteps);
    }
    delay(aux_delay);
  }
  delay(2000);
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
  flexR[servoIndex] = R_DIV[servoIndex] * (VCC / flexV - 1.0);                                                             // Conversão para Resistência
  angle[servoIndex] = map(flexR[servoIndex], STRAIGHT_RESISTANCE[servoIndex], BEND_RESISTANCE[servoIndex], MinClearance, MaxClearance);  // Connversão para Ângulo
  return angle[servoIndex];
}