#include <Servo.h> 
#define B_Max 13
#define B_Min 12

// Declaração dos Flex
const int FLEX_1 = A0; // Pin connected to voltage divider output
const int FLEX_2 = A1; // Pin connected to voltage divider output
const int FLEX_3 = A2; // Pin connected to voltage divider output
const int FLEX_4 = A3; // Pin connected to voltage divider output
const int FLEX_5 = A4; // Pin connected to voltage divider output
const int POTENCIOMETRO = A5; // Pin connected to voltage divider output

// Declaração dos Servos
Servo SERVO_1;
Servo SERVO_2;
Servo SERVO_3;
Servo SERVO_4;
Servo SERVO_5;

// Variáveis dos Servos
const int numServos = 5;
const int Clearance = 80;
const int ClearanceMin[numServos] = {130, 50, 130, 50, 170};
const int ClearanceMax[numServos] = {0, 0, 0, 0, 0};

// Variáveis dos Flex
const float VCC = 5.16; // Measured voltage of Ardunio 5V line
const int numSensors = 5; // Número total de sensores

// Flex_1
const float R_DIV_1 = 45600.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_1 = 8480.0; // resistência com a mão aberta
const float BEND_RESISTANCE_1 = 18300.0; // resistência com a mão fechada

// Flex_2
const float R_DIV_2 = 45200.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_2 = 10230.0; // resistência com a mão aberta
const float BEND_RESISTANCE_2 = 29000.0; // resistência com a mão fechada

// Flex_3
const float R_DIV_3 = 45900.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_3 = 12020.0; // resistência com a mão aberta
const float BEND_RESISTANCE_3 = 28000.0; // resistência com a mão fechada

// Flex_4
const float R_DIV_4 = 46100.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_4 = 10570.0; // resistência com a mão aberta
const float BEND_RESISTANCE_4 = 28000.0; // resistência com a mão fechada

// Flex_5
const float R_DIV_5 = 46700.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_5 = 8570.0; // resistência com a mão aberta
const float BEND_RESISTANCE_5 = 15500.0; // resistência com a mão fechada

// Declaração das Variáveis
String ValString;
String ValString_Res;
String ValString_Ang;
String selec = "A";
int flag_min = 0;
int flag_max = 0;
int aux_eureka = 0;
int sensorIndex;
int servoIndex;
int botoes = 0;
int setPoint[numServos] = {ClearanceMin[0], ClearanceMin[1], ClearanceMin[2], ClearanceMin[3], ClearanceMin[4]};
int setPoint_ant[numServos] = {0, 0, 0, 0,0};
int input[numServos];
int cmd[5];
int erro[numServos];
int maiorErro;
int angle[numSensors];
int flexR[numSensors];
int FLEX_PIN[numSensors] = {FLEX_1, FLEX_2, FLEX_3, FLEX_4, FLEX_5};
float R_DIV[numSensors] = {R_DIV_1, R_DIV_2, R_DIV_3, R_DIV_4, R_DIV_5};
float STRAIGHT_RESISTANCE[numSensors] = {STRAIGHT_RESISTANCE_1, STRAIGHT_RESISTANCE_2, STRAIGHT_RESISTANCE_3, STRAIGHT_RESISTANCE_4, STRAIGHT_RESISTANCE_5};
float BEND_RESISTANCE[numSensors] = {BEND_RESISTANCE_1, BEND_RESISTANCE_2, BEND_RESISTANCE_3, BEND_RESISTANCE_4, BEND_RESISTANCE_5};
int MaxBend[numSensors]; // = {ClearanceMax[0], ClearanceMax[1], ClearanceMax[2], ClearanceMax[3], ClearanceMax[4]};
int MedBend[numSensors];
int MinBend[numSensors]; // = {ClearanceMin[0], ClearanceMin[1], ClearanceMin[2], ClearanceMin[3], ClearanceMin[4]};


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
  SERVO_1.attach(5);
  SERVO_2.attach(6);
  SERVO_3.attach(9);
  SERVO_4.attach(10);
  SERVO_5.attach(11);
}

void Reset(){
    SERVO_1.write(ClearanceMax[0]);
    // delay(500);
    SERVO_2.write(ClearanceMax[1]);
    // delay(500);
    SERVO_3.write(ClearanceMax[2]);
    // delay(500);
    SERVO_4.write(ClearanceMax[3]);
    // delay(500);
    SERVO_5.write(ClearanceMax[4]);
    // delay(500);
}

void ValorFlex() {
  ValString_Res = "";
  ValString_Ang = "";

  for (sensorIndex = 0; sensorIndex < numSensors; sensorIndex++) {
    // Leitura do sensor
    int flexADC = analogRead(FLEX_PIN[sensorIndex]);
    float flexV = flexADC * VCC / 1023.0;
    flexR[sensorIndex] = R_DIV[sensorIndex] * (VCC / flexV - 1.0);
    angle[sensorIndex] = map(flexR[sensorIndex], STRAIGHT_RESISTANCE[sensorIndex], BEND_RESISTANCE[sensorIndex], 0, 180);
  }
  // Imprime os valores na porta serial
  for (sensorIndex = 0; sensorIndex < numSensors; sensorIndex++) {
    ValString_Res.concat(flexR[sensorIndex]);
    ValString_Res.concat(" ; ");
    ValString_Ang.concat(angle[sensorIndex]);
    ValString_Ang.concat(" ; ");
  }
  // Aguarde um breve momento para evitar sobrecarga da porta serial
  Serial.println("Resistance; " + ValString_Res + " ohms");
  Serial.println("Bend; " + ValString_Ang + " graus");
  // Serial.println();
  delay(500);
}

void loop() {
  int aux_selec = 1;
  if (aux_selec == 0){
    Reset();
  }
  else if (aux_selec == 1){
    Calibrar();
  }
  else if (aux_selec == 2){
    // Debug();
  }
  else if (aux_selec == 3){
    ValorFlex();
  }
  else if (aux_selec == 4){
    Eureka();
  }
  
}

void Calibrar() {  
  // Mede Potenciometro
  int Pot_Calib = analogRead(POTENCIOMETRO);
  int pot_angle = map(Pot_Calib, 0, 1023, 20, 110);
  Serial.println(pot_angle);

  // Acionamento dos motores
  SERVO_1.write(pot_angle);
  SERVO_2.write(180-pot_angle-10);
  SERVO_3.write(pot_angle);
  SERVO_4.write(180-pot_angle-10);
  SERVO_5.write(pot_angle);
  delay(15);
  if (digitalRead(B_Min) == 0 && digitalRead(B_Max) == 0){
    botoes = 0;
    }
  if (botoes == 0){
      // Define Mínimo do paciente
      if (digitalRead(B_Min) == 1){
        ValorFlex();
        for (sensorIndex = 0; sensorIndex < numSensors; sensorIndex++) {
          MinBend[sensorIndex] = angle[sensorIndex];
          Serial.print("MinBend:" + String(MinBend[sensorIndex]) + " ; ");
        }
        flag_min = 1;
        botoes=1;
        Serial.println();
        delay(500);
      }
      // Define Máximo do paciente
      else if (digitalRead(B_Max) == 1){
        ValorFlex();
        for (sensorIndex = 0; sensorIndex < numSensors; sensorIndex++) {
          MaxBend[sensorIndex] = angle[sensorIndex];
          Serial.print("MaxBend: " + String(MaxBend[sensorIndex]) + " ; ");
        }
        flag_max = 1;
        botoes=1;
        Serial.println();
        delay(500);
        }
      // Define ponto Médio do paciente
      else if (flag_max == 1 && flag_min == 1) {
        for (sensorIndex = 0; sensorIndex < numSensors; sensorIndex++) {
          MedBend[sensorIndex] = (MinBend[sensorIndex] + MaxBend[sensorIndex]) / 2;
          Serial.print("MedBend: " + String(MedBend[sensorIndex]) + " ; ");
        }
        flag_max = 0;
        flag_min = 0;
        botoes=1;
        Serial.println();
        delay(500);
        }
    } 
}

void Eureka(){
  // Define a variável de comando
  Serial.println();
  Serial.print("Eureka: ");
  Serial.println(aux_eureka);
  switch (aux_eureka) {
    case 0:
      // Reset();
      for (int i = 0; i < 5; i++) {
          cmd[i] = 0; // Completamente fechado
      } break;
    case 1:
      for (int i = 0; i < 5; i++) {
          cmd[i] = 2; // Completamente aberto
      } break;
    case 2:
      for (int i = 0; i < 5; i++) {
          cmd[i] = 1; // Completamente no ponto médio
      } break;
    default:
      aux_eureka = -1;
      break;
  }
  for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
    switch (cmd[servoIndex]) { // Mapeia a variável de comando para o input de cada motor
      case 0: // Fechar
        input[servoIndex] = ClearanceMin[servoIndex];
        // Serial.print(" - input_min: ");
        break;
      case 1: // Ponto Médio
        input[servoIndex] = (ClearanceMin[servoIndex] + ClearanceMax[servoIndex]) / 2;
        // Serial.print(" - input_med: ");
        break;
      case 2: // Abrir
        input[servoIndex] = ClearanceMax[servoIndex];
        // Serial.print(" - input_max: ");
        break;
    }
    

    erro[servoIndex] = input[servoIndex] - setPoint_ant[servoIndex]; // Calcula erro
    if (abs(erro[servoIndex]) > abs(maiorErro)) {
      maiorErro = erro[servoIndex]; // Calcula Maior Erro
    }
    setPoint[servoIndex] = setPoint_ant[servoIndex] + erro[servoIndex]; // Ajusta setPoint conforme o input e o erro
    // Limita o setPoint pela Clearance de cada motor
    if ((servoIndex + 1) % 2 == 1) { // Se servoIndex + 1 for ímpar
      if (setPoint[servoIndex] <= ClearanceMax[servoIndex]) {
        setPoint[servoIndex] = ClearanceMax[servoIndex];
      }
    } else { // Se servoIndex for par
      if (setPoint[servoIndex] >= ClearanceMax[servoIndex]) {
        setPoint[servoIndex] = ClearanceMax[servoIndex];
      }
    }
    Serial.print(servoIndex);
    Serial.print(" - input: ");
    Serial.print(input[servoIndex]);
    Serial.print(" - setPoint_ant: ");
    Serial.print(setPoint_ant[servoIndex]);
    Serial.print(" - erro: ");
    Serial.print(erro[servoIndex]);
    Serial.print(" - setPoint: ");
    Serial.println(setPoint[servoIndex]);
  }
  // Serial.println();
  // Aciona os motores 
  for (int servoAngle = 0; servoAngle <= abs(maiorErro); servoAngle++) {
    for (int servoIndex = 0; servoIndex < numServos; servoIndex++) {
      if (erro[servoIndex] >= 0) {
        setPoint_ant[servoIndex] += servoAngle;
      } else {
        setPoint_ant[servoIndex] -= servoAngle;
      }
      // Atualiza setPoint_ant após o cálculo com a Clearance Mínima de cada motor
      setPoint_ant[servoIndex] += ClearanceMin[servoIndex];
      if (setPoint_ant[servoIndex] >= setPoint[servoIndex]) {
          setPoint_ant[servoIndex] = setPoint[servoIndex];
      }
    }
    Serial.println(setPoint_ant[3]); 
    SERVO_1.write(setPoint_ant[0]); 
    SERVO_2.write(setPoint_ant[1]); 
    SERVO_3.write(setPoint_ant[2]); 
    SERVO_4.write(setPoint_ant[3]); 
    SERVO_5.write(setPoint_ant[4]); 
    delay(30);
  }
  aux_eureka++; 
  delay(3000);
}

// void Debug(){
//   int aux_debug = 1;
//   int cmd[5];
//   // Define a variável de comando 
//   if (aux_debug == 0){
//     int cmd[5] = {0, 0, 0, 0, 0}; // Completamente aberto
//   }
//   if (aux_debug == 1){
//     int cmd[5] = {2, 2, 2, 2, 2}; // Completamente fechado
//   }
//   if (aux_debug == 2){
//     int cmd[5] = {1, 1, 1, 1, 1}; // Completamente no ponto médio
//   }
//   if (aux_debug == 3){
//     int cmd[5] = {2, 1, 0, 0, 2}; // Sinal de OK com as mãos
//   }

//   // Mapeia a variável de comando para o input de cada motor
//   for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
//     if (cmd[servoIndex] == 0) {
//       input[servoIndex] = MinBend[servoIndex];
//     }
//     else if (cmd[servoIndex] == 1) {
//       input[servoIndex] = MedBend[servoIndex];
//     }
//     else if (cmd[servoIndex] == 2) {
//       input[servoIndex] = MaxBend[servoIndex];
//     }
//   }
//   // Calcula a resistência de cada Flex Sensor e estima a flexão em graus
//   ValorFlex();
//   // Calcula o erro de posição
//   for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
//     erro[servoIndex] = input[servoIndex] - angle[servoIndex];
//   }
//   // Ajusta a escala do erro para se adequar a escala dos motores
//   for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
//     erro[servoIndex] = map(erro[servoIndex], MinBend[servoIndex], MaxBend[servoIndex], ClearanceMin[servoIndex], ClearanceMax[servoIndex]);
//   }
//   // Ajusta setPoint conforme o erro
//   for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
//     setPoint[servoIndex] = setPoint[servoIndex] + erro[servoIndex];
//   }
//     // Ajusta setPoint conforme a clearance de cada motor
//   for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
//     if (setPoint[servoIndex] >= ClearanceMax[servoIndex]){
//       setPoint[servoIndex] = ClearanceMax[servoIndex];
//     }
//     else if (setPoint[servoIndex] <= ClearanceMin[servoIndex]){
//       setPoint[servoIndex] = ClearanceMin[servoIndex];
//     }
//   }
//   // Define Maior Erro
//   int maiorErro = erro[0];
//   for (int i = 1; i < 5; i++) {
//     if (abs(erro[i]) > abs(maiorErro)) {
//       maiorErro = erro[i]; 
//     }
//   } 
//   // Aciona o motor
//   if (maiorErro <= 0) {
//     // Abrir a mão
//     for (int servoAngle = Clearance; servoAngle >= 0; servoAngle--) { 
//       int servoAngle_2 = map(servoAngle, 0, Clearance, 180, (180 - Clearance));
//       // Servo 1
//       if (servoAngle >= setPoint_ant[0]){
//         SERVO_1.write(setPoint_ant[0]);
//       } else if (servoAngle <= ClearanceMax[0]) {
//         SERVO_1.write(ClearanceMax[0]);
//       } else {
//         SERVO_1.write(servoAngle);
//       } 
//       // Servo 2
//       if (servoAngle >= setPoint_ant[1]){
//         SERVO_2.write(setPoint_ant[1]);
//       } else if (servoAngle <= ClearanceMax[1]) {
//         SERVO_2.write(ClearanceMax[1]);
//       } else {
//         SERVO_2.write(servoAngle);
//       } 
//       // Servo 3
//       if (servoAngle >= setPoint_ant[2]){
//         SERVO_3.write(setPoint_ant[2]);
//       } else if (servoAngle <= ClearanceMax[2]) {
//         SERVO_3.write(ClearanceMax[2]);
//       } else {
//         SERVO_3.write(servoAngle);
//       } 
//       // Servo 4
//       if (servoAngle >= setPoint_ant[3]){
//         SERVO_4.write(setPoint_ant[3]);
//       } else if (servoAngle <= ClearanceMax[3]) {
//         SERVO_4.write(ClearanceMax[3]);
//       } else {
//         SERVO_4.write(servoAngle);
//       } 
//       // Servo 5
//       if (servoAngle >= setPoint_ant[4]){
//         SERVO_5.write(setPoint_ant[4]);
//       } else if (servoAngle <= ClearanceMax[4]) {
//         SERVO_5.write(ClearanceMax[4]);
//       } else {
//         SERVO_5.write(servoAngle);
//       } 
//       delay(15);
//     }
//   }
//   else if (maiorErro > 0) {
//     // Fechar a mão
//     for (int servoAngle = 0; servoAngle >= Clearance; servoAngle++) { 
//       int servoAngle_2 = map(servoAngle, Clearance, 0, (180 - Clearance), 180);
//       // Servo 1
//         if (servoAngle <= setPoint_ant[0]){
//           SERVO_1.write(setPoint_ant[0]);
//         } else if (servoAngle >= ClearanceMax[0]) {
//           SERVO_1.write(ClearanceMax[0]);
//         } else {
//           SERVO_1.write(servoAngle);
//         } 
//         // Servo 2
//         if (servoAngle <= setPoint_ant[1]){
//           SERVO_2.write(setPoint_ant[1]);
//         } else if (servoAngle >= ClearanceMax[1]) {
//           SERVO_2.write(ClearanceMax[1]);
//         } else {
//           SERVO_2.write(servoAngle);
//         } 
//         // Servo 3
//         if (servoAngle <= setPoint_ant[2]){
//           SERVO_3.write(setPoint_ant[2]);
//         } else if (servoAngle >= ClearanceMax[2]) {
//           SERVO_3.write(ClearanceMax[2]);
//         } else {
//           SERVO_3.write(servoAngle);
//         } 
//         // Servo 4
//         if (servoAngle <= setPoint_ant[3]){
//           SERVO_4.write(setPoint_ant[3]);
//         } else if (servoAngle >= ClearanceMax[3]) {
//           SERVO_4.write(ClearanceMax[3]);
//         } else {
//           SERVO_4.write(servoAngle);
//         } 
//         // Servo 5
//         if (servoAngle <= setPoint_ant[4]){
//           SERVO_5.write(setPoint_ant[4]);
//         } else if (servoAngle >= ClearanceMax[4]) {
//           SERVO_5.write(ClearanceMax[4]);
//         } else {
//           SERVO_5.write(servoAngle);
//         }
//         delay(15);
//     }
//   }
//   for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
//     setPoint_ant[numServos] = setPoint[servoIndex];
//   }
// }