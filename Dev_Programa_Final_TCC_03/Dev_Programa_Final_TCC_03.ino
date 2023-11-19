#include <Servo.h> 
#define B_Max 13
#define B_Min 12
#define B_selec_1 1
#define B_selec_2 2

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
const float STRAIGHT_RESISTANCE_1 = 10000.0; // resistência com a mão aberta
const float BEND_RESISTANCE_1 = 14000.0; // resistência com a mão fechada

// Flex_2
const float R_DIV_2 = 45200.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_2 = 11000.0; // resistência com a mão aberta
const float BEND_RESISTANCE_2 = 21500.0; // resistência com a mão fechada

// Flex_3
const float R_DIV_3 = 45900.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_3 = 12000.0; // resistência com a mão aberta
const float BEND_RESISTANCE_3 = 24500.0; // resistência com a mão fechada

// Flex_4
const float R_DIV_4 = 46100.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_4 = 10000.0; // resistência com a mão aberta
const float BEND_RESISTANCE_4 = 25500.0; // resistência com a mão fechada

// Flex_5
const float R_DIV_5 = 46700.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_5 = 9000.0; // resistência com a mão aberta
const float BEND_RESISTANCE_5 = 15000.0; // resistência com a mão fechada

// Declaração das Variáveis
String ValString;
String ValString_Res;
String ValString_Ang;
String rotina;
int flag_min = 0;
int flag_max = 0;
int sensorIndex;
int servoIndex;
int botoes = 0;
int setPoint[numServos] = {ClearanceMin[0], ClearanceMin[1], ClearanceMin[2], ClearanceMin[3], ClearanceMin[4]};
int setPoint_ant[numServos] = {0, 0, 0, 0, 0};
int servoAngle[numServos];
int input[numServos];
int erro[numServos];
int somaErros = 0;
int maiorErro = 0;
int erroMedio;
int setPoint_ant_min = 0;
int setPoint_ant_max = 0;
int setPoint_min; // Mão Aberta
int setPoint_max; // Mão fechada
int angle[numSensors];
int flexR[numSensors];
int FLEX_PIN[numSensors] = {FLEX_1, FLEX_2, FLEX_3, FLEX_4, FLEX_5};
float R_DIV[numSensors] = {R_DIV_1, R_DIV_2, R_DIV_3, R_DIV_4, R_DIV_5};
float STRAIGHT_RESISTANCE[numSensors] = {STRAIGHT_RESISTANCE_1, STRAIGHT_RESISTANCE_2, STRAIGHT_RESISTANCE_3, STRAIGHT_RESISTANCE_4, STRAIGHT_RESISTANCE_5};
float BEND_RESISTANCE[numSensors] = {BEND_RESISTANCE_1, BEND_RESISTANCE_2, BEND_RESISTANCE_3, BEND_RESISTANCE_4, BEND_RESISTANCE_5};
int MaxBend[numSensors] = {0, 0, 0, 0, 0}; // = {ClearanceMax[0], ClearanceMax[1], ClearanceMax[2], ClearanceMax[3], ClearanceMax[4]};
int MedBend[numSensors] = {90, 90, 90, 90, 90};
int MinBend[numSensors] = {180, 180, 180, 180, 180}; // = {ClearanceMin[0], ClearanceMin[1], ClearanceMin[2], ClearanceMin[3], ClearanceMin[4]};
int cmd[5][numServos];

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
  pinMode(B_selec_1, INPUT);
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
  Serial.println("Resistance: " + ValString_Res + " ohms");
  Serial.println("Bend: " + ValString_Ang + " graus");
  // Serial.println();
  // delay(1000);
}

void loop() {
  int aux_selec = 4;
  if (aux_selec == 0){
    Reset();
  }
  else if (aux_selec == 1){
    Calibrar();
  }
  else if (aux_selec == 2){
    ValorFlex();
  }
  else if (aux_selec == 3){
    Run();
  }  
  else if (aux_selec == 4){
    Show();
  }
  
}

void Calibrar() {  
  // Mede Potenciometro
  int Pot_Calib = analogRead(POTENCIOMETRO);
  int pot_angle = map(Pot_Calib, 0, 1023, 20, 110);
  // Serial.println(pot_angle);

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

void Run(){
  // if (Serial.available() > 0){
  //   rotina = Serial.readString();
  //   Serial.println(rotina);
  // }
  rotina = "0";
  int n;

  // Define Matriz de Comando 
  if (rotina == "0"){
    n = 3;
    cmd[0][0] = 2;
    cmd[0][1] = 2;
    cmd[0][2] = 2;
    cmd[0][3] = 2;
    cmd[0][4] = 2;

    cmd[1][0] = 0;
    cmd[1][1] = 0;
    cmd[1][2] = 0;
    cmd[1][3] = 0;
    cmd[1][4] = 0;

    cmd[2][0] = 2;
    cmd[2][1] = 2;
    cmd[2][2] = 2;
    cmd[2][3] = 2;
    cmd[2][4] = 2;

    // Exibe matriz de comando
    // for (int d = 0; d < 3; d++) {
    //   for (int j = 0; j < 5; j++) {
    //     Serial.print(cmd[d][j]);
    //     Serial.print(" ");
    //   }
    //   Serial.println();
    // }
  }
  // Executa rotina
  for (int i = 0; i < n; i++) {
    Serial.println(i);
    // Mapeia a variável de comando para o input de cada motor
    for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
      switch (cmd[i][servoIndex]) {
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
          input[servoIndex] = MedBend[servoIndex];
          break;
      }
      // Serial.print("cmd: ");
      // Serial.println(cmd[i][servoIndex]);
      // Serial.print("Input: ");
      // Serial.println(input[servoIndex]);
    }

    // Define setPoint e erro
    ValorFlex(); // Obtém valor real da posição
    for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
      erro[servoIndex] = input[servoIndex] - angle[servoIndex]; // Calcula o erro de posição
      setPoint[servoIndex] = setPoint_ant[servoIndex] + erro[servoIndex]; // Calcula setPoint conforme o erro
      somaErros += erro[servoIndex]; // Calcula a soma dos erros
      if (abs(erro[servoIndex]) > abs(maiorErro)) {
        maiorErro = abs(erro[servoIndex]); // Define maior erro
      }
      if (setPoint_ant[servoIndex] > setPoint_ant_min) {
        setPoint_ant_min = setPoint_ant[servoIndex]; // Define o menor setPoint anterior - Mão Aberta
      }
      if (setPoint_ant[servoIndex] < setPoint_ant_max) {
        setPoint_ant_max = setPoint_ant[servoIndex]; // Define o maior setPoint anterior - Mão Fechada
      }
      Serial.print("setPoint: ");
      Serial.println(setPoint[servoIndex]);
      Serial.print("Erro: ");
      Serial.println(erro[servoIndex]);
    } 
    erroMedio = somaErros/numServos; // Calcula erro médio
    setPoint_min = setPoint_ant_min - maiorErro; // Define o menor setPoint - Mão Aberta
    setPoint_max = setPoint_ant_max + maiorErro; // Define o maioir setPoint - Mão Fechada
    
    bool aux = false;
    while (aux == false){
      Serial.println(aux);
      if (-10 <= erroMedio <= 10) {
        SERVO_1.write(servoAngle[0]);
        SERVO_2.write(servoAngle[1]);
        SERVO_3.write(servoAngle[2]);
        SERVO_4.write(servoAngle[3]);
        SERVO_5.write(servoAngle[4]);
        delay(30);
        aux = true;
        Serial.println(aux);
      }
      else if (erroMedio > 10) { // fechando a mão
        for (int servoAngle_aux = setPoint_ant_min; servoAngle <= setPoint_max; servoAngle_aux++) {
          Serial.println(servoAngle_aux);
          servoAngle[0] = servoAngle_aux;
          servoAngle[1] = 180 - servoAngle_aux;
          servoAngle[2] = servoAngle_aux;
          servoAngle[3] = 180 - servoAngle_aux;
          servoAngle[4] = servoAngle_aux;

          // Ajusta setPoint conforme a clearance de cada motor
          for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
            if (servoAngle[servoIndex] >= setPoint[servoIndex]){
              servoAngle[servoIndex] = setPoint[servoIndex];
            }
          }

          SERVO_1.write(servoAngle[0]);
          SERVO_2.write(servoAngle[1]);
          SERVO_3.write(servoAngle[2]);
          SERVO_4.write(servoAngle[3]);
          SERVO_5.write(servoAngle[4]);
          delay(15);

          if (servoAngle >= setPoint_max - 5) {
            setPoint_ant[0] = servoAngle[0];
            setPoint_ant[1] = servoAngle[1];
            setPoint_ant[2] = servoAngle[2];
            setPoint_ant[3] = servoAngle[3];
            setPoint_ant[4] = servoAngle[4];
          }
        }
      }
      else if (erroMedio < -10) { // abrindo a mão
        for (int servoAngle_aux = setPoint_ant_max; servoAngle >= setPoint_min; servoAngle_aux--) {
          Serial.println(servoAngle_aux);
          servoAngle[0] = servoAngle_aux;
          servoAngle[1] = 180 - servoAngle_aux;
          servoAngle[2] = servoAngle_aux;
          servoAngle[3] = 180 - servoAngle_aux;
          servoAngle[4] = servoAngle_aux;

          // Ajusta setPoint conforme a clearance de cada motor
          for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
            if (servoAngle[servoIndex] <= setPoint[servoIndex]){
              servoAngle[servoIndex] = setPoint[servoIndex];
            }
          }
          
          SERVO_1.write(servoAngle[0]);
          SERVO_2.write(servoAngle[1]);
          SERVO_3.write(servoAngle[2]);
          SERVO_4.write(servoAngle[3]);
          SERVO_5.write(servoAngle[4]);
          delay(15);

          if (servoAngle <= setPoint_min + 5) {
            setPoint_ant[0] = servoAngle[0];
            setPoint_ant[1] = servoAngle[1];
            setPoint_ant[2] = servoAngle[2];
            setPoint_ant[3] = servoAngle[3];
            setPoint_ant[4] = servoAngle[4];
          }
        }
      }
      // Define setPoint e erro
      ValorFlex(); // Obtém valor real da posição
      for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
        erro[servoIndex] = input[servoIndex] - angle[servoIndex]; // Calcula o erro de posição
        setPoint[servoIndex] = setPoint_ant[servoIndex] + erro[servoIndex]; // Calcula setPoint conforme o erro
        somaErros += erro[servoIndex]; // Calcula a soma dos erros
        if (abs(erro[servoIndex]) > abs(maiorErro)) {
          maiorErro = abs(erro[servoIndex]); // Define maior erro
        }
        if (setPoint_ant[servoIndex] > setPoint_ant_min) {
          setPoint_ant_min = setPoint_ant[servoIndex]; // Define o menor setPoint anterior - Mão Aberta
        }
        if (setPoint_ant[servoIndex] < setPoint_ant_max) {
          setPoint_ant_max = setPoint_ant[servoIndex]; // Define o maior setPoint anterior - Mão Fechada
        }
      } 
      erroMedio = somaErros/numServos; // Calcula erro médio
      setPoint_min = setPoint_ant_min - maiorErro; // Define o menor setPoint - Mão Aberta
      setPoint_max = setPoint_ant_max + maiorErro; // Define o maioir setPoint - Mão Fechada
    }
    SERVO_1.write(servoAngle[0]);
    SERVO_2.write(servoAngle[1]);
    SERVO_3.write(servoAngle[2]);
    SERVO_4.write(servoAngle[3]);
    SERVO_5.write(servoAngle[4]);
    delay(3000);
  }
}

void Show(){
  // if (Serial.available() > 0){
  //   rotina = Serial.readString();
  //   Serial.println(rotina);
  // }
  rotina = "0";
  int n;

  // Define Matriz de Comando 
  if (rotina == "0"){
    n = 3;
    cmd[0][0] = 2;
    cmd[0][1] = 2;
    cmd[0][2] = 2;
    cmd[0][3] = 2;
    cmd[0][4] = 2;

    cmd[1][0] = 0;
    cmd[1][1] = 0;
    cmd[1][2] = 0;
    cmd[1][3] = 0;
    cmd[1][4] = 0;

    cmd[2][0] = 2;
    cmd[2][1] = 2;
    cmd[2][2] = 2;
    cmd[2][3] = 2;
    cmd[2][4] = 2;

    // Exibe matriz de comando
    // for (int d = 0; d < 3; d++) {
    //   for (int j = 0; j < 5; j++) {
    //     Serial.print(cmd[d][j]);
    //     Serial.print(" ");
    //   }
    //   Serial.println();
    // }
  }
  // Executa rotina
  for (int i = 0; i < n; i++) {
    Serial.println(i);
    // Mapeia a variável de comando para o input de cada motor
    for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
      switch (cmd[i][servoIndex]) {
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
          input[servoIndex] = MedBend[servoIndex];
          break;
      }
      // Serial.print("cmd: ");
      // Serial.println(cmd[i][servoIndex]);
      // Serial.print("Input: ");
      // Serial.println(input[servoIndex]);
    }

    // Define setPoint e erro
    // somaErros = 0;
    for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
      erro[servoIndex] 
      setPoint[servoIndex] = input[servoIndex]
      somaErros += erro[servoIndex]; // Calcula a soma dos erros
      if (abs(erro[servoIndex]) > abs(maiorErro)) {
        maiorErro = abs(erro[servoIndex]); // Define maior erro
      }
      if (setPoint_ant[servoIndex] > setPoint_ant_min) {
        setPoint_ant_min = setPoint_ant[servoIndex]; // Define o menor setPoint anterior - Mão Aberta
      }
      if (setPoint_ant[servoIndex] < setPoint_ant_max) {
        setPoint_ant_max = setPoint_ant[servoIndex]; // Define o maior setPoint anterior - Mão Fechada
      }
      // Serial.print("setPoint: ");
      // Serial.println(setPoint[servoIndex]);
      // Serial.print("Erro: ");
      // Serial.println(erro[servoIndex]);
    } 
    erroMedio = somaErros/numServos; // Calcula erro médio
    setPoint_min = setPoint_ant_min - maiorErro; // Define o menor setPoint - Mão Aberta
    setPoint_max = setPoint_ant_max + maiorErro; // Define o maioir setPoint - Mão Fechada
    
    Serial.print("erroMedio: ");
    Serial.println(erroMedio);
    Serial.print("setPoint_min: ");
    Serial.println(setPoint_min);
    Serial.print("setPoint_max: ");
    Serial.println(setPoint_max);
    Serial.print("setPoint_ant_min: ");
    Serial.println(setPoint_ant_min);
    Serial.print("setPoint_ant_max: ");
    Serial.println(setPoint_ant_max);

    if (erroMedio >= - 10 and erroMedio <= 10) {
      Serial.println("Fim");
      SERVO_1.write(servoAngle[0]);
      SERVO_2.write(servoAngle[1]);
      SERVO_3.write(servoAngle[2]);
      SERVO_4.write(servoAngle[3]);
      SERVO_5.write(servoAngle[4]);
      delay(30);
    }
    else if (erroMedio > 10) { // fechando a mão
      for (int servoAngle_aux = setPoint_ant_min; servoAngle <= setPoint_max; servoAngle_aux++) {
        Serial.print("Abrido: ");
        Serial.println(servoAngle_aux);
        servoAngle[0] = servoAngle_aux;
        servoAngle[1] = 180 - servoAngle_aux;
        servoAngle[2] = servoAngle_aux;
        servoAngle[3] = 180 - servoAngle_aux;
        servoAngle[4] = servoAngle_aux;
        // Ajusta setPoint conforme a clearance de cada motor
        for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
          if (servoAngle[servoIndex] >= setPoint[servoIndex]){
            servoAngle[servoIndex] = setPoint[servoIndex];
          }
        }
        SERVO_1.write(servoAngle[0]);
        SERVO_2.write(servoAngle[1]);
        SERVO_3.write(servoAngle[2]);
        SERVO_4.write(servoAngle[3]);
        SERVO_5.write(servoAngle[4]);
        delay(15);
        if (servoAngle >= setPoint_max - 5) {
          setPoint_ant[0] = servoAngle[0];
          setPoint_ant[1] = servoAngle[1];
          setPoint_ant[2] = servoAngle[2];
          setPoint_ant[3] = servoAngle[3];
          setPoint_ant[4] = servoAngle[4];
        }
      }
    }
    if (erroMedio < -10) { // abrindo a mão
      for (int servoAngle_aux = setPoint_ant_max; servoAngle >= setPoint_min; servoAngle_aux--) {
        Serial.print("Fechando: ");
        Serial.println(servoAngle_aux);
        servoAngle[0] = servoAngle_aux;
        servoAngle[1] = 180 - servoAngle_aux;
        servoAngle[2] = servoAngle_aux;
        servoAngle[3] = 180 - servoAngle_aux;
        servoAngle[4] = servoAngle_aux;

        // Ajusta setPoint conforme a clearance de cada motor
        for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
          if (servoAngle[servoIndex] <= setPoint[servoIndex]){
            servoAngle[servoIndex] = setPoint[servoIndex];
          }
        }
        
        SERVO_1.write(servoAngle[0]);
        SERVO_2.write(servoAngle[1]);
        SERVO_3.write(servoAngle[2]);
        SERVO_4.write(servoAngle[3]);
        SERVO_5.write(servoAngle[4]);
        delay(15);

        if (servoAngle <= setPoint_min + 5) {
          setPoint_ant[0] = servoAngle[0];
          setPoint_ant[1] = servoAngle[1];
          setPoint_ant[2] = servoAngle[2];
          setPoint_ant[3] = servoAngle[3];
          setPoint_ant[4] = servoAngle[4];
        }
      }
    }
    // // Define setPoint e erro
    // ValorFlex(); // Obtém valor real da posição
    // for (servoIndex = 0; servoIndex < numServos; servoIndex++) {
    //   erro[servoIndex] = input[servoIndex] - setPoint_ant[servoIndex]; // Calcula o erro de posição
    //   setPoint[servoIndex] = setPoint_ant[servoIndex] + erro[servoIndex]; // Calcula setPoint conforme o erro
    //   somaErros += erro[servoIndex]; // Calcula a soma dos erros
    //   if (abs(erro[servoIndex]) > abs(maiorErro)) {
    //     maiorErro = abs(erro[servoIndex]); // Define maior erro
    //   }
    //   if (setPoint_ant[servoIndex] > setPoint_ant_min) {
    //     setPoint_ant_min = setPoint_ant[servoIndex]; // Define o menor setPoint anterior - Mão Aberta
    //   }
    //   if (setPoint_ant[servoIndex] < setPoint_ant_max) {
    //     setPoint_ant_max = setPoint_ant[servoIndex]; // Define o maior setPoint anterior - Mão Fechada
    //   }
    // } 
    // erroMedio = somaErros/numServos; // Calcula erro médio
    // setPoint_min = setPoint_ant_min - maiorErro; // Define o menor setPoint - Mão Aberta
    // setPoint_max = setPoint_ant_max + maiorErro; // Define o maioir setPoint - Mão Fechada
    delay(3000);
  }
}