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
int ClaranceMin_1 = 0;
int ClaranceMax_1 = 180;
Servo SERVO_2;
int ClaranceMin_2 = 0;
int ClaranceMax_2 = 180;
Servo SERVO_3;
int ClaranceMin_3 = 0;
int ClaranceMax_3 = 180;
Servo SERVO_4;
int ClaranceMin_4 = 0;
int ClaranceMax_4 = 180;
Servo SERVO_5;
int ClaranceMin_5 = 0;
int ClaranceMax_5 = 180;

// Declaração das Variáveis
String ValString;
String ValString_Res;
String ValString_Ang;
String input;
String selec = "A";
int botoes = 0;
int setPoint_1 = ClaranceMin_1;
int erro_1 = 0;
int setPoint_2 = ClaranceMin_2;
int erro_2 = 0;
int setPoint_3 = ClaranceMin_3;
int erro_3 = 0;
int setPoint_4 = ClaranceMin_4;
int erro_4 = 0;
int setPoint_5 = ClaranceMin_5;
int erro_5 = 0;
float angle_1, angle_2, angle_3, angle_4, angle_5;
int setPoint[5] = {setPoint_1, setPoint_2, setPoint_3, setPoint_4, setPoint_5};
int setPoint_ant;

// Declaração das Variáveis Constantes e Mensuráveis
const float VCC = 5.16; // Measured voltage of Ardunio 5V line

// Flex_1
const float R_DIV_1 = 45600.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_1 = 8480.0; // resistência com a mão aberta
const float BEND_RESISTANCE_1 = 18300.0; // resistência com a mão fechada
int MaxBend_1 = 90;
int MinBend_1 = 0;

// Flex_2
const float R_DIV_2 = 45200.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_2 = 10230.0; // resistência com a mão aberta
const float BEND_RESISTANCE_2 = 29000.0; // resistência com a mão fechada
int MaxBend_2 = 90;
int MinBend_2 = 0;

// Flex_3
const float R_DIV_3 = 45900.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_3 = 12020.0; // resistência com a mão aberta
const float BEND_RESISTANCE_3 = 28000.0; // resistência com a mão fechada
int MaxBend_3 = 90;
int MinBend_3 = 0;

// Flex_4
const float R_DIV_4 = 46100.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_4 = 10570.0; // resistência com a mão aberta
const float BEND_RESISTANCE_4 = 22800.0; // resistência com a mão fechada
int MaxBend_4 = 90;
int MinBend_4 = 0;

// Flex_5
const float R_DIV_5 = 46700.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_5 = 9570.0; // resistência com a mão aberta
const float BEND_RESISTANCE_5 = 17500.0; // resistência com a mão fechada
int MaxBend_5 = 90;
int MinBend_5 = 0;

void setup() 
{
  Serial.begin(9600);
  pinMode(FLEX_1, INPUT);
  pinMode(FLEX_2, INPUT);
  pinMode(FLEX_3, INPUT);
  pinMode(FLEX_4, INPUT);
  pinMode(FLEX_5, INPUT);
  pinMode(POTENCIOMETRO, INPUT);
  SERVO_1.attach(3);
  SERVO_2.attach(5);
  SERVO_3.attach(6);
  SERVO_4.attach(9);
  SERVO_5.attach(10);
}

void reset(){
  SERVO_1.write(ClaranceMin_1);
  delay(1500);
  SERVO_2.write(ClaranceMin_2);
  delay(1500);
  SERVO_3.write(ClaranceMin_3);
  delay(1500);
  SERVO_4.write(ClaranceMin_4);
  delay(1500);
  SERVO_5.write(ClaranceMin_5);
  delay(1500);
}

void ValorFlex() 
{
  // Zera a ValString
  ValString_Res = "";
  ValString_Ang = "";

  // Calcula a resistência de cada Flex Sensor e estima a flexão em graus
  // Flex_1
  int flexADC_1 = analogRead(FLEX_1);
  float flexV_1 = flexADC_1 * VCC / 1023.0;
  float flexR_1 = R_DIV_1 * (VCC / flexV_1 - 1.0);
  angle_1 = map(flexR_1, STRAIGHT_RESISTANCE_1, BEND_RESISTANCE_1, 0, 90.0);

  // Flex_2
  int flexADC_2 = analogRead(FLEX_2);
  float flexV_2 = flexADC_2 * VCC / 1023.0;
  float flexR_2 = R_DIV_2 * (VCC / flexV_2 - 1.0);
  angle_2 = map(flexR_2, STRAIGHT_RESISTANCE_2, BEND_RESISTANCE_2, 0, 90.0);

  // Flex_3
  int flexADC_3 = analogRead(FLEX_3);
  float flexV_3 = flexADC_3 * VCC / 1023.0;
  float flexR_3 = R_DIV_3 * (VCC / flexV_3 - 1.0);
  angle_3 = map(flexR_3, STRAIGHT_RESISTANCE_3, BEND_RESISTANCE_3, 0, 90.0);

  // Flex_4
  int flexADC_4 = analogRead(FLEX_4);
  float flexV_4 = flexADC_4 * VCC / 1023.0;
  float flexR_4 = R_DIV_4 * (VCC / flexV_4 - 1.0);
  angle_4 = map(flexR_4, STRAIGHT_RESISTANCE_4, BEND_RESISTANCE_4, 0, 90.0);

  // Flex_5
  int flexADC_5 = analogRead(FLEX_5);
  float flexV_5 = flexADC_5 * VCC / 1023.0;
  float flexR_5 = R_DIV_5 * (VCC / flexV_5 - 1.0);
  angle_5 = map(flexR_5, STRAIGHT_RESISTANCE_5, BEND_RESISTANCE_5, 0, 90.0);
  
  ValString_Res.concat(flexR_1);
  ValString_Res.concat(" ; ");
  ValString_Res.concat(flexR_2);
  ValString_Res.concat(" ; ");
  ValString_Res.concat(flexR_3);
  ValString_Res.concat(" ; ");
  ValString_Res.concat(flexR_4);
  ValString_Res.concat(" ; ");
  ValString_Res.concat(flexR_5);

  ValString_Ang.concat(angle_1);
  ValString_Ang.concat(" ; ");
  ValString_Ang.concat(angle_2);
  ValString_Ang.concat(" ; ");
  ValString_Ang.concat(angle_3);
  ValString_Ang.concat(" ; ");
  ValString_Ang.concat(angle_4);
  ValString_Ang.concat(" ; ");
  ValString_Ang.concat(angle_5);
  
  Serial.println("Resistance: " + ValString_Res + " ohms");
  Serial.println("Bend: " + ValString_Ang + " degrees");
  Serial.println();
}

void calibrar() {
  String flag_Max = "Não Gravado";
  String flag_Min = "Não Gravado";
  int Pot_Calib = analogRead(POTENCIOMETRO);
  int setPoint = map(Pot_Calib, 0, 1023, ClaranceMin_1, ClaranceMax_1);
  int erro_calib = setPoint - setPoint_ant;

  if (erro_calib >= 0) {
    // Inicia um loop onde i varia de setPoint_ant até setPoint
    for (int i = setPoint_ant; i <= setPoint; i++) {
      // Calcula as posições desejadas (setPoint_X) dos servos com base no valor de i
      setPoint_1 = map(i, 0, 1023, ClaranceMin_1, ClaranceMax_1);
      setPoint_2 = map(i, 0, 1023, ClaranceMin_2, ClaranceMax_2);
      setPoint_3 = map(i, 0, 1023, ClaranceMin_3, ClaranceMax_3);
      setPoint_4 = map(i, 0, 1023, ClaranceMin_4, ClaranceMax_4);
      setPoint_5 = map(i, 0, 1023, ClaranceMin_5, ClaranceMax_5);

      // Define as posições dos servos com base nos setPoints calculados
      SERVO_1.write(setPoint_1);
      SERVO_2.write(setPoint_2);
      SERVO_3.write(setPoint_3);
      SERVO_4.write(setPoint_4);
      SERVO_5.write(setPoint_5);
    }
  } 
  
  else if (erro_calib < 0) {
    // Inicia um loop onde i varia de setPoint_ant até setPoint (decremento)
    for (int i = setPoint_ant; i >= setPoint; i--) {
      // Calcula as posições desejadas (setPoint_X) dos servos com base no valor de i
      setPoint_1 = map(i, 0, 1023, ClaranceMin_1, ClaranceMax_1);
      setPoint_2 = map(i, 0, 1023, ClaranceMin_2, ClaranceMax_2);
      setPoint_3 = map(i, 0, 1023, ClaranceMin_3, ClaranceMax_3);
      setPoint_4 = map(i, 0, 1023, ClaranceMin_4, ClaranceMax_4);
      setPoint_5 = map(i, 0, 1023, ClaranceMin_5, ClaranceMax_5);

      // Define as posições dos servos com base nos setPoints calculados
      SERVO_1.write(setPoint_1);
      SERVO_2.write(setPoint_2);
      SERVO_3.write(setPoint_3);
      SERVO_4.write(setPoint_4);
      SERVO_5.write(setPoint_5);
    }
  }
  
  ValString.concat(setPoint_1);
  ValString.concat(" ; ");
  ValString.concat(setPoint_2);
  ValString.concat(" ; ");
  ValString.concat(setPoint_3);
  ValString.concat(" ; ");
  ValString.concat(setPoint_4);
  ValString.concat(" ; ");
  ValString.concat(setPoint_5);
  Serial.println("setPoints: " + ValString);

  if (digitalRead(B_Max) == HIGH) {
   
    ValorFlex();
    MaxBend_1 = angle_1;
    MaxBend_2 = angle_2;
    MaxBend_3 = angle_3;
    MaxBend_4 = angle_4;
    MaxBend_5 = angle_5;

    Serial.println("MaxBend Gravado");
    flag_Max = "Gravado";
    delay(3000);

  } else if (digitalRead(B_Min) == HIGH) {
   
    ValorFlex();
    MinBend_1 = angle_1;
    MinBend_2 = angle_2;
    MinBend_3 = angle_3;
    MinBend_4 = angle_4;
    MinBend_5 = angle_5;

    Serial.println("MinBend Gravado");
    flag_Min = "Gravado";
    delay(3000);
  }

  if (flag_Max == "Gravado" and flag_Min == "Gravado"){
    selec = "B";
  } else {
    selec = "C";
  }
  Serial.println("MaxBend: " + flag_Max + "MinBend: " + flag_Min + "Modo: " + selec);
  setPoint_ant = setPoint;
}

void loop(){
  if (Serial.available() > 0){
    Serial.println("Serial aberta para seleção do modo de operação");
    selec = Serial.readString();
  }
  
  Serial.println(selec);

  if (selec == "A") {
    Serial.println("Caso A - Reset");
    reset();  
  } else if (selec == "B") {
    Serial.println("Caso B - Run");
    run();
  } else if (selec == "C") {
    Serial.println("Caso C - Calibrar");
    calibrar();
  } else {
    Serial.println("Erro - Valor fora dos intervalos especificados");
  }
}

void run(){
  if (Serial.available() > 0){
    Serial.println("Serial aberta para recepção do input.");
    input = Serial.readString();
    Serial.println("input: " + input);
  }

  int input_1 = map(input.toInt(), 0, 90, MinBend_1, MaxBend_1);
  int input_2 = map(input.toInt(), 0, 90, MinBend_2, MaxBend_2);
  int input_3 = map(input.toInt(), 0, 90, MinBend_3, MaxBend_3);
  int input_4 = map(input.toInt(), 0, 90, MinBend_4, MaxBend_4);
  int input_5 = map(input.toInt(), 0, 90, MinBend_5, MaxBend_5);

  // Calcula a resistência de cada Flex Sensor e estima a flexão em graus
  ValorFlex();

  // Calcula o erro de posição
  erro_1 = input_1 - angle_1;
  erro_2 = input_2 - angle_2;
  erro_3 = input_3 - angle_3;
  erro_4 = input_4 - angle_4;
  erro_5 = input_5 - angle_5; 

  int erros[5] = {abs(erro_1), abs(erro_2), abs(erro_3), abs(erro_4), abs(erro_5)};
  int maiorErro = erros[0];

  for (int i = 1; i < 5; i++) {
    if (erros[i] > maiorErro) {
      maiorErro = erros[i]; 
    }
  }
  
  if (maiorErro <= 10) {
    // Ajusta a escala do setPoint para se adequar a escala dos motores
    setPoint_1 = map(setPoint_1, MinBend_1, MaxBend_1, ClaranceMin_1, ClaranceMax_1);
    setPoint_2 = map(setPoint_2, MinBend_2, MaxBend_2, ClaranceMin_2, ClaranceMax_2);
    setPoint_3 = map(setPoint_3, MinBend_3, MaxBend_3, ClaranceMin_3, ClaranceMax_3);
    setPoint_4 = map(setPoint_4, MinBend_4, MaxBend_4, ClaranceMin_4, ClaranceMax_4);
    setPoint_5 = map(setPoint_5, MinBend_5, MaxBend_5, ClaranceMin_5, ClaranceMax_5);
    selec = "D";
  }
  else {
    // Ajusta a escala do setPoint para se adequar a escala dos sensores
    setPoint_1 = map(setPoint_1, ClaranceMin_1, ClaranceMax_1, MinBend_1, MaxBend_1);
    setPoint_2 = map(setPoint_2, ClaranceMin_2, ClaranceMax_2, MinBend_2, MaxBend_2);
    setPoint_3 = map(setPoint_3, ClaranceMin_3, ClaranceMax_3, MinBend_3, MaxBend_3);
    setPoint_4 = map(setPoint_4, ClaranceMin_4, ClaranceMax_4, MinBend_4, MaxBend_4);
    setPoint_5 = map(setPoint_5, ClaranceMin_5, ClaranceMax_5, MinBend_5, MaxBend_5);

    // Ajusta setPoint cconforme o erro
    setPoint_1 = setPoint_1 + erro_1;
    setPoint_2 = setPoint_2 + erro_2;
    setPoint_3 = setPoint_3 + erro_3;
    setPoint_4 = setPoint_4 + erro_4;
    setPoint_5 = setPoint_5 + erro_5;

    // Ajusta a escala do setPoint para se adequar a escala dos motores
    setPoint_1 = map(setPoint_1, MinBend_1, MaxBend_1, ClaranceMin_1, ClaranceMax_1);
    setPoint_2 = map(setPoint_2, MinBend_2, MaxBend_2, ClaranceMin_2, ClaranceMax_2);
    setPoint_3 = map(setPoint_3, MinBend_3, MaxBend_3, ClaranceMin_3, ClaranceMax_3);
    setPoint_4 = map(setPoint_4, MinBend_4, MaxBend_4, ClaranceMin_4, ClaranceMax_4);
    setPoint_5 = map(setPoint_5, MinBend_5, MaxBend_5, ClaranceMin_5, ClaranceMax_5);

    selec = "B";
  }

  // Limitador de software para segurança
  // Limite inferior
  if (setPoint_1 <= MinBend_1){
    setPoint_1 = MinBend_1;}
  if (setPoint_2 <= MinBend_2){
    setPoint_2 = MinBend_2;}
  if (setPoint_3 <= MinBend_3){
    setPoint_3 = MinBend_3;}
  if (setPoint_4 <= MinBend_4){
    setPoint_4 = MinBend_4;}
  if (setPoint_5 <= MinBend_5){
    setPoint_5 = MinBend_5;}

  //Limite Superior
  if (setPoint_1 >= MaxBend_1){
    setPoint_1 = MaxBend_1;}
  if (setPoint_1 >= MaxBend_1){
    setPoint_2 = MaxBend_2;}
  if (setPoint_1 >= MaxBend_1){
    setPoint_3 = MaxBend_3;}
  if (setPoint_1 >= MaxBend_1){
    setPoint_4 = MaxBend_4;}
  if (setPoint_1 >= MaxBend_1){
    setPoint_5 = MaxBend_5;}
  
  ValString.concat(setPoint_1);
  ValString.concat(" ; ");
  ValString.concat(setPoint_2);
  ValString.concat(" ; ");
  ValString.concat(setPoint_3);
  ValString.concat(" ; ");
  ValString.concat(setPoint_4);
  ValString.concat(" ; ");
  ValString.concat(setPoint_5);
  Serial.println("setPoints: " + ValString);

  for (int i = 0; i < 5; i++){ 
    SERVO_1.write(setPoint_1);
    SERVO_2.write(setPoint_2);
    SERVO_3.write(setPoint_3);
    SERVO_4.write(setPoint_4);
    SERVO_5.write(setPoint_5);
  }
}