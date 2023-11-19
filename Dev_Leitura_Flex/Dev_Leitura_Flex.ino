/******************************************************************************
Flex_Sensor_Example.ino
Example sketch for SparkFun's flex sensors
  (https://www.sparkfun.com/products/10264)
Jim Lindblom @ SparkFun Electronics
April 28, 2016

Create a voltage divider circuit combining a flex sensor with a 47k resistor.
- The resistor should connect from A0 to GND.
- The flex sensor should connect from A0 to 3.3V
As the resistance of the flex sensor increases (meaning it's being bent), the
voltage at A0 should decrease.

Development environment specifics:
Arduino 1.6.7
******************************************************************************/
const int FLEX_1 = A0; // Pin connected to voltage divider output
const int FLEX_2 = A1; // Pin connected to voltage divider output
const int FLEX_3 = A2; // Pin connected to voltage divider output
const int FLEX_4 = A3; // Pin connected to voltage divider output
const int FLEX_5 = A4; // Pin connected to voltage divider output

// Measure the voltage at 5V and the actual resistance of your 47k resistor, and enter them below

// Variáveis dos Flex
const float VCC = 5;  // Measured voltage of Ardunio 5V line

// Flex_1
const float R_DIV_1 = 46500.0;                // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_1 = 14000.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_1 = 30000.0;      // resistência com a mão fechada

// Flex_2
const float R_DIV_2 = 46700.0;                // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_2 = 18000.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_2 = 35000.0;      // resistência com a mão fechada

// Flex_3
const float R_DIV_3 = 46300.0;                // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_3 = 13000.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_3 = 35000.0;      // resistência com a mão fechada

// Flex_4
const float R_DIV_4 = 45800.0;                // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_4 = 10000.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_4 = 25000.0;      // resistência com a mão fechada

// Flex_5
const float R_DIV_5 = 47300.0;               // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE_5 = 12000.0;  // resistência com a mão aberta
const float BEND_RESISTANCE_5 = 20000.0;     // resistência com a mão fechada


void setup() 
{
  Serial.begin(9600);
  pinMode(FLEX_1, INPUT);
  pinMode(FLEX_2, INPUT);
  pinMode(FLEX_3, INPUT);
  pinMode(FLEX_4, INPUT);
  pinMode(FLEX_5, INPUT);
}

void loop() 
{
  // Read the ADC, and calculate voltage and resistance from it
  int flexADC_1 = analogRead(FLEX_1);
  float flexV_1 = flexADC_1 * VCC / 1023.0;
  float flexR_1 = R_DIV_1 * (VCC / flexV_1 - 1.0);

  int flexADC_2 = analogRead(FLEX_2);
  float flexV_2 = flexADC_2 * VCC / 1023.0;
  float flexR_2 = R_DIV_2 * (VCC / flexV_2 - 1.0);

  int flexADC_3 = analogRead(FLEX_3);
  float flexV_3 = flexADC_3 * VCC / 1023.0;
  float flexR_3 = R_DIV_3 * (VCC / flexV_3 - 1.0);

  int flexADC_4 = analogRead(FLEX_4);
  float flexV_4 = flexADC_4 * VCC / 1023.0;
  float flexR_4 = R_DIV_4 * (VCC / flexV_4 - 1.0);

  int flexADC_5 = analogRead(FLEX_5);
  float flexV_5 = flexADC_5 * VCC / 1023.0;
  float flexR_5 = R_DIV_5 * (VCC / flexV_5 - 1.0);

  Serial.print(flexR_1);
  Serial.print(" ");
  Serial.print(flexR_2);
  Serial.print(" ");
  Serial.print(flexR_3);
  Serial.print(" ");
  Serial.print(flexR_4);
  Serial.print(" ");
  Serial.print(flexR_5);


  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  float angle_1 = map(flexR_1, STRAIGHT_RESISTANCE_1, BEND_RESISTANCE_1, 0, 90.0);
  float angle_2 = map(flexR_2, STRAIGHT_RESISTANCE_2, BEND_RESISTANCE_2, 0, 90.0);
  float angle_3 = map(flexR_3, STRAIGHT_RESISTANCE_3, BEND_RESISTANCE_3, 0, 90.0);
  float angle_4 = map(flexR_4, STRAIGHT_RESISTANCE_4, BEND_RESISTANCE_4, 0, 90.0);
  float angle_5 = map(flexR_5, STRAIGHT_RESISTANCE_5, BEND_RESISTANCE_5, 0, 90.0);
  
  Serial.print(angle_1);
  Serial.print(" ; ");
  Serial.print(angle_2);
  Serial.print(" ; ");
  Serial.print(angle_3);
  Serial.print(" ; ");
  Serial.print(angle_4);
  Serial.print(" ; ");
  Serial.print(angle_5);

  Serial.println();

  delay(2000);
}
