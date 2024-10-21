#include <Arduino.h>
#include <ESP32Encoder.h>

// Encoder pins
#define LEFT_ENC_PIN_A 18
#define LEFT_ENC_PIN_B 19
#define RIGHT_ENC_PIN_A 34
#define RIGHT_ENC_PIN_B 35

// Variables PWM
const int frecuenciaPWM = 490; // Frecuencia de PWM en Hz
const int canalPWM1 = 0;       // Canal de PWM para el motor 1
const int canalPWM2 = 1;       // Canal de PWM para el motor 2
const int resolucionPWM = 8;   // Resolución del PWM (8 bits)

// Definición de los pines para el segundo motor
const int pinEnable2 = 15;   // Pin PWM para ENB (Motor 2)
const int pinMotor2A = 2;    // IN4
const int pinMotor2B = 3;    // IN3


// Encoder objects
ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;

void setup() {
  Serial.begin(115200);

  leftEncoder.attachHalfQuad(LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);
  leftEncoder.clearCount();

  rightEncoder.attachHalfQuad(RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B);
  rightEncoder.clearCount();

    // Configurar los pines del segundo motor como salida
  pinMode(pinMotor2A, OUTPUT);
  pinMode(pinMotor2B, OUTPUT);
  
  ledcSetup(canalPWM2, frecuenciaPWM, resolucionPWM); // Canal 1, frecuencia, resolución para motor 2
  ledcAttachPin(pinEnable2, canalPWM2);               // Asignar pin PWM ENB al canal 1


}

void loop() {
  // Read and print encoder values
  int64_t leftCount = leftEncoder.getCount();
  int64_t rightCount = rightEncoder.getCount();

  Serial.print("Left Encoder: ");
  Serial.print(leftCount);
  Serial.print("\tRight Encoder: ");
  Serial.println(rightCount);

  delay(100); // Adjust delay as needed
} 
