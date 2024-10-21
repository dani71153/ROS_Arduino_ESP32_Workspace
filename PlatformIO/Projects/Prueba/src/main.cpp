#include <Arduino.h>

// Definición de los pines para el primer motor
const int pinEnable1 = 21;   // Pin PWM para EN1
const int pinMotor1A = 23;   // IN1
const int pinMotor1B = 22;   // IN2

// Definición de los pines para el segundo motor
const int pinEnable2 = 15;   // Pin PWM para ENB (Motor 2)
const int pinMotor2A = 2;    // IN4
const int pinMotor2B = 3;    // IN3

// Variables PWM
const int frecuenciaPWM = 490; // Frecuencia de PWM en Hz
const int canalPWM1 = 0;       // Canal de PWM para el motor 1
const int canalPWM2 = 1;       // Canal de PWM para el motor 2
const int resolucionPWM = 8;   // Resolución del PWM (8 bits)

// Velocidad máxima
const int velocidadMaxima = 255; // Rango de 0 a 255 para velocidad con 8 bits

void setup() {
  // Configurar los pines del primer motor como salida
  pinMode(pinMotor1A, OUTPUT);
  pinMode(pinMotor1B, OUTPUT);
  
  // Configurar los pines del segundo motor como salida
  pinMode(pinMotor2A, OUTPUT);
  pinMode(pinMotor2B, OUTPUT);
  
  // Configurar PWM en los pines EN1 y ENB
  ledcSetup(canalPWM1, frecuenciaPWM, resolucionPWM); // Canal 0, frecuencia, resolución para motor 1
  ledcAttachPin(pinEnable1, canalPWM1);               // Asignar pin PWM EN1 al canal 0

  ledcSetup(canalPWM2, frecuenciaPWM, resolucionPWM); // Canal 1, frecuencia, resolución para motor 2
  ledcAttachPin(pinEnable2, canalPWM2);               // Asignar pin PWM ENB al canal 1

  // Iniciar la comunicación serie
  Serial.begin(115200);
}

void loop() {
  // Girar el primer motor en una dirección
  Serial.println("Girando el motor 1 hacia adelante...");
  digitalWrite(pinMotor1A, HIGH); // IN1 = HIGH
  digitalWrite(pinMotor1B, LOW);  // IN2 = LOW
  
  // Ajustar velocidad del primer motor
  ledcWrite(canalPWM1, 500); // Velocidad máxima para motor 1 (255)

  // Girar el segundo motor en una dirección
  Serial.println("Girando el motor 2 hacia adelante...");
  digitalWrite(pinMotor2A, HIGH); // IN4 = HIGH
  digitalWrite(pinMotor2B, LOW);  // IN3 = LOW
  
  // Ajustar velocidad del segundo motor
  ledcWrite(canalPWM2, 0); // Velocidad máxima para motor 2 (255)
  delay(2000);

  // Detener ambos motores
  Serial.println("Deteniendo ambos motores...");
  ledcWrite(canalPWM1, 0); // Detener motor 1
  ledcWrite(canalPWM2, 0); // Detener motor 2
  delay(1000);
}
