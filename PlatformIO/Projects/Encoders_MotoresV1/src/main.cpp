#include <Arduino.h>
#include <Encoder.h>
// Motor 1
int pinIN1_M1 = 7;  // Pin IN1 del motor 1
int pinIN2_M1 = 8;  // Pin IN2 del motor 1
int pinPWM_M1 = 5;  // Pin PWM del motor 1

// Motor 2
int pinIN1_M2 = A13;  // Pin IN1 del motor 2
int pinIN2_M2 = A12;  // Pin IN2 del motor 2
int pinPWM_M2 = 44;   // Pin PWM del motor 2

// Motor 3 Ya esta en el ESP32. Para más informacion ve a Prueba.
int pinIN1_M3 = A15;  // Pin IN1 del motor 3
int pinIN2_M3 = A14;  // Pin IN2 del motor 3
int pinPWM_M3 = 46;   // Pin PWM del motor 3

// Motor 4 Ya esta en el ESP32. Para más informacion ve a Prueba.
int pinIN1_M4 = 9;  // Pin IN1 del motor 4
int pinIN2_M4 = 10; // Pin IN2 del motor 4
int pinPWM_M4 = 4;  // Pin PWM del motor 4


void detenerMotores();
void avanzarMotor(int pinIN1, int pinIN2, int pinPWM, int velocidad, bool adelante) ;


void setup() {
  // Configurar los pines de los motores como salidas
  pinMode(pinIN1_M1, OUTPUT);
  pinMode(pinIN2_M1, OUTPUT);
  pinMode(pinPWM_M1, OUTPUT);

  pinMode(pinIN1_M2, OUTPUT);
  pinMode(pinIN2_M2, OUTPUT);
  pinMode(pinPWM_M2, OUTPUT);

  pinMode(pinIN1_M3, OUTPUT);
  pinMode(pinIN2_M3, OUTPUT);
  pinMode(pinPWM_M3, OUTPUT);

  pinMode(pinIN1_M4, OUTPUT);
  pinMode(pinIN2_M4, OUTPUT);
  pinMode(pinPWM_M4, OUTPUT);
}

void loop() {

  /*Asumiendo una referencia.
  
  El arduino va a ser el origen de nuestro sistema coordenado. El norte seria en direccion al puerto USB C de entrada, y en direccion de 
  la PC. El Sur seria por donde va el ESP32. El oeste seria donde esta la bornera negra. Y el este es donde estan los fusibles.

  Primer motor es el que esta debajo de los fusibles. 
  Segudo motor es el que esta debajo de la computadora.
  El Motor tres es el que esta debajo de la bornera y el boton de encendido.
  EL motor 4 es el que esta debajo del la salida del stepdown.

   Pines de los Encoders.
   Amarillo = A
   Blanco = B
  
   Motor 1.   
   Entrada_A = 19
   Entrada_B = 18
   
   Motor 2.
   Entrada_ A = 2
   Entrada_B = 3

  Pines de los Encoders en el ESP32
  Motor 3
  #include <ESP32Encoder.h>
  #define EncoderPinA 18
  #define EncoderPinB 19
  Motor 4. 
  Encoder_PinA= D34
  Encoder_PinB= D35
  
  */
  // Personalizar el sentido y velocidad de cada motor
  avanzarMotor(pinIN1_M1, pinIN2_M1, pinPWM_M1, 255, true);  // Motor 1 adelante a velocidad máxima 
  avanzarMotor(pinIN1_M2, pinIN2_M2, pinPWM_M2, 255, true);  // Motor 2 adelante a velocidad 200
  avanzarMotor(pinIN1_M3, pinIN2_M3, pinPWM_M3, 0, false); // Motor 3 atrás a velocidad 150
  avanzarMotor(pinIN1_M4, pinIN2_M4, pinPWM_M4, 0, false);   // Motor 4 detenido
  delay(2000);

  // Detener todos los motores
  detenerMotores();
  delay(2000);
}

// Función para avanzar o retroceder motores individualmente
void avanzarMotor(int pinIN1, int pinIN2, int pinPWM, int velocidad, bool adelante) {
  if (adelante) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
  } else {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
  }
  analogWrite(pinPWM, velocidad);
}

// Función para detener todos los motores
void detenerMotores() {
  // Motor 1 detenido
  digitalWrite(pinIN1_M1, LOW);
  digitalWrite(pinIN2_M1, LOW);
  analogWrite(pinPWM_M1, 0);

  // Motor 2 detenido
  digitalWrite(pinIN1_M2, LOW);
  digitalWrite(pinIN2_M2, LOW);
  analogWrite(pinPWM_M2, 0);

  // Motor 3 detenido
  digitalWrite(pinIN1_M3, LOW);
  digitalWrite(pinIN2_M3, LOW);
  analogWrite(pinPWM_M3, 0);

  // Motor 4 detenido
  digitalWrite(pinIN1_M4, LOW);
  digitalWrite(pinIN2_M4, LOW);
  analogWrite(pinPWM_M4, 0);
}
