#include <Arduino.h>
#include <string>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <stdio.h>

//#include <iostream>
//#include <vector>
//using namespace std;

//#include <WiFi.h>
//#include "esp_wifi.h"

/* FOR BLUETOOTH CONNECTION */
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

/* For LCD */
#include <Wire.h>              //for ESP8266 use bug free i2c driver https://github.com/enjoyneering/ESP8266-I2C-Driver
#include <LiquidCrystal_I2C.h>

#define COLUMS 16
#define ROWS   2

#define PAGE   ((COLUMS) * (ROWS))

LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

String modo = "AUTOMATIC";
#define pinBTN 18
//

#define EnableA  27   // Enable motors Right        GPIO15(D8)        
#define EnableB  4    // Enable motors Left         GPIO4 (D2)
#define IN1  14       // L298N in1 motors Right     GPIO13(D7)
#define IN2  12        // L298N in2 motors Right     GPIO12(D6)
#define IN3  13        // L298N in3 motors Left      GPIO14(D5)
#define IN4  15         // L298N in4 motors Left      GPIO0 (D3)

/*----------------------*/
// Setting PWM properties
int freq = 3000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int resolution = 8;
int dutyCycle = (pow(2, resolution) - 1) * 0.65; // -2500- duty cycle value from 0 to 255, dependiendo de la resolución
/*----------------------*/
/*----------------------*/

int speedCar = dutyCycle;         // (400 - 1023)( 10bits resolution )

// for ultrasonic
/*const int trigPin1 = 26;
const int echoPin1 = 32;
const int trigPin2 = 25;
const int echoPin2 = 35;
const int trigPin3 = 33;
const int echoPin3 = 34;*/
#define trigPin1 GPIO_NUM_26
#define echoPin1 GPIO_NUM_32
#define trigPin2 GPIO_NUM_25
#define echoPin2 GPIO_NUM_35
#define trigPin3 GPIO_NUM_33
#define echoPin3 GPIO_NUM_34

//#define MOTOR_PIN 18 

float distanceLeft; //distancia en centímetros
float distanceFront; //distancia en centímetros
float distanceRight; //distancia en centímetros

long t = 0;
float distancia = 0;
//#define SOUND_SPEED 0.034 // 343.2 m/s

float getDistance(gpio_num_t trig, gpio_num_t echo){

  // Read data from the HC-SR04 distance sensor 
  gpio_set_level(trig, 0);
  delayMicroseconds(2);
  gpio_set_level(trig, 1); 
  delayMicroseconds(10); 
  gpio_set_level(trig, 0); 
  // Measure the duration of the echo pulse 
  t = pulseIn(echo, HIGH); 
  //Calculate the distance 

  distancia = t * 0.034 / 2; 

  return distancia;
}

void getFrontDistance(){
  distanceFront = getDistance(trigPin2, echoPin2);
  /* Serial.print("Front Distance: ");
  Serial.println(distanceFront); */
}

void getLeftDistance(){
  distanceLeft = getDistance(trigPin1, echoPin1);
  /* Serial.print("Left Distance: ");
  Serial.println(distanceLeft); */
}

void getRightDistance(){
  distanceRight = getDistance(trigPin3, echoPin3);
  /* Serial.print("Right Distance: ");
  Serial.println(distanceRight); */
}

void updateDistances(){
  getFrontDistance();
  getLeftDistance();
  getRightDistance();
}

String currentMovement = "nothing";

void showMovementLCD(const char movement[]){
  lcd.clear();
  delay(20);
  lcd.setCursor(3,0);	//columna - fila
  lcd.print(modo);
  lcd.setCursor(0,1);
  lcd.print(movement);
}

void stopRobot(){  

  if(currentMovement != "STOP"){

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    showMovementLCD("Detenido");
    currentMovement = "STOP";
  }

}

void goAhead(){ 

  //Para evitar demasiados cambios
  if(currentMovement != "FORWARD"){

    //stopRobot(); // Evitar cambio brusco

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    showMovementLCD("Hacia Adelante");
    currentMovement = "FORWARD";
  }
  
  // Se actualizan mediciones solo si está en automático
  if(modo == "AUTOMATIC") updateDistances();

}

void goBack(){ 

    if(currentMovement != "BACKWARD"){

      //stopRobot(); // Evitar cambio brusco

      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    showMovementLCD("Reversa");
    currentMovement = "BACKWARD";
  }

  // Se actualizan mediciones solo si está en automático;
  if(modo == "AUTOMATIC") updateDistances();

}

void goRight(){ 

  if(currentMovement != "RIGHT"){

    //stopRobot(); // Evitar cambio brusco

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    showMovementLCD("A la derecha");
    currentMovement = "RIGHT";
  }

  // Se actualizan mediciones solo si está en automático;
  if(modo == "AUTOMATIC") updateDistances();

}

void goLeft(){

  if(currentMovement != "LEFT"){

    //stopRobot(); // Evitar cambio brusco

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    showMovementLCD("A la Izquierda");
    currentMovement = "LEFT";
  }

  // Se actualizan mediciones solo si está en automático;
  if(modo == "AUTOMATIC") updateDistances();

}

bool modoCambio = false;

void setup() {

  Serial.begin(9600);

  Serial.println("");
  delay(100);

  //pinMode(EnableA, OUTPUT);
  //pinMode(EnableB, OUTPUT);  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(pinBTN, INPUT_PULLUP);

  // Sensores de distancia
  /*
  pinMode(trigPin1, OUTPUT); //pin como salida
  pinMode(echoPin1, INPUT);  //pin como entrada

  pinMode(trigPin2, OUTPUT); //pin como salida
  pinMode(echoPin2, INPUT);  //pin como entrada

  pinMode(trigPin3, OUTPUT); //pin como salida
  pinMode(echoPin3, INPUT);  //pin como entrada
  */
  gpio_set_direction(trigPin1, GPIO_MODE_OUTPUT); 
  gpio_set_direction(echoPin1, GPIO_MODE_INPUT); 
  gpio_set_direction(trigPin2, GPIO_MODE_OUTPUT); 
  gpio_set_direction(echoPin2, GPIO_MODE_INPUT); 
  gpio_set_direction(trigPin3, GPIO_MODE_OUTPUT); 
  gpio_set_direction(echoPin3, GPIO_MODE_INPUT); 

  /*-----------------------------------------------*/
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel0, freq, resolution);
  ledcSetup(pwmChannel1, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(EnableA, pwmChannel0);
  ledcAttachPin(EnableB, pwmChannel1);
   // Create the selected output voltage
  ledcWrite(pwmChannel0, speedCar); // x V
   // Create the selected output voltage
  ledcWrite(pwmChannel1, speedCar); // x V
  /*-----------------------------------------------*/

  /* For LCD */
  //
  while (lcd.begin(COLUMS, ROWS) != 1) //colums - 20, rows - 4
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(5000);   
  }

  lcd.print(F("PCF8574 is OK..."));    //(F()) saves string to flash & keeps dynamic memory free
  delay(2000);

  lcd.clear();
  //

  lcd.print(F("Testing Motors"));
  delay(1000);
  stopRobot(); delay(500);
  goLeft(); delay(500);
  stopRobot(); delay(500);
  goRight();delay(500);
  stopRobot(); delay(500);
  lcd.clear();
  lcd.print(F("Motors Ready"));
  delay(1000);
  lcd.clear();
  lcd.print(F("RANGER1 -> ON"));
  delay(1000);
  lcd.clear();

  /* FOR BLUETOOTH CONNECTION */
  //
  SerialBT.begin("ESP32tVeraGalvez"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with Bluetooth!");
  //

  //INTERRUPCIÓN para el cambio del modo
  //attachInterrupt(digitalPinToInterrupt(pinBTN),isr,FALLING); // Se define la interrupción

}

//boolean sensorDisable = false;
void disableSensors(){
  gpio_set_direction(trigPin1, GPIO_MODE_DISABLE); 
  gpio_set_direction(echoPin1, GPIO_MODE_DISABLE); 
  gpio_set_direction(trigPin2, GPIO_MODE_DISABLE); 
  gpio_set_direction(echoPin2, GPIO_MODE_DISABLE); 
  gpio_set_direction(trigPin3, GPIO_MODE_DISABLE); 
  gpio_set_direction(echoPin3, GPIO_MODE_DISABLE);
  //sensorDisable = true;
}

void enableSensors(){
  gpio_set_direction(trigPin1, GPIO_MODE_OUTPUT); 
  gpio_set_direction(echoPin1, GPIO_MODE_INPUT); 
  gpio_set_direction(trigPin2, GPIO_MODE_OUTPUT); 
  gpio_set_direction(echoPin2, GPIO_MODE_INPUT); 
  gpio_set_direction(trigPin3, GPIO_MODE_OUTPUT); 
  gpio_set_direction(echoPin3, GPIO_MODE_INPUT); 
  //sensorDisable = true;
}

//For manual control
char command = 'W'; //Waiting 

//bool comandoRecibido = false; // Detectar que se está cambiando el comando enviado por Bluetooth
bool estuvoConectado = false; // Variable para evitar limpiar la pantalla demasiadas veces al perder conexión


// Para el funcionamiento por temporizador:
long previousMillis = 0; // almacenará tiempo transcurrido en cada iteración
long intervalOn = 0; // tiempo en ms que el motor estará encendido


void loop(){

  // Cambiar de modo al detectar conexión a Bluetooth
  if (SerialBT.available()) {
    command = SerialBT.read(); //comandoRecibido = true;
    //Serial.println(command);
    if(command == 'C'){ // Se espera el envío de C - de Conexión a Bluetooth
      /*modo = "MANUAL";
      stopRobot();*/
      //Serial.println("Conectado a dispositivo Bluetooth");
      modo = "AUTOMATIC"; // Previene el caso en el que se haya perdido la conexión y por ende el modo MANUAL ha quedado
      modoCambio = true;
      estuvoConectado = true;
    }
    else if(command == 'M'){ // Se espera el envío de M - de Modo (Alterado de Manual a Automático)
      /*modo = "MANUAL";
      stopRobot();*/
      modoCambio = true;
    }
  }else if(!SerialBT.connected() && estuvoConectado){
      stopRobot();
      currentMovement = ""; // Se actualiza porque pudo haber desconectado en cualquier momento
      lcd.setCursor(0,0);	//columna - fila
      lcd.print("lost connection");
      estuvoConectado = false;
      // Send Message para ocultar controles
  }
  
  /*IMPLEMENTAR EL CAMBIO DE MODO DE FUNCIONAMIENTO CON INTERRUPCIONES*/
  // De manera Inicial se encuentra en automático y cada vez que se presione el botón se produce un cambio del modo
  /*
  if(digitalRead(pinBTN)==LOW){
    while(digitalRead(pinBTN)==LOW){};
    if(digitalRead(pinBTN)==HIGH ){
      modoCambio = true;
    }
    Serial.println("Cambiando modo...");
  }*/

  //Serial.print(modo);
  
  //Cada vez que se cambie de modo Limpiar LCDy escribir el modo de funcionamiento
  if(modoCambio) {
    
    /* Cada vez que se cambie de modo:  */
    // Actualizar modo
    if(modo == "AUTOMATIC") {
      modo = "MANUAL";
      // Disable the HC-SR04 distance sensor when not in use 
      disableSensors();
    }
    else {
      modo = "AUTOMATIC";
      enableSensors();
      //sensorDisable = false;
      SerialBT.disconnect(); command = 'W';
      //emparejado = false;
    }

    //Limpiar LCD y escribir el modo de funcionamiento actualizado -> Basta con llamar a detener el robot
    stopRobot();

    modoCambio = false;
    Serial.print("Modo Cambiado a: ");
    Serial.println(modo);
  }

  if (modo == "AUTOMATIC")
  {
    if ((distanceLeft >= 15 && distanceRight >= 15) && distanceFront >= 25)
    {
      goAhead();
      //delay(200);
    }
    else if(distanceLeft < 15 || distanceRight < 15)
    {
      if(distanceLeft >= distanceRight)
      {
        goLeft();
        delay(400);
        //stopRobot();
        //goAhead();
      }
      else if(distanceRight >= distanceLeft)
      {
        
        goRight();
        delay(400);
        //stopRobot();
        //goAhead();
      }
    }
    else if (distanceFront < 25)
    {
      stopRobot();
      delay(150);
      goBack();
      delay(400);
      stopRobot();
      delay(250);

      if(distanceRight >= distanceLeft)
        {
          goRight();
          delay(300);
          //stopRobot();
        }
      else if(distanceLeft >= distanceRight)
      {
        goLeft();
        delay(300);
        //stopRobot();
      }
    }
    
  }else if(modo == "MANUAL"){

    if(command != 'W'){
      if (command == 'F') goAhead(); // Forward
      else if (command == 'B') goBack(); //Backward
      else if (command == 'L') goLeft();
      else if (command == 'R') goRight();
      else if (command == 'S') stopRobot();

      command = 'W';
    }
  }

}