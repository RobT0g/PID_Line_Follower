#include "motor.h"
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
 #include <pins_arduino.h>
#endif

MotorActual::MotorActual(int* speed, int pinA, int pinB, bool invert){
    this->speed = speed;
    this->pinA = pinA;
    this->pinB = pinB;
    this->invert = invert;
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
}
MotorActual::~MotorActual(){}

bool MotorActual::move(){
  if(*speed == 0) return false;
  if(*speed > 255) *speed = 255;
  else if(*speed < -255) *speed = -255;
  if(*speed > 0 == invert){
    analogWrite(pinA, *speed);
    analogWrite(pinB, 0);
  }else{
    analogWrite(pinB, *speed);
    analogWrite(pinA, 0);
  }
  Serial.print(motorId);
  Serial.print(" - ");
  Serial.println(*speed);
  return true;
}

void MotorActual::stop(){
    *speed = 0;
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
}

void MotorActual::setId(int id){
  this->motorId = id;
}
