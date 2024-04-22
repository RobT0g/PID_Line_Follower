#include <Arduino.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h"
#include <motor.h>
#include <calculation.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

bool invertDirMotor = false;
bool invertEsqMotor = false;
int motorPinA0 = 0;
int motorPinA1 = 1;
int motorPinB0 = 2;
int motorPinB1 = 3;
const int offsetA = 1;
const int offsetB = 1;
int speedA = 200;
int speedB = 200;

//PIDCalc pid{PIDCalc(&speedA, &speedB)};
MotorActual mA{MotorActual(&speedA, motorPinA0, motorPinA1, invertDirMotor)};
MotorActual mB{MotorActual(&speedB, motorPinB0, motorPinB1,  invertEsqMotor)};

void setup(){
    Serial.begin(115200);
    //pid.setup();
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(motorPinA0, OUTPUT);
    pinMode(motorPinA1, OUTPUT);
}

void loop(){
    //pid.iterate();
    //mA.move();
    //mB.move();
    analogWrite(motorPinA0, 200);
    analogWrite(motorPinA1, 0);
    delay(1000);
}
