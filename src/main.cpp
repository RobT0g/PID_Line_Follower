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
int motorPinA0 = 11;
int motorPinA1 = 10;
int motorPinB0 = 9;
int motorPinB1 = 6;
const int offsetA = 1;
const int offsetB = 1;
int speedA = 0;
int speedB = 0;

PIDCalc pid{PIDCalc(&speedA, &speedB)};
MotorActual mA{MotorActual(&speedA, motorPinA0, motorPinA1, invertDirMotor)};
MotorActual mB{MotorActual(&speedB, motorPinB0, motorPinB1,  invertEsqMotor)};

void setup(){
    Serial.begin(115200);
    //pid.setup();
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop(){
    //pid.iterate();
    //mA.move();
    //mB.move();
    digitalWrite(LED_BUILTIN, 1);
    delay(500);
    digitalWrite(LED_BUILTIN, 0);
    delay(500);
}
