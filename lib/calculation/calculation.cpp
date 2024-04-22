#include <Arduino.h>
#include <stdint.h>
#include "calculation.h"

PIDCalc::PIDCalc(int *speedA, int *speedB){
    this->error = error;
    this->speedA = speedA;
    this->speedB = speedB;
}

PIDCalc::~PIDCalc(){}

void PIDCalc::calculate(){
    int *calculated = new int[1];
    accumulator += error;
    *calculated = (kp/pow(10, multiP))*error;                   //P
    *calculated += (ki/pow(10, multiI))*accumulator;            //I
    *calculated += (kd/pow(10,multiD))*(error - lastError);     //D
    Serial.print("New output: ");
    Serial.println(*calculated);
    *speedA = defaultSpeed + *calculated;
    *speedB = defaultSpeed - *calculated;
}

void PIDCalc::iterate(){
    position = qtr.readLineBlack(sensorValues);
    isOutOfLine = sensorValues[0]>=980 && sensorValues[1]>=980 && sensorValues[2]>=980 && sensorValues[3]>=980 && sensorValues[4]>=980;
    if(isOutOfLine){
        if(lastError < 0){
            *speedA = 230;
            *speedB = -230;
        }else{
            *speedA = -230;
            *speedB = 230;

        }
    }else{
        error = 2000 - position;
        this->calculate();
    }
}

void PIDCalc::setup(){
    // configure the sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){26, 27, 14, 12, 13}, SensorCount);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
    
    //SerialBT.begin();
    //Serial.println("Bluetooth Started! Ready to pair...");
    
    // analogRead() takes about 0.1 ms on an AVR.
    // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
    // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 10 seconds.
    for (uint16_t i = 0; i < 400; i++){
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

    // print the calibration minimum values measured when emitters were on
    Serial.print("Calibration done:: ");
    for (uint8_t i = 0; i < SensorCount; i++){
        threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i])/2;
        Serial.print(threshold[i]);
        Serial.print("  ");
    }
    Serial.println();
    delay(1000);
}
