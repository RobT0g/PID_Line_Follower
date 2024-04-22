#include <Arduino.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h"
#include <stdint.h>

const int defaultSpeed = 230;
const uint8_t SensorCount = 5;

class PIDCalc{
    private:
        float kp = 1, ki = 0.01, kd = 0.25;
        int multiP = 1, multiI = 1, multiD = 1;
        int accumulator = 0;
        int lastError = 0;
        int error = 0;
        int *speedA;
        int *speedB;
        
        uint16_t position = 0;
        uint16_t sensorValues[SensorCount];
        int threshold[SensorCount];
        bool isOutOfLine = false;

        QTRSensors qtr;
        BluetoothSerial SerialBT;

    public:
        PIDCalc(int*, int*);
        ~PIDCalc(void);
        void calculate();
        void iterate();
        void setup();
};
