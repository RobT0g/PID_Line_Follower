#include <Arduino.h>
#include <stdint.h>

#define printMotor true

class MotorActual{
    private:
        int *speed;
        int pinA;
        int pinB;
        bool invert = false;
        int motorId = 0;

    public:
        bool move();
        void stop();
        void setId(int);
        MotorActual(int*, int, int, bool);
        ~MotorActual();
};