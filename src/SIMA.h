#ifndef SIMA_H
#define SIMA_H

#include "pathfind.h"
#include "motorControl.h"

class SIMA{
private:
    float x;
    float y;
    float angle;

    MotorControl motors;
    Grid grid;

public:
    SIMA(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, int, HardwareSerial&, float, float, float);

    bool translate(float, int, bool);
    bool move(float, float, float, int, uint8_t, bool);
    bool rotate(float, int, bool);
    bool navigate(float, float, float);
};

#endif