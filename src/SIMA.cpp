#include "SIMA.h"


SIMA::SIMA(uint8_t ID1, uint8_t ID2, uint8_t dir_pin, uint8_t TX_pin, uint8_t RX_pin, int Baudrate, HardwareSerial& port, float left_w, float percent, float separation)
 : motors(ID1, ID2, dir_pin, TX_pin, RX_pin, Baudrate, port, left_w, percent, separation)
{
 x = 0;
 y = 0;
 angle = 0;
 motors.init();   
}

bool SIMA::rotate(float goal_angle, int speed, bool detection){
    motors.changeVelocity(speed, speed);
    float delta_angle = goal_angle - angle;

    if(!motors.rotateMotorsMM(delta_angle, detection)){
        return false;
    }

    angle = goal_angle;

    return true;
}

bool SIMA::translate(float distance, int speed, bool detection){
    motors.changeVelocity(speed, speed);
    motors.readPosition();

    if(!motors.moveMotorsMM(distance, distance, detection)){
        float traversed = motors.traversed_distance();
        x += traversed * cos(angle * 2 * PI / 360.0);
        y += traversed * sin(angle * 2 * PI / 360.0); 

        return false;
    }

    x += distance * cos(angle * 2 * PI / 360.0);
    y += distance * sin(angle * 2 * PI / 360.0); 

    return true;
}

bool SIMA::move(float goal_x, float goal_y, float goal_angle, int speed, uint8_t mode, bool detection){
    bool rotate_on_start = mode & 1; // mode works with 3 bits (001) is for rotate on start
    bool walk = mode & 2;       // (010) is for translate and (100) is for rotate on end of movement
    bool rotate_on_end = mode & 4;

    float dx = goal_x - x;
    float dy = goal_y - y;

    float distance = hypotf(dx, dy);
    float start_angle = atan2f(dy, dx) * 180/PI;

    if(rotate_on_start){
        if(!rotate(start_angle, speed, false)){
            return false;
        }
    }
    if(walk){
        if(!translate(distance, speed, detection)){
            return false;
        }
    }
    if(rotate_on_end){
        if(!rotate(goal_angle, speed, false)){
            return false;
        }
    }

    return true;
}