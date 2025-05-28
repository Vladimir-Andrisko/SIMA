#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "Dynamixel2Arduino/Dynamixel2Arduino.h"

using namespace ControlTableItem;


typedef struct sr_data{
    int data;
} sr_data_t;

typedef struct sw_data{
    int data;
} sw_data_t;


class MotorControl{
private:
    static const uint8_t id_count = 2;
    static const uint8_t protocol = 2;
    static const uint8_t buffer_cap = 128;

    Dynamixel2Arduino dxl;

    HardwareSerial& serial;
    uint8_t id1;
    uint8_t id2;
    int baudrate;
    uint8_t rx_pin;
    uint8_t tx_pin;

    uint8_t ID_list[id_count]; 
    uint8_t user_pkt_buf[buffer_cap];

    int position[id_count];
    int begin_of_position[id_count];

    double left_wheel_radius;
    double right_wheel_radius;
    double wheel_separation;


    // Sync read variables
    sr_data_t sr_data[id_count];
    DYNAMIXEL::InfoSyncReadInst_t sr_infos;
    DYNAMIXEL::XELInfoSyncRead_t sr_infos_xels[id_count];

    // Sync write variables
    sw_data_t sw_data_vel[id_count];
    sw_data_t sw_data_acc[id_count];
    sw_data_t sw_data_pos[id_count];

    DYNAMIXEL::InfoSyncWriteInst_t sw_infos_vel;
    DYNAMIXEL::InfoSyncWriteInst_t sw_infos_acc;
    DYNAMIXEL::InfoSyncWriteInst_t sw_infos_pos;
    DYNAMIXEL::XELInfoSyncWrite_t sw_infos_xels_vel[id_count];
    DYNAMIXEL::XELInfoSyncWrite_t sw_infos_xels_acc[id_count];
    DYNAMIXEL::XELInfoSyncWrite_t sw_infos_xels_pos[id_count];
public:
    MotorControl(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, int, HardwareSerial&, double, double, double);

    void setupSyncRead();
    void setupSyncAcceleration();
    void setupSyncVelocity();
    void setupSyncPosition();
    void setupSync();

    bool init();
    bool changeVelocity(int, int);
    bool changeAcceleration(int, int);
    bool resetMotors();

    bool readPosition();
    bool readBeginOfPosition();
    bool moveMotorsMM(double, double, bool);
    bool rotateMotorsMM(float, bool);
    float traversed_distance();

    bool serial_debugging(HardwareSerial& port = Serial);
    
};

bool readSensorsTemp();


#endif