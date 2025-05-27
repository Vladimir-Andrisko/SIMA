#include "motorControl.h"

MotorControl::MotorControl(uint8_t ID1, uint8_t ID2, uint8_t dir_pin, uint8_t TX_pin, uint8_t RX_pin, int Baudrate, HardwareSerial& port, double left_w, double percent, double separation) : dxl(port, dir_pin), serial(port){
  id1 = ID1; id2 = ID2;
  ID_list[0] = id1; ID_list[1] = id2;
  baudrate = Baudrate;
  rx_pin = RX_pin; tx_pin = TX_pin;
  wheel_separation = separation;
  left_wheel_radius = left_w;
  right_wheel_radius = left_w * percent / 100;
}

bool MotorControl::init(){
  serial.begin(baudrate, SERIAL_8N1, rx_pin, tx_pin);
  delay(100);   // Allow time for serial port to initialize

  dxl.begin(baudrate);
  delay(100);   // Allow time for Dynamixel to stabilize

  dxl.setPortProtocolVersion(protocol);

  bool connected = true;
  uint8_t max_attempts = 100;

  // pings the motors 100 times and returns fail if only one motor fails
  for(int i = 0; i < id_count; i++){
    connected = false;
    for(int attempt = 0; attempt < max_attempts; attempt++){
      if(dxl.ping(ID_list[i])){
        connected = true;
        break;
      }
      delay(50);
    }

    if(!connected){
      return false;
    }
  }

  for(int i = 0; i < id_count; i++){
    dxl.torqueOff(ID_list[i]);
    dxl.setOperatingMode(ID_list[i], OP_EXTENDED_POSITION);
    dxl.torqueOn(ID_list[i]);
  }

  setupSync();
  if(!resetMotors()){
    return false;
  }

  return true;
}

void MotorControl::setupSyncRead(){
    sr_infos.packet.p_buf = user_pkt_buf;
    sr_infos.packet.buf_capacity = buffer_cap;
    sr_infos.packet.is_completed = false;
    sr_infos.addr = 132;
    sr_infos.addr_length = 4;
    sr_infos.p_xels = sr_infos_xels;
    sr_infos.xel_count = 0;

    for(int i = 0; i < id_count; i++){
        sr_infos_xels[i].id = ID_list[i];
        sr_infos_xels[i].p_recv_buf = (uint8_t*)&sr_data[i];
        sr_infos.xel_count++;
    }
    sr_infos.is_info_changed = true;  
}

void MotorControl::setupSyncVelocity(){
  sw_infos_vel.packet.p_buf = nullptr;
  sw_infos_vel.packet.is_completed = false;
  sw_infos_vel.addr = 112;
  sw_infos_vel.addr_length = 4;
  sw_infos_vel.p_xels = sw_infos_xels_vel;
  sw_infos_vel.xel_count = 0;

  for (int i = 0; i < id_count; i++) {
    sw_infos_xels_vel[i].id = ID_list[i];
    sw_infos_xels_vel[i].p_data = (uint8_t*)&sw_data_vel[i].data;
    sw_infos_vel.xel_count++;
  }
  sw_infos_vel.is_info_changed = true;
}

void MotorControl::setupSyncAcceleration(){
  sw_infos_acc.packet.p_buf = nullptr;
  sw_infos_acc.packet.is_completed = false;
  sw_infos_acc.addr = 108;
  sw_infos_acc.addr_length = 4;
  sw_infos_acc.p_xels = sw_infos_xels_acc;
  sw_infos_acc.xel_count = 0;

  for (int i = 0; i < id_count; i++) {
  sw_infos_xels_acc[i].id = ID_list[i];
  sw_infos_xels_acc[i].p_data = (uint8_t*)&sw_data_acc[i].data;
  sw_infos_acc.xel_count++;
  }
  sw_infos_acc.is_info_changed = true;
}

void MotorControl::setupSyncPosition(){
  sw_infos_pos.packet.p_buf = nullptr;
  sw_infos_pos.packet.is_completed = false;
  sw_infos_pos.addr = 116;
  sw_infos_pos.addr_length = 4;
  sw_infos_pos.p_xels = sw_infos_xels_pos;
  sw_infos_pos.xel_count = 0;

  for (int i = 0; i < id_count; i++) {
    sw_infos_xels_pos[i].id = ID_list[i];
    sw_infos_xels_pos[i].p_data = (uint8_t*)&sw_data_pos[i].data;
    sw_infos_pos.xel_count++;
  }
  sw_infos_pos.is_info_changed = true;
}

void MotorControl::setupSync(){
  setupSyncRead();
  setupSyncVelocity();
  setupSyncPosition();
  setupSyncAcceleration();
}

bool MotorControl::readPosition(){
  uint8_t recv_cnt = dxl.syncRead(&sr_infos);

  if(recv_cnt == id_count){
    for(int i = 0; i < id_count; i++){
      position[i] = sr_data[i].data;
    }
    return true;
  }

  return false;
}

bool MotorControl::readBeginOfPosition(){
  uint8_t recv_cnt = dxl.syncRead(&sr_infos);

  if(recv_cnt == id_count){
    for(int i = 0; i < id_count; i++){
      begin_of_position[i] = sr_data[i].data;
    }
    return true;
  }

  return false;
}

bool MotorControl::changeVelocity(int vel1, int vel2){
  sw_data_vel[0].data = vel1;
  sw_data_vel[1].data = vel2;

  sw_infos_vel.is_info_changed = true;

  if(dxl.syncWrite(&sw_infos_vel)){
    return true;
  }

  return false; 
}

bool MotorControl::changeAcceleration(int acc1, int acc2){
  sw_data_acc[0].data = acc1;
  sw_data_acc[1].data = acc2;

  sw_infos_acc.is_info_changed = true;

  if(dxl.syncWrite(&sw_infos_acc)){
    return true;
  }
  return false;
}

bool MotorControl::resetMotors(){
  int goal_positions[2] = {0, 0};

  sw_data_pos[0].data = goal_positions[0];
  sw_data_pos[1].data = goal_positions[1];

  sw_infos_pos.is_info_changed = true;

  if(!dxl.syncWrite(&sw_infos_pos)){
    return false;
  }

  while(abs(position[0] - goal_positions[0]) > 20 || abs(position[0] - goal_positions[0]) > 20){
    if(!readPosition()){
      continue;
    }
  }
  readBeginOfPosition();

  return true;

}

bool MotorControl::moveMotorsMM(double left, double right, bool detection = false){

  double ticks_per_mm_left = 4095.0 / left_wheel_radius;
  double ticks_per_mm_right = 4095.0 / right_wheel_radius;

  int offset_left = (int)round(left*ticks_per_mm_left);
  int offset_right = (int)round(right*ticks_per_mm_right);

  int velocity[2];

  readPosition();
  readBeginOfPosition();

  int goal_positions[2] = {position[0] + offset_left, position[1] - offset_right};

  sw_data_pos[0].data = goal_positions[0];
  sw_data_pos[1].data = goal_positions[1];
  sw_infos_pos.is_info_changed = true;
  dxl.syncWrite(&sw_infos_pos);

  while (abs(position[0] - goal_positions[0] > 20) ||  abs(position[1] - goal_positions[1] > 20)){
    readPosition();

    if(readSensorsTemp() && detection){
      changeAcceleration(300, 300);
      changeVelocity(0, 0);
      return false;
    }
  }

  return true;
}

bool MotorControl::rotateMotorsMM(float angle_deg, bool detection){
  float arch = 3.141592 * wheel_separation * angle_deg / 360;
  if(moveMotorsMM(arch, -arch, detection)){
    return true;
  }
  return false;
}

float MotorControl::traversed_distance(){
  double ticks_per_mm_left = 4095.0 / left_wheel_radius;
  double ticks_per_mm_right = 4095.0 / right_wheel_radius;

  readPosition();
  
  float dp_left = abs(position[0] - begin_of_position[0]);
  float dp_right = abs(position[1] - begin_of_position[1]);

  return (dp_left*ticks_per_mm_left + dp_right*ticks_per_mm_right)/2.0;
  
}

bool readSensorsTemp(){
  return false;       // will implement logic for sensors later
}                     // hopefully
