#ifndef mavlink_DIY_H
#define mavlink_DIY_H

#include <Arduino.h>
#include <MAVLink.h>
#include <HardwareSerial.h>

extern HardwareSerial SerialPX4;
extern unsigned long now;
extern unsigned long last_setpoint;

// MAVLink Parameters
const uint8_t system_id = 1;       // ESP32 system ID
const uint8_t component_id = 200;  // ESP32 componentID
const uint8_t target_system = 1;   // PX4 system ID
const uint8_t target_component = 1;// PX4 component ID

typedef enum {
    POSITION_ONLY = 0,      //(x,y,z)
    VELOCITY_ONLY = 1,      // (vx,vy,vz)
    POSITION_YAW = 2,  // (x,y,z,yaw)
    VELOCITY_YAW = 3,  // (vx,vy,vz,yaw)
    POSITION_YAWRATE = 4, // (x,y,z,yaw_rate)
  } local_ned_mode;

void send_heartbeat();
void send_request_data_stream();
void command_set_mode(uint8_t base_mode, uint32_t custom_mode);
void command_set_mode_test(uint8_t base_mode, uint8_t main_mode, uint8_t sub_mode);
void command_arm();
void command_position_target_local_ned(float x, float y, float z);
void command_position_target_local_ned_test(local_ned_mode mode, const float params[11]);
bool position_check(float target_x, float target_y, float target_z); 
bool position_step_approach(float target_x, float target_y, float target_z, float v_desired);
void set_attitude_target(float roll, float pitch, float yaw, bool rate_control, float yaw_rate, float thrust);
bool attitude_check(float target_roll, float target_pitch, float target_yaw);
void receive_mavlink();

#endif
