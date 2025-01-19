#ifndef _P2OS_HPP_
#define _P2OS_HPP_

#include "p2os_comm.hpp"
#include "p2os_msgs.hpp"
#include <Arduino.h>

class P2OS {
public:
    P2OS(HardwareSerial& debug_serial, HardwareSerial& pioneer_serial);

    ~P2OS();

    int setup();

    int shutdown();

    void loop();

    nav_msgs::ros_p2os_data_t get_p2dx_data();

    void set_vel(geometry_msgs::Twist* msg);

    void set_motor_state(p2os_msgs::MotorState* msg);

protected:
    HardwareSerial*    debug_serial;
    HardwareSerial*    pioneer_serial;
    P2OSCommunication* p2os_comm;
    uint32_t           last_time_pulse = 0;
    uint32_t           last_time_vel = 0;
    uint32_t           current_time;
    // uint32_t last_time;
};

#endif  // _P2OS_HPP_
