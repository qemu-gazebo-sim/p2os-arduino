#include "p2os.hpp"
#include "p2os_config.hpp"
#include <ArduinoLog.h>

P2OS::P2OS(HardwareSerial& debug_serial, HardwareSerial& pioneer_serial) {
    this->debug_serial = &debug_serial;
    this->pioneer_serial = &pioneer_serial;
    this->p2os_comm = new P2OSCommunication(*(this->debug_serial), *(this->pioneer_serial));
}

P2OS::~P2OS() {
    delete this->p2os_comm;
}

int P2OS::setup() {
    if (this->p2os_comm->Setup()) {
        Log.errorln("p2os setup failed...");
        return -1;
    }

    Log.traceln("p2os_constructor");

    this->p2os_comm->ResetRawPositions();
    this->last_time_pulse = millis();
    this->p2os_comm->pulse = PULSE_INTERVAL;

    return 0;
}

int P2OS::shutdown() {
    if (this->p2os_comm->Shutdown()) {
        Log.errorln("p2os shutdown failed...");
        return -1;
    }

    return 0;
}

void P2OS::loop() {
    this->current_time = millis();  // ros::Time currentTime = ros::Time::now();

    this->p2os_comm->check_and_set_vel();
    this->p2os_comm->check_and_set_motor_state();

    if (this->p2os_comm->get_pulse() > 0) {
        if (this->p2os_comm->millis2Sec(this->current_time - this->last_time_pulse) > this->p2os_comm->get_pulse()) {
            Log.verboseln("sending pulse");
            this->p2os_comm->SendPulse();
            this->last_time_pulse = this->current_time;
        }
    }

    // Hack fix to get around the fact that if no commands are sent to the
    // robot via SendReceive, the driver will never read SIP packets and so
    // never send data back to clients. We need a better way of doing regular
    // checks of the serial port - peek in sendreceive, maybe? Because if there
    // is no data waiting this will sit around waiting until one comes
    this->p2os_comm->SendReceive(NULL, true);
    this->p2os_comm->updateDiagnostics();
}

nav_msgs::ros_p2os_data_t P2OS::get_p2dx_data() {
    return this->p2os_comm->p2os_data;
}

void P2OS::set_vel(geometry_msgs::Twist* msg) {
    this->p2os_comm->cmdvel_cb(msg);
}

void P2OS::set_motor_state(p2os_msgs::MotorState* msg) {
    this->p2os_comm->cmdmotor_state(msg);
}