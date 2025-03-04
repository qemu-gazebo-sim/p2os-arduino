#include <Arduino.h>
#include <p2os.hpp>
#include <HardwareSerial.h>
#include <ArduinoLog.h>

#define PIONEER_SERIAL_RX 16
#define PIONEER_SERIAL_TX 17

HardwareSerial debug_serial(0);    // define a Serial for UART0
HardwareSerial pioneer_serial(2);  // define a Serial for UART2

P2OS* p2os;

void setup() {
    debug_serial.begin(9600);
    pioneer_serial.begin(9600, SERIAL_8N1, PIONEER_SERIAL_RX, PIONEER_SERIAL_TX);
    debug_serial.flush();
    pioneer_serial.flush();

    Log.begin(LOG_LEVEL_INFO, &debug_serial);

    p2os = new P2OS(pioneer_serial);

    Log.infoln("Ready!");
}

bool update_velocity(unsigned long loop_time, geometry_msgs::Twist* velocity) {
    static double vel_param[][2] = {
        {0.0, 0.0},  {0.1, 0.0},  {0.2, 0.0},   {0.3, 0.0},  {0.4, 0.0},  {0.5, 0.0},  {0.4, 0.0},
        {0.3, 0.0},  {0.2, 0.0},  {0.1, 0.0},   {0.0, 0.0},  {-0.1, 0.0}, {-0.2, 0.0}, {-0.3, 0.0},
        {-0.4, 0.0}, {-0.5, 0.0}, {-0.5, 0.0},  {-0.4, 0.0}, {-0.3, 0.0}, {-0.2, 0.0}, {-0.1, 0.0},
        {0.0, 0.0},  {0.0, 0.6},  {0.0, 1.2},   {0.0, 1.75}, {0.0, 1.2},  {0.0, 0.6},  {0.0, 0.0},
        {0.0, -0.6}, {0.0, -1.2}, {0.0, -1.75}, {0.0, -1.2}, {0.0, -0.6}, {0.0, 0.0},
    };

    static uint8_t num_total_pos = sizeof(vel_param) / sizeof(vel_param[0]);
    uint8_t        current_pos = uint8_t(loop_time / 1000);

    velocity->linear.x = vel_param[current_pos][0];   // 3.0;
    velocity->angular.z = vel_param[current_pos][1];  // 0.0;

    return ((current_pos + 1) < num_total_pos);
}

void loop() {
    geometry_msgs::Twist  msg_vel;
    p2os_msgs::MotorState msg_motor_state;

    bool is_send_velocity = !(p2os->setup());

    uint32_t current_loop_time = millis();
    uint32_t last_time_motor_state = millis();
    uint32_t velocity_update_loop_time = millis();

    while (is_send_velocity) {
        p2os->loop();

        is_send_velocity = update_velocity(millis() - velocity_update_loop_time, &msg_vel);
        p2os->set_vel(&msg_vel);

        if (millis() - last_time_motor_state > 100) {
            msg_motor_state.state = 1;
            p2os->set_motor_state(&msg_motor_state);
            last_time_motor_state = millis();
        }

        Log.infoln("current_loop_time: %d", (millis() - current_loop_time));
        current_loop_time = millis();
    }

    while (true) {
        msg_vel.linear.x = 0;
        msg_vel.angular.z = 0;
        p2os->set_vel(&msg_vel);
        Log.infoln("Done");
        sleep(1);
    }
}
