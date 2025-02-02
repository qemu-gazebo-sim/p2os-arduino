#include <Arduino.h>
#include <p2os.hpp>
#include <HardwareSerial.h>
#include <vector>

#define PIONEER_SERIAL_RX 16
#define PIONEER_SERIAL_TX 17

HardwareSerial            debug_serial(0);    // define a Serial for UART0
HardwareSerial            pioneer_serial(2);  // define a Serial for UART2
P2OS*                     p2os;
uint32_t                  print_time;
nav_msgs::ros_p2os_data_t current_data;

void setup() {
    debug_serial.begin(9600);
    pioneer_serial.begin(9600, SERIAL_8N1, PIONEER_SERIAL_RX, PIONEER_SERIAL_TX);
    debug_serial.flush();
    pioneer_serial.flush();

    p2os = new P2OS(debug_serial, pioneer_serial);

    while (p2os->setup()) {
        debug_serial.println("p2os setup failed...");
    }

    debug_serial.println("Ready!");
}

double check_value(double val) {
    if (val > pow(2, 16)) {
        return -1;
    }

    return val;
}

void loop() {
    p2os->loop();

    current_data = p2os->get_p2dx_data();
    if ((millis() - print_time) > 500) {
        double vel_lin_x = current_data.position.twist.twist.linear.x;
        double vel_lin_y = current_data.position.twist.twist.linear.y;
        double vel_lin_z = current_data.position.twist.twist.linear.z;

        double vel_ang_x = current_data.position.twist.twist.angular.x;
        double vel_ang_y = current_data.position.twist.twist.angular.y;
        double vel_ang_z = current_data.position.twist.twist.angular.z;

        double pose_x = current_data.position.pose.pose.position.x;
        double pose_y = current_data.position.pose.pose.position.y;
        double pose_z = current_data.position.pose.pose.position.z;

        int32_t            sonar_count = current_data.sonar.ranges_count;
        std::vector<float> sonar_data = current_data.sonar.ranges;

        debug_serial.printf(
            "Velocity Linear:\n\tx: %.6lf\n\t y:%.6lf\n\t z:%.6lf\n", check_value(vel_lin_x), check_value(vel_lin_y),
            check_value(vel_lin_z)
        );

        debug_serial.printf(
            "Velocity Angular:\n\tx: %.6lf \n\ty:%.6lf \n\tz:%.6lf\n", check_value(vel_ang_x), check_value(vel_ang_y),
            check_value(vel_ang_z)
        );

        debug_serial.printf(
            "Position:\n\tx: %.6lf \n\ty:%.6lf \n\tz:%.6lf \n", check_value(pose_x), check_value(pose_y),
            check_value(pose_z)
        );

        if (sonar_count >= 0) {
            String sonar_info;
            for (int i = 0; i < sonar_count; i++) {
                sonar_info += " ";
                sonar_info += static_cast<int>(sonar_data[i]);
            }
            debug_serial.printf("Sonars: %s\n", sonar_info.c_str());
        }

        print_time = millis();
    }
}
