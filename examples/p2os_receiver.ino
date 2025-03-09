#include <p2os.hpp>
#include <HardwareSerial.h>
#include <ArduinoLog.h>

HardwareSerial            debug_serial(0);    // define a Serial for UART0
HardwareSerial            pioneer_serial(2);  // define a Serial for UART2

P2OS*                     p2os;
uint32_t                  print_time;
nav_msgs::ros_p2os_data_t current_data;

void setup() {
    debug_serial.begin(9600);
    debug_serial.flush();

    Log.begin(LOG_LEVEL_INFO, &debug_serial);

    p2os = new P2OS(pioneer_serial);
    
    while (p2os->setup()) {
        Log.infoln("p2os setup failed...");
    }

    Log.infoln("Ready!");
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

        Log.infoln(
            "Velocity Linear:\n\tx: %D\n\t y: %D\n\t z: %D", 
            check_value(vel_lin_x), 
            check_value(vel_lin_y),
            check_value(vel_lin_z)
        );

        Log.infoln(
            "Velocity Angular:\n\tx: %D \n\ty: %D \n\tz: %D", 
            check_value(vel_ang_x), 
            check_value(vel_ang_y),
            check_value(vel_ang_z)
        );

        Log.infoln(
            "Position:\n\tx: %D \n\ty: %D \n\tz: %D", 
            check_value(pose_x), 
            check_value(pose_y),
            check_value(pose_z)
        );

        if (sonar_count >= 0) {
            String sonar_info;
            for (int i = 0; i < sonar_count; i++) {
                sonar_info += " ";
                sonar_info += static_cast<int>(sonar_data[i]);
            }
            Log.infoln("Sonars: %s\n", sonar_info.c_str());
        }

        print_time = millis();
    }
}
