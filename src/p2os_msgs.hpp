#ifndef _P2OS_MSGS_HPP_
#define _P2OS_MSGS_HPP_

#include <Arduino.h>
#include <vector>

namespace p2os_msgs {

// struct Header {
//     uint32_t seq;       // Sequence number
//     uint64_t stamp;     // Timestamp
//     char frame_id[50];  // Frame ID, adjust size as necessary
// };

struct AIO {
    uint8_t            voltages_count;
    std::vector<float> voltages;
};

struct ArmState {
    bool arm_power;
};

struct BatteryState {
    // Header header
    float voltage;
};

struct DIO {
    uint32_t count;
    uint16_t bits;
};

struct GripState {
    uint32_t state;
    int32_t  dir;  // Direction: -1 for inward, +1 for outward, 0 for stationary
    bool     inner_beam;
    bool     outer_beam;
    bool     left_contact;
    bool     right_contact;
};

struct LiftState {
    int32_t state;
    int32_t dir;  // direction: -1 (downward), +1 (upward), 0 (stationary)
    float   position;
};

struct GripperState {
    GripState grip;
    LiftState lift;
};

struct MotorState {
    int32_t state;
};

struct PTZState {
    int32_t pan;
    int32_t tilt;
    int32_t zoom;
    bool    relative;
};

struct SonarArray {
    // Header header;
    int32_t            ranges_count;
    std::vector<float> ranges;
};

struct TCMState {
    bool tcm_power;
};
}  // namespace p2os_msgs

namespace geometry_msgs {

struct Point {
    double x;
    double y;
    double z;
};

struct Quaternion {
    double x;
    double y;
    double z;
    double w;
};

struct Pose {
    Point position;
    // Quaternion orientation;
};

struct PoseWithCovariance {
    Pose                   pose;
    std::array<double, 36> covariance;
};

struct Vector3 {
    double x;
    double y;
    double z;
};

struct Twist {
    Vector3 linear;
    Vector3 angular;
};

struct TwistWithCovariance {
    Twist                  twist;
    std::array<double, 36> covariance;
};

struct Transform {
    Vector3    translation;
    Quaternion rotation;
};

struct TransformStamped {
    // Header header
    std::string child_frame_id;  // the frame id of the child frame
    Transform   transform;
};

}  // namespace geometry_msgs

namespace nav_msgs {

struct Odometry {
    // Header header
    std::string                        child_frame_id;
    geometry_msgs::PoseWithCovariance  pose;
    geometry_msgs::TwistWithCovariance twist;
};

/*! \brief Container struct.
 *
 *  Create a struct that holds the
 *  Robot's sensors.
 */
struct ros_p2os_data_t {
    Odometry                        position;    //! Provides the position of the robot
    p2os_msgs::BatteryState         batt;        //! Provides the battery voltage
    p2os_msgs::MotorState           motors;      //! Provides the state of the motors (enabled or disabled)
    p2os_msgs::GripperState         gripper;     //! Provides the state of the gripper
    p2os_msgs::SonarArray           sonar;       //! Container for sonar data
    p2os_msgs::DIO                  dio;         //! Digital In/Out
    p2os_msgs::AIO                  aio;         //! Analog In/Out
    geometry_msgs::TransformStamped odom_trans;  //! Transformed odometry frame.
};
}  // namespace nav_msgs

#endif  // _P2OS_MSGS_HPP_