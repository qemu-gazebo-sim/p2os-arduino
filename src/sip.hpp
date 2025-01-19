#ifndef _SIP_HPP_
#define _SIP_HPP_

#include "p2os_msgs.hpp"

struct ArmJoint {
    char          speed;
    unsigned char home;
    unsigned char min;
    unsigned char centre;
    unsigned char max;
    unsigned char ticksPer90;
};

enum PlayerGripperStates {
    PLAYER_GRIPPER_STATE_OPEN = 1,
    PLAYER_GRIPPER_STATE_CLOSED,
    PLAYER_GRIPPER_STATE_MOVING,
    PLAYER_GRIPPER_STATE_ERROR
};

enum PlayerActArrayStates {
    PLAYER_ACTARRAY_ACTSTATE_IDLE = 1,
    PLAYER_ACTARRAY_ACTSTATE_MOVING,
    PLAYER_ACTARRAY_ACTSTATE_STALLED
};

class SIP {
private:
    int PositionChange(uint16_t, uint16_t);
    int param_idx;  // index of our robot's data in the parameter table

public:
    HardwareSerial* debug_serial;
    // these values are returned in every standard SIP
    bool          lwstall, rwstall;
    unsigned char motors_enabled, sonar_flag;
    unsigned char status, battery, sonarreadings, analog, digin, digout;
    uint16_t      ptu, compass, timer, rawxpos;
    uint16_t      rawypos, frontbumpers, rearbumpers;
    int16_t       angle, lvel, rvel, control;
    uint16_t*     sonars;
    int           xpos, ypos;
    int           x_offset, y_offset, angle_offset;
    std::string   odom_frame_id;
    std::string   base_link_frame_id;

    // these values are returned in a CMUcam serial string extended SIP
    // (in host byte-order)
    uint16_t     blobmx, blobmy;                  // Centroid
    uint16_t     blobx1, blobx2, bloby1, bloby2;  // Bounding box
    uint16_t     blobarea, blobconf;              // Area and confidence
    unsigned int blobcolor;

    // This value is filled by ParseGyro()
    int32_t gyro_rate;

    // This information comes from the ARMpac and ARMINFOpac packets
    bool          armPowerOn, armConnected;
    bool          armJointMoving[6];
    unsigned char armJointPos[6];
    double        armJointPosRads[6];
    unsigned char armJointTargetPos[6];
    char*         armVersionString;
    unsigned char armNumJoints;
    ArmJoint*     armJoints;

    // Need this value to calculate approx position of lift when in between up
    // and down
    double lastLiftPos;

    // Timestamping SIP packets
    // double timeStandardSIP, timeGyro, timeSERAUX, timeArm;

    /* returns 0 if Parsed correctly otherwise 1 */
    void ParseStandard(unsigned char* buffer);
    void ParseSERAUX(unsigned char* buffer);
    void ParseGyro(unsigned char* buffer);
    void ParseArm(unsigned char* buffer);
    void ParseArmInfo(unsigned char* buffer);
    void Print();
    void PrintSonars();
    void PrintArm();
    void PrintArmInfo();
    void FillStandard(nav_msgs::ros_p2os_data_t* data);

    // void FillSERAUX(player_p2os_data_t* data);
    // void FillGyro(player_p2os_data_t* data);
    // void FillArm(player_p2os_data_t* data);

    explicit SIP(int idx, HardwareSerial& debug_serial) :
        param_idx(idx),
        sonarreadings(0),
        sonars(NULL),
        xpos(0),
        ypos(0),
        x_offset(0),
        y_offset(0),
        angle_offset(0),
        blobmx(0),
        blobmy(0),
        blobx1(0),
        blobx2(0),
        bloby1(0),
        bloby2(0),
        blobarea(0),
        blobconf(0),
        blobcolor(0),
        armPowerOn(false),
        armConnected(false),
        armNumJoints(0),
        armJoints(NULL),
        lastLiftPos(0.0f) {
        for (int i = 0; i < 6; ++i) {
            armJointMoving[i] = false;
            armJointPos[i] = 0;
            armJointPosRads[i] = 0;
            armJointTargetPos[i] = 0;
        }
        this->debug_serial = &debug_serial;
        armVersionString = new char[strlen("") + 1];
        strcpy(armVersionString, "");
    }

    ~SIP() { delete[] sonars; }
};

#endif  // _SIP_HPP_
