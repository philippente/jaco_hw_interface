#pragma once

#include "KinovaTypes.h"
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"

#include "hardware_interface/robot_hw.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"

namespace jaco_hw_interface
{

class JacoHWInterface : public hardware_interface::RobotHW
{
private:
    bool loadUSBCommandLayerFunctions();
    bool initUSBConnection();
    bool initTorqueControlParameters();
    bool registerRosControlInterfaces();

    // Handle for the jaco command layer library
    void* commandLayer_handle;
    // Function pointers to the functions from the jaco command layer library
    int(*myInitAPI)();
    int(*myCloseAPI)();
    int(*myMoveHome)();
    int(*mySetTorqueControlType)(TORQUECONTROL_TYPE type);
    int(*mySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
    int(*mySetTorqueInactivityType)(int);
    int(*mySetTorqueRobotProtection)(int protectionLevel);
    int(*mySetTorqueSafetyFactor)(float factor);
    int(*mySetTorqueVibrationController)(float value);
    int(*mySetTorqueActuatorDamping)(float Command[COMMAND_SIZE]);
    int(*mySetTorqueFeedVelocity)(float Command[COMMAND_SIZE]);
    int(*mySetPositionLimitDistance)(float Command[COMMAND_SIZE]);
    int(*mySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
    int(*mySetGravityType)(GRAVITY_TYPE Type);
    int(*myGetAngularCommand)(AngularPosition &);
    int(*myGetAngularPosition)(AngularPosition &);
    int(*mySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);

    double position[6];
    double velocity[6]; // velocity is not monitored (yet?)
    double effort[6]; // effort is not monitored (yet?)
    
    double effort_command[6];

    // ROS Control Interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::EffortJointInterface effort_joint_interface;

    bool torqueMode;

public:
    JacoHWInterface();
    ~JacoHWInterface();

    void read_state();
    void execute_command();
};

} // namespace jaco_hw_interface


