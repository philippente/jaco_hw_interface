#include "jaco_hw_interface/jaco_hw_interface.h"

#include <dlfcn.h>

#include "ros/ros.h"
#include "controller_manager/controller_manager.h"

namespace jaco_hw_interface
{

JacoHWInterface::JacoHWInterface()
{
    ROS_INFO_STREAM("Initializing Kinova Jaco 2");

    loadUSBCommandLayerFunctions();
    initUSBConnection();

    initTorqueControlParameters();

    ROS_INFO_STREAM("The robot will switch to torque control mode and move. Be cautious.");

    registerRosControlInterfaces();
}

JacoHWInterface::~JacoHWInterface()
{
    int result = (*myCloseAPI)();
    ROS_INFO("Closed Kinova USB API: %d", result);
}

bool JacoHWInterface::loadUSBCommandLayerFunctions()
{
    // load functions from Kinova USB command layer library
    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);

    myInitAPI = (int(*)()) dlsym(commandLayer_handle, "InitAPI");
    myCloseAPI = (int(*)()) dlsym(commandLayer_handle, "CloseAPI");
    myMoveHome = (int(*)()) dlsym(commandLayer_handle, "MoveHome");
    mySetTorqueControlType = (int(*)(TORQUECONTROL_TYPE)) dlsym(commandLayer_handle, "SetTorqueControlType");
    mySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
    mySetTorqueInactivityType = (int(*)(int)) dlsym(commandLayer_handle, "SetTorqueInactivityType");
    mySetTorqueRobotProtection = (int (*)(int)) dlsym(commandLayer_handle, "SetTorqueRobotProtection");
    mySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
    mySetTorqueVibrationController = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueVibrationController");
    mySetTorqueActuatorDamping = (int (*)(float[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetTorqueActuatorDamping");
    mySetTorqueFeedVelocity = (int (*)(float[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetTorqueFeedVelocity");
    mySetPositionLimitDistance = (int (*)(float[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetPositionLimitDistance");
    mySetGravityOptimalZParam = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "SetGravityOptimalZParam");
    mySetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");
    myGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
    myGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
    mySendAngularTorqueCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendAngularTorqueCommand");

    if ((myInitAPI == NULL) || (myInitAPI == NULL) ||
	(myCloseAPI == NULL) || (myMoveHome == NULL))
    {
	ROS_ERROR("* * * ERROR DURING FUNCTION LOADING * * *"); 
	return false;
    }

    return true;
}

bool JacoHWInterface::initUSBConnection()
{
    int resultInit = (*myInitAPI)();
    ROS_DEBUG("Initialization result: %d", resultInit);
    if (resultInit != 1)
    {
	ROS_ERROR("INITIALIZATION FAILED");
	return false;
    }

    AngularPosition currentCommand;
    int resultCommunication = myGetAngularCommand(currentCommand);
    ROS_DEBUG("Communcation result: %d", resultCommunication);
    if (resultCommunication != 1)
    {
	ROS_ERROR("COMMUNICATION NOT WORKING");
	return false;
    }

    // Set to position mode
    mySwitchTrajectoryTorque(POSITION);
    torqueMode = false;

    myMoveHome();

    ROS_INFO_STREAM("INITIALIZATION COMPLETED AND COMMUNICATION WORKING");
    return true;
}

bool JacoHWInterface::initTorqueControlParameters()
{
    // Choose between direct torque control and impedance control
    mySetTorqueControlType(DIRECTTORQUE);

    // Set torque commands to zero when not receiving commands
    mySetTorqueInactivityType(1);

    // Deactivate self collision protection
    mySetTorqueRobotProtection(0);

    // Set allowed velocity to 100%
    mySetTorqueSafetyFactor(1.0);

    mySetTorqueVibrationController(0.5);

    float actuatorDamping[COMMAND_SIZE];
    for (int i = 0; i < COMMAND_SIZE; i++)
    {
	actuatorDamping[i] = 1.0;
    }
    mySetTorqueActuatorDamping(actuatorDamping);

    float feedVelocity[COMMAND_SIZE];
    for (int i = 0; i < COMMAND_SIZE; i++)
    {
	feedVelocity[i] = 0.3;
    }
    mySetTorqueFeedVelocity(feedVelocity);

    float positionLimitDistance[COMMAND_SIZE];
    for (int i = 0; i < COMMAND_SIZE; i++)
    {
	positionLimitDistance[i] = 0.0;
    }
    mySetPositionLimitDistance(positionLimitDistance);

    // TODO: Make optimal gravity mode work or set payload parameters
    //float optimalParam[OPTIMAL_Z_PARAM_SIZE] =
    //{1.35422, 0.0154553, -0.0179652, -1.59187, 0.00649075,
    // 0.855273, 0.00401852, 0.329462, 0.00558146, -0.00856051,
    // 2.50616, -1.35007, 0.26923, -0.935937, -0.120401, 0.0643233};
    //mySetGravityOptimalZParam(optimalParam);
    //mySetGravityType(OPTIMAL);

    return true;
}

bool JacoHWInterface::registerRosControlInterfaces()
{
    hardware_interface::JointStateHandle state_handle_joint1("jaco_joint_1", &position[0], &velocity[0], &effort[0]);
    joint_state_interface.registerHandle(state_handle_joint1);
    hardware_interface::JointStateHandle state_handle_joint2("jaco_joint_2", &position[1], &velocity[1], &effort[1]);
    joint_state_interface.registerHandle(state_handle_joint2);
    hardware_interface::JointStateHandle state_handle_joint3("jaco_joint_3", &position[2], &velocity[2], &effort[2]);
    joint_state_interface.registerHandle(state_handle_joint3);
    hardware_interface::JointStateHandle state_handle_joint4("jaco_joint_4", &position[3], &velocity[3], &effort[3]);
    joint_state_interface.registerHandle(state_handle_joint4);
    hardware_interface::JointStateHandle state_handle_joint5("jaco_joint_5", &position[4], &velocity[4], &effort[4]);
    joint_state_interface.registerHandle(state_handle_joint5);
    hardware_interface::JointStateHandle state_handle_joint6("jaco_joint_6", &position[5], &velocity[5], &effort[5]);
    joint_state_interface.registerHandle(state_handle_joint6);

    registerInterface(&joint_state_interface);

    hardware_interface::JointHandle effort_handle_joint1(joint_state_interface.getHandle("jaco_joint_1"), &effort_command[0]);
    effort_joint_interface.registerHandle(effort_handle_joint1);
    hardware_interface::JointHandle effort_handle_joint2(joint_state_interface.getHandle("jaco_joint_2"), &effort_command[1]);
    effort_joint_interface.registerHandle(effort_handle_joint2);
    hardware_interface::JointHandle effort_handle_joint3(joint_state_interface.getHandle("jaco_joint_3"), &effort_command[2]);
    effort_joint_interface.registerHandle(effort_handle_joint3);
    hardware_interface::JointHandle effort_handle_joint4(joint_state_interface.getHandle("jaco_joint_4"), &effort_command[3]);
    effort_joint_interface.registerHandle(effort_handle_joint4);
    hardware_interface::JointHandle effort_handle_joint5(joint_state_interface.getHandle("jaco_joint_5"), &effort_command[4]);
    effort_joint_interface.registerHandle(effort_handle_joint5);
    hardware_interface::JointHandle effort_handle_joint6(joint_state_interface.getHandle("jaco_joint_6"), &effort_command[5]);
    effort_joint_interface.registerHandle(effort_handle_joint6);

    registerInterface(&effort_joint_interface);

    return true;
}

void JacoHWInterface::read_state()
{
    AngularPosition positionData;
    myGetAngularPosition(positionData);
    position[0] = positionData.Actuators.Actuator1 / 180 * 3.14159;
    position[1] = positionData.Actuators.Actuator2 / 180 * 3.14159;
    position[2] = positionData.Actuators.Actuator3 / 180 * 3.14159;
    position[3] = positionData.Actuators.Actuator4 / 180 * 3.14159;
    position[4] = positionData.Actuators.Actuator5 / 180 * 3.14159;
    position[5] = positionData.Actuators.Actuator6 / 180 * 3.14159;
}

void JacoHWInterface::execute_command()
{
    float torqueCommand[COMMAND_SIZE];
    for (int i = 0; i < COMMAND_SIZE; i++)
    {
	torqueCommand[i] = (float)effort_command[i];
	if (torqueCommand[i] > 4.0) torqueCommand[i] = 4.0;
	if (torqueCommand[i] < -4.0) torqueCommand[i] = -4.0;
    }

    if (!torqueMode)
    {
	mySwitchTrajectoryTorque(TORQUE);
	torqueMode = true;
    }

    ROS_INFO("Sending Torque Command: %f %f %f %f %f %f",
	     torqueCommand[0], torqueCommand[1], torqueCommand[2],
	     torqueCommand[3], torqueCommand[4], torqueCommand[5]);
    mySendAngularTorqueCommand(torqueCommand);
}

} // namespace jaco_hw_interface

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jaco_hw_interface");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    jaco_hw_interface::JacoHWInterface robot;
    controller_manager::ControllerManager cm(&robot);

    ros::Time cur_time;
    ros::Time prev_time = ros::Time::now();
    ros::Duration period;

    // Set control rate to 20Hz
    ros::Rate rate(20.0);

    while (ros::ok())
    {
	// Calculate period since last update
	cur_time = ros::Time::now();
	period = cur_time - prev_time;
	prev_time = cur_time;

	//ROS_INFO("Updating robot at time %f with period %f", cur_time.toSec(), period.toSec());
	
	robot.read_state();
	cm.update(cur_time, period);
	robot.execute_command();

	rate.sleep();
    }

    return 0;
}
