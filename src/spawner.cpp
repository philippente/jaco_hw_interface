#include "ros/ros.h"
#include "controller_manager_msgs/LoadController.h"
#include "controller_manager_msgs/SwitchController.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_spawner");

    if (argc != 2)
    {
        ROS_ERROR("Wrong number of arguments");
        ROS_ERROR("Usage: spawner <ControllerName>");
    }

    char* controllerName = argv[1];

    ros::NodeHandle nh;
    ros::service::waitForService("/controller_manager/load_controller");
    ros::ServiceClient load_client = nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
    ros::service::waitForService("/controller_manager/switch_controller");
    ros::ServiceClient switch_client = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

    controller_manager_msgs::LoadController load_service;
    load_service.request.name = controllerName;
    controller_manager_msgs::SwitchController switch_service;
    switch_service.request.start_controllers = std::vector<std::basic_string<char> >(1, controllerName);
    switch_service.request.stop_controllers = std::vector<std::basic_string<char> >(0);
    switch_service.request.strictness = controller_manager_msgs::SwitchControllerRequest::STRICT;

    if (!load_client.call(load_service))
    {
        ROS_ERROR("Failed to call service /controller_manager/load_controller");
        return 1;
    }

    if (!load_service.response.ok)
    {
        ROS_ERROR("Failed to load controller: %s", controllerName);
        return 1;
    }

    if (!switch_client.call(switch_service))
    {
        ROS_ERROR("Failed to call service /controller_manager/switch_controller");
        return 1;
    }

    if (!switch_service.response.ok)
    {
        ROS_ERROR("Failed to start controller: %s", controllerName);
        return 1;
    }

    ROS_INFO("Successfully loaded and started controller: %s", controllerName);
    return 0;
}
