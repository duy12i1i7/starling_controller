#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <cmath>

#include <geometry_msgs/msg/point.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <libInterpolate/Interpolate.hpp>
#include <libInterpolate/AnyInterpolator.hpp>

// Include custom ros msgs from template_msgs
#include "template_msgs/msg/notify_vehicles.hpp"
#include "template_msgs/msg/target_angle.hpp"

using namespace std;
using namespace rclcpp;

// Forward Declare Parent UAVController class to avoid cyclic include
class UAVController;

class UserController
{
    public:
        UserController(UAVController *node);

        // Reset this controller
        void reset();

        // User Controller Checking If The Start Location is Registered
        bool smReady(const rclcpp::Time& stamp);

        // User Controller Execute One Control Loop
        bool smExecute(const rclcpp::Time& stamp, const rclcpp::Duration& time_elapsed);
    
    private:
        // ROS Helper Functions
        rclcpp::Logger get_logger();
        rclcpp::Time now();

        // The ROSnode itself
        UAVController* node;

        // Publishers
        rclcpp::Publisher<template_msgs::msg::TargetAngle>::SharedPtr          notify_angle_pub;

        // Subscriptions
        rclcpp::Subscription<template_msgs::msg::TargetAngle>::SharedPtr       sync_angle_sub;
        rclcpp::Subscription<template_msgs::msg::NotifyVehicles>::SharedPtr    notify_vehicles_sub;

        //// Functionality

        // Handle Subscription Callbacks
        void handleTargetAngle(const template_msgs::msg::TargetAngle::SharedPtr s);
        void handleNotifyVehicles(const template_msgs::msg::NotifyVehicles::SharedPtr s);

        // Variables
        geometry_msgs::msg::Point origin;
        const double circle_radius = 1.5; // In meters
        const double height = 1.0;
        uint8_t system_vehicle_id;

        // Received initial positions
        bool received_circle_id = false;

        // Dynamic Variables
        double vehicle_start_theta;
        double vehicle_setpoint_theta; 

        // Control on velocity
        double sync_angle_P;
        double vehicle_velocity = 0.5;
};

#endif