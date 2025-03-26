//
// Created by arno on 3/14/25.
//

#ifndef JETRACERCONTROLLER_H
#define JETRACERCONTROLLER_H
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "IJetracer.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

using std::placeholders::_1;

namespace JetracerController {
    class JetracerController : public rclcpp::Node {
    public:
        JetracerController();

    private:
        void init();
        void topic_Callback(const geometry_msgs::msg::Twist);
        void time_Callback();

    private:
        // cmd_vel subscription
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;

        // timer
        rclcpp::TimerBase::SharedPtr timer;
        // publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motorLvel_publisher;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motorRvel_publisher;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motorLset_publisher;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motorRset_publisher;

        // tf broadcaster
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        // jetracer hardware
        std::unique_ptr<IJetracer> jetracer;
    };
} // jetracerController

#endif //JETRACERCONTROLLER_H
