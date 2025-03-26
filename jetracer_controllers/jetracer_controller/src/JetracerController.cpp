//
// Created by arno on 3/14/25.
//

#include "jetracer_controller/JetracerController.h"
#include "jetracer_controller/JetracerSerial.h"
#include "jetracer_controller/JetracerMock.h"
#include "tf2/LinearMath/Quaternion.h"

namespace JetracerController {
    JetracerController::JetracerController()  : Node("jetracer_control") {
        // define parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 0);
        this->declare_parameter<int>("kp", 0);
        this->declare_parameter<int>("ki", 0);
        this->declare_parameter<int>("kd", 0);
        this->declare_parameter<double>("linear_correction", 0.0);
        this->declare_parameter<int>("servo_bias", 0);
        this->declare_parameter<float>("a", 0);
        this->declare_parameter<float>("b", 0);
        this->declare_parameter<float>("c", 0);
        this->declare_parameter<float>("d", 0);
        this->declare_parameter<bool>("mock", true);

        // create subscriber
        subscription = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&JetracerController::topic_Callback, this, _1));

        // create jetson interface
        JetracerCreateInfo create_info;
        create_info.serial_port = this->get_parameter("serial_port").as_string();
        create_info.baud_rate = this->get_parameter("baud_rate").as_int();
        create_info.kp = this->get_parameter("kp").as_int();
        create_info.ki = this->get_parameter("ki").as_int();
        create_info.kd = this->get_parameter("kd").as_int();
        create_info.linear_correction = this->get_parameter("linear_correction").as_double();
        create_info.servo_bias = this->get_parameter("servo_bias").as_int();
        create_info.a = this->get_parameter("a").as_double();
        create_info.b = this->get_parameter("b").as_double();
        create_info.c = this->get_parameter("c").as_double();
        create_info.d = this->get_parameter("d").as_double();

        bool mock = this->get_parameter("mock").as_bool();
        if (mock) {
            jetracer = std::make_unique<JetracerMock>(create_info, this->get_logger());
        } else {
            jetracer = std::make_unique<JetracerSerial>(create_info, this->get_logger());
        }
        // setup publishers
        odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        motorLvel_publisher = this->create_publisher<std_msgs::msg::Int32>("motor/lvel", 10);
        motorRvel_publisher = this->create_publisher<std_msgs::msg::Int32>("motor/rvel", 10);
        motorLset_publisher = this->create_publisher<std_msgs::msg::Int32>("motor/lset", 10);
        motorRset_publisher = this->create_publisher<std_msgs::msg::Int32>("motor/rset", 10);

        // tf2 broadcaster
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // init the jetracer interface
        this->init();
    }

    void JetracerController::init() {
        // set up jetracer interface
        jetracer->init();
        jetracer->activate();

        // start publisher timer
        timer =  this->create_wall_timer(1ms, std::bind(&JetracerController::time_Callback, this));
    }


    void JetracerController::topic_Callback(const geometry_msgs::msg::Twist msg) {
        jetracer->setVelocity(msg.linear.x, msg.linear.y, msg.angular.z);
    }

    void JetracerController::time_Callback() {
        // odom
        auto odom_msg = nav_msgs::msg::Odometry();
        Odom odom_data = jetracer->getOdometry();
        odom_msg.header.frame_id = "odom";
        // position
        odom_msg.pose.pose.position.x = odom_data.position.x;
        odom_msg.pose.pose.position.y = odom_data.position.y;
        odom_msg.pose.pose.position.z = odom_data.position.z;
        // orientation
        odom_msg.pose.pose.orientation.x = odom_data.orientation.x;
        odom_msg.pose.pose.orientation.y = odom_data.orientation.y;
        odom_msg.pose.pose.orientation.z = odom_data.orientation.z;
        // linear
        odom_msg.twist.twist.linear.x = odom_data.linear.x;
        odom_msg.twist.twist.linear.y = odom_data.linear.y;
        odom_msg.twist.twist.linear.z = odom_data.linear.z;
        // angular
        odom_msg.twist.twist.angular.x = odom_data.angular.x;
        odom_msg.twist.twist.angular.y = odom_data.angular.y;
        odom_msg.twist.twist.angular.z = odom_data.angular.z;
        // covarianc
        odom_msg.twist.covariance = jetracer->getCovarianceOdometry().twist;
        odom_msg.pose.covariance = jetracer->getCovarianceOdometry().pose;
        odom_publisher->publish(odom_msg);

        // odom tf2 frame
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        // position
        t.transform.translation.x = odom_data.position.x;
        t.transform.translation.y = odom_data.position.y;
        t.transform.translation.z = odom_data.position.z;
        // orientation
        tf2::Quaternion quat;
        quat.setRPY(odom_data.orientation.x, odom_data.orientation.y, odom_data.orientation.z);
        t.transform.rotation.x = quat.x();
        t.transform.rotation.y = quat.y();
        t.transform.rotation.z = quat.z();
        t.transform.rotation.w = quat.w();

        tf_broadcaster->sendTransform(t);

        // imu
        auto imu_msg = sensor_msgs::msg::Imu();
        IMU imu_data = jetracer->getIMU();
        // acc
        imu_msg.linear_acceleration.x  = imu_data.acc.x;
        imu_msg.linear_acceleration.y = imu_data.acc.y;
        imu_msg.linear_acceleration.z = imu_data.acc.z;
        // gyro
        imu_msg.angular_velocity.x = imu_data.gyro.x;
        imu_msg.angular_velocity.y = imu_data.gyro.y;
        imu_msg.angular_velocity.z = imu_data.gyro.z;
        // angle
        imu_msg.orientation.x = imu_data.angle.x;
        imu_msg.orientation.y = imu_data.angle.y;
        imu_msg.orientation.z = imu_data.angle.z;
        // covariance
        imu_msg.angular_velocity_covariance = jetracer->getCovarianceImu().angularVelocity;
        imu_msg.linear_acceleration_covariance = jetracer->getCovarianceImu().linearAcceleration;
        imu_msg.orientation_covariance = jetracer->getCovarianceImu().orientation;
        imu_publisher->publish(imu_msg);

        MotorStates motorStates_data = jetracer->getMotorStates();
        auto motorLvel_msg = std_msgs::msg::Int32();
        motorLvel_msg.data = (int32_t)motorStates_data.motor_lvel;
        motorLvel_publisher->publish(motorLvel_msg);

        auto motorRvel_msg = std_msgs::msg::Int32();
        motorRvel_msg.data = (int32_t)motorStates_data.motor_rvel;
        motorRvel_publisher->publish(motorRvel_msg);

        auto motorLset_msg = std_msgs::msg::Int32();
        motorLset_msg.data = (int32_t)motorStates_data.motor_lset;
        motorLset_publisher->publish(motorLset_msg);

        auto motorRset_msg = std_msgs::msg::Int32();
        motorRset_msg.data = (int32_t)motorStates_data.motor_rset;
        motorRset_publisher->publish(motorRset_msg);
    }

} // jetracerController

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JetracerController::JetracerController>());
    rclcpp::shutdown();
    return 0;
}