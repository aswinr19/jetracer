//
// Created by arno on 3/14/25.
//

#ifndef JETRACERMOCK_H
#define JETRACERMOCK_H
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "IJetracer.hpp"

namespace JetracerController {
    class JetracerMock : public IJetracer {
    public:
        explicit JetracerMock(JetracerCreateInfo create_info, rclcpp::Logger logger);
        ~JetracerMock() final;

        bool init() final;
        bool activate() final;
        bool deactivate() final;

        bool SetParams(int kp, int ki, int kd, double linear_correction, int servo_bias) final;
        bool SetCoefficient(float a, float b, float c, float d) final;

        bool setVelocity(double x, double y, double yaw) final;

        inline IMU getIMU() final {
            return imu;
        }

        inline Odom getOdometry() final {
            return odom;
        }
        inline MotorStates getMotorStates() final {
            return motorStates;
        }

    private:
        // Simulation methods
        void simulationLoop();
        void updateOdometry(double linear_vel, double steering_angle, double dt);
        void updateIMU(double linear_vel, double steering_angle, double angular_vel, double dt);
        double getRandomNoise(double magnitude);
        double calculateAngularVelocity(double linear_vel, double steering_angle);

    private:
        // config
        std::string serial_port;
        int baud_rate;
        int kp, ki, kd;
        double linear_correction;
        int servo_bias;
        float a, b, c, d;

        // states
        IMU imu;
        Odom odom;
        MotorStates motorStates;

        // Vehicle parameters
        double wheelbase_ = 0.17;  // Distance between front and rear axles (m)
        double track_width_ = 0.15; // Distance between left and right wheels (m)
        double max_steering_angle_ = 30.0 * M_PI / 180.0; // Maximum steering angle in radians

        // Simulation state
        std::mutex mutex_;
        std::thread simulation_thread_;
        bool running_ = false;
        std::chrono::steady_clock::time_point last_update_time_;

        // Target velocities and steering
        double target_linear_vel_ = 0.0;    // Forward velocity (m/s)
        double target_steering_angle_ = 0.0; // Steering angle (rad)
        double current_steering_angle_ = 0.0; // Current steering angle (rad)
    };
} // jetracerController

#endif //JETRACERMOCK_H
