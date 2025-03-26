//
// Created by arno on 3/14/25.
//

#ifndef JETRACERSERIAL_H
#define JETRACERSERIAL_H
#include <string>
#include <mutex>
#include <thread>
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include "IJetracer.hpp"

namespace JetracerController {
    class JetracerSerial : public IJetracer {
    public:
        explicit JetracerSerial(JetracerCreateInfo create_info, rclcpp::Logger logger);
        ~JetracerSerial() final;

        bool init() final;
        bool activate() final;
        bool deactivate() final;

        bool SetParams(int kp, int ki, int kd, double linear_correction, int servo_bias) final;
        bool SetCoefficient(float a, float b, float c, float d) final;

        bool setVelocity(double x, double y, double yaw) final;

        inline IMU getIMU() final {
            std::lock_guard lock(imu_mutex);
            return imu;
        }

        inline Odom getOdometry() final {
            std::lock_guard lock(odom_mutex);
            return odom;
        }

        inline MotorStates getMotorStates() final {
            std::lock_guard lock(motor_mutex);
            return motorStates;
        }

        void serial_task(); // do not execute!!!

    private:
        uint8_t checksum(uint8_t* buf, size_t len);

    private:
        // config
        std::string serial_port;
        int baud_rate;
        int kp, ki, kd;
        double linear_correction;
        int servo_bias;
        float a, b, c, d;

        // states
        std::mutex imu_mutex;
        IMU imu;
        std::mutex odom_mutex;
        Odom odom;
        std::mutex motor_mutex;
        MotorStates motorStates;


        // constants for serial commands
        static constexpr uint8_t HEAD1 = 0xAA;
        static constexpr uint8_t HEAD2 = 0x55;
        static constexpr uint8_t SEND_TYPE_VELOCITY = 0x11;
        static constexpr uint8_t SEND_TYPE_PARAMS = 0x12;
        static constexpr uint8_t SEND_TYPE_COEFFICIENT = 0x13;

        // serial
        std::thread serial_task_thread;
        boost::asio::io_service iosev;
        boost::asio::serial_port sp;
        bool running = false;

        // ros
        rclcpp::Clock clock;
    };
} // jetracerController

#endif //JETRACERSERIAL_H
