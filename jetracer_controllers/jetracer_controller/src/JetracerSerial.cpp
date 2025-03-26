//
// Created by arno on 3/14/25.
//

#include "jetracer_controller/JetracerSerial.h"
#include <rclcpp/rclcpp.hpp>

namespace JetracerController {
    JetracerSerial::JetracerSerial(JetracerCreateInfo create_info, rclcpp::Logger logger)
        : IJetracer(logger), serial_port(create_info.serial_port), baud_rate(create_info.baud_rate),
          kp(create_info.kp), ki(create_info.ki), kd(create_info.kd),
          linear_correction(create_info.linear_correction), servo_bias(create_info.servo_bias),
          a(create_info.a), b(create_info.b), c(create_info.c), d(create_info.d),
          sp(iosev) {
    }

    JetracerSerial::~JetracerSerial() {
        // closing serial port
        try {
            if (sp.is_open()) {
                sp.close();
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Serial port is not opened");
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception closing serial port: %s", e.what());
        }
    }


    bool JetracerSerial::init() {
        try {
            // opening serial poort
            boost::system::error_code ec;
            sp.open(serial_port, ec);
            if (ec) {
                RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", ec.message().c_str());
                return false;
            }

            // set serial properties
            sp.set_option(boost::asio::serial_port::baud_rate(baud_rate));
            sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
            sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
            sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
            sp.set_option(boost::asio::serial_port::character_size(8));
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");

            // set params and coefficients
            if (!SetParams(kp, ki, kd, linear_correction, servo_bias)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set controller parameters");
                return false;
            }
            if (!SetCoefficient(a, b, c, d)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set coefficients");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Params and coefficients set successfully");
        }
        catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Exception opening serial port: %s", e.what());
            return false;
        }
        return true;
    }

    bool JetracerSerial::activate() {
        // start reading thread
        running = true;
        serial_task_thread = std::thread(std::bind(&JetracerSerial::serial_task, this));
        return true;
    }

    bool JetracerSerial::deactivate() {
        // stop reading thread
        running = false;
        if (serial_task_thread.joinable()) {
            serial_task_thread.join();
        }
        return true;
    }

    bool JetracerSerial::setVelocity(double x, double y, double yaw) {
        try {
            static uint8_t tmp[11];
            tmp[0] = HEAD1;
            tmp[1] = HEAD2;
            tmp[2] = 0x0b;
            tmp[3] = SEND_TYPE_VELOCITY;
            tmp[4] = ((int16_t)(x * 1000) >> 8) & 0xff;
            tmp[5] = ((int16_t)(x * 1000)) & 0xff;
            tmp[6] = ((int16_t)(y * 1000) >> 8) & 0xff;
            tmp[7] = ((int16_t)(y * 1000)) & 0xff;
            tmp[8] = ((int16_t)(yaw * 1000) >> 8) & 0xff;
            tmp[9] = ((int16_t)(yaw * 1000)) & 0xff;
            tmp[10] = checksum(tmp, 10);
            write(sp, boost::asio::buffer(tmp, 11));
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception write Velocity to serial port: %s", e.what());
            return false;
        }
        return true;
    }

    uint8_t JetracerSerial::checksum(uint8_t* buf, size_t len) {
        uint8_t sum = 0x00;
        for (size_t i = 0; i < len; i++)
            sum += *(buf + i);
        return sum;
    }

    bool JetracerSerial::SetParams(int kp, int ki, int kd, double linear_correction, int servo_bias) {
        try {
            static uint8_t tmp[15];
            tmp[0] = HEAD1;
            tmp[1] = HEAD1;
            tmp[2] = 0x0F;
            tmp[3] = SEND_TYPE_PARAMS;
            tmp[4] = (kp >> 8) & 0xff;
            tmp[5] = kp & 0xff;
            tmp[6] = (ki >> 8) & 0xff;
            tmp[7] = ki & 0xff;
            tmp[8] = (kd >> 8) & 0xff;
            tmp[9] = kd & 0xff;
            tmp[10] = (int16_t)((int16_t)(linear_correction * 1000) >> 8) & 0xff;
            tmp[11] = (int16_t)(linear_correction * 1000) & 0xff;
            tmp[12] = ((int16_t)((int16_t)servo_bias) >> 8) & 0xff;
            tmp[13] = ((int16_t)servo_bias) & 0xff;
            tmp[14] = checksum(tmp, 14);
            write(sp, boost::asio::buffer(tmp, 15));
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception write Params to serial port: %s", e.what());
            return false;
        }
        return true;
    }

    bool JetracerSerial::SetCoefficient(float a, float b, float c, float d) {
        try {
            static uint8_t tmp[21];
            char* p;
            tmp[0] = HEAD1;
            tmp[1] = HEAD2;
            tmp[2] = 0x15;
            tmp[3] = SEND_TYPE_COEFFICIENT;
            p = (char*)&a;
            tmp[4] = p[0];
            tmp[5] = p[1];
            tmp[6] = p[2];
            tmp[7] = p[3];
            p = (char*)&b;
            tmp[8] = p[0];
            tmp[9] = p[1];
            tmp[10] = p[2];
            tmp[11] = p[3];
            p = (char*)&c;
            tmp[12] = p[0];
            tmp[13] = p[1];
            tmp[14] = p[2];
            tmp[15] = p[3];
            p = (char*)&d;
            tmp[16] = p[0];
            tmp[17] = p[1];
            tmp[18] = p[2];
            tmp[19] = p[3];
            tmp[20] = checksum(tmp, 20);
            write(sp, boost::asio::buffer(tmp, 21));
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception write Coefficients to serial port: %s", e.what());
            return false;
        }
        return true;
    }

    void JetracerSerial::serial_task() {
        enum frameState {
            State_Head1, State_Head2, State_Size, State_Data, State_CheckSum, State_Handle
        };

        frameState state = State_Head1;

        uint8_t frame_size, frame_sum, frame_type [[maybe_unused]];
        uint8_t data[50];
        rclcpp::Time previousTime;

        RCLCPP_INFO(this->get_logger(), "Starting serial receive task");

        while (running) {
            // State machine
            // [head1 head2 size type data checksum ]
            // [0xAA  0x55  0x2D 0x01 ....  0xXX    ]
            switch (state) {
            case State_Head1: // waiting for frame header 1
                frame_sum = 0x00;
                read(sp, boost::asio::buffer(&data[0], 1));
                state = (data[0] == HEAD1 ? State_Head2 : State_Head1);
                if (state == State_Head1) {
                    // couldn't find header
                    RCLCPP_DEBUG(this->get_logger(), "Received invalid header1: 0x%02X", data[0]);
                }
                break;

            case State_Head2: // waiting for frame header 2
                read(sp, boost::asio::buffer(&data[1], 1));
                state = (data[1] == HEAD2 ? State_Size : State_Head1);
                if (state == State_Head1) {
                    // couldn't find header2
                    RCLCPP_DEBUG(this->get_logger(), "Received invalid header2: 0x%02X", data[1]);
                }
                break;

            case State_Size: // waiting for frame Size
                read(sp, boost::asio::buffer(&data[2], 1));
                frame_size = data[2];
                state = State_Data;
                break;

            case State_Data: // waiting for frame data
                read(sp, boost::asio::buffer(&data[3], frame_size - 4));
                frame_type = data[3];
                state = State_CheckSum;
                break;

            case State_CheckSum: // waiting for frame CheckSum
                read(sp, boost::asio::buffer(&data[frame_size - 1], 1));
                frame_sum = checksum(data, frame_size - 1);
                state = data[frame_size - 1] == frame_sum ? State_Handle : State_Head1;
                if (state == State_Head1) {
                    RCLCPP_WARN(this->get_logger(), "Checksum error! Received: 0x%02X, Calculated: 0x%02X",
                                data[frame_size -1], frame_sum);
                }
                break;

            case State_Handle: // processing frame data
            // If the message is a standard format, try to parse key values
                if (frame_size >= 34) {
                    rclcpp::Time currentTime = clock.now();

                    // IMU data
                    imu_mutex.lock();
                    imu.gyro.x = ((double)((int16_t)(data[4]*256+data[5]))/32768*2000/180*3.1415);
                    imu.gyro.y = ((double)((int16_t)(data[6]*256+data[7]))/32768*2000/180*3.1415);
                    imu.gyro.z = ((double)((int16_t)(data[8]*256+data[9]))/32768*2000/180*3.1415);
                    imu.acc.x = ((double)((int16_t)(data[10]*256+data[11]))/32768*2*9.8);
                    imu.acc.y = ((double)((int16_t)(data[12]*256+data[13]))/32768*2*9.8);
                    imu.acc.z = ((double)((int16_t)(data[14]*256+data[15]))/32768*2*9.8);
                    imu.angle.x = ((double)((int16_t)(data[16]*256+data[17]))/10.0);
                    imu.angle.y = ((double)((int16_t)(data[18]*256+data[19]))/10.0);
                    imu.angle.z = ((double)((int16_t)(data[20]*256+data[21]))/10.0);
                    imu_mutex.unlock();

                    // Odometry data
                    double x = ((double)((int16_t)(data[22]*256+data[23]))/1000);
                    double y = ((double)((int16_t)(data[24]*256+data[25]))/1000);
                    double yaw = ((double)((int16_t)(data[26]*256+data[27]))/1000);
                    double velX = ((double)((int16_t)(data[28]*256+data[29]))/1000);
                    double velY = ((double)((int16_t)(data[30]*256+data[31]))/1000);
                    double velYaw = ((double)((int16_t)(data[32]*256+data[33]))/1000);
                    odom_mutex.lock();
                    // pos
                    odom.position.x = x;
                    odom.position.y = y;
                    odom.position.z = 0.0;
                    odom.orientation.x = 0.0;
                    odom.orientation.y = 0.0;
                    odom.orientation.z = yaw;
                    // vel
                    odom.linear.x = velX/(currentTime-previousTime).seconds();
                    odom.linear.y = velY/(currentTime-previousTime).seconds();
                    odom.linear.z = 0.0;
                    odom.angular.x = 0.0;
                    odom.angular.y = 0.0;
                    odom.angular.z = velYaw/(currentTime-previousTime).seconds();
                    odom_mutex.unlock();

                    // motor data if available
                    if (frame_size >= 42) {
                        std::lock_guard lock(motor_mutex);
                        motorStates.motor_lvel = (double)((int16_t)(data[34]*256+data[35]));
                        motorStates.motor_rvel = (double)((int16_t)(data[36]*256+data[37]));
                        motorStates.motor_lset = (double)((int16_t)(data[38]*256+data[39]));
                        motorStates.motor_rset = (double)((int16_t)(data[40]*256+data[41]));
                    }
                    previousTime = currentTime;
                }

                state = State_Head1;
                break;

            default:
                state = State_Head1;
                break;
            }
        }
    }
} // JetracerController