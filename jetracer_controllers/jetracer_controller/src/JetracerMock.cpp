//
// Created by arno on 3/14/25.
//

#include "jetracer_controller/JetracerMock.h"

namespace JetracerController {
    JetracerMock::JetracerMock(JetracerCreateInfo create_info, rclcpp::Logger logger)
        : IJetracer(logger), serial_port(create_info.serial_port), baud_rate(create_info.baud_rate),
          kp(create_info.kp), ki(create_info.ki), kd(create_info.kd),
          linear_correction(create_info.linear_correction), servo_bias(create_info.servo_bias),
          a(create_info.a), b(create_info.b), c(create_info.c), d(create_info.d){

        // Initialize odometry data
        odom.position = {0.0, 0.0, 0.0};
        odom.orientation = {0.0, 0.0, 0.0};
        odom.linear = {0.0, 0.0, 0.0};
        odom.angular = {0.0, 0.0, 0.0};

        // Initialize IMU data with some realistic noise
        imu.gyro = {0.01, -0.02, 0.005};
        imu.acc = {0.0, 0.0, 9.81}; // gravity along z-axis
        imu.angle = {0.0, 0.0, 0.0};

        // Initialize motor states
        motorStates = {0, 0, 0, 0};
    }

    JetracerMock::~JetracerMock() {
        RCLCPP_INFO(this->get_logger(), "destroy");
    }


    bool JetracerMock::init() {
        RCLCPP_INFO(this->get_logger(), "init");
        SetParams(kp, ki, kd, linear_correction, servo_bias);
        SetCoefficient(a, b, c, d);
        return true;
    }

    bool JetracerMock::activate() {
        RCLCPP_INFO(this->get_logger(), "activate");
        // Start the simulation thread
        running_ = true;
        simulation_thread_ = std::thread(&JetracerMock::simulationLoop, this);
        return true;
    }

    bool JetracerMock::deactivate() {
        RCLCPP_INFO(this->get_logger(), "deactivate");
        return true;
    }


    bool JetracerMock::SetParams(int kp, int ki, int kd, double linear_correction, int servo_bias) {
        RCLCPP_INFO(this->get_logger(), "Set params");
        RCLCPP_INFO(this->get_logger(), "serial_port: %s", serial_port.c_str());
        RCLCPP_INFO(this->get_logger(), "baud_rate: %d", baud_rate);
        RCLCPP_INFO(this->get_logger(), "kp: %d", kp);
        RCLCPP_INFO(this->get_logger(), "ki: %d", ki);
        RCLCPP_INFO(this->get_logger(), "kd: %d", kd);
        RCLCPP_INFO(this->get_logger(), "linear_correction: %f", linear_correction);
        RCLCPP_INFO(this->get_logger(), "servo_bias: %d", servo_bias);
        return true;
    }

    bool JetracerMock::SetCoefficient(float a, float b, float c, float d) {
        RCLCPP_INFO(this->get_logger(), "set coefficient");
        RCLCPP_INFO(this->get_logger(), "a: %f", a);
        RCLCPP_INFO(this->get_logger(), "b: %f", b);
        RCLCPP_INFO(this->get_logger(), "c: %f", c);
        RCLCPP_INFO(this->get_logger(), "d: %f", d);
        return true;
    }

    bool JetracerMock::setVelocity(const double x, const double y, const double yaw) {
        RCLCPP_INFO(this->get_logger(), "set_velocity: x=%f, y=%f, yaw=%f", x, y, yaw);
        std::lock_guard<std::mutex> lock(mutex_);

        // Set target velocities
        target_linear_vel_ = x;

        // Convert angular velocity (yaw) to steering angle using Ackermann model
        // For a small steering angle α and wheelbase L:
        // ω (angular velocity) = v * tan(α) / L
        // Solving for α: α = arctan(ω * L / v)

        if (fabs(x) > 0.1) {  // Only calculate steering when moving
            double angle = atan2(yaw * wheelbase_, x);
            // Clamp to maximum steering angle
            target_steering_angle_ = std::clamp(angle, -max_steering_angle_, max_steering_angle_);
        } else {
            // When not moving or moving very slowly, keep previous steering angle
            target_steering_angle_ = current_steering_angle_;
        }

        // For Ackermann steering, we primarily control the throttle (forward speed)
        // and the steering angle. In our mock model, we'll map these to motor values:
        // - motor_lset: Throttle control
        // - motor_rset: Steering control

        // Scale to appropriate unit range (assuming -1000 to 1000 range)
        motorStates.motor_lset = target_linear_vel_ * 1000.0;
        motorStates.motor_rset = target_steering_angle_ * (1000.0 / max_steering_angle_);

        return true;
    }

    double JetracerMock::calculateAngularVelocity(double linear_vel, double steering_angle) {
        // Calculate angular velocity from linear velocity and steering angle
        // For Ackermann steering: ω = v * tan(α) / L
        // where α is steering angle, L is wheelbase, v is linear velocity

        if (fabs(linear_vel) < 0.001) {
            return 0.0;  // Avoid division by zero when not moving
        }

        return linear_vel * tan(steering_angle) / wheelbase_;
    }

    void JetracerMock::simulationLoop() {
        // Main simulation loop
        while (running_) {
            // Sleep for a realistic update interval (10ms = 100Hz)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            std::lock_guard<std::mutex> lock(mutex_);

            // Calculate elapsed time since last update
            auto current_time = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(current_time - last_update_time_).count();
            last_update_time_ = current_time;

            // Define maximum rates of change
            double max_accel = 2.0;  // m/s²
            double max_steering_rate = 1.0;  // rad/s - how fast steering can change

            // Update linear velocity with acceleration limit
            double current_linear = odom.linear.x;
            double linear_accel = std::clamp(target_linear_vel_ - current_linear,
                                            -max_accel * dt, max_accel * dt);
            double new_linear = current_linear + linear_accel;

            // Update steering angle with rate limit
            double steering_change = std::clamp(target_steering_angle_ - current_steering_angle_,
                                               -max_steering_rate * dt, max_steering_rate * dt);
            current_steering_angle_ += steering_change;

            // Calculate angular velocity from steering angle and linear velocity
            double angular_vel = calculateAngularVelocity(new_linear, current_steering_angle_);

            // Update motor velocities based on current state
            // In a real Ackermann model, all wheels have different velocities in turns
            double throttle_value = new_linear * 1000.0;  // Scale to internal units

            // For simplicity in mock, both motorLvel and motorRvel represent the same forward speed
            // In reality, inner and outer wheels would have different speeds during turning
            motorStates.motor_lvel = throttle_value;
            motorStates.motor_rvel = current_steering_angle_ * (1000.0 / max_steering_angle_);

            // Update odometry based on velocities
            updateOdometry(new_linear, current_steering_angle_, dt);

            // Update IMU data
            updateIMU(new_linear, current_steering_angle_, angular_vel, dt);
        }
    }

    void JetracerMock::updateOdometry(double linear_vel, double steering_angle, double dt) {
        // Calculate angular velocity from steering angle
        double angular_vel = calculateAngularVelocity(linear_vel, steering_angle);

        // Update orientation (yaw) based on angular velocity
        odom.orientation.z += angular_vel * dt;

        // Normalize orientation to [-π, π]
        while (odom.orientation.z > M_PI) odom.orientation.z -= 2.0 * M_PI;
        while (odom.orientation.z < -M_PI) odom.orientation.z += 2.0 * M_PI;

        // Update position based on linear velocity and orientation
        // For Ackermann steering, the vehicle follows a circular path during turns
        double dx = linear_vel * cos(odom.orientation.z) * dt;
        double dy = linear_vel * sin(odom.orientation.z) * dt;

        odom.position.x += dx;
        odom.position.y += dy;

        // Update velocity parts of odometry
        odom.linear.x = linear_vel;
        odom.linear.y = 0.0;  // No sideways motion in Ackermann steering
        odom.linear.z = 0.0;

        odom.angular.x = 0.0;
        odom.angular.y = 0.0;
        odom.angular.z = angular_vel;
    }

    void JetracerMock::updateIMU(double linear_vel, double steering_angle, double angular_vel, double dt) {
        // Update gyroscope readings (add some noise)
        imu.gyro.x = getRandomNoise(0.01);
        imu.gyro.y = getRandomNoise(0.01);
        imu.gyro.z = angular_vel + getRandomNoise(0.02);

        // Update accelerometer readings
        // For a car with Ackermann steering on a flat surface:
        // - Linear acceleration in x-axis direction
        // - Centripetal acceleration during turning
        // - Gravity along z-axis (9.81 m/s²)

        // Calculate linear acceleration (simple model with some lag)
        static double prev_linear_vel = 0.0;
        double linear_accel = (linear_vel - prev_linear_vel) / dt;
        prev_linear_vel = linear_vel;

        // Calculate centripetal acceleration (a = v²/r)
        // For Ackermann steering: r = L/tan(α) where L is wheelbase, α is steering angle
        double radius = fabs(steering_angle) > 0.001 ?
                        wheelbase_ / tan(fabs(steering_angle)) :
                        std::numeric_limits<double>::max();

        double centripetal_accel = (radius < std::numeric_limits<double>::max()) ?
                                   (linear_vel * linear_vel) / radius : 0.0;

        // Apply accelerations with appropriate signs based on motion
        imu.acc.x = linear_accel + getRandomNoise(0.05);

        // Lateral acceleration depends on turning direction
        if (steering_angle > 0) {
            imu.acc.y = -centripetal_accel + getRandomNoise(0.05);  // Turning left
        } else {
            imu.acc.y = centripetal_accel + getRandomNoise(0.05);   // Turning right
        }

        imu.acc.z = 9.81 + getRandomNoise(0.05);  // Gravity + noise

        // Update orientation angles
        // In a real IMU, these would come from sensor fusion,
        // but we can just use our odometry values with some noise
        imu.angle.x = getRandomNoise(0.5);  // Small roll variations
        imu.angle.y = getRandomNoise(0.5);  // Small pitch variations
        imu.angle.z = odom.orientation.z * (180.0 / M_PI) + getRandomNoise(1.0);  // Convert to degrees
    }

    double JetracerMock::getRandomNoise(double magnitude) {
        // Generate random noise with given magnitude
        return ((double)rand() / RAND_MAX * 2.0 - 1.0) * magnitude;
    }
} // jetracerController