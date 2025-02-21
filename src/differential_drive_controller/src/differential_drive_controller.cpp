#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <algorithm>

class DifferentialDriveController : public rclcpp::Node {
public:
    DifferentialDriveController() : Node("differential_drive_controller") {
        // Parameters with safety limits
        declare_parameter("wheelbase", 0.5);
        declare_parameter("wheel_radius", 0.1);
        declare_parameter("max_rpm", 150.0);  // Added max_rpm parameter
        
        // Get and validate parameters
        wheelbase_ = get_parameter("wheelbase").as_double();
        wheel_radius_ = get_parameter("wheel_radius").as_double();
        max_rpm_ = get_parameter("max_rpm").as_double();
        
        // Parameter validation
        if (wheelbase_ <= 0 || wheel_radius_ <= 0 || max_rpm_ <= 0) {
            RCLCPP_ERROR(get_logger(), "Invalid parameters! All parameters must be positive.");
            throw std::runtime_error("Invalid parameters");
        }

        // Create publishers and subscribers
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&DifferentialDriveController::cmdVelCallback, this, std::placeholders::_1));

        left_rpm_pub_ = create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
        right_rpm_pub_ = create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);

        RCLCPP_INFO(get_logger(), "Controller initialized with:");
        RCLCPP_INFO(get_logger(), "Wheelbase: %.2f m", wheelbase_);
        RCLCPP_INFO(get_logger(), "Wheel radius: %.2f m", wheel_radius_);
        RCLCPP_INFO(get_logger(), "Max RPM: %.2f", max_rpm_);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        try {
            // Get velocities
            double linear = msg->linear.x;
            double angular = msg->angular.z;

            // Safety check for input velocities
            if (std::isnan(linear) || std::isnan(angular)) {
                RCLCPP_WARN(get_logger(), "Received NaN velocity commands!");
                return;
            }

            // Calculate wheel velocities
            double left_velocity = (1.0 * linear - angular * wheelbase_) / (1.0 * wheel_radius_);
            double right_velocity = (1.0 * linear + angular * wheelbase_) / (1.0 * wheel_radius_);

            // Convert to RPM
            double left_rpm = (left_velocity * 60.0) / (2.0 * M_PI);
            double right_rpm = (right_velocity * 60.0) / (2.0 * M_PI);

            // Apply RPM limits
            left_rpm = std::clamp(left_rpm, -max_rpm_, max_rpm_);
            right_rpm = std::clamp(right_rpm, -max_rpm_, max_rpm_);

            // Create messages
            auto left_msg = std_msgs::msg::Float64();
            auto right_msg = std_msgs::msg::Float64();
            left_msg.data = left_rpm;
            right_msg.data = right_rpm;

            // Publish
            left_rpm_pub_->publish(left_msg);
            right_rpm_pub_->publish(right_msg);

            // Log only if velocities are non-zero (reduce spam)
            if (linear != 0.0 || angular != 0.0) {
                RCLCPP_INFO(get_logger(), 
                    "Cmd: lin=%.2f ang=%.2f -> RPM: left=%.2f right=%.2f",
                    linear, angular, left_rpm, right_rpm);
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error processing velocity command: %s", e.what());
        }
    }

    // Member variables
    double wheelbase_;
    double wheel_radius_;
    double max_rpm_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_rpm_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<DifferentialDriveController>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("differential_drive_controller"), 
            "Node crashed with error: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}