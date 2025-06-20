#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

class DistanceController : public rclcpp::Node {
public:
    DistanceController() : Node("distance_controller") {
        // Define relative movement goals (delta_x, delta_y)
        relative_goals = {
            {0.0, -1.0},
            {0.0, 1.0},
            {1.0, 0.0},
            {-1.0, 0.0},
            {1.0, -1.0},
            {-1.0, 1.0},
            {1.0, 1.0},
            {-1.0, -1.0},
            {1.0, 0.0},
            {-1.0, 0.0}
        };

        vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&DistanceController::odomCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DistanceController::controlLoop, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    struct Goal {
        float x;
        float y;
    };

    std::vector<Goal> relative_goals;
    std::vector<Goal> absolute_goals;

    std::size_t current_goal_index = 0;
    bool initialized = false;

    float initial_x = 0.0, initial_y = 0.0;
    float current_x = 0.0, current_y = 0.0;

    float prev_error_x = 0.0, integral_x = 0.0;
    float prev_error_y = 0.0, integral_y = 0.0;

    float kp = 0.5, ki = 0.0125, kd = 0.0025;
    float max_linear_speed = 0.5;
    float goal_tolerance = 0.04;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x = msg->pose.pose.position.x;
        current_y = msg->pose.pose.position.y;

        // Only store the initial position once
        if (!initialized) {
            initial_x = current_x;
            initial_y = current_y;

            // Convert relative to absolute goals
            for (const auto& g : relative_goals) {
                absolute_goals.push_back({initial_x + g.x, initial_y + g.y});
            }

            initialized = true;
            RCLCPP_INFO(this->get_logger(), "Initial position recorded: (%.2f, %.2f)", initial_x, initial_y);
        }
    }

    void controlLoop() {
        if (!initialized || current_goal_index >= absolute_goals.size())
            return;

        auto goal = absolute_goals[current_goal_index];

        float error_x = goal.x - current_x;
        float error_y = goal.y - current_y;
        float distance_to_goal = std::sqrt(error_x * error_x + error_y * error_y);
        RCLCPP_INFO(this->get_logger(),
        "delta X: %.3f, delta Y: %.3f, Distance: %.3f",
        error_x, error_y, distance_to_goal);

        // PID for X
        integral_x += error_x;
        float derivative_x = (error_x - prev_error_x) / 0.1;
        float control_x = kp * error_x + ki * integral_x + kd * derivative_x;

        // PID for Y
        integral_y += error_y;
        float derivative_y = (error_y - prev_error_y) / 0.1;
        float control_y = kp * error_y + ki * integral_y + kd * derivative_y;

        // Clamp
        control_x = std::clamp(control_x, -max_linear_speed, max_linear_speed);
        control_y = std::clamp(control_y, -max_linear_speed, max_linear_speed);

        geometry_msgs::msg::Twist vel;
        vel.linear.x = control_x;
        vel.linear.y = control_y;
        vel_pub->publish(vel);

        if (std::abs(error_x) < goal_tolerance && std::abs(error_y) < goal_tolerance) {
            RCLCPP_INFO(this->get_logger(), "Relative Goal %zu reached", current_goal_index);

            geometry_msgs::msg::Twist stop;
            for (int i = 0; i < 2; ++i) {
                vel_pub->publish(stop);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }

            current_goal_index++;
            integral_x = 0.0;
            integral_y = 0.0;

            if (current_goal_index >= absolute_goals.size()) {
                vel_pub->publish(stop);
                timer_->cancel();
                rclcpp::shutdown();
            }
        }

        prev_error_x = error_x;
        prev_error_y = error_y;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
