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
            {0.0, 1.0, +0.00000},
            {0.0, -1.0, +0.00000},
            {0.0, -1.0, +0.00000},
            {0.0, 1.0, +0.00000},
            {1.0, 1.0, +0.00000},
            {-1.0, -1.0, +0.00000},
            {1.0, -1.0, +0.00000},
            {-1.0, 1.0, +0.00000},
            {1.0, 0.0, +0.00000},
            {-1.0, 0.0, +0.00000}
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
    struct Goal {
        float x;
        float y;
        float theta;
    };

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Goal> relative_goals;
    Goal current_target;

    std::size_t current_goal_index = 0;
    bool initialized = false;
    bool goal_active = false;

    float current_x = 0.0, current_y = 0.0;

    float prev_error_x = 0.0, integral_x = 0.0;
    float prev_error_y = 0.0, integral_y = 0.0;

    // float kp = 0.5, ki = 0.0125, kd = 0.0025;

    float kp = 0.8;   // higier than 1.0 overshoots in xy motion
    float ki = 0.01;
    float kd = 0.08;

    float max_linear_speed = 0.5;
    float goal_tolerance = 0.04;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x = msg->pose.pose.position.x;
        current_y = msg->pose.pose.position.y;

        if (!initialized) {
            RCLCPP_INFO(this->get_logger(), "Initial position: (%.3f, %.3f)", current_x, current_y);
            initialized = true;
        }
    }

    void controlLoop() {
        if (!initialized || current_goal_index >= relative_goals.size())
            return;

        // Set new goal relative to current position
        if (!goal_active) {
            current_target.x = current_x + relative_goals[current_goal_index].x;
            current_target.y = current_y + relative_goals[current_goal_index].y;
            goal_active = true;
            RCLCPP_INFO(this->get_logger(), "New target #%zu set to (%.3f, %.3f)",
                        current_goal_index, current_target.x, current_target.y);
        }

        float error_x = current_target.x - current_x;
        float error_y = current_target.y - current_y;
        float distance = std::sqrt(error_x * error_x + error_y * error_y);

        RCLCPP_INFO(this->get_logger(),
            "delta X: %.3f, delta Y: %.3f, Distance: %.3f",
            error_x, error_y, distance);

        // PID control
        integral_x += error_x;
        float derivative_x = (error_x - prev_error_x) / 0.1;
        float control_x = kp * error_x + ki * integral_x + kd * derivative_x;

        integral_y += error_y;
        float derivative_y = (error_y - prev_error_y) / 0.1;
        float control_y = kp * error_y + ki * integral_y + kd * derivative_y;

        control_x = std::clamp(control_x, -max_linear_speed, max_linear_speed);
        control_y = std::clamp(control_y, -max_linear_speed, max_linear_speed);

        geometry_msgs::msg::Twist vel;
        vel.linear.x = control_x;
        vel.linear.y = control_y;
        vel_pub->publish(vel);

        if (distance < goal_tolerance) {
            RCLCPP_INFO(this->get_logger(), "Relative Goal %zu reached", current_goal_index);

            geometry_msgs::msg::Twist stop;
            for (int i = 0; i < 2; ++i) {
                vel_pub->publish(stop);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }

            current_goal_index++;
            goal_active = false;
            integral_x = 0.0;
            integral_y = 0.0;

            if (current_goal_index >= relative_goals.size()) {
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