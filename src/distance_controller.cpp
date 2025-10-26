#include <algorithm>
#include <cmath>
#include <cstdlib>   // std::atoi
#include <vector>
#include <chrono>    // <-- added

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class DistanceController : public rclcpp::Node {
public:
  DistanceController(int scene_number)
      : Node("distance_controller"), scene_number_(scene_number) {
    selectWaypoints();

    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&DistanceController::odomCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DistanceController::controlLoop, this));

    last_time_ = std::chrono::steady_clock::now();  // <-- init for dt
  }

private:
  struct Goal { float x; float y; float theta; };

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Goal> relative_goals;
  Goal current_target;

  std::size_t current_goal_index = 0;
  bool initialized = false;
  bool goal_active = false;

  float current_x = 0.0f, current_y = 0.0f;

  float prev_error_x = 0.0f, integral_x = 0.0f;
  float prev_error_y = 0.0f, integral_y = 0.0f;

  // PID gains
  float kp = 1.2f;
  float ki = 0.001f;
  float kd = 0.001f;

  // Limits
  float max_linear_speed = 0.2f;  // m/s
  float goal_tolerance    = 0.02f; // m

  // NEW: Acceleration limit (m/s^2). Lower = gentler acceleration.
  float max_accel = 0.2f;

  // NEW: store last commanded velocities for slew limiting
  float last_cmd_x_ = 0.0f;
  float last_cmd_y_ = 0.0f;

  // NEW: for dt computation
  std::chrono::steady_clock::time_point last_time_;

  int scene_number_;

  void selectWaypoints() {
    switch (scene_number_) {
    case 1:
      RCLCPP_INFO(this->get_logger(), "Scene 1: Simulation waypoints selected.");
      relative_goals = {{0.0, 1.0, 0.0},  {0.0, -1.0, 0.0}, {0.0, -1.0, 0.0},
                        {0.0, 1.0, 0.0},  {1.0, 1.0, 0.0},  {-1.0, -1.0, 0.0},
                        {1.0, -1.0, 0.0}, {-1.0, 1.0, 0.0}, {1.0, 0.0, 0.0},
                        {-1.0, 0.0, 0.0}};
      break;
    case 2:
      RCLCPP_INFO(this->get_logger(), "Scene 2: CyberWorld waypoints selected.");
      relative_goals = {{0.98, 0.0, 0.0}, {0.0, -0.5, 0.0}, {0.0, 0.5, 0.0}, {-0.98, 0.0, 0.0}};
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d", scene_number_);
      rclcpp::shutdown();
      break;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    if (!initialized) {
      RCLCPP_INFO(this->get_logger(), "Initial position: (%.3f, %.3f)", current_x, current_y);
      initialized = true;
    }
  }

  static inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(v, hi));
  }

  // NEW: apply acceleration (slew-rate) limiting
  float slewToward(float current, float target, float max_rate, float dt) {
    const float dv = target - current;
    const float max_step = max_rate * dt;
    if (dv >  max_step) return current + max_step;
    if (dv < -max_step) return current - max_step;
    return target;
  }

  void controlLoop() {
    if (!initialized || current_goal_index >= relative_goals.size()) return;

    // dt since last loop
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_time_).count();
    if (dt <= 0.0f || dt > 1.0f) dt = 0.1f;  // guard against pauses; default to 100 ms
    last_time_ = now;

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

    RCLCPP_INFO(this->get_logger(), "delta X: %.3f, delta Y: %.3f, Distance: %.3f",
                error_x, error_y, distance);

    // PID compute (using dt for derivative)
    integral_x += error_x * dt;
    float derivative_x = (error_x - prev_error_x) / dt;
    float control_x = kp * error_x + ki * integral_x + kd * derivative_x;

    integral_y += error_y * dt;
    float derivative_y = (error_y - prev_error_y) / dt;
    float control_y = kp * error_y + ki * integral_y + kd * derivative_y;

    // Clamp to speed limits (these are the TARGET velocities)
    float target_vx = clampf(control_x, -max_linear_speed, max_linear_speed);
    float target_vy = clampf(control_y, -max_linear_speed, max_linear_speed);

    // === NEW: Acceleration limiting ===
    // Limit change per cycle: |Δv| <= max_accel * dt
    float cmd_vx = slewToward(last_cmd_x_, target_vx, max_accel, dt);
    float cmd_vy = slewToward(last_cmd_y_, target_vy, max_accel, dt);

    geometry_msgs::msg::Twist vel;
    vel.linear.x = cmd_vx;
    vel.linear.y = cmd_vy;
    vel_pub->publish(vel);

    // store for next loop
    last_cmd_x_ = cmd_vx;
    last_cmd_y_ = cmd_vy;

    if (distance < goal_tolerance) {
      RCLCPP_INFO(this->get_logger(), "Relative Goal %zu reached", current_goal_index);

      geometry_msgs::msg::Twist stop;
      // Optional: ramp down instead of hard stop (kept simple here)
      for (int i = 0; i < 2; ++i) {
        vel_pub->publish(stop);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }

      current_goal_index++;
      goal_active = false;
      integral_x = 0.0f;
      integral_y = 0.0f;
      prev_error_x = 0.0f;
      prev_error_y = 0.0f;
      last_cmd_x_ = 0.0f;  // reset ramp
      last_cmd_y_ = 0.0f;

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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int scene_number = 1; // Default: Simulation
  if (argc > 1) scene_number = std::atoi(argv[1]);

  auto node = std::make_shared<DistanceController>(scene_number);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
