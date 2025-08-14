#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>

#include "fastbot_waypoints/action/waypoint_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using Waypoint = fastbot_waypoints::action::WaypointAction;
using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class WaypointActionServer : public rclcpp::Node {
public:
  explicit WaypointActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("fastbot_as", options), yaw_precision_(M_PI / 90.0),
        dist_precision_(0.05), state_("idle") {
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<Waypoint>(
        this, "fastbot_as",
        std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
        std::bind(&WaypointActionServer::handle_cancel, this, _1),
        std::bind(&WaypointActionServer::handle_accepted, this, _1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot/cmd_vel", 1);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        std::bind(&WaypointActionServer::odom_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Fastbot Waypoint Action Server started");
  }

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  geometry_msgs::msg::Point position_;
  double yaw_{0.0};
  double initial_yaw = 1.52; // The robot spawns with an offset
  std::string state_;

  double yaw_precision_, dist_precision_;
  geometry_msgs::msg::Point des_pos_;

  bool success = false; // If the robot successfully reached the goal

  enum class State { FIX_YAW, GO_TO_POINT, FIX_FINAL_YAW };
  State state = State::FIX_YAW;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Waypoint::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal point: x=%.2f y=%.2f",
                goal->position.x, goal->position.y);
    success = false;
    // state = State::FIX_YAW;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    std::thread{std::bind(&WaypointActionServer::execute, this, _1),
                goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");
    rclcpp::Rate rate(25);

    des_pos_ = goal_handle->get_goal()->position;

    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto result = std::make_shared<Waypoint::Result>();

    while (rclcpp::ok() && !success) {

      double dx = des_pos_.x - position_.x;
      double dy = des_pos_.y - position_.y;
      double err_pos = std::hypot(dx, dy);
      double desired_yaw = normalize_angle(std::atan2(dy, dx));
      double err_yaw = normalize_angle(desired_yaw - (yaw_ - initial_yaw));

      // RCLCPP_INFO(this->get_logger(), "err_pos = %f", err_pos);
      // RCLCPP_INFO(this->get_logger(), "desired_yaw = %f", desired_yaw);
      // RCLCPP_INFO(this->get_logger(), "err_yaw = %f", err_yaw);

      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      geometry_msgs::msg::Twist twist;

      switch (state) {
      case State::FIX_YAW:
        if (std::fabs(err_yaw) > yaw_precision_) {
          state_ = "fix yaw";
          RCLCPP_INFO(this->get_logger(), "State: fix yaw");
          twist.angular.z = (err_yaw > 0) ? 0.25 : -0.25;
          twist.linear.x = 0.0;

        } else {
          twist.angular.z = 0.0;
          state = State::GO_TO_POINT;
          continue;
        }
        break;

      case State::GO_TO_POINT:
        if (err_pos < dist_precision_) {
          state = State::FIX_FINAL_YAW;
          continue;
        }
        if (std::fabs(err_yaw) > yaw_precision_) {
          state = State::FIX_YAW;
          continue;
        }
        twist.angular.z = 0.0;
        twist.linear.x = 0.4;
        RCLCPP_INFO(this->get_logger(), "State: Go to Point");
        break;

      case State::FIX_FINAL_YAW:
        RCLCPP_INFO(this->get_logger(), "State: fix final yaw");
        if (std::fabs(err_yaw) > yaw_precision_) {
          state_ = "fix final yaw";
          RCLCPP_INFO(this->get_logger(), "State: fix final yaw 2");
          twist.angular.z = (err_yaw > 0) ? 0.15 : -0.15;
          twist.linear.x = 0.0;
        } else {
          // Finished movement
          twist.angular.z = 0.0;
          twist.linear.x = 0.0;
          success = true;
        }
        break;
      }

      cmd_vel_pub_->publish(twist);

      feedback->position = position_;
      feedback->state = state_;
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    // Stop robot
    geometry_msgs::msg::Twist stop_twist;
    cmd_vel_pub_->publish(stop_twist);

    // Return success
    result->success = success;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Reached goal successfully");
    state = State::FIX_YAW;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position_ = msg->pose.pose.position;
    // Extract yaw from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_);
  }

  // normalize angles to range [-pi, pi]
  double normalize_angle(double angle) {

    while (angle > M_PI) {
      angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2 * M_PI;
    }
    return angle;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointActionServer>());
  rclcpp::shutdown();
  return 0;
}
