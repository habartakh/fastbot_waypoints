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

#include "gtest/gtest.h"

using Waypoint = fastbot_waypoints::action::WaypointAction;
using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;
