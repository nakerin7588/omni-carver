#include "lqr_controller/iterative_lqr_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

using iterative_lqr_controller::IterativeLQRController;
using nav2_util::declare_parameter_if_not_declared;

IterativeLQRController::IterativeLQRController()
: logger_(rclcpp::get_logger("IterativeLQRController"))
{}

IterativeLQRController::~IterativeLQRController() = default;

void IterativeLQRController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::ControllerException("iLQR: failed to lock node");
  }
  plugin_name_  = name;
  logger_       = node->get_logger();
  clock_        = node->get_clock();
  tf_buffer_    = tf;
  costmap_ros_  = costmap_ros;

  // declare & read parameters
  declare_parameter_if_not_declared(node, name + ".horizon",       rclcpp::ParameterValue(10));
  declare_parameter_if_not_declared(node, name + ".dt",            rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, name + ".Q_x",           rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(node, name + ".Q_y",           rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(node, name + ".R_vx",          rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name + ".R_vy",          rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name + ".max_vel_x",     rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name + ".max_vel_y",     rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name + ".max_vel_theta", rclcpp::ParameterValue(1.0));

  node->get_parameter(name + ".horizon",       horizon_);
  node->get_parameter(name + ".dt",            dt_);
  double Qx, Qy, Rvx, Rvy;
  node->get_parameter(name + ".Q_x",           Qx);
  node->get_parameter(name + ".Q_y",           Qy);
  node->get_parameter(name + ".R_vx",          Rvx);
  node->get_parameter(name + ".R_vy",          Rvy);
  node->get_parameter(name + ".max_vel_x",     max_vel_x_);
  node->get_parameter(name + ".max_vel_y",     max_vel_y_);
  node->get_parameter(name + ".max_vel_theta", max_vel_theta_);

  // build weight matrices
  Q_ = Eigen::Matrix2d::Zero();
  Q_(0,0) = Qx;  Q_(1,1) = Qy;
  R_ = Eigen::Matrix2d::Zero();
  R_(0,0) = Rvx; R_(1,1) = Rvy;

  // optional laser subscription
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::QoS(10),
    std::bind(&IterativeLQRController::scanCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "'%s' configured (horizon=%d, dt=%.2f)",
              plugin_name_.c_str(), horizon_, dt_);
}

void IterativeLQRController::cleanup()
{
  scan_sub_.reset();
  RCLCPP_INFO(logger_, "Cleaned up '%s'", plugin_name_.c_str());
}

void IterativeLQRController::activate()
{
  RCLCPP_INFO(logger_, "Activating '%s'", plugin_name_.c_str());
}

void IterativeLQRController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating '%s'", plugin_name_.c_str());
}

void IterativeLQRController::setPlan(const nav_msgs::msg::Path & path)
{
  current_plan_ = path;
}

void IterativeLQRController::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_ = *msg;
}

nav_msgs::msg::Path IterativeLQRController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  // ─── copy/paste Nav2's pure_pursuit transformGlobalPlan here ───
  //    prune & TF into costmap_ros_->getBaseFrameID()
  nav_msgs::msg::Path local_plan;
  // TODO: implement exactly as in nav2_pure_pursuit_controller
  return local_plan;
}

Eigen::VectorXd IterativeLQRController::solveILQR(
  const nav_msgs::msg::Path & local_plan,
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Build an initial U = zeros(2*horizon)
  Eigen::VectorXd U = Eigen::VectorXd::Zero(2 * horizon_);

  // ─── TODO: Implement the backward/forward pass of iLQR here ───

  return U;
}

geometry_msgs::msg::TwistStamped IterativeLQRController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::ControllerException("iLQR: node expired");
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;
  cmd.header.stamp = clock_->now();

  // 1. transform plan
  auto local_plan = transformGlobalPlan(pose);
  if (local_plan.poses.empty()) {
    RCLCPP_WARN(logger_, "Local plan empty");
    return cmd;
  }

  // 2. solve iLQR
  Eigen::VectorXd U = solveILQR(local_plan, pose);

  // 3. apply first control
  double vx = std::clamp(U(0), -max_vel_x_, max_vel_x_);
  double vy = std::clamp(U(1), -max_vel_y_, max_vel_y_);
  cmd.twist.linear.x  = vx;
  cmd.twist.linear.y  = vy;
  cmd.twist.angular.z = 0.0;

  return cmd;
}

void IterativeLQRController::setSpeedLimit(const double &, const bool &){}

PLUGINLIB_EXPORT_CLASS(
  iterative_lqr_controller::IterativeLQRController,
  nav2_core::Controller)
