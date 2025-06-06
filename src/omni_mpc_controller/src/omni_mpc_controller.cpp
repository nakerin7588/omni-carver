#include "omni_mpc_controller/omni_mpc_controller.hpp"

#include "rclcpp/logging.hpp"                // for RCLCPP_INFO
#include "tf2/utils.h"
#include "pluginlib/class_list_macros.hpp"

namespace omni_mpc_controller
{

void OmniMPCController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  tf_buffer_   = tf_buffer;
  costmap_ros_ = costmap_ros;

  // declare & read parameters under "<name>.*"
  horizon_     = node->declare_parameter(name + ".horizon", horizon_);
  dt_          = node->declare_parameter(name + ".model_dt", dt_);
  num_v_       = node->declare_parameter(name + ".num_samples_v", num_v_);
  num_w_       = node->declare_parameter(name + ".num_samples_w", num_w_);
  vx_max_      = node->declare_parameter(name + ".vx_max", vx_max_);
  vx_min_      = node->declare_parameter(name + ".vx_min", vx_min_);
  vy_max_      = node->declare_parameter(name + ".vy_max", vy_max_);
  vy_min_      = node->declare_parameter(name + ".vy_min", vy_min_);
  wz_max_      = node->declare_parameter(name + ".wz_max", wz_max_);
  wz_min_      = node->declare_parameter(name + ".wz_min", wz_min_);
  obstacle_weight_         = node->declare_parameter(name + ".obstacle_weight", obstacle_weight_);
  path_weight_             = node->declare_parameter(name + ".path_weight", path_weight_);
  forward_reward_          = node->declare_parameter(name + ".forward_reward", forward_reward_);
  obstacle_threshold_      = node->declare_parameter(name + ".obstacle_threshold", obstacle_threshold_);
  penalty_factor_          = node->declare_parameter(name + ".penalty_factor", penalty_factor_);
  emergency_ttc_           = node->declare_parameter(name + ".emergency_ttc", emergency_ttc_);
  clearance_penalty_weight_= node->declare_parameter(name + ".clearance_penalty_weight", clearance_penalty_weight_);

  RCLCPP_INFO(
    node->get_logger(), "Configured OmniMPCController '%s'", name.c_str());
}

geometry_msgs::msg::TwistStamped OmniMPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * checker)
{
  // 1) if goal reached, return zero
  if (checker && checker->isGoalReached(pose.pose, last_goal_.pose, geometry_msgs::msg::Twist{})) {
    return geometry_msgs::msg::TwistStamped();
  }

  // 2) sample (v,w) grid
  double best_cost = std::numeric_limits<double>::infinity();
  double best_v = 0.0, best_w = 0.0;

  for (int i = 0; i < num_v_; ++i) {
    double v = vx_min_ + (vx_max_ - vx_min_) * double(i) / (num_v_ - 1);
    for (int j = 0; j < num_w_; ++j) {
      double w = wz_min_ + (wz_max_ - wz_min_) * double(j) / (num_w_ - 1);

      // apply global speed limit if set
      double v_cmd = v, w_cmd = w;
      if (speed_limit_ > 0.0) {
        if (speed_percentage_) {
          v_cmd *= speed_limit_;
          w_cmd *= speed_limit_;
        } else {
          v_cmd = std::copysign(std::min(std::abs(v_cmd), speed_limit_), v_cmd);
          w_cmd = std::copysign(std::min(std::abs(w_cmd), speed_limit_), w_cmd);
        }
      }

      auto result = simulateTrajectory(pose.pose, v_cmd, w_cmd);
      double cost = result.first;
      if (cost < best_cost) {
        best_cost = cost;
        best_v = v_cmd;
        best_w = w_cmd;
      }
    }
  }

  // 3) publish best
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;
  cmd.twist.linear.x  = best_v;
  cmd.twist.linear.y  = 0.0;
  cmd.twist.angular.z = best_w;
  return cmd;
}

void OmniMPCController::setPlan(const nav_msgs::msg::Path & path)
{
  path_ = path;
  if (!path_.poses.empty()) {
    last_goal_ = path_.poses.back();
  }
}

void OmniMPCController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  speed_limit_      = speed_limit;
  speed_percentage_ = percentage;
}

std::pair<double, OmniMPCController::Trajectory>
OmniMPCController::simulateTrajectory(
  const geometry_msgs::msg::Pose & start,
  double v, double w)
{
  double x   = start.position.x;
  double y   = start.position.y;
  double yaw = tf2::getYaw(start.orientation);

  int steps = int(horizon_ / dt_);
  double total_cost = 0.0;
  int    min_cost   = std::numeric_limits<int>::max();
  Trajectory traj; traj.reserve(steps);

  // costmap info
  auto cm     = costmap_ros_->getCostmap()->getCharMap();
  auto width  = costmap_ros_->getCostmap()->getSizeInCellsX();
  auto height = costmap_ros_->getCostmap()->getSizeInCellsY();
  double res  = costmap_ros_->getCostmap()->getResolution();
  double ox   = costmap_ros_->getCostmap()->getOriginX();
  double oy   = costmap_ros_->getCostmap()->getOriginY();

  for (int i = 0; i < steps; ++i) {
    // forward simulate
    x   += v * std::cos(yaw) * dt_;
    y   += v * std::sin(yaw) * dt_;
    yaw += w * dt_;

    int mx = int((x - ox) / res);
    int my = int((y - oy) / res);
    int c  = 100;  // default obstacle
    if (mx >= 0 && my >= 0 && mx < int(width) && my < int(height)) {
      int idx = my * width + mx;
      c = cm[idx] < 0 ? 50 : cm[idx];
    }
    min_cost = std::min(min_cost, c);

    // obstacle penalty or collision
    if (c >= obstacle_threshold_) {
      double ttc = i * dt_;
      if (ttc < emergency_ttc_) {
        return {std::numeric_limits<double>::infinity(), {}};
      }
      total_cost += (horizon_ - ttc) * penalty_factor_;
    } else {
      total_cost += obstacle_weight_ * (c / 100.0);
    }

    // path‐tracking error to final goal
    double dx = x - last_goal_.pose.position.x;
    double dy = y - last_goal_.pose.position.y;
    total_cost += path_weight_ * std::hypot(dx, dy);

    // forward reward
    total_cost -= forward_reward_ * v;

    // record trajectory (for viz)
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.orientation = tf2::toMsg(
      tf2::Quaternion(0, 0, std::sin(yaw/2), std::cos(yaw/2)));
    traj.push_back(p);
  }

  // clearance penalty
  total_cost += clearance_penalty_weight_ * min_cost;
  return {total_cost, traj};
}

}  // namespace omni_mpc_controller

PLUGINLIB_EXPORT_CLASS(
  omni_mpc_controller::OmniMPCController,
  nav2_core::Controller)
