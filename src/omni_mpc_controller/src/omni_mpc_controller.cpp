#include "omni_mpc_controller/omni_mpc_controller.hpp"
#include <algorithm>      // std::clamp
#include <cmath>          // std::cos, std::sin
#include <limits>

namespace omni_mpc_controller
{

OmniMPCController::OmniMPCController()
: noise_dist_(0.0, 1.0)  // unit Gaussian
{
}

void OmniMPCController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  node_       = parent.lock();
  tf_buffer_  = tf;
  costmap_ros_= costmap;

  // Declare & read parameters under <name> namespace
  node_->declare_parameter(name + ".time_steps", 10);
  node_->declare_parameter(name + ".model_dt", 0.1);
  node_->declare_parameter(name + ".batch_size", 500);
  node_->declare_parameter(name + ".lambda", 1.0);

  node_->declare_parameter(name + ".vx_min", -0.5);
  node_->declare_parameter(name + ".vx_max",  0.5);
  node_->declare_parameter(name + ".vy_min", -0.5);
  node_->declare_parameter(name + ".vy_max",  0.5);
  node_->declare_parameter(name + ".omega_min", -1.0);
  node_->declare_parameter(name + ".omega_max",  1.0);

  node_->declare_parameter(name + ".vx_std", 0.1);
  node_->declare_parameter(name + ".vy_std", 0.1);
  node_->declare_parameter(name + ".omega_std", 0.2);

  node_->get_parameter(name + ".time_steps", time_steps_);
  node_->get_parameter(name + ".model_dt",    model_dt_);
  node_->get_parameter(name + ".batch_size",  batch_size_);
  node_->get_parameter(name + ".lambda",      lambda_);

  node_->get_parameter(name + ".vx_min",      vx_min_);
  node_->get_parameter(name + ".vx_max",      vx_max_);
  node_->get_parameter(name + ".vy_min",      vy_min_);
  node_->get_parameter(name + ".vy_max",      vy_max_);
  node_->get_parameter(name + ".omega_min",   omega_min_);
  node_->get_parameter(name + ".omega_max",   omega_max_);

  node_->get_parameter(name + ".vx_std",      vx_std_);
  node_->get_parameter(name + ".vy_std",      vy_std_);
  node_->get_parameter(name + ".omega_std",   omega_std_);

  // Initialize nominal control sequence
  U_.assign(time_steps_, Control{0.0, 0.0, 0.0});
  generator_.seed(random_device_());
}

geometry_msgs::msg::TwistStamped
OmniMPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // Silence unused‐parameter warnings
  (void)velocity;
  (void)goal_checker;

  // Prepare storage
  std::vector<std::vector<Control>> noise(batch_size_,
    std::vector<Control>(time_steps_));
  std::vector<std::vector<Control>> candidates = noise;
  std::vector<double> costs(batch_size_, 0.0);
  std::vector<double> weights(batch_size_, 0.0);

  // 1) Sample K trajectories
  for (int k = 0; k < batch_size_; ++k) {
    for (int t = 0; t < time_steps_; ++t) {
      noise[k][t] = sampleNoise();
      candidates[k][t] = {
        U_[t].vx + noise[k][t].vx,
        U_[t].vy + noise[k][t].vy,
        U_[t].omega + noise[k][t].omega
      };
    }
  }

  // 2) Evaluate cost of each trajectory
#ifdef _OPENMP
  #pragma omp parallel for
#endif
  for (int k = 0; k < batch_size_; ++k) {
    auto x = pose;
    double S = 0.0;
    for (int t = 0; t < time_steps_; ++t) {
      x = propagate(x, candidates[k][t], model_dt_);
      // Placeholder cost: distance to final path point
      double dx = x.pose.position.x -
        global_path_.poses.back().pose.position.x;
      double dy = x.pose.position.y -
        global_path_.poses.back().pose.position.y;
      S += std::hypot(dx, dy);
    }
    costs[k] = S;
  }

  // 3) Compute weights exp(–S/λ)
  double Z = 0.0;
  for (int k = 0; k < batch_size_; ++k) {
    weights[k] = std::exp(-costs[k] / lambda_);
    Z += weights[k];
  }
  for (auto & w : weights) {
    w /= (Z + std::numeric_limits<double>::epsilon());
  }

  // 4) Update nominal controls
  for (int t = 0; t < time_steps_; ++t) {
    Control delta{0.0, 0.0, 0.0};
    for (int k = 0; k < batch_size_; ++k) {
      delta.vx    += weights[k] * noise[k][t].vx;
      delta.vy    += weights[k] * noise[k][t].vy;
      delta.omega += weights[k] * noise[k][t].omega;
    }
    U_[t].vx    += delta.vx;
    U_[t].vy    += delta.vy;
    U_[t].omega += delta.omega;
  }

  // 5) Build the output (first command, clamped)
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp    = node_->now();
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();

  cmd.twist.linear.x  = std::clamp(U_[0].vx,    vx_min_,    vx_max_);
  cmd.twist.linear.y  = std::clamp(U_[0].vy,    vy_min_,    vy_max_);
  cmd.twist.angular.z = std::clamp(U_[0].omega, omega_min_, omega_max_);

  // 6) Shift the sequence for next cycle
  for (int t = 0; t + 1 < time_steps_; ++t) {
    U_[t] = U_[t + 1];
  }
  U_[time_steps_ - 1] = {0.0, 0.0, 0.0};

  return cmd;
}

void OmniMPCController::setPlan(const nav_msgs::msg::Path & path)
{
  global_path_ = path;
}

void OmniMPCController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    vx_max_    *= speed_limit;
    vy_max_    *= speed_limit;
    omega_max_ *= speed_limit;
  } else {
    vx_max_    = speed_limit;
    vy_max_    = speed_limit;
    omega_max_ = speed_limit;
  }
}

void OmniMPCController::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "OmniMPCController::cleanup()");
}

void OmniMPCController::activate()
{
  RCLCPP_INFO(node_->get_logger(), "OmniMPCController::activate()");
}

void OmniMPCController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "OmniMPCController::deactivate()");
}

geometry_msgs::msg::PoseStamped
OmniMPCController::propagate(
  const geometry_msgs::msg::PoseStamped & pose,
  const Control & u,
  double dt)
{
  auto result = pose;
  double x   = pose.pose.position.x;
  double y   = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);

  x   += (u.vx * std::cos(yaw) - u.vy * std::sin(yaw)) * dt;
  y   += (u.vx * std::sin(yaw) + u.vy * std::cos(yaw)) * dt;
  yaw += u.omega * dt;

  result.pose.position.x = x;
  result.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  result.pose.orientation = tf2::toMsg(q);
  return result;
}

Control OmniMPCController::sampleNoise()
{
  return Control{
    vx_std_    * noise_dist_(generator_),
    vy_std_    * noise_dist_(generator_),
    omega_std_ * noise_dist_(generator_)
  };
}

}  // namespace omni_mpc_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  omni_mpc_controller::OmniMPCController,
  nav2_core::Controller
)
