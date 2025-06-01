#ifndef OMNI_SBMPO_CONTROLLER__OMNI_SBMPO_CONTROLLER_HPP_
#define OMNI_SBMPO_CONTROLLER__OMNI_SBMPO_CONTROLLER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <limits>
#include <cmath>

#include "nav2_core/controller.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace omni_sbmpo_controller
{

class OmniSBMPOController : public nav2_core::Controller
{
public:
  OmniSBMPOController() = default;
  ~OmniSBMPOController() override = default;

  // nav2_core::Controller interface
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  using Trajectory = std::vector<geometry_msgs::msg::Pose>;

  // simulate one (v,w) from start, return (cost, trajectory)
  std::pair<double, Trajectory> simulateTrajectory(
    const geometry_msgs::msg::Pose & start,
    double v, double w);

  // pointers injected by Nav2
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // current global plan + goal
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped last_goal_;

  // SBMPO parameters
  double horizon_{2.5}, dt_{0.1}, v_max_{0.5}, w_max_{1.0};
  int num_v_{7}, num_w_{7};
  double obstacle_weight_{15.0}, path_weight_{0.05}, forward_reward_{0.1};
  int obstacle_threshold_{30};
  double penalty_factor_{300.0}, emergency_ttc_{0.5}, clearance_penalty_weight_{2.0};

  // optional speed‐limit override
  double speed_limit_{0.0};
  bool speed_percentage_{false};
};

}  // namespace omni_sbmpo_controller

#endif  // OMNI_SBMPO_CONTROLLER__OMNI_SBMPO_CONTROLLER_HPP_