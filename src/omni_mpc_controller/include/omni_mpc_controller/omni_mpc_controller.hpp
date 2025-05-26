#ifndef OMNI_MPC_CONTROLLER__OMNI_MPC_CONTROLLER_HPP_
#define OMNI_MPC_CONTROLLER__OMNI_MPC_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <random>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace omni_mpc_controller
{

struct Control
{
  double vx;
  double vy;
  double omega;
};

class OmniMPCController : public nav2_core::Controller
{
public:
  OmniMPCController();
  ~OmniMPCController() override = default;

  // nav2_core::Controller overrides
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  void cleanup()   override;
  void activate()  override;
  void deactivate()override;

private:
  // Helper for forward‐integrating one step
  geometry_msgs::msg::PoseStamped propagate(
    const geometry_msgs::msg::PoseStamped & pose,
    const Control & u,
    double dt);

  // Draw zero‐mean Gaussian noise for one control
  Control sampleNoise();

  // Node handles & TF / costmap
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // The global path (set by setPlan)
  nav_msgs::msg::Path global_path_;

  // MPPI parameters (loaded from YAML)
  int time_steps_;
  double model_dt_;
  int batch_size_;
  double lambda_;

  double vx_min_, vx_max_;
  double vy_min_, vy_max_;
  double omega_min_, omega_max_;

  double vx_std_, vy_std_, omega_std_;

  // Nominal control sequence
  std::vector<Control> U_;

  // RNG for noise
  std::mt19937 generator_;
  std::normal_distribution<double> noise_dist_;
  std::random_device random_device_;
};

}  // namespace omni_mpc_controller

#endif  // OMNI_MPC_CONTROLLER__OMNI_MPC_CONTROLLER_HPP_
