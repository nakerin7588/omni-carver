#ifndef OMNI_CARVER_PURE_PURSUIT__OMNI_CARVER_PURE_PURSUIT_HPP_
#define OMNI_CARVER_PURE_PURSUIT__OMNI_CARVER_PURE_PURSUIT_HPP_

#include <string>
#include <vector>
#include <memory>

#include <angles/angles.h>
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace omni_carver_pure_pursuit
{

class PurePursuitController : public nav2_core::Controller
{
public:
  PurePursuitController() = default;
  ~PurePursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;


  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

protected:
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  bool transformPose(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    const rclcpp::Duration & transform_tolerance
  ) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("PurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  double target_linear_velocity_;
  double lookahead_distance_;
  double max_angular_velocity_;
  double yaw_gain_;
  rclcpp::Duration transform_tolerance_ {0, 0};

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
};

}  // namespace omni_carver_pure_pursuit

#endif  // OMNI_CARVER_PURE_PURSUIT__OMNI_CARVER_PURE_PURSUIT_HPP_