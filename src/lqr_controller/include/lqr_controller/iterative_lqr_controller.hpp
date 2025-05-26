#ifndef ITERATIVE_LQR_CONTROLLER__ITERATIVE_LQR_CONTROLLER_HPP_
#define ITERATIVE_LQR_CONTROLLER__ITERATIVE_LQR_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_core/controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace iterative_lqr_controller
{

class IterativeLQRController : public nav2_core::Controller
{
public:
  IterativeLQRController();
  ~IterativeLQRController() override;

  // Lifecycle hooks
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // Controller interface
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed, const bool & percentage) override;

private:
  // Node handle & logging
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;

  // TF & costmap
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Parameters
  int    horizon_;
  double dt_;
  Eigen::Matrix2d Q_, R_;
  double max_vel_x_, max_vel_y_, max_vel_theta_;

  // Current plan & optional laser
  nav_msgs::msg::Path current_plan_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan last_scan_;

  // Helpers
  nav_msgs::msg::Path      transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
  Eigen::VectorXd          solveILQR(const nav_msgs::msg::Path & local_plan,
                                     const geometry_msgs::msg::PoseStamped & pose);
  void                     scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

}  // namespace iterative_lqr_controller

#endif  // ITERATIVE_LQR_CONTROLLER__ITERATIVE_LQR_CONTROLLER_HPP_
