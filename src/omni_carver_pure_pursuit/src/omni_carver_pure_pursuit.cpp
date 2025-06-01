#include <algorithm>
#include <string>
#include <memory>

#include <angles/angles.h>
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "omni_carver_pure_pursuit/omni_carver_pure_pursuit.hpp"
#include "nav2_util/geometry_utils.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace omni_carver_pure_pursuit
{

/**
* Find element in iterator with the minimum calculated value
*/
template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

void PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;

  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".target_linear_velocity", rclcpp::ParameterValue(
      0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_distance",
    rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".yaw_gain", rclcpp::ParameterValue(
      1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity", rclcpp::ParameterValue(
      1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(
      0.1));

  node->get_parameter(plugin_name_ + ".target_linear_velocity", target_linear_velocity_);
  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".yaw_gain", yaw_gain_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
}

void PurePursuitController::cleanup(){}

void PurePursuitController::activate(){}

void PurePursuitController::deactivate(){}

void PurePursuitController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  (void) speed_limit;
  (void) percentage;
}

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;

  auto transformed_plan = transformGlobalPlan(pose);

  // Find the first pose which is at a distance greater than the specified lookahed distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_distance_;
    });

  // If the last pose is still within lookahed distance, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }
  auto goal_pose = goal_pose_it->pose;

  double linear_vel_x, linear_vel_y, angular_vel;
//  if (goal_pose.position.x > 0) {
//   auto curvature = 2.0 * goal_pose.position.y /
//     (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
//   linear_vel = desired_linear_vel_;
//   angular_vel = desired_linear_vel_ * curvature;
// } else {
//   linear_vel = 0.0;
//   angular_vel = max_angular_vel_;
// }

RCLCPP_INFO(
    logger_,
    "Goal pose: x: %.2f, y: %.2f, orientation: %.2f",
    goal_pose.position.x,
    goal_pose.position.y,
    tf2::getYaw(goal_pose.orientation));

if (goal_pose.position.x > 0.02 || goal_pose.position.y > 0.02 || goal_pose.position.x < -0.02 || goal_pose.position.y < -0.02) {
  // Calculate the linear vector towards the goal pose
  // double ux = goal_pose.position.x / lookahead_distance_;
  // double uy = goal_pose.position.y / lookahead_distance_;

  // Calculate the linear velocity components
  linear_vel_x = target_linear_velocity_ * std::cos(atan2(goal_pose.position.y, goal_pose.position.x));
  linear_vel_y = target_linear_velocity_ * std::sin(atan2(goal_pose.position.y, goal_pose.position.x));

  // double goal_yaw    = tf2::getYaw(goal_pose.orientation);
  // double current_yaw = tf2::getYaw(pose.pose.orientation);

  // double yaw_error = angles::shortest_angular_distance(current_yaw, goal_yaw);
  // double angular_vel = yaw_gain_ * yaw_error;
  // auto curvature = 2.0 * goal_pose.position.y /
  //   (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
  // linear_vel_x = desired_linear_vel_;
  // angular_vel = target_linear_velocity_ * curvature;
}
else {
  linear_vel_x = 0.0;
  linear_vel_y = 0.0;
  angular_vel = max_angular_velocity_;
  // if (tf2::getYaw(goal_pose.orientation) > 0){
  //   // If the goal is in the positive x direction, we can set a positive angular velocity
  //   angular_vel = max_angular_velocity_;
  // } else {
  //   // If the goal is in the negative x direction, we can set a negative angular velocity
  //   angular_vel = -max_angular_velocity_;
  // }
}

// Create and publish a TwistStamped message with the desired velocity
geometry_msgs::msg::TwistStamped cmd_vel;
cmd_vel.header.frame_id = pose.header.frame_id;
cmd_vel.header.stamp = clock_->now();
cmd_vel.twist.linear.x = linear_vel_x;
cmd_vel.twist.linear.y = linear_vel_y;
cmd_vel.twist.angular.z = std::clamp(angular_vel, -max_angular_velocity_, +max_angular_velocity_);;

return cmd_vel;
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_pub_->publish(path);
  global_plan_ = path;
}

nav_msgs::msg::Path
PurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Original mplementation taken fron nav2_dwb_controller

  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(
      tf_, global_plan_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin =
    min_by(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // From the closest point, look for the first point that's further then dist_threshold from the
  // robot. These points are definitely outside of the costmap so we won't transform them.
  auto transformation_end = std::find_if(
    transformation_begin, end(global_plan_.poses),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
    });

  // Helper function for the transform below. Transforms a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      // We took a copy of the pose, let's lookup the transform at the current time
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
        tf_, costmap_ros_->getBaseFrameID(),
        stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool PurePursuitController::transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & transform_tolerance
) const
{
  // Implementation taken as is from nav_2d_utils in nav2_dwb_controller

  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (tf2::ExtrapolationException & ex) {
    auto transform = tf->lookupTransform(
      frame,
      in_pose.header.frame_id,
      tf2::TimePointZero
    );
    if (
      (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      transform_tolerance)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Transform data too old when converting from %s to %s",
        in_pose.header.frame_id.c_str(),
        frame.c_str()
      );
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Data time: %ds %uns, Transform time: %ds %uns",
        in_pose.header.stamp.sec,
        in_pose.header.stamp.nanosec,
        transform.header.stamp.sec,
        transform.header.stamp.nanosec
      );
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("tf_help"),
      "Exception in transformPose: %s",
      ex.what()
    );
    return false;
  }
  return false;
}

}  // namespace omni_carver_pure_pursuit
// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(omni_carver_pure_pursuit::PurePursuitController, nav2_core::Controller)