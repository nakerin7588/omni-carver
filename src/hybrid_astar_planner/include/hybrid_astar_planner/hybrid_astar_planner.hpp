// include/nav2_hybrid_planner/hybrid_astar_planner.hpp
#ifndef NAV2_HYBRID_PLANNER__HYBRID_ASTAR_PLANNER_HPP_
#define NAV2_HYBRID_PLANNER__HYBRID_ASTAR_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_hybrid_planner
{

class HybridAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  HybridAStarPlanner();
  ~HybridAStarPlanner() override;

  /**
   * @brief Configure the planner prior to activation
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup resources
   */
  void cleanup() override;

  /**
   * @brief Prepare for activation
   */
  void activate() override;

  /**
   * @brief Teardown after deactivation
   */
  void deactivate() override;

  /**
   * @brief Compute a global plan from start to goal
   * @return A nav_msgs::msg::Path containing waypoints
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> done) override;

private:
  // Internal node representation (x, y, θ)
  struct Node {
    double x, y, theta;
    double g, f;
    std::shared_ptr<Node> parent;
  };

  // Parameters loaded in configure()
  double step_size_;
  int analytic_expansion_freq_;

  // Pointers to Nav2 resources
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Core search routines
  std::vector<std::shared_ptr<Node>> computeHybridAStar(
    const Node &start, const Node &goal);
  double gridHeuristic(double x, double y);
  bool tryAnalyticExpansion(
    const Node &current, const Node &goal,
    std::vector<std::shared_ptr<Node>> &path);
};

}  // namespace nav2_hybrid_planner

#endif  // NAV2_HYBRID_PLANNER__HYBRID_ASTAR_PLANNER_HPP_
