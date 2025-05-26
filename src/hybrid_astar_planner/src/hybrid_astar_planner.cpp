// src/hybrid_astar_planner.cpp
#include "hybrid_astar_planner/hybrid_astar_planner.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <functional>
#include <cmath>
#include <algorithm>

namespace nav2_hybrid_planner
{

HybridAStarPlanner::HybridAStarPlanner() = default;
HybridAStarPlanner::~HybridAStarPlanner() = default;

void HybridAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("HybridAStarPlanner: failed to lock lifecycle node");
  }

  tf_ = tf;
  costmap_ros_ = costmap_ros;

  node->declare_parameter(name + ".step_size", 0.5);
  node->declare_parameter(name + ".analytic_expansion_freq", 20);
  node->get_parameter(name + ".step_size", step_size_);
  node->get_parameter(name + ".analytic_expansion_freq", analytic_expansion_freq_);
}

void HybridAStarPlanner::cleanup()   {}
void HybridAStarPlanner::activate()  {}
void HybridAStarPlanner::deactivate(){}

nav_msgs::msg::Path HybridAStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> done)
{
  Node s{
    start.pose.position.x,
    start.pose.position.y,
    tf2::getYaw(start.pose.orientation),
    0.0, 0.0, nullptr
  };
  Node g{
    goal.pose.position.x,
    goal.pose.position.y,
    tf2::getYaw(goal.pose.orientation),
    0.0, 0.0, nullptr
  };

  auto node_path = computeHybridAStar(s, g);

  nav_msgs::msg::Path path_msg;
  path_msg.header = start.header;
  for (auto &n : node_path) {
    geometry_msgs::msg::PoseStamped p;
    p.header = start.header;
    p.pose.position.x = n->x;
    p.pose.position.y = n->y;
    p.pose.orientation = tf2::toMsg(
      tf2::Quaternion{0, 0, std::sin(n->theta / 2.0), std::cos(n->theta / 2.0)}
    );
    path_msg.poses.push_back(p);
  }
  return path_msg;
}

std::vector<std::shared_ptr<HybridAStarPlanner::Node>>
HybridAStarPlanner::computeHybridAStar(
  const Node & start, const Node & goal)
{
  auto cmp = [](auto & a, auto & b) { return a->f > b->f; };
  std::priority_queue<
    std::shared_ptr<Node>,
    std::vector<std::shared_ptr<Node>>,
    decltype(cmp)
  > open_list(cmp);

  open_list.push(std::make_shared<Node>(start));
  std::vector<std::shared_ptr<Node>> closed;
  int expansions = 0;

  while (!open_list.empty()) {
    auto current = open_list.top();
    open_list.pop();

    if (std::hypot(current->x - goal.x, current->y - goal.y) < step_size_) {
      std::vector<std::shared_ptr<Node>> final_path;
      if (tryAnalyticExpansion(*current, goal, final_path)) {
        return final_path;
      }
    }

    closed.push_back(current);

    if (++expansions % analytic_expansion_freq_ == 0) {
      std::vector<std::shared_ptr<Node>> final_path;
      if (tryAnalyticExpansion(*current, goal, final_path)) {
        return final_path;
      }
    }

    for (double steer : {-0.3, 0.0, 0.3}) {
      double new_theta = current->theta + step_size_ * std::tan(steer);
      double new_x = current->x + step_size_ * std::cos(new_theta);
      double new_y = current->y + step_size_ * std::sin(new_theta);

      unsigned int mx, my;
      if (!costmap_ros_->getCostmap()->worldToMap(new_x, new_y, mx, my)) {
        continue;
      }
      if (costmap_ros_->getCostmap()->getCost(mx, my) >=
          nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        continue;
      }

      auto child = std::make_shared<Node>(Node{
        new_x, new_y, new_theta, 0.0, 0.0, current
      });
      child->g = current->g + step_size_;
      child->f = child->g +
                 gridHeuristic(new_x, new_y) +
                 std::fabs(new_theta - goal.theta);

      open_list.push(child);
    }
  }

  return {};
}

double HybridAStarPlanner::gridHeuristic(double x, double y)
{
  double ox = costmap_ros_->getCostmap()->getOriginX();
  double oy = costmap_ros_->getCostmap()->getOriginY();
  return std::hypot(x - ox, y - oy);
}

bool HybridAStarPlanner::tryAnalyticExpansion(
  const Node & current,
  const Node & goal,
  std::vector<std::shared_ptr<Node>> & path)
{
  auto node = std::make_shared<Node>(goal);
  node->parent = std::make_shared<Node>(current);

  for (auto it = node; it; it = it->parent) {
    path.push_back(it);
  }
  std::reverse(path.begin(), path.end());
  return true;
}

}  // namespace nav2_hybrid_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_hybrid_planner::HybridAStarPlanner,
  nav2_core::GlobalPlanner
)
