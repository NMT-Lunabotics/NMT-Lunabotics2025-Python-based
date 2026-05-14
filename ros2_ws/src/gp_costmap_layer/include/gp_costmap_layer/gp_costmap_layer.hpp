#pragma once

#include <mutex>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gp_costmap_layer
{

class GPCostmapLayer : public nav2_costmap_2d::Layer
{
public:
  GPCostmapLayer() = default;
  ~GPCostmapLayer() override = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j) override;

  void reset() override;

  bool isClearable() override
  {
    return false;
  }

private:
  void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  std::string topic_{"/gp_costmap"};

  bool enabled_{true};
  bool use_unknown_{true};
  bool trinary_{false};

  // true: GP layer overwrites costs in its local grid window.
  // false: GP layer only raises costs compared to existing master costmap.
  bool overwrite_{true};

  // GP mapping module usually publishes:
  // -1 = unknown
  // 0 = free
  // 1-95 = increasing terrain/traversability cost
  int lethal_threshold_{90};

  // Converts GP 0-95 values into Nav2 0-252 cost range.
  double cost_scale_{2.0};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;

  std::mutex grid_mutex_;

  double res_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};

  unsigned int width_{0};
  unsigned int height_{0};
};

}  // namespace gp_costmap_layer
