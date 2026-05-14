#include "gp_costmap_layer/gp_costmap_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

namespace gp_costmap_layer
{

void GPCostmapLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/gp_costmap")));
  declareParameter("use_unknown", rclcpp::ParameterValue(true));

  // GP costmap usually publishes 0-95, with -1 as unknown.
  // A lethal threshold of 100 may never trigger.
  declareParameter("lethal_threshold", rclcpp::ParameterValue(90));

  // Scales GP occupancy values into Nav2 cost values.
  declareParameter("cost_scale", rclcpp::ParameterValue(2.0));

  // false = preserve gradient cost, true = convert all nonzero costs to inflated obstacle
  declareParameter("trinary", rclcpp::ParameterValue(false));

  // For first testing, overwrite lets GP directly define its local window.
  // Later, set this false if you only want GP to increase cost.
  declareParameter("overwrite", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".use_unknown", use_unknown_);
  node->get_parameter(name_ + ".lethal_threshold", lethal_threshold_);
  node->get_parameter(name_ + ".cost_scale", cost_scale_);
  node->get_parameter(name_ + ".trinary", trinary_);
  node->get_parameter(name_ + ".overwrite", overwrite_);

  // Match this to the GP mapping module publisher.
  // If /gp_costmap publisher is BEST_EFFORT, change this to .best_effort().
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_,
    qos,
    std::bind(&GPCostmapLayer::gridCallback, this, std::placeholders::_1));

  current_ = false;
  matchSize();

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] Subscribing to %s",
    name_.c_str(),
    topic_.c_str());
}

void GPCostmapLayer::gridCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  auto node = node_.lock();

  if (node && layered_costmap_) {
    const std::string costmap_frame = layered_costmap_->getGlobalFrameID();

    if (!costmap_frame.empty() && msg->header.frame_id != costmap_frame) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        2000,
        "[%s] GP costmap frame '%s' does not match Nav2 costmap frame '%s'",
        name_.c_str(),
        msg->header.frame_id.c_str(),
        costmap_frame.c_str());
    }
  }

  std::lock_guard<std::mutex> lk(grid_mutex_);

  grid_ = msg;
  res_ = msg->info.resolution;
  origin_x_ = msg->info.origin.position.x;
  origin_y_ = msg->info.origin.position.y;
  width_ = msg->info.width;
  height_ = msg->info.height;

  current_ = true;
}

void GPCostmapLayer::updateBounds(
  double /*robot_x*/,
  double /*robot_y*/,
  double /*robot_yaw*/,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lk(grid_mutex_);

  if (!grid_) {
    return;
  }

  const double gx0 = origin_x_;
  const double gy0 = origin_y_;
  const double gx1 = origin_x_ + static_cast<double>(width_) * res_;
  const double gy1 = origin_y_ + static_cast<double>(height_) * res_;

  *min_x = std::min(*min_x, gx0);
  *min_y = std::min(*min_y, gy0);
  *max_x = std::max(*max_x, gx1);
  *max_y = std::max(*max_y, gy1);
}

void GPCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr grid;
  double res;
  double ox;
  double oy;
  unsigned int w;
  unsigned int h;

  {
    std::lock_guard<std::mutex> lk(grid_mutex_);

    if (!grid_) {
      return;
    }

    grid = grid_;
    res = res_;
    ox = origin_x_;
    oy = origin_y_;
    w = width_;
    h = height_;
  }

  if (res <= 0.0 || w == 0 || h == 0 || grid->data.empty()) {
    return;
  }

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx;
      double wy;

      master_grid.mapToWorld(i, j, wx, wy);

      const int gx = static_cast<int>(std::floor((wx - ox) / res));
      const int gy = static_cast<int>(std::floor((wy - oy) / res));

      if (
        gx < 0 || gy < 0 ||
        gx >= static_cast<int>(w) ||
        gy >= static_cast<int>(h))
      {
        continue;
      }

      const int idx = gy * static_cast<int>(w) + gx;

      if (idx < 0 || idx >= static_cast<int>(grid->data.size())) {
        continue;
      }

      const int8_t occ = grid->data[idx];
      const unsigned char old_cost = master_grid.getCost(i, j);

      // GP mapping module uses -1 for unknown/high-uncertainty/no-support cells.
      if (occ < 0) {
        if (use_unknown_) {
          master_grid.setCost(i, j, nav2_costmap_2d::NO_INFORMATION);
        }
        continue;
      }

      unsigned char new_cost = nav2_costmap_2d::FREE_SPACE;

      if (occ >= lethal_threshold_) {
        new_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
      } else if (occ <= 0) {
        new_cost = nav2_costmap_2d::FREE_SPACE;
      } else if (trinary_) {
        new_cost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      } else {
        const int scaled = static_cast<int>(
          std::round(static_cast<double>(occ) * cost_scale_));

        // 252 is the highest non-lethal Nav2 cost.
        new_cost = static_cast<unsigned char>(
          std::clamp(scaled, 1, 252));
      }

      if (overwrite_) {
        // Best for testing GP as the main local terrain costmap.
        master_grid.setCost(i, j, new_cost);
      } else {
        // Safer for combining with obstacle_layer/voxel_layer.
        if (
          old_cost == nav2_costmap_2d::NO_INFORMATION ||
          new_cost > old_cost)
        {
          master_grid.setCost(i, j, new_cost);
        }
      }
    }
  }
}

void GPCostmapLayer::reset()
{
  std::lock_guard<std::mutex> lk(grid_mutex_);

  grid_.reset();
  current_ = false;
}

}  // namespace gp_costmap_layer

PLUGINLIB_EXPORT_CLASS(gp_costmap_layer::GPCostmapLayer, nav2_costmap_2d::Layer)
