#ifndef RADIATION_LAYER_H_
#define RADIATION_LAYER_H_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

namespace rad_costmap_plugin
{

class RadiationLayer : public nav2_costmap_2d::Layer
{
public:
  RadiationLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset();

  virtual bool isClearable();

private:
  void heatmapCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  std::string heatmap_topic_;
  double warning_threshold_;
  double evacuation_threshold_;
  bool new_heatmap_received_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr heatmap_sub_;
  cv::Mat latest_heatmap_;
  std::mutex heatmap_mutex_;
};

}  // namespace rad_costmap_plugin

#endif  // RADIATION_LAYER_H_
