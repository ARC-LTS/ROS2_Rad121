#include "rad_costmap_plugin/RadiationLayer.h"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rad_costmap_plugin::RadiationLayer, nav2_costmap_2d::Layer)

namespace rad_costmap_plugin
{

RadiationLayer::RadiationLayer()
: new_heatmap_received_(false), map_received_(false)
{
}

void RadiationLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node in radiation layer");
  }

  declareParameter("heatmap_topic", rclcpp::ParameterValue("/rad_heatmap/raw_mean_map"));
  declareParameter("warning_threshold_mrem_hr", rclcpp::ParameterValue(0.25));
  declareParameter("evacuation_threshold_mrem_hr", rclcpp::ParameterValue(2.0));
  
  node->get_parameter(name_ + "." + "heatmap_topic", heatmap_topic_);
  node->get_parameter(name_ + "." + "warning_threshold_mrem_hr", warning_threshold_);
  node->get_parameter(name_ + "." + "evacuation_threshold_mrem_hr", evacuation_threshold_);

  RCLCPP_INFO(node->get_logger(), "Radiation Layer: Subscribing to heatmap topic: %s", heatmap_topic_.c_str());

  rclcpp::QoS heatmap_qos(rclcpp::KeepLast(1));
  heatmap_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    heatmap_topic_, heatmap_qos,
    std::bind(&RadiationLayer::heatmapCallback, this, std::placeholders::_1));
  
  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&RadiationLayer::mapCallback, this, std::placeholders::_1));
  
  // Set this layer as current
  current_ = true;
  enabled_ = true;
}

void RadiationLayer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  map_info_ = msg->info;
  map_received_ = true;
}

void RadiationLayer::heatmapCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    std::lock_guard<std::mutex> lock(heatmap_mutex_);
    latest_heatmap_ = cv_ptr->image;
    new_heatmap_received_ = true;
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_.lock()->get_logger(), "cv_bridge exception in radiation layer: %s", e.what());
  }
}

void RadiationLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (latest_heatmap_.empty()) {
    return;
  }

  // Force update of the entire costmap area to ensure radiation costs are persistent
  *min_x = std::min(*min_x, layered_costmap_->getCostmap()->getOriginX());
  *min_y = std::min(*min_y, layered_costmap_->getCostmap()->getOriginY());
  *max_x = std::max(*max_x, layered_costmap_->getCostmap()->getOriginX() + layered_costmap_->getCostmap()->getSizeInMetersX());
  *max_y = std::max(*max_y, layered_costmap_->getCostmap()->getOriginY() + layered_costmap_->getCostmap()->getSizeInMetersY());
}

void RadiationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock_heatmap(heatmap_mutex_);
  std::lock_guard<std::mutex> lock_map(map_mutex_);

  if (latest_heatmap_.empty() || !map_received_) {
    return;
  }

  unsigned char* master_array = master_grid.getCharMap();
  double res = master_grid.getResolution();
  double origin_x = master_grid.getOriginX();
  double origin_y = master_grid.getOriginY();

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      // 1. Get world coordinates of the current costmap cell
      double wx = origin_x + (i + 0.5) * res;
      double wy = origin_y + (j + 0.5) * res;

      // 2. Transform world coordinate to heatmap pixel coordinate (floating point)
      double h_x = (wx - map_info_.origin.position.x) / map_info_.resolution;
      double h_y = (wy - map_info_.origin.position.y) / map_info_.resolution;

      if (h_x < 0 || h_x >= latest_heatmap_.cols - 1 || h_y < 0 || h_y >= latest_heatmap_.rows - 1) {
        continue;
      }

      // Bilinear Interpolation for smoother, non-voxelized costs
      int x1 = static_cast<int>(std::floor(h_x));
      int y1 = static_cast<int>(std::floor(h_y));
      int x2 = x1 + 1;
      int y2 = y1 + 1;

      float dx = static_cast<float>(h_x - x1);
      float dy = static_cast<float>(h_y - y1);

      float f11 = latest_heatmap_.at<float>(y1, x1);
      float f12 = latest_heatmap_.at<float>(y2, x1);
      float f21 = latest_heatmap_.at<float>(y1, x2);
      float f22 = latest_heatmap_.at<float>(y2, x2);

      float radiation = (1 - dx) * (1 - dy) * f11 +
                        (1 - dx) * dy * f12 +
                        dx * (1 - dy) * f21 +
                        dx * dy * f22;

      if (radiation > warning_threshold_) {
        int index = master_grid.getIndex(i, j);
        unsigned char current_cost = master_array[index];

        if (current_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
          continue;
        }

        unsigned char radiation_cost = 0;
        if (radiation >= evacuation_threshold_) {
          // Use a high cost but not LETHAL to allow robot to move/recover
          radiation_cost = 250; 
        } else {
          double scale = (radiation - warning_threshold_) / (evacuation_threshold_ - warning_threshold_);
          // Scale from 1 up to 249
          radiation_cost = 1 + static_cast<unsigned char>(scale * 248.0);
        }
        
        master_array[index] = std::max(current_cost, radiation_cost);
      }
    }
  }
}

void RadiationLayer::reset()
{
  // This method is called when the costmap is reset.
  // For this layer, we can clear the latest heatmap to force it to be refreshed.
  std::lock_guard<std::mutex> lock(heatmap_mutex_);
  latest_heatmap_ = cv::Mat();
  new_heatmap_received_ = false;
}

bool RadiationLayer::isClearable()
{
  return false;
}

}  // namespace rad_costmap_plugin
