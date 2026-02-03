#include "rad_costmap_plugin/RadiationLayer.h"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rad_costmap_plugin::RadiationLayer, nav2_costmap_2d::Layer)

namespace rad_costmap_plugin
{

RadiationLayer::RadiationLayer()
: new_heatmap_received_(false)
{
}

void RadiationLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node in radiation layer");
  }

  declareParameter("heatmap_topic", rclcpp::ParameterValue("/rad_heatmap/raw_mean_map"));
  declareParameter("warning_threshold_mrem_hr", rclcpp::ParameterValue(2.0));
  declareParameter("evacuation_threshold_mrem_hr", rclcpp::ParameterValue(10.0));
  
  node->get_parameter(name_ + "." + "heatmap_topic", heatmap_topic_);
  node->get_parameter(name_ + "." + "warning_threshold_mrem_hr", warning_threshold_);
  node->get_parameter(name_ + "." + "evacuation_threshold_mrem_hr", evacuation_threshold_);

  RCLCPP_INFO(node->get_logger(), "Subscribing to heatmap topic: %s", heatmap_topic_.c_str());
  RCLCPP_INFO(node->get_logger(), "Warning threshold: %.2f, Evacuation threshold: %.2f", warning_threshold_, evacuation_threshold_);

  rclcpp::QoS heatmap_qos(rclcpp::KeepLast(1));
  heatmap_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    heatmap_topic_, heatmap_qos,
    std::bind(&RadiationLayer::heatmapCallback, this, std::placeholders::_1));
  
  // Set this layer as current
  current_ = true;
  // Set the layer as enabled by default
  enabled_ = true;
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
  if (!new_heatmap_received_) {
    return;
  }

  // Update the entire map area
  *min_x = layered_costmap_->getCostmap()->getOriginX();
  *min_y = layered_costmap_->getCostmap()->getOriginY();
  *max_x = *min_x + layered_costmap_->getCostmap()->getSizeInMetersX();
  *max_y = *min_y + layered_costmap_->getCostmap()->getSizeInMetersY();
}

void RadiationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !new_heatmap_received_) {
    return;
  }

  std::lock_guard<std::mutex> lock(heatmap_mutex_);
  if (latest_heatmap_.empty()) {
    return;
  }

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  if (latest_heatmap_.rows != static_cast<int>(size_y) || latest_heatmap_.cols != static_cast<int>(size_x)) {
    RCLCPP_WARN_ONCE(
      node_.lock()->get_logger(),
      "Radiation heatmap dimensions (%dx%d) do not match the costmap dimensions (%dx%d). Cannot apply radiation costs.",
      latest_heatmap_.cols, latest_heatmap_.rows, size_x, size_y);
    return;
  }

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      float radiation = latest_heatmap_.at<float>(j, i);

      if (radiation > warning_threshold_) {
        int index = master_grid.getIndex(i, j);
        unsigned char current_cost = master_array[index];

        // Do not overwrite lethal obstacles
        if (current_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
          continue;
        }

        unsigned char radiation_cost = 0;
        if (radiation >= evacuation_threshold_) {
          radiation_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        } else {
          // Linearly scale cost from 1 up to INSCRIBED_INFLATED_OBSTACLE - 1
          double scale = (radiation - warning_threshold_) / (evacuation_threshold_ - warning_threshold_);
          radiation_cost = 1 + scale * (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 2);
        }
        
        master_array[index] = std::max(current_cost, radiation_cost);
      }
    }
  }
  new_heatmap_received_ = false; // Reset flag after updating costs
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
