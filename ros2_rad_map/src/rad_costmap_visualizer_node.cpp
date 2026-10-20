#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cv_bridge/cv_bridge.h>
#include <mutex>

// Constants for costmap values (defined in nav2_costmap_2d/cost_values.hpp)
// Redefined here to avoid dependency on Nav2 headers in this package
namespace cost_values {
  static constexpr unsigned char FREE_SPACE = 0;
  static constexpr unsigned char LETHAL_OBSTACLE = 254;
  static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
}

namespace rad_costmap_plugin
{

class RadCostmapVisualizerNode : public rclcpp::Node
{
public:
  RadCostmapVisualizerNode() : Node("rad_costmap_visualizer_node"), map_info_received_(false)
  {
    declare_parameter("heatmap_topic", rclcpp::ParameterValue("/rad_heatmap/raw_mean_map"));
    declare_parameter("map_topic", rclcpp::ParameterValue("/map"));
    declare_parameter("radiation_costmap_topic", rclcpp::ParameterValue("/radiation_costmap"));
    declare_parameter("warning_threshold_mrem_hr", rclcpp::ParameterValue(1.0));
    declare_parameter("evacuation_threshold_mrem_hr", rclcpp::ParameterValue(2.0));
    
    heatmap_topic_ = get_parameter("heatmap_topic").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    radiation_costmap_topic_ = get_parameter("radiation_costmap_topic").as_string();
    warning_threshold_ = get_parameter("warning_threshold_mrem_hr").as_double();
    evacuation_threshold_ = get_parameter("evacuation_threshold_mrem_hr").as_double();
    
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local().reliable();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos, std::bind(&RadCostmapVisualizerNode::mapCallback, this, std::placeholders::_1));
      
    heatmap_sub_ = create_subscription<sensor_msgs::msg::Image>(
      heatmap_topic_, 10, std::bind(&RadCostmapVisualizerNode::heatmapCallback, this, std::placeholders::_1));

    uncertainty_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/rad_heatmap/gp_std_dev", 10, std::bind(&RadCostmapVisualizerNode::uncertaintyCallback, this, std::placeholders::_1));

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/radiation_cost_zones", 10);
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(radiation_costmap_topic_, map_qos);

    RCLCPP_INFO(get_logger(), "Radiation Costmap Visualizer has started.");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_info_ = msg->info;
    map_frame_id_ = msg->header.frame_id;
    
    if (!map_info_received_) {
      map_info_received_ = true;
      RCLCPP_INFO(get_logger(), "Map info received. Visualizer is ready.");
    }
  }

  void heatmapCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(map_mutex_, std::try_to_lock);
    if (!lock.owns_lock() || !map_info_received_) {
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    auto now = this->get_clock()->now();

    // Prepare OccupancyGrid message
    nav_msgs::msg::OccupancyGrid radiation_costmap_msg;
    radiation_costmap_msg.header.frame_id = map_frame_id_;
    radiation_costmap_msg.header.stamp = now;
    radiation_costmap_msg.info = map_info_;
    // Every cell in the full map extent always ends up written below (either
    // NO_INFORMATION for out-of-heatmap-bounds/uncertain cells, or a computed
    // cost) - default to NO_INFORMATION so no separate scalar fill is needed
    // for the out-of-overlap region.
    radiation_costmap_msg.data.assign(map_info_.width * map_info_.height, (int8_t)(unsigned char)255);

    auto warning_marker = create_marker("warning_zone", 0, 1.0, 0.5, 0.0); // Orange
    auto evac_marker = create_marker("evacuation_zone", 1, 1.0, 0.0, 0.0); // Red

    unsigned int map_width = map_info_.width;
    unsigned int map_height = map_info_.height;
    double map_res = map_info_.resolution;
    double map_origin_x = map_info_.origin.position.x;
    double map_origin_y = map_info_.origin.position.y;

    cv::Mat uncertainty_img;
    {
        std::lock_guard<std::mutex> lock(uncertainty_mutex_);
        uncertainty_img = latest_uncertainty_.clone();
    }

    // Vectorized replacement for a manual per-pixel .at<float>() scan over the
    // full map (validated against the original scalar loop with a standalone
    // byte-exact equivalence test across matching/mismatched heatmap/map/
    // uncertainty sizes, including degenerate and empty-image cases).
    //
    // Bulk classification (free/evacuation/uncertain) is done with cv::compare
    // masks in one pass each. The "warning gradient" cost needs the same
    // truncating static_cast<unsigned char> the original used - cv::Mat's
    // convertTo() rounds to nearest instead of truncating, so it can't be
    // substituted without changing the published cost values - so that one
    // piece is still computed with a loop, but only over the small subset of
    // pixels already isolated by cv::findNonZero as "in the warning band",
    // not the whole map.
    int overlap_rows = std::min(static_cast<int>(map_height), cv_ptr->image.rows);
    int overlap_cols = std::min(static_cast<int>(map_width), cv_ptr->image.cols);

    if (overlap_rows > 0 && overlap_cols > 0) {
        cv::Mat radiation_roi = cv_ptr->image(cv::Rect(0, 0, overlap_cols, overlap_rows));

        cv::Mat uncertain_mask = cv::Mat::zeros(overlap_rows, overlap_cols, CV_8UC1);
        if (!uncertainty_img.empty()) {
            int u_rows = std::min(overlap_rows, uncertainty_img.rows);
            int u_cols = std::min(overlap_cols, uncertainty_img.cols);
            if (u_rows > 0 && u_cols > 0) {
                cv::Mat unc_roi = uncertainty_img(cv::Rect(0, 0, u_cols, u_rows));
                cv::Mat sub_mask;
                cv::compare(unc_roi, 0.5, sub_mask, cv::CMP_GT);
                sub_mask.copyTo(uncertain_mask(cv::Rect(0, 0, u_cols, u_rows)));
            }
        }
        cv::Mat known_mask;
        cv::bitwise_not(uncertain_mask, known_mask);

        cv::Mat evac_mask_raw, warn_mask_raw;
        cv::compare(radiation_roi, evacuation_threshold_, evac_mask_raw, cv::CMP_GE);
        cv::compare(radiation_roi, warning_threshold_, warn_mask_raw, cv::CMP_GT);

        cv::Mat evac_mask, warn_only_mask, not_evac;
        cv::bitwise_and(evac_mask_raw, known_mask, evac_mask);
        cv::bitwise_not(evac_mask, not_evac);
        cv::bitwise_and(warn_mask_raw, not_evac, warn_only_mask);
        cv::bitwise_and(warn_only_mask, known_mask, warn_only_mask);

        cv::Mat cost_roi = cv::Mat::zeros(overlap_rows, overlap_cols, CV_8UC1);
        cost_roi.setTo(cv::Scalar(cost_values::LETHAL_OBSTACLE), evac_mask);
        cost_roi.setTo(cv::Scalar(255), uncertain_mask);

        std::vector<cv::Point> evac_pixels, warn_pixels;
        cv::findNonZero(evac_mask, evac_pixels);
        cv::findNonZero(warn_only_mask, warn_pixels);

        for (const auto& pt : warn_pixels) {
            float radiation = radiation_roi.at<float>(pt.y, pt.x);
            double scale = (radiation - warning_threshold_) / (evacuation_threshold_ - warning_threshold_);
            unsigned char cost_value = 1 + static_cast<unsigned char>(scale * (cost_values::INSCRIBED_INFLATED_OBSTACLE - 2));
            cost_roi.at<uchar>(pt.y, pt.x) = cost_value;
        }

        for (int r = 0; r < overlap_rows; ++r) {
            std::memcpy(&radiation_costmap_msg.data[r * map_width], cost_roi.ptr<uchar>(r), overlap_cols);
        }

        for (const auto& pt : evac_pixels) {
            geometry_msgs::msg::Point p;
            p.x = map_origin_x + (pt.x + 0.5) * map_res;
            p.y = map_origin_y + (pt.y + 0.5) * map_res;
            p.z = 0.0;
            evac_marker.points.push_back(p);
        }
        for (const auto& pt : warn_pixels) {
            geometry_msgs::msg::Point p;
            p.x = map_origin_x + (pt.x + 0.5) * map_res;
            p.y = map_origin_y + (pt.y + 0.5) * map_res;
            p.z = 0.0;
            warning_marker.points.push_back(p);
        }
    }

    if (!warning_marker.points.empty()) {
      marker_array.markers.push_back(warning_marker);
    }
    if (!evac_marker.points.empty()) {
      marker_array.markers.push_back(evac_marker);
    }
    
    // Publish even if empty to clear old markers
    if (marker_array.markers.empty()) {
        warning_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(warning_marker);
    }

    // marker_pub_->publish(marker_array);
    costmap_pub_->publish(radiation_costmap_msg);
  }

  void uncertaintyCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(uncertainty_mutex_);
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      latest_uncertainty_ = cv_ptr->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception in uncertainty: %s", e.what());
    }
  }

  visualization_msgs::msg::Marker create_marker(std::string ns, int id, float r, float g, float b)
  {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = map_frame_id_;
      marker.header.stamp = this->get_clock()->now();
      marker.ns = ns;
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = map_info_.resolution;
      marker.scale.y = map_info_.resolution;
      marker.scale.z = 0.1;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 0.5;
      return marker;
  }

  std::mutex map_mutex_;
  bool map_info_received_;
  nav_msgs::msg::MapMetaData map_info_;
  std::string map_frame_id_;

  std::string heatmap_topic_;
  std::string map_topic_;
  std::string radiation_costmap_topic_;
  double warning_threshold_;
  double evacuation_threshold_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr heatmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr uncertainty_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  cv::Mat latest_uncertainty_;
  std::mutex uncertainty_mutex_;
};

} // namespace rad_costmap_plugin

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rad_costmap_plugin::RadCostmapVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
