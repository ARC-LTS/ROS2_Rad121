#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "ros2_rad_map/srv/find_peaks.hpp"
#include "ros2_rad_map/msg/rad_zone.hpp"
#include "ros2_rad_map/msg/rad_zone_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <vector>
#include <mutex>
#include <iomanip>
#include <cmath> // For std::sqrt
#include <algorithm> // For std::remove_if

// Helper function to map a radiation value to a color
std_msgs::msg::ColorRGBA valueToColor(double value, double min_val, double max_val) {
    std_msgs::msg::ColorRGBA color;
    double normalized_value = (value - min_val) / (max_val - min_val);
    normalized_value = std::max(0.0, std::min(1.0, normalized_value)); // Clamp between 0 and 1

    // Simple Jet-like colormap (blue-green-yellow-red)
    if (normalized_value < 0.25) { // Blue to Cyan
        color.r = 0.0;
        color.g = normalized_value * 4.0;
        color.b = 1.0;
    } else if (normalized_value < 0.5) { // Cyan to Green
        color.r = 0.0;
        color.g = 1.0;
        color.b = 1.0 - (normalized_value - 0.25) * 4.0;
    } else if (normalized_value < 0.75) { // Green to Yellow
        color.r = (normalized_value - 0.5) * 4.0;
        color.g = 1.0;
        color.b = 0.0;
    } else { // Yellow to Red
        color.r = 1.0;
        color.g = 1.0 - (normalized_value - 0.75) * 4.0;
        color.b = 0.0;
    }
    color.a = 1.0; // Fully opaque
    return color;
}


struct RadSample {
    double x, y, value;
};

class PeakFinderClientNode : public rclcpp::Node
{
public:
    PeakFinderClientNode() : Node("peak_finder_client_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>("sensor_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<int>("num_starts", 20);
        this->declare_parameter<double>("marker_scale", 0.5);
        this->declare_parameter<std::vector<double>>("marker_color_r", {1.0, 0.0, 0.0, 1.0});
        this->declare_parameter<double>("matching_threshold_m", 1.5);
        this->declare_parameter<double>("min_peak_mrem_hr", 0.4); // New parameter

        sensor_frame_ = this->get_parameter("sensor_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        num_starts_ = this->get_parameter("num_starts").as_int();
        marker_scale_ = this->get_parameter("marker_scale").as_double();
        marker_color_r_ = this->get_parameter("marker_color_r").as_double_array();
        matching_threshold_m_ = this->get_parameter("matching_threshold_m").as_double();
        min_peak_mrem_hr_ = this->get_parameter("min_peak_mrem_hr").as_double(); // Get new parameter

        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rad", 10, std::bind(&PeakFinderClientNode::radCallback, this, std::placeholders::_1));

        peak_finder_client_ = this->create_client<ros2_rad_map::srv::FindPeaks>("/find_rad_peaks");

        zone_waypoints_pub_ = this->create_publisher<ros2_rad_map::msg::RadZoneArray>("/radiation_zones", 10);
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/radiation_peak_markers", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(15),
            std::bind(&PeakFinderClientNode::callPeakFinderService, this));
            
        RCLCPP_INFO(this->get_logger(), "Peak Finder Client initialized.");
        if (!peak_finder_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Service /find_rad_peaks not available. Exiting.");
            rclcpp::shutdown();
        }
    }

private:
    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) return;

        geometry_msgs::msg::PoseStamped pose_in_sensor, pose_in_map;
        try {
            pose_in_sensor.header.frame_id = sensor_frame_;
            pose_in_sensor.header.stamp = rclcpp::Time(0);
            tf_buffer_.transform(pose_in_sensor, pose_in_map, map_frame_, std::chrono::milliseconds(500));

            std::lock_guard<std::mutex> lock(data_mutex_);
            historical_samples_.push_back({pose_in_map.pose.position.x, pose_in_map.pose.position.y, msg->data[1]});
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error for logging sample: %s", ex.what());
        }
    }

    void callPeakFinderService()
    {
        if (is_service_running_) {
            RCLCPP_INFO(this->get_logger(), "Peak finding service is already running.");
            return;
        }

        std::vector<RadSample> samples_copy;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (historical_samples_.empty()) {
                RCLCPP_INFO(this->get_logger(), "No samples collected yet, skipping peak finding.");
                return;
            }
            samples_copy = historical_samples_;
        }

        is_service_running_ = true;
        auto request = std::make_shared<ros2_rad_map::srv::FindPeaks::Request>();
        request->num_starts = num_starts_;

        for (const auto &s : samples_copy) {
            request->observed_x.push_back(s.x);
            request->observed_y.push_back(s.y);
            request->observed_values_mean.push_back(s.value);
        }

        RCLCPP_INFO(this->get_logger(), "Calling /find_rad_peaks service with %zu samples.", samples_copy.size());

        peak_finder_client_->async_send_request(request, [this](rclcpp::Client<ros2_rad_map::srv::FindPeaks>::SharedFuture future) {
            try {
                auto response = future.get();
                
                std::lock_guard<std::mutex> lock(data_mutex_);

                // --- DATA ASSOCIATION LOGIC ---
                std::vector<bool> matched_new_peaks(response->peaks.size(), false);
                std::vector<bool> matched_persistent_zones(persistent_zones_.size(), false);

                // For each new peak, find the closest persistent zone
                for (size_t i = 0; i < response->peaks.size(); ++i) {
                    const auto& new_peak = response->peaks[i];
                    double min_dist_sq = std::pow(matching_threshold_m_, 2);
                    int best_match_idx = -1;

                    for (size_t j = 0; j < persistent_zones_.size(); ++j) {
                        if (matched_persistent_zones[j]) continue;

                        double dx = new_peak.x - persistent_zones_[j].center.x;
                        double dy = new_peak.y - persistent_zones_[j].center.y;
                        double dist_sq = dx*dx + dy*dy;

                        if (dist_sq < min_dist_sq) {
                            min_dist_sq = dist_sq;
                            best_match_idx = j;
                        }
                    }

                    if (best_match_idx != -1) {
                        // Found a match. Update the persistent zone.
                        persistent_zones_[best_match_idx].center.x = new_peak.x;
                        persistent_zones_[best_match_idx].center.y = new_peak.y;
                        persistent_zones_[best_match_idx].center.z = new_peak.z;
                        
                        matched_new_peaks[i] = true;
                        matched_persistent_zones[best_match_idx] = true;
                    }
                }

                // Add any unmatched new peaks as new persistent zones
                for (size_t i = 0; i < response->peaks.size(); ++i) {
                    if (!matched_new_peaks[i]) {
                        ros2_rad_map::msg::RadZone new_zone;
                        new_zone.label = std::string(1, next_zone_label_++);
                        new_zone.center = response->peaks[i];
                        persistent_zones_.push_back(new_zone);
                    }
                }

                // --- FILTERING ---
                std::vector<ros2_rad_map::msg::RadZone> zones_to_publish;
                for (const auto& zone : persistent_zones_) {
                    if (zone.center.z >= min_peak_mrem_hr_) {
                        zones_to_publish.push_back(zone);
                    }
                }

                // --- PUBLISH PERSISTENT DATA ---
                auto zone_array_msg = std::make_unique<ros2_rad_map::msg::RadZoneArray>();
                zone_array_msg->header.stamp = this->get_clock()->now();
                zone_array_msg->header.frame_id = map_frame_;
                zone_array_msg->zones = zones_to_publish; // Publish filtered zones

                visualization_msgs::msg::MarkerArray marker_array;
                visualization_msgs::msg::Marker clear_marker;
                clear_marker.header.stamp = zone_array_msg->header.stamp;
                clear_marker.header.frame_id = map_frame_;
                clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
                marker_array.markers.push_back(clear_marker);

                int marker_id = 0;
                double max_rad_val = min_peak_mrem_hr_;
                for (const auto& zone : zones_to_publish) {
                    if (zone.center.z > max_rad_val) max_rad_val = zone.center.z;
                }

                for (const auto& zone : zones_to_publish) {
                    // Sphere marker
                    visualization_msgs::msg::Marker point_marker;
                    point_marker.header.stamp = zone_array_msg->header.stamp;
                    point_marker.header.frame_id = map_frame_;
                    point_marker.ns = "radiation_peaks";
                    point_marker.id = marker_id++;
                    point_marker.type = visualization_msgs::msg::Marker::SPHERE;
                    point_marker.action = visualization_msgs::msg::Marker::ADD;
                    point_marker.pose.position = zone.center;
                    point_marker.pose.orientation.w = 1.0;
                    point_marker.scale.x = marker_scale_;
                    point_marker.scale.y = marker_scale_;
                    point_marker.scale.z = marker_scale_;
                    point_marker.color = valueToColor(zone.center.z, min_peak_mrem_hr_, max_rad_val); // Color by intensity
                    marker_array.markers.push_back(point_marker);

                    // Text marker
                    visualization_msgs::msg::Marker text_marker;
                    text_marker.header.stamp = zone_array_msg->header.stamp;
                    text_marker.header.frame_id = map_frame_;
                    text_marker.ns = "radiation_peak_labels";
                    text_marker.id = marker_id++;
                    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                    text_marker.action = visualization_msgs::msg::Marker::ADD;
                    text_marker.pose.position = zone.center;
                    text_marker.pose.position.z += marker_scale_ * 1.5;
                    text_marker.pose.orientation.w = 1.0;
                    text_marker.scale.z = marker_scale_ * 0.75;
                    text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; text_marker.color.a = 1.0; // White text
                    std::stringstream ss;
                    ss << zone.label << ": " << std::fixed << std::setprecision(2) << zone.center.z;
                    text_marker.text = ss.str();
                    marker_array.markers.push_back(text_marker);
                }

                // --- 3D GRADIENT (LINE_LIST) VISUALIZATION ---
                if (zones_to_publish.size() > 1) {
                    visualization_msgs::msg::Marker line_list_marker;
                    line_list_marker.header.stamp = zone_array_msg->header.stamp;
                    line_list_marker.header.frame_id = map_frame_;
                    line_list_marker.ns = "radiation_gradient";
                    line_list_marker.id = marker_id++;
                    line_list_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
                    line_list_marker.action = visualization_msgs::msg::Marker::ADD;
                    line_list_marker.pose.orientation.w = 1.0;
                    line_list_marker.scale.x = 0.05; // Line width

                    for (size_t i = 0; i < zones_to_publish.size(); ++i) {
                        const auto& zone1 = zones_to_publish[i];
                        double min_dist_sq = std::numeric_limits<double>::max();
                        int nearest_neighbor_idx = -1;

                        for (size_t j = 0; j < zones_to_publish.size(); ++j) {
                            if (i == j) continue;

                            const auto& zone2 = zones_to_publish[j];
                            double dx = zone1.center.x - zone2.center.x;
                            double dy = zone1.center.y - zone2.center.y;
                            double dist_sq = dx*dx + dy*dy;

                            if (dist_sq < min_dist_sq) {
                                min_dist_sq = dist_sq;
                                nearest_neighbor_idx = j;
                            }
                        }

                        if (nearest_neighbor_idx != -1) {
                            const auto& zone2 = zones_to_publish[nearest_neighbor_idx];
                            line_list_marker.points.push_back(zone1.center);
                            line_list_marker.colors.push_back(valueToColor(zone1.center.z, min_peak_mrem_hr_, max_rad_val));
                            line_list_marker.points.push_back(zone2.center);
                            line_list_marker.colors.push_back(valueToColor(zone2.center.z, min_peak_mrem_hr_, max_rad_val));
                        }
                    }
                    marker_array.markers.push_back(line_list_marker);
                }
                
                RCLCPP_INFO(this->get_logger(), "Publishing %zu persistent zones (filtered).", zones_to_publish.size());
                zone_waypoints_pub_->publish(std::move(zone_array_msg));
                marker_array_pub_->publish(marker_array);

            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
            is_service_running_ = false;
        });
    }

    std::mutex data_mutex_;
    std::vector<RadSample> historical_samples_;
    std::vector<ros2_rad_map::msg::RadZone> persistent_zones_;
    char next_zone_label_ = 'A';
    bool is_service_running_ = false;

    // ROS Comms
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Client<ros2_rad_map::srv::FindPeaks>::SharedPtr peak_finder_client_;
    rclcpp::Publisher<ros2_rad_map::msg::RadZoneArray>::SharedPtr zone_waypoints_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Params
    std::string sensor_frame_, map_frame_;
    int num_starts_;
    double marker_scale_;
    double matching_threshold_m_;
    double min_peak_mrem_hr_; // New parameter
    std::vector<double> marker_color_r_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PeakFinderClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
