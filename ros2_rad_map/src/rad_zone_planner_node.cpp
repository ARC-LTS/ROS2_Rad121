#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"
#include "ros2_rad_map/msg/rad_zone.hpp"
#include "ros2_rad_map/msg/rad_zone_array.hpp"
#include <opencv2/opencv.hpp>
#include <mutex>
#include <vector>

class RadZonePlannerNode : public rclcpp::Node
{
public:
    RadZonePlannerNode() : Node("rad_zone_planner_node")
    {
        this->declare_parameter<std::string>("heatmap_topic", "/rad_heatmap/raw_mean_map");
        this->declare_parameter<std::string>("map_topic", "/map");
        this->declare_parameter<std::string>("peaks_topic", "/radiation_zones");
        this->declare_parameter<double>("radiation_threshold", 2.0);
        this->declare_parameter<double>("min_zone_area", 50.0);
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<int>("noise_removal_kernel_size", 3);
        this->declare_parameter<int>("noise_removal_iterations", 2);
        this->declare_parameter<int>("background_dilation_iterations", 3);
        this->declare_parameter<int>("peak_seed_radius_px", 5);
        this->declare_parameter<double>("safety_threshold_mrem_hr", 5.0);

        auto heatmap_topic = this->get_parameter("heatmap_topic").as_string();
        auto map_topic = this->get_parameter("map_topic").as_string();
        auto peaks_topic = this->get_parameter("peaks_topic").as_string();
        radiation_threshold_ = this->get_parameter("radiation_threshold").as_double();
        min_zone_area_ = this->get_parameter("min_zone_area").as_double();
        map_frame_ = this->get_parameter("map_frame").as_string();
        noise_removal_kernel_size_ = this->get_parameter("noise_removal_kernel_size").as_int();
        noise_removal_iterations_ = this->get_parameter("noise_removal_iterations").as_int();
        background_dilation_iterations_ = this->get_parameter("background_dilation_iterations").as_int();
        peak_seed_radius_px_ = this->get_parameter("peak_seed_radius_px").as_int();
        safety_threshold_mrem_hr_ = this->get_parameter("safety_threshold_mrem_hr").as_double();
        
        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local().reliable();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, map_qos, std::bind(&RadZonePlannerNode::mapCallback, this, std::placeholders::_1));
        heatmap_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            heatmap_topic, rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&RadZonePlannerNode::heatmapCallback, this, std::placeholders::_1));
        peak_sub_ = this->create_subscription<ros2_rad_map::msg::RadZoneArray>(
            peaks_topic, 10,
            std::bind(&RadZonePlannerNode::peakCallback, this, std::placeholders::_1));

        zone_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/radiation_zone_markers", 10);
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/radiation_aware_costmap", map_qos);

        RCLCPP_INFO(this->get_logger(), "Radiation Zone Planner has started.");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        latest_map_ = msg;
        map_info_ = msg->info;
        if (!map_info_received_) {
            map_info_received_ = true;
        }
    }

    void peakCallback(const ros2_rad_map::msg::RadZoneArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(peak_mutex_);
        latest_peaks_.clear();
        for (const auto& zone : msg->zones) {
            latest_peaks_.push_back(zone.center);
        }
    }

    void heatmapCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        nav_msgs::msg::OccupancyGrid::SharedPtr map_to_modify;
        {
            std::unique_lock<std::mutex> lock(map_mutex_, std::try_to_lock);
            if (!lock.owns_lock() || !map_info_received_ || !latest_map_) {
                return;
            }
            map_to_modify = std::make_shared<nav_msgs::msg::OccupancyGrid>(*latest_map_);
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat heatmap_raw = cv_ptr->image;
        if (heatmap_raw.rows == static_cast<int>(map_to_modify->info.height) && heatmap_raw.cols == static_cast<int>(map_to_modify->info.width)) {
            for (int r = 0; r < heatmap_raw.rows; ++r) {
                for (int c = 0; c < heatmap_raw.cols; ++c) {
                    if (heatmap_raw.at<float>(r, c) > safety_threshold_mrem_hr_) {
                        unsigned int index = r * map_to_modify->info.width + c;
                        if (index < map_to_modify->data.size()) {
                            map_to_modify->data[index] = 100; // LETHAL_OBSTACLE
                        }
                    }
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Heatmap dimensions (%d, %d) do not match map dimensions (%d, %d). Cannot generate radiation costmap.",
                heatmap_raw.cols, heatmap_raw.rows, map_to_modify->info.width, map_to_modify->info.height);
        }
        map_to_modify->header.stamp = this->get_clock()->now();
        costmap_pub_->publish(*map_to_modify);
        
        cv::Mat flipped_heatmap;
        cv::flip(cv_ptr->image, flipped_heatmap, 0);

        cv::Mat binary_map;
        cv::threshold(flipped_heatmap, binary_map, radiation_threshold_, 255, cv::THRESH_BINARY);
        binary_map.convertTo(binary_map, CV_8U);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(noise_removal_kernel_size_, noise_removal_kernel_size_));
        cv::Mat opening;
        cv::morphologyEx(binary_map, opening, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), noise_removal_iterations_);

        cv::Mat sure_background;
        cv::dilate(opening, sure_background, kernel, cv::Point(-1, -1), background_dilation_iterations_);

        cv::Mat sure_foreground = cv::Mat::zeros(opening.size(), CV_8U);
        {
            std::lock_guard<std::mutex> lock(peak_mutex_);
            if(latest_peaks_.empty()) {
                // Publish a cleared marker array so rviz2 sees a consistent
                // update on this topic whenever the costmap updates.
                visualization_msgs::msg::MarkerArray empty_array;
                visualization_msgs::msg::Marker clear_marker;
                clear_marker.header.frame_id = map_frame_;
                clear_marker.header.stamp = this->get_clock()->now();
                clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
                empty_array.markers.push_back(clear_marker);
                zone_marker_pub_->publish(empty_array);
                return;
            }

            double res = map_info_.resolution;
            double origin_x = map_info_.origin.position.x;
            double origin_y = map_info_.origin.position.y;
            int map_height = map_info_.height;

            for(const auto& peak : latest_peaks_) {
                int px = static_cast<int>((peak.x - origin_x) / res);
                int py = map_height - 1 - static_cast<int>((peak.y - origin_y) / res);

                if (px >= 0 && px < sure_foreground.cols && py >= 0 && py < sure_foreground.rows) {
                    cv::circle(sure_foreground, cv::Point(px, py), peak_seed_radius_px_, cv::Scalar(255), -1);
                }
            }
        }

        cv::Mat unknown;
        cv::subtract(sure_background, sure_foreground, unknown);

        cv::Mat markers;
        cv::connectedComponents(sure_foreground, markers);
        markers = markers + 1;
        markers.setTo(0, unknown == 255);

        cv::Mat flipped_heatmap_8uc3, scaled_heatmap;
        cv::normalize(flipped_heatmap, scaled_heatmap, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::cvtColor(scaled_heatmap, flipped_heatmap_8uc3, cv::COLOR_GRAY2BGR);
        cv::watershed(flipped_heatmap_8uc3, markers);

        std::vector<std::vector<cv::Point>> final_contours;
        double num_components;
        cv::minMaxLoc(markers, NULL, &num_components);
        for (int i = 2; i <= num_components; ++i) {
            cv::Mat segment_mask = (markers == i);
            std::vector<std::vector<cv::Point>> segment_contours;
            cv::findContours(segment_mask, segment_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            for (const auto &cont : segment_contours) {
                if (cv::contourArea(cont) > min_zone_area_) {
                    final_contours.push_back(cont);
                }
            }
        }

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = map_frame_;
        clear_marker.header.stamp = this->get_clock()->now();
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        int marker_id = 1;
        for (const auto& contour : final_contours) {
            visualization_msgs::msg::Marker zone_marker;
            zone_marker.header.stamp = this->get_clock()->now();
            zone_marker.header.frame_id = map_frame_;
            zone_marker.ns = "zone_boundaries";
            zone_marker.id = marker_id++;
            zone_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            zone_marker.action = visualization_msgs::msg::Marker::ADD;
            
            zone_marker.pose.orientation.w = 1.0;
            zone_marker.scale.x = 0.05;
            zone_marker.color.r = 1.0; zone_marker.color.g = 0.0; zone_marker.color.b = 1.0; zone_marker.color.a = 1.0;

            for (const auto& pixel_point : contour) {
                geometry_msgs::msg::Point world_point;
                world_point.x = map_info_.origin.position.x + pixel_point.x * map_info_.resolution;
                world_point.y = map_info_.origin.position.y + (map_info_.height - pixel_point.y) * map_info_.resolution;
                world_point.z = 0.1;
                zone_marker.points.push_back(world_point);
            }
            if (!contour.empty()) {
                zone_marker.points.push_back(zone_marker.points.front());
            }
            
            marker_array.markers.push_back(zone_marker);
        }

        zone_marker_pub_->publish(marker_array);
    }

    std::mutex map_mutex_;
    std::mutex peak_mutex_;
    nav_msgs::msg::MapMetaData map_info_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    bool map_info_received_{false};
    double radiation_threshold_;
    double min_zone_area_;
    std::string map_frame_;
    std::vector<geometry_msgs::msg::Point> latest_peaks_;

    int noise_removal_kernel_size_;
    int noise_removal_iterations_;
    int background_dilation_iterations_;
    int peak_seed_radius_px_;
    double safety_threshold_mrem_hr_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr heatmap_sub_;
    rclcpp::Subscription<ros2_rad_map::msg::RadZoneArray>::SharedPtr peak_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zone_marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadZonePlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}