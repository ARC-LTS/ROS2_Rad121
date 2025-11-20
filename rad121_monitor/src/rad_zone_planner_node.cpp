#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rad121_monitor/msg/rad_zone.hpp"
#include "rad121_monitor/msg/rad_zone_array.hpp"
#include <opencv2/opencv.hpp>
#include <mutex>

class RadZonePlannerNode : public rclcpp::Node
{
public:
    RadZonePlannerNode() : Node("rad_zone_planner_node")
    {
        this->declare_parameter<std::string>("heatmap_topic", "/rad_heatmap/raw_mean_map");
        this->declare_parameter<std::string>("map_topic", "/map");
        this->declare_parameter<double>("radiation_threshold", 2.0);
        this->declare_parameter<double>("min_zone_area", 50.0);
        this->declare_parameter<std::string>("map_frame", "map");

        auto heatmap_topic = this->get_parameter("heatmap_topic").as_string();
        auto map_topic = this->get_parameter("map_topic").as_string();
        radiation_threshold_ = this->get_parameter("radiation_threshold").as_double();
        min_zone_area_ = this->get_parameter("min_zone_area").as_double();
        map_frame_ = this->get_parameter("map_frame").as_string();
        
        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local().reliable();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, map_qos, std::bind(&RadZonePlannerNode::mapCallback, this, std::placeholders::_1));
        heatmap_sub_ = this->create_subscription<sensor_msgs::msg::Image>(heatmap_topic, 10, std::bind(&RadZonePlannerNode::heatmapCallback, this, std::placeholders::_1));

        zone_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/radiation_zone_markers", 10);
        zone_waypoints_pub_ = this->create_publisher<rad121_monitor::msg::RadZoneArray>("/radiation_zones", 10);

        RCLCPP_INFO(this->get_logger(), "Radiation Zone Planner has started.");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!map_info_received_) {
            map_info_ = msg->info;
            map_info_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Map info received.");
        }
    }

    void heatmapCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::unique_lock<std::mutex> lock(map_mutex_, std::try_to_lock);
        if (!lock.owns_lock() || !map_info_received_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Map info not yet received, skipping heatmap processing.");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat flipped_heatmap;
        cv::flip(cv_ptr->image, flipped_heatmap, 0);

        cv::Mat binary_map;
        cv::threshold(flipped_heatmap, binary_map, radiation_threshold_, 255, cv::THRESH_BINARY);
        binary_map.convertTo(binary_map, CV_8U);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_map, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        auto zone_array_msg = std::make_unique<rad121_monitor::msg::RadZoneArray>();
        zone_array_msg->header.stamp = this->get_clock()->now();
        zone_array_msg->header.frame_id = map_frame_;

        visualization_msgs::msg::MarkerArray marker_array;
        
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = map_frame_;
        clear_marker.header.stamp = zone_array_msg->header.stamp;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        int marker_id = 1;
        char zone_label_char = 'A';
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < min_zone_area_) {
                continue;
            }

            rad121_monitor::msg::RadZone zone_msg;
            zone_msg.label = std::string(1, zone_label_char);
            cv::Moments moments = cv::moments(contour);
            if (moments.m00 > 0) {
                cv::Point2f pc(moments.m10 / moments.m00, moments.m01 / moments.m00);
                zone_msg.center.x = map_info_.origin.position.x + pc.x * map_info_.resolution;
                zone_msg.center.y = map_info_.origin.position.y + (map_info_.height - pc.y) * map_info_.resolution;
                zone_msg.center.z = 0.1;
            }
            zone_array_msg->zones.push_back(zone_msg);

            visualization_msgs::msg::Marker zone_marker;
            zone_marker.header = zone_array_msg->header;
            zone_marker.ns = "zone_boundaries";
            zone_marker.id = marker_id++;
            zone_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            zone_marker.action = visualization_msgs::msg::Marker::ADD;
            zone_marker.pose.orientation.w = 1.0;
            zone_marker.scale.x = 0.05;
            zone_marker.color.r = 1.0; zone_marker.color.g = 0.0; zone_marker.color.b = 0.0; zone_marker.color.a = 1.0;

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

            visualization_msgs::msg::Marker label_marker;
            label_marker.header = zone_array_msg->header;
            label_marker.ns = "zone_labels";
            label_marker.id = marker_id++;
            label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label_marker.action = visualization_msgs::msg::Marker::ADD;
            label_marker.pose.position = zone_msg.center;
            label_marker.pose.position.z += 0.5;
            label_marker.scale.z = 0.5;
            label_marker.color.r = 1.0; label_marker.color.g = 1.0; label_marker.color.b = 1.0; label_marker.color.a = 1.0;
            label_marker.text = std::string(1, zone_label_char);
            marker_array.markers.push_back(label_marker);

            zone_label_char++;
        }

        zone_marker_pub_->publish(marker_array);
        zone_waypoints_pub_->publish(std::move(zone_array_msg));
    }

    std::mutex map_mutex_;
    nav_msgs::msg::MapMetaData map_info_;
    bool map_info_received_{false};
    double radiation_threshold_;
    double min_zone_area_;
    std::string map_frame_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr heatmap_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zone_marker_pub_;
    rclcpp::Publisher<rad121_monitor::msg::RadZoneArray>::SharedPtr zone_waypoints_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadZonePlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
