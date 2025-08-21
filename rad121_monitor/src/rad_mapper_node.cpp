#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/occupancy_grid.hpp" // Required for the map
#include "std_srvs/srv/trigger.hpp"         // Required for the save service
#include "cv_bridge/cv_bridge.h"            // Required to convert ROS images
#include <opencv2/opencv.hpp>               // Required for image processing
#include <vector>
#include <mutex>
#include <chrono>
#include <filesystem>
#include <ctime>
#include <iomanip>
#include <sstream>

class RadPointSaverNode : public rclcpp::Node
{
public:
    RadPointSaverNode()
        : Node("rad_point_saver_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("robot_frame", "base_link");
        this->declare_parameter<std::string>("output_directory", "");
        this->declare_parameter<double>("tf_timeout_seconds", 0.2);
        this->declare_parameter<double>("marker_scale", 0.05);
        this->declare_parameter<std::vector<double>>("marker_color", {1.0, 0.2, 0.0, 1.0}); // RGBA Orange

        map_frame_ = this->get_parameter("map_frame").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        output_directory_ = this->get_parameter("output_directory").as_string();
        tf_timeout_seconds_ = this->get_parameter("tf_timeout_seconds").as_double();
        marker_scale_ = this->get_parameter("marker_scale").as_double();
        marker_color_ = this->get_parameter("marker_color").as_double_array();

        if(marker_color_.size() != 4) {
            marker_color_ = {1.0, 0.2, 0.0, 1.0};
        }
        
        createSessionFolder();

        // Set up ROS Communications
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rad", 10, std::bind(&RadPointSaverNode::radCallback, this, std::placeholders::_1));

        // Subscribe to the map topic with transient_local QoS to get the latest map
        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local().reliable();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos, std::bind(&RadPointSaverNode::mapCallback, this, std::placeholders::_1));

        rclcpp::QoS marker_qos(rclcpp::KeepLast(1));
        marker_qos.transient_local();
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/rad_points", marker_qos);

        publish_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RadPointSaverNode::publishMarkers, this));
        
        // Service to trigger saving the image
        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_map_with_points",
            std::bind(&RadPointSaverNode::saveServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Radiation Point Saver has started.");
        RCLCPP_INFO(this->get_logger(), "Call the /save_map_with_points service to save the image.");
    }

private:
    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.empty()) return;

        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform(
                map_frame_, robot_frame_, tf2::TimePoint(), tf2::durationFromSec(tf_timeout_seconds_));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "TF lookup failed: %s", ex.what());
            return;
        }

        geometry_msgs::msg::Point new_point;
        new_point.x = transform_stamped.transform.translation.x;
        new_point.y = transform_stamped.transform.translation.y;
        new_point.z = 0.0;

        std::lock_guard<std::mutex> lock(points_mutex_);
        collected_points_.push_back(new_point);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (map_received_) return; // Only process the first map received
        
        RCLCPP_INFO(this->get_logger(), "Map received! Ready to save images.");
        map_data_ = *msg;
        map_received_ = true;
    }

    void publishMarkers()
    {
        std::lock_guard<std::mutex> lock(points_mutex_);
        if (collected_points_.empty()) return;

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = map_frame_;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "rad_points";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.scale.x = marker_scale_;
        marker.scale.y = marker_scale_;
        marker.color.r = marker_color_[0];
        marker.color.g = marker_color_[1];
        marker.color.b = marker_color_[2];
        marker.color.a = marker_color_[3];
        marker.lifetime = rclcpp::Duration(0, 0);
        marker.points = collected_points_;

        marker_pub_->publish(marker);
    }

    void saveServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Save service called.");
        
        // Lock both resources to ensure data consistency
        std::scoped_lock lock(map_mutex_, points_mutex_);

        if (!map_received_) {
            response->success = false;
            response->message = "Map has not been received yet. Cannot save image.";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        // 1. Convert ROS OccupancyGrid to a grayscale OpenCV image
        cv::Mat map_image_gray(map_data_.info.height, map_data_.info.width, CV_8UC1);
        for (unsigned int y = 0; y < map_data_.info.height; y++) {
            for (unsigned int x = 0; x < map_data_.info.width; x++) {
                int8_t occupancy = map_data_.data[y * map_data_.info.width + x];
                if (occupancy == -1) {
                    map_image_gray.at<uchar>(y, x) = 205; // Unknown space (gray)
                } else {
                    // Invert: higher occupancy (darker) -> lower pixel value (black)
                    map_image_gray.at<uchar>(y, x) = static_cast<uchar>(255 - (occupancy * 2.55));
                }
            }
        }
        
        // 2. Convert to color image to draw colored points
        cv::Mat map_image_bgr;
        cv::cvtColor(map_image_gray, map_image_bgr, cv::COLOR_GRAY2BGR);
        
        // 3. Draw each collected point on the image
        double res = map_data_.info.resolution;
        double origin_x = map_data_.info.origin.position.x;
        double origin_y = map_data_.info.origin.position.y;
        cv::Scalar point_color( // BGR format for OpenCV
            marker_color_[2] * 255, 
            marker_color_[1] * 255, 
            marker_color_[0] * 255
        );

        for (const auto& point : collected_points_) {
            // Convert world coordinates (meters) to pixel coordinates
            int px = static_cast<int>((point.x - origin_x) / res);
            int py = static_cast<int>((point.y - origin_y) / res);
            
            // Draw a circle for the point
            // Note: OpenCV's origin is top-left, ROS map's is bottom-left, so we flip the y-axis
            cv::circle(map_image_bgr, cv::Point(px, map_data_.info.height - 1 - py), 2, point_color, -1);
        }
        
        // 4. Save the final image
        std::string filename = "map_with_rad_points.png";
        std::string full_path = (std::filesystem::path(session_folder_) / filename).string();
        
        if (cv::imwrite(full_path, map_image_bgr)) {
            response->success = true;
            response->message = "Successfully saved image to " + full_path;
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        } else {
            response->success = false;
            response->message = "Failed to save image to " + full_path;
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }
    
    void createSessionFolder()
    {
        std::stringstream ss;
        std::time_t now = std::time(nullptr);
        ss << "rad_points_" << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
        if (output_directory_.empty()) {
            output_directory_ = std::filesystem::current_path().string();
        }
        session_folder_ = (std::filesystem::path(output_directory_) / ss.str()).string();
        std::filesystem::create_directories(session_folder_);
        RCLCPP_INFO(this->get_logger(), "Saving output to: %s", session_folder_.c_str());
    }

    // ROS objects
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Parameters & Data
    std::string map_frame_, robot_frame_, output_directory_, session_folder_;
    double tf_timeout_seconds_, marker_scale_;
    std::vector<double> marker_color_;
    std::vector<geometry_msgs::msg::Point> collected_points_;
    std::mutex points_mutex_;
    nav_msgs::msg::OccupancyGrid map_data_;
    bool map_received_{false};
    std::mutex map_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadPointSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}