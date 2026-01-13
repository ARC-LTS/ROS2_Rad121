#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <vector>
#include <random>
#include <string>
#include <memory>

// Struct to hold information about a single radiation source
struct RadSource
{
    int id;
    geometry_msgs::msg::Point position;
    double intensity; // mrem/hr
};

class RadFieldServer : public rclcpp::Node
{
public:
    RadFieldServer() : Node("rad_field_server"), tf_buffer_(this->get_clock()), next_source_id_(0)
    {
        // Initialize TF listener
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        // Initialize random number generation
        rng_.seed(std::random_device()());
        dist_ = std::uniform_real_distribution<double>(2.5, 10.0);

        // Declare parameters
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("robot_frame", "base_link");
        map_frame_ = this->get_parameter("map_frame").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();

        // Create the service to spawn a new source
        spawn_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/spawn_source",
            std::bind(&RadFieldServer::spawn_source_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Create the publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/sources", 10);

        // Create a timer to periodically publish markers
        marker_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RadFieldServer::publish_markers, this));

        RCLCPP_INFO(this->get_logger(), "Radiation Field Server has started.");
        RCLCPP_INFO(this->get_logger(), "Call service '%s' to spawn a source.", spawn_service_->get_service_name());
        RCLCPP_INFO(this->get_logger(), "Publishing source markers to '%s'.", marker_pub_->get_topic_name());
    }

private:
    void spawn_source_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // Unused

        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tf_buffer_.lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                         robot_frame_.c_str(), map_frame_.c_str(), ex.what());
            response->success = false;
            response->message = "TF lookup failed.";
            return;
        }

        RadSource new_source;
        new_source.id = next_source_id_++;
        new_source.position.x = t.transform.translation.x;
        new_source.position.y = t.transform.translation.y;
        new_source.position.z = t.transform.translation.z;
        new_source.intensity = dist_(rng_); // Random intensity

        sources_.push_back(new_source);

        RCLCPP_INFO(this->get_logger(), "Spawned source %d at (%.2f, %.2f, %.2f) with intensity %.2f mrem/hr.",
                    new_source.id, new_source.position.x, new_source.position.y, new_source.position.z, new_source.intensity);

        // Immediately publish markers after adding a new one
        publish_markers();

        response->success = true;
        response->message = "Radiation source spawned successfully.";
    }

    void publish_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        auto now = this->get_clock()->now();

        // Add a DELETEALL marker to clear previous markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = map_frame_;
        clear_marker.header.stamp = now;
        clear_marker.id = 0;
        clear_marker.ns = "rad_sources";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        // Create a new marker for each source
        for (const auto &source : sources_)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_frame_;
            marker.header.stamp = now;
            marker.ns = "rad_sources";
            marker.id = source.id + 1; // ID 0 is reserved for DELETEALL
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = source.position;
            marker.pose.orientation.w = 1.0;

            // Scale and color based on intensity (2.5 -> 10.0)
            double normalized_intensity = (source.intensity - 2.5) / (10.0 - 2.5);
            marker.scale.x = 0.2 + normalized_intensity * 0.8; // Scale from 0.2m to 1.0m
            marker.scale.y = 0.2 + normalized_intensity * 0.8;
            marker.scale.z = 0.2 + normalized_intensity * 0.8;
            
            // Color from green (low) to red (high)
            marker.color.a = 0.8; // Alpha
            marker.color.r = static_cast<float>(normalized_intensity);
            marker.color.g = static_cast<float>(1.0 - normalized_intensity);
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    // ROS Comms
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr spawn_service_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string map_frame_;
    std::string robot_frame_;

    // State
    std::vector<RadSource> sources_;
    int next_source_id_;

    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadFieldServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}