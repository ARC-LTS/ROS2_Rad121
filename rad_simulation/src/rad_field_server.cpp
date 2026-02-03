#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <vector>
#include <random>
#include <string>
#include <memory>
#include <mutex>

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
        this->declare_parameter<int>("num_random_sources_to_spawn", 3);

        map_frame_ = this->get_parameter("map_frame").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        num_random_sources_ = this->get_parameter("num_random_sources_to_spawn").as_int();

        // Create the service to spawn a new source at robot's pose
        spawn_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/spawn_source",
            std::bind(&RadFieldServer::spawn_source_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        // Create the service to spawn random sources on the map
        spawn_random_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/spawn_random_sources",
            std::bind(&RadFieldServer::spawn_random_sources_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Create the publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/sources", 10);

        // Create a timer to periodically publish markers
        marker_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RadFieldServer::publish_markers, this));
        
        // Subscribe to the map
        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local().reliable();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos, std::bind(&RadFieldServer::map_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Radiation Field Server has started.");
        RCLCPP_INFO(this->get_logger(), "Call service '%s' to spawn a source at robot.", spawn_service_->get_service_name());
        RCLCPP_INFO(this->get_logger(), "Call service '%s' to spawn random sources on map.", spawn_random_service_->get_service_name());
        RCLCPP_INFO(this->get_logger(), "Publishing source markers to '%s'.", marker_pub_->get_topic_name());
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_map_ = *msg;
        map_received_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "Map received. Ready to spawn random sources.");
    }

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

    void spawn_random_sources_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // Unused

        if (!map_received_) {
            response->success = false;
            response->message = "Map not received yet. Cannot spawn random sources.";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        std::lock_guard<std::mutex> lock(map_mutex_);
        unsigned int map_width = current_map_.info.width;
        unsigned int map_height = current_map_.info.height;
        double map_res = current_map_.info.resolution;
        double map_origin_x = current_map_.info.origin.position.x;
        double map_origin_y = current_map_.info.origin.position.y;

        std::uniform_int_distribution<unsigned int> x_dist(0, map_width - 1);
        std::uniform_int_distribution<unsigned int> y_dist(0, map_height - 1);

        int spawned_count = 0;
        for (int i = 0; i < num_random_sources_; ++i) {
            unsigned int rand_mx, rand_my;
            int attempts = 0;
            const int max_attempts = 100;

            do {
                rand_mx = x_dist(rng_);
                rand_my = y_dist(rng_);
                attempts++;
            } while (current_map_.data[rand_my * map_width + rand_mx] != 0 && attempts < max_attempts);

            if (attempts == max_attempts) {
                RCLCPP_WARN(this->get_logger(), "Failed to find free space for random source after %d attempts. Skipping source.", max_attempts);
                continue;
            }

            RadSource new_source;
            new_source.id = next_source_id_++;
            new_source.position.x = map_origin_x + (rand_mx + 0.5) * map_res;
            new_source.position.y = map_origin_y + (rand_my + 0.5) * map_res;
            new_source.position.z = 0.0; // Assume 2D plane
            new_source.intensity = dist_(rng_); // Random intensity

            sources_.push_back(new_source);
            spawned_count++;

            RCLCPP_INFO(this->get_logger(), "Spawned random source %d at (%.2f, %.2f) with intensity %.2f mrem/hr.",
                        new_source.id, new_source.position.x, new_source.position.y, new_source.intensity);
        }

        if (spawned_count > 0) {
            publish_markers();
            response->success = true;
            response->message = "Successfully spawned " + std::to_string(spawned_count) + " random radiation sources.";
        } else {
            response->success = false;
            response->message = "No random sources were spawned.";
        }
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
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr spawn_random_service_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;


    // TF
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string map_frame_;
    std::string robot_frame_;

    // State
    std::vector<RadSource> sources_;
    int next_source_id_;
    nav_msgs::msg::OccupancyGrid current_map_;
    bool map_received_;
    int num_random_sources_;

    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_;
    
    // Mutex for map access
    std::mutex map_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadFieldServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}