#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <cmath>

// Struct to hold information about a single radiation source
struct RadSource
{
    int id;
    geometry_msgs::msg::Point position;
    double intensity; // mrem/hr
};

class RadSensorSimulator : public rclcpp::Node
{
public:
    RadSensorSimulator() : Node("rad_sensor_simulator"), tf_buffer_(this->get_clock())
    {
        // Initialize TF listener
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        // Declare parameters
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("robot_frame", "base_link");
        this->declare_parameter<double>("publish_rate_hz", 10.0);
        this->declare_parameter<double>("min_sensing_distance_m", 0.58);
        
        map_frame_ = this->get_parameter("map_frame").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        double publish_rate = this->get_parameter("publish_rate_hz").as_double();
        double min_sensing_distance = this->get_parameter("min_sensing_distance_m").as_double();
        min_distance_sq_ = min_sensing_distance * min_sensing_distance;


        // Subscription to the source markers from the field server
        source_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/rad_field_server/sources", 10,
            std::bind(&RadSensorSimulator::sources_callback, this, std::placeholders::_1));

        // Publisher for the simulated /rad topic
        rad_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rad", 10);

        // Timer for the main sensor update loop
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&RadSensorSimulator::sensor_update_callback, this));

        RCLCPP_INFO(this->get_logger(), "Radiation Sensor Simulator has started.");
    }

private:
    void sources_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(source_mutex_);
        sources_.clear();

        for (const auto &marker : msg->markers)
        {
            // We only care about the ADD actions for SPHERE markers
            if (marker.type == visualization_msgs::msg::Marker::SPHERE &&
                marker.action == visualization_msgs::msg::Marker::ADD)
            {
                RadSource source;
                source.id = marker.id;
                source.position = marker.pose.position;
                // Reverse the normalization logic from the server to get intensity
                // scale = 0.2 + norm * 0.8  => norm = (scale - 0.2) / 0.8
                // intensity = norm * (10.0 - 2.5) + 2.5
                double normalized_intensity = (marker.scale.x - 0.2) / 0.8;
                source.intensity = normalized_intensity * 7.5 + 2.5;

                sources_.push_back(source);
            }
        }
    }

    void sensor_update_callback()
    {
        geometry_msgs::msg::Point sensor_position;
        try
        {
            geometry_msgs::msg::TransformStamped t = tf_buffer_.lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
            sensor_position.x = t.transform.translation.x;
            sensor_position.y = t.transform.translation.y;
            sensor_position.z = t.transform.translation.z;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Could not get sensor position: %s", ex.what());
            return;
        }

        std::vector<RadSource> sources_copy;
        {
            std::lock_guard<std::mutex> lock(source_mutex_);
            sources_copy = sources_;
        }

        double total_mrem_hr = 0.0;

        for (const auto &source : sources_copy)
        {
            double dx = source.position.x - sensor_position.x;
            double dy = source.position.y - sensor_position.y;
            double dz = source.position.z - sensor_position.z;
            double dist_sq = dx * dx + dy * dy + dz * dz;

            if (dist_sq < min_distance_sq_)
            {
                dist_sq = min_distance_sq_;
            }

            // Inverse square law
            total_mrem_hr += source.intensity / dist_sq;
        }

        // Publish the message
        auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        // The real sensor publishes [cpm_compensated, mR/hr]. We simulate this.
        // We can invent a CPM value or just put 0, as the second value is what's used.
        msg->data.push_back(total_mrem_hr * 450.0); // Dummy CPM based on a sensitivity of 450
        msg->data.push_back(total_mrem_hr);
        rad_pub_->publish(std::move(msg));
    }

    // ROS Comms
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr source_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rad_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string map_frame_;
    std::string robot_frame_;

    // State
    std::vector<RadSource> sources_;
    std::mutex source_mutex_;
    double min_distance_sq_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadSensorSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}