#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rad_simulation/msg/rad_source_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <cmath>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

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
    RadSensorSimulator() : Node("rad_sensor_simulator"), tf_buffer_(this->get_clock()), background_radiation_(0.0)
    {
        // Initialize TF listener
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        // Declare parameters
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("robot_frame", "base_link");
        this->declare_parameter<double>("publish_rate_hz", 10.0);
        this->declare_parameter<double>("min_sensing_distance_m", 0.58);
        this->declare_parameter<double>("effective_range_m", 5.0);
        this->declare_parameter<bool>("background_radiation_enabled", true);
        this->declare_parameter<double>("background_radiation_min", 0.05);
        this->declare_parameter<double>("background_radiation_max", 1.0);

        map_frame_ = this->get_parameter("map_frame").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        double publish_rate = this->get_parameter("publish_rate_hz").as_double();
        double min_sensing_distance = this->get_parameter("min_sensing_distance_m").as_double();
        min_distance_sq_ = min_sensing_distance * min_sensing_distance;
        double effective_range = this->get_parameter("effective_range_m").as_double();
        effective_range_sq_ = effective_range * effective_range;

        // Cache background parameters to avoid get_parameter() calls in the 10 Hz hot path
        bg_enabled_ = this->get_parameter("background_radiation_enabled").as_bool();
        bg_min_ = this->get_parameter("background_radiation_min").as_double();
        bg_max_ = this->get_parameter("background_radiation_max").as_double();

        param_cb_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params) {
                for (const auto & p : params) {
                    if (p.get_name() == "background_radiation_enabled")
                        bg_enabled_ = p.as_bool();
                    else if (p.get_name() == "background_radiation_min")
                        bg_min_ = p.as_double();
                    else if (p.get_name() == "background_radiation_max")
                        bg_max_ = p.as_double();
                }
                rcl_interfaces::msg::SetParametersResult r;
                r.successful = true;
                return r;
            });


        // Subscription to the raw source data from the field server
        source_sub_ = this->create_subscription<rad_simulation::msg::RadSourceArray>(
            "/rad_field_server/sources_data", 10,
            std::bind(&RadSensorSimulator::sources_callback, this, std::placeholders::_1));
        
        // Subscription to background radiation
        background_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/rad_field_server/background_radiation", 10,
            std::bind(&RadSensorSimulator::background_callback, this, std::placeholders::_1));

        // Publisher for the simulated /rad topic
        rad_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rad", 10);

        // Timer for the main sensor update loop
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&RadSensorSimulator::sensor_update_callback, this));

        RCLCPP_INFO(this->get_logger(), "Radiation Sensor Simulator has started.");
    }

private:
    double get_spatial_background(double x, double y)
    {
        if (!bg_enabled_) {
            return 0.0;
        }

        // Spatial noise function to simulate "infinite number of fixed point sources".
        // Deterministic based on coordinates (no temporal noise).
        // Using a multi-scale sine sum to create a natural-looking background field.
        double val = std::sin(x * 0.43) * std::cos(y * 0.37) +
                     std::sin(x * 1.21 + 0.5) * std::sin(y * 1.33 + 1.2) +
                     std::cos(x * 2.87 + 2.1) * std::sin(y * 3.11 + 0.8) +
                     std::sin(x * 5.71 + 3.4) * std::cos(y * 6.23 + 4.1);

        // val is in roughly [-4, 4]. Normalize to [0, 1]
        double normalized = (val + 4.0) / 8.0;
        normalized = std::max(0.0, std::min(1.0, normalized));

        return bg_min_ + normalized * (bg_max_ - bg_min_);
    }

    void background_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(source_mutex_);
        background_radiation_ = msg->data;
    }

    void sources_callback(const rad_simulation::msg::RadSourceArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(source_mutex_);
        sources_.clear();

        for (const auto &s : msg->sources)
        {
            RadSource source;
            source.id = s.id;
            source.position = s.position;
            source.intensity = s.intensity;
            sources_.push_back(source);
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

        // Use spatial background based on position (removes temporal sensor noise)
        double total_mrem_hr = get_spatial_background(sensor_position.x, sensor_position.y);

        for (const auto &source : sources_copy)
        {
            double dx = source.position.x - sensor_position.x;
            double dy = source.position.y - sensor_position.y;
            double dz = source.position.z - sensor_position.z;
            double dist_sq = dx * dx + dy * dy + dz * dz;

            // Clamp at effective_range_sq_ so reading never exceeds source.intensity.
            // min_distance_sq_ is a fallback in case effective_range_m is set to 0.
            double clamp_sq = std::max(effective_range_sq_, min_distance_sq_);
            if (dist_sq < clamp_sq)
            {
                dist_sq = clamp_sq;
            }

            // Point source model: I(r) = I_source * (effective_range^2 / r^2)
            // At r <= effective_range_m the reading is capped at source.intensity.
            // Beyond effective_range_m it falls off as inverse-square.
            total_mrem_hr += source.intensity * (effective_range_sq_ / dist_sq);
        }

        // Publish the message
        auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        msg->data.push_back(total_mrem_hr * 450.0); // Dummy CPM
        msg->data.push_back(total_mrem_hr);
        rad_pub_->publish(std::move(msg));
    }

    // ROS Comms
    rclcpp::Subscription<rad_simulation::msg::RadSourceArray>::SharedPtr source_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr background_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rad_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string map_frame_;
    std::string robot_frame_;

    // State
    std::vector<RadSource> sources_;
    double background_radiation_;
    std::mutex source_mutex_;
    double min_distance_sq_;
    double effective_range_sq_;

    // Cached background parameters (updated via parameter callback)
    bool bg_enabled_;
    double bg_min_;
    double bg_max_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadSensorSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}