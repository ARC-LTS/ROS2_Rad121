#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <mutex>
#include <vector>
#include <fstream>
#include <iomanip>

// Struct to hold the combined data.
struct TaggedData
{
    rclcpp::Time timestamp;
    double cpm;
    double mrem_hr;
    double x;
    double y;
};

/**
 * @class RadMapperNode
 * @brief Subscribes to radiation data, uses TF2 to get the robot's position,
 * and saves all collected data to a file on shutdown. This node performs
 * no visualization and is designed to be lightweight and fast.
 */
class RadMapperNode : public rclcpp::Node
{
public:
    RadMapperNode()
        : Node("rad_mapper_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("output_file_path", "rad_data.csv");
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<std::string>("source_frame", "odom");

        // Get parameters
        output_file_path_ = this->get_parameter("output_file_path").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        source_frame_ = this->get_parameter("source_frame").as_string();

        RCLCPP_INFO(this->get_logger(), "Saving data to: %s", output_file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Looking up transform from '%s' -> '%s'",
                    source_frame_.c_str(), target_frame_.c_str());

        // Initialize TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscription to the /rad topic.
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rad", 10, std::bind(&RadMapperNode::rad_callback, this, std::placeholders::_1));

        // Register the shutdown handler to save data.
        rclcpp::on_shutdown([this]()
                            {
            RCLCPP_INFO(this->get_logger(), "Shutdown signal received. Saving data...");
            this->save_data(); });

        RCLCPP_INFO(this->get_logger(), "Rad Mapper Node has been initialized.");
    }

private:
    /**
     * @brief Callback for /rad messages. Looks up the transform and stores the data.
     */
    void rad_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received /rad message with insufficient data size.");
            return;
        }

        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tf_buffer_->lookupTransform(source_frame_, target_frame_, tf2::TimePoint());
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Could not get transform from '%s' to '%s': %s",
                                 source_frame_.c_str(), target_frame_.c_str(), ex.what());
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);

        collected_data_.push_back({this->get_clock()->now(),
                                   msg->data[0],
                                   msg->data[1],
                                   t.transform.translation.x,
                                   t.transform.translation.y});

        RCLCPP_DEBUG(this->get_logger(), "Logged data point #%zu", collected_data_.size());
    }

    void save_data()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::ofstream output_file(output_file_path_);
        if (!output_file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", output_file_path_.c_str());
            return;
        }

        output_file << "timestamp_sec,timestamp_nanosec,cpm,mrem_hr,x,y\n";
        for (const auto &data : collected_data_)
        {
            output_file << data.timestamp.seconds() << ","
                        << (data.timestamp.nanoseconds() % 1000000000) << ","
                        << std::fixed << std::setprecision(8)
                        << data.cpm << ","
                        << data.mrem_hr << ","
                        << data.x << ","
                        << data.y << "\n";
        }
        output_file.close();
        RCLCPP_INFO(this->get_logger(), "Successfully saved %zu data points to %s", collected_data_.size(), output_file_path_.c_str());
    }

    // Member Variables
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    std::string output_file_path_;
    std::string target_frame_;
    std::string source_frame_;
    std::vector<TaggedData> collected_data_;
    std::mutex data_mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadMapperNode>());
    rclcpp::shutdown();
    return 0;
}
