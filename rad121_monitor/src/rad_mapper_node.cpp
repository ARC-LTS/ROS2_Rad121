#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_srvs/srv/trigger.hpp"  // ## NEW ## Required for the service
#include <mutex>
#include <vector>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <ctime>

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
 * and saves all collected data to a timestamped session folder via a service call.
 */
class RadMapperNode : public rclcpp::Node
{
public:
    RadMapperNode()
        : Node("rad_mapper_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("output_directory", "");
        this->declare_parameter<std::string>("output_filename", "rad_data.csv");
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<std::string>("source_frame", "odom");

        // Get parameters
        output_directory_ = this->get_parameter("output_directory").as_string();
        output_filename_ = this->get_parameter("output_filename").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        source_frame_ = this->get_parameter("source_frame").as_string();

        createSessionFolder();

        RCLCPP_INFO(this->get_logger(), "Saving data to session folder: %s", session_folder_.c_str());
        RCLCPP_INFO(this->get_logger(), "Looking up transform from '%s' -> '%s'",
                    source_frame_.c_str(), target_frame_.c_str());

        // Initialize TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscription to the /rad topic.
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rad", 10, std::bind(&RadMapperNode::rad_callback, this, std::placeholders::_1));

        // ## NEW ## Create the service server to trigger data saving.
        store_data_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/store_data",
            std::bind(&RadMapperNode::storeDataCallback, this, std::placeholders::_1, std::placeholders::_2));

        // ## REMOVED ## The on_shutdown hook is no longer needed.
        // rclcpp::on_shutdown([...]);

        RCLCPP_INFO(this->get_logger(), "Rad Mapper Node has been initialized.");
        RCLCPP_INFO(this->get_logger(), "Call the /store_data service to save the collected data.");
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
            t = tf_buffer_->lookupTransform(source_frame_, target_frame_, tf2::TimePoint(), tf2::durationFromSec(0.1));
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

    /**
     * @brief Creates a unique timestamped directory for the session's output.
     */
    void createSessionFolder()
    {
        std::stringstream ss;
        std::time_t now = std::time(nullptr);
        ss << "rad_" << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
        std::string session_name = ss.str();

        if (output_directory_.empty()) {
            output_directory_ = std::filesystem::current_path().string();
        }
        
        session_folder_ = (std::filesystem::path(output_directory_) / session_name).string();
        
        try {
            std::filesystem::create_directories(session_folder_);
             RCLCPP_INFO(this->get_logger(), "Session folder created: %s", session_folder_.c_str());
        } catch (const std::filesystem::filesystem_error &e) {
             RCLCPP_ERROR(this->get_logger(), "Could not create session folder. Data saving will be disabled. Error: %s", e.what());
             session_folder_ = "";
        }
    }

    /**
     * @brief Saves all collected data points to a CSV file in the session folder.
     */
    void save_data()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (session_folder_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Session folder is not set. Cannot save data.");
            return;
        }
        std::filesystem::path full_path = std::filesystem::path(session_folder_) / output_filename_;

        std::ofstream output_file(full_path.string());
        if (!output_file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", full_path.c_str());
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
        RCLCPP_INFO(this->get_logger(), "Successfully saved %zu data points to %s", collected_data_.size(), full_path.c_str());
    }

    /**
     * @brief The callback function for the /store_data service.
     */
    void storeDataCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // Request is unused, suppress warning

        RCLCPP_INFO(this->get_logger(), "/store_data service called. Saving collected data...");
        save_data();

        response->success = true;
        response->message = "Data saved to session folder: " + session_folder_;
    }

    // Member Variables
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr store_data_service_; // ## NEW ##
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    std::string output_directory_;
    std::string output_filename_;
    std::string session_folder_;
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