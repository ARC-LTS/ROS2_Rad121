#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "ros2_rad_map/srv/gp_predict.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <vector>
#include <deque>
#include <mutex>
#include <thread>
#include "std_msgs/msg/float64.hpp"

// Historical sample buffer cap: previously unbounded, growing (and getting
// copied wholesale + re-marshalled into a service request) for the entire
// mission. Mirrors gp_predictor_node.py's equivalent 1000-sample cap.
static constexpr size_t kMaxHistoricalSamples = 1000;

struct RadSample {
    double x, y, value;
};

void pack_rgb(float r, float g, float b, float& rgb_float) {
    uint8_t r_uint = static_cast<uint8_t>(r * 255.0f);
    uint8_t g_uint = static_cast<uint8_t>(g * 255.0f);
    uint8_t b_uint = static_cast<uint8_t>(b * 255.0f);
    uint32_t rgb = (static_cast<uint32_t>(r_uint) << 16) |
                   (static_cast<uint32_t>(g_uint) << 8) |
                   static_cast<uint32_t>(b_uint);
    memcpy(&rgb_float, &rgb, sizeof(uint32_t));
}

float valueToColor(double value, double min_val, double max_val) {
    double normalized_value = (max_val > min_val) ? (value - min_val) / (max_val - min_val) : 0.0;
    normalized_value = std::max(0.0, std::min(1.0, normalized_value));
    float r, g, b;

    if (normalized_value < 0.25) {
        r = 0.0; g = normalized_value * 4.0; b = 1.0;
    } else if (normalized_value < 0.5) {
        r = 0.0; g = 1.0; b = 1.0 - (normalized_value - 0.25) * 4.0;
    } else if (normalized_value < 0.75) {
        r = (normalized_value - 0.5) * 4.0; g = 1.0; b = 0.0;
    } else {
        r = 1.0; g = 1.0 - (normalized_value - 0.75) * 4.0; b = 0.0;
    }
    
    float rgb_float;
    pack_rgb(r, g, b, rgb_float);
    return rgb_float;
}


class GPVisualizerNode : public rclcpp::Node
{
public:
    GPVisualizerNode() : Node("gp_visualizer_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>("sensor_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<double>("cloud_resolution_m", 0.25);
        this->declare_parameter<double>("update_period_s", 10.0);

        sensor_frame_ = this->get_parameter("sensor_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        cloud_resolution_ = this->get_parameter("cloud_resolution_m").as_double();
        double update_period = this->get_parameter("update_period_s").as_double();

        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local().reliable();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, std::bind(&GPVisualizerNode::mapCallback, this, std::placeholders::_1));
        
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/rad", 10, std::bind(&GPVisualizerNode::radCallback, this, std::placeholders::_1));

        background_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/rad_field_server/background_radiation", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                this->current_background_level_ = msg->data;
            });

        gp_client_ = this->create_client<ros2_rad_map::srv::GPPredict>("predict_gp");

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/gp_surface_cloud", 10);

        timer_ = this->create_wall_timer(std::chrono::duration<double>(update_period), std::bind(&GPVisualizerNode::triggerCloudGeneration, this));

        RCLCPP_INFO(this->get_logger(), "GP Visualizer Node has started.");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        map_ = *msg;
        map_received_ = true;
    }

    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) return;

        geometry_msgs::msg::PoseStamped pose_in_map;
        try {
            geometry_msgs::msg::PoseStamped pose_in_sensor;
            pose_in_sensor.header.frame_id = sensor_frame_;
            pose_in_sensor.header.stamp = rclcpp::Time(0);
            // No timeout (defaults to 0): avoids a 500ms blocking wait on
            // every /rad message. TF is virtually always already available by
            // the time a sample arrives; if not, skip this one sample.
            tf_buffer_.transform(pose_in_sensor, pose_in_map, map_frame_);

            std::lock_guard<std::mutex> lock(data_mutex_);
            historical_samples_.push_back({pose_in_map.pose.position.x, pose_in_map.pose.position.y, msg->data[1]});
            if (historical_samples_.size() > kMaxHistoricalSamples) {
                historical_samples_.pop_front();
            }
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error for logging sample: %s", ex.what());
        }
    }

    void triggerCloudGeneration()
    {
        std::lock_guard<std::mutex> lock(update_mutex_);
        if (is_updating_) {
            return;
        }
        is_updating_ = true;

        std::thread{ [this]() {
            this->generateCloud();
        }}.detach();
    }

    void generateCloud()
    {
        if (!gp_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "GP prediction service is not available.");
            std::lock_guard<std::mutex> lock(update_mutex_);
            is_updating_ = false;
            return;
        }

        nav_msgs::msg::OccupancyGrid map_copy;
        std::deque<RadSample> samples_copy;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!map_received_ || historical_samples_.empty()) {
                RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for map and radiation samples to generate visualization.");
                std::lock_guard<std::mutex> update_lock(update_mutex_);
                is_updating_ = false;
                return;
            }
            map_copy = map_;
            samples_copy = historical_samples_;
        }

        auto request = std::make_shared<ros2_rad_map::srv::GPPredict::Request>();
        request->mean_value = current_background_level_;
        for (const auto &s : samples_copy) {
            request->observed_x.push_back(s.x);
            request->observed_y.push_back(s.y);
            request->observed_values_mean.push_back(s.value);
        }

        std::vector<geometry_msgs::msg::Point> points_to_predict;
        double res = map_copy.info.resolution;
        double origin_x = map_copy.info.origin.position.x;
        double origin_y = map_copy.info.origin.position.y;
        int step = std::max(1, (int)(cloud_resolution_/res));
        
        for (unsigned int y = 0; y < map_copy.info.height; y += step) {
            for (unsigned int x = 0; x < map_copy.info.width; x += step) {
                if (map_copy.data[y * map_copy.info.width + x] == 0) {
                    geometry_msgs::msg::Point p;
                    p.x = origin_x + (x + 0.5) * res;
                    p.y = origin_y + (y + 0.5) * res;
                    points_to_predict.push_back(p);
                }
            }
        }
        
        if (points_to_predict.empty()) {
            std::lock_guard<std::mutex> lock(update_mutex_);
            is_updating_ = false;
            return;
        }
        request->points_to_predict = points_to_predict;

        auto future = gp_client_->async_send_request(request);
        
        // The MultiThreadedExecutor in main() is processing the service response.
        // We can just wait for the future to be ready in this background thread.
        // Let's use a timeout to avoid blocking forever if the service is down.
        if (future.wait_for(std::chrono::seconds(9)) == std::future_status::ready)
        {
            auto response = future.get();
            if (!response || response->predicted_mean.size() != points_to_predict.size()) {
                RCLCPP_ERROR(this->get_logger(), "GP service call failed or returned invalid data.");
            } else {
                publishCloud(points_to_predict, response->predicted_mean);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call GP service: request timed out.");
        }

        std::lock_guard<std::mutex> lock(update_mutex_);
        is_updating_ = false;
    }

    void publishCloud(const std::vector<geometry_msgs::msg::Point>& points, const std::vector<double>& values)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->get_clock()->now();
        cloud_msg.header.frame_id = map_frame_;
        cloud_msg.height = 1;
        cloud_msg.width = points.size();
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;
        
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);

        auto minmax = std::minmax_element(values.begin(), values.end());
        double min_val = *minmax.first;
        double max_val = *minmax.second;

        for(size_t i = 0; i < points.size(); ++i) {
            const auto& point = points[i];
            float z = values[i];
            float color = valueToColor(z, min_val, max_val);

            uint8_t* ptr = &cloud_msg.data[i * cloud_msg.point_step];
            float x_float = point.x;
            float y_float = point.y;
            memcpy(ptr + 0, &x_float, sizeof(float));
            memcpy(ptr + 4, &y_float, sizeof(float));
            memcpy(ptr + 8, &z, sizeof(float));
            memcpy(ptr + 12, &color, sizeof(float));
        }
        
        cloud_pub_->publish(cloud_msg);
    }

    std::mutex data_mutex_;
    std::mutex update_mutex_;
    bool is_updating_{false};
    nav_msgs::msg::OccupancyGrid map_;
    bool map_received_{false};
    std::deque<RadSample> historical_samples_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr background_sub_;
    rclcpp::Client<ros2_rad_map::srv::GPPredict>::SharedPtr gp_client_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    std::string sensor_frame_, map_frame_;
    double cloud_resolution_;
    double current_background_level_{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Use a MultiThreadedExecutor to allow the service client to be called from a separate thread
    // without blocking the main subscription/timer callbacks.
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<GPVisualizerNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}