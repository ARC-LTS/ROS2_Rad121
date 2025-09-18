#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rad121_monitor/srv/gp_predict.hpp"

#include <vector>
#include <mutex>
#include <thread>
#include <future>
#include <opencv2/opencv.hpp>

struct RadiationSample { double x, y; };
cv::Mat createBaseMapImage(const nav_msgs::msg::OccupancyGrid &map);
cv::Mat createColorHeatmapImage(const cv::Mat &field, double threshold);

class RadiationHeatmapNode : public rclcpp::Node
{
public:
    RadiationHeatmapNode() : Node("rad_heatmap_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {}

    void init()
    {
        this->declare_parameter<std::string>("sensor_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<double>("background_threshold_mrem_hr", 0.1);
        this->declare_parameter<int>("raycasting_max_sources", 20);
        this->declare_parameter<double>("raycasting_update_radius_m", 20.0);

        sensor_frame_ = this->get_parameter("sensor_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        background_threshold_mrem_hr_ = this->get_parameter("background_threshold_mrem_hr").as_double();
        raycasting_max_sources_ = this->get_parameter("raycasting_max_sources").as_int();
        raycasting_update_radius_m_ = this->get_parameter("raycasting_update_radius_m").as_double();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), std::bind(&RadiationHeatmapNode::mapCallback, this, std::placeholders::_1));
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/rad", 10, std::bind(&RadiationHeatmapNode::radCallback, this, std::placeholders::_1));

        image_transport::ImageTransport it(this->shared_from_this());
        heatmap_pub_ = it.advertise("/rad_heatmap/map", 1);
        
        gp_client_ = this->create_client<rad121_monitor::srv::GPPredict>("predict_gp");
        RCLCPP_INFO(this->get_logger(), "Waiting for GP service...");
        gp_client_->wait_for_service();
        RCLCPP_INFO(this->get_logger(), "Node initialized.");
    }

private:
    std::mutex data_mutex_;
    nav_msgs::msg::OccupancyGrid base_map_;
    bool map_received_ = false;
    std::vector<RadiationSample> collected_samples_;
    
    std::mutex update_mutex_;
    bool is_updating_ = false;
    
    cv::Mat persistent_occluded_heatmap_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            base_map_ = *msg;
            map_received_ = true;
        }
        
        triggerGPUpdate();
    }

    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!map_received_ || msg->data.size() < 2) {
            return;
        }

        geometry_msgs::msg::PoseStamped pose_in_map;
        try {
            geometry_msgs::msg::PoseStamped pose_in_sensor;
            pose_in_sensor.header.frame_id = sensor_frame_;
            pose_in_sensor.header.stamp = this->get_clock()->now();
            tf_buffer_.transform(pose_in_sensor, pose_in_map, map_frame_, std::chrono::milliseconds(200));
            
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                collected_samples_.push_back({pose_in_map.pose.position.x, pose_in_map.pose.position.y});
            } 

            RCLCPP_INFO(this->get_logger(), "New radiation sample logged. Triggering heatmap update.");
            triggerGPUpdate();

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        }
    }

    void triggerGPUpdate()
    {
        {
            std::lock_guard<std::mutex> lock(update_mutex_);
            if (is_updating_) { return; }
            is_updating_ = true;
        }

        nav_msgs::msg::OccupancyGrid map_copy;
        std::vector<RadiationSample> samples_copy;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!map_received_ || collected_samples_.empty()) {
                std::lock_guard<std::mutex> update_lock(update_mutex_);
                is_updating_ = false;
                return;
            }
            map_copy = base_map_;
            samples_copy = collected_samples_;
        }
        
        auto request = std::make_shared<rad121_monitor::srv::GPPredict::Request>();
        std::vector<std::pair<int, int>> valid_pixel_coords;
        double res = map_copy.info.resolution;
        double origin_x = map_copy.info.origin.position.x;
        double origin_y = map_copy.info.origin.position.y;

        for (unsigned int y = 0; y < map_copy.info.height; ++y) {
            for (unsigned int x = 0; x < map_copy.info.width; ++x) {
                if (map_copy.data[y * map_copy.info.width + x] < 50) {
                    geometry_msgs::msg::Point p;
                    p.x = origin_x + (x + 0.5) * res;
                    p.y = origin_y + (y + 0.5) * res;
                    request->points_to_predict.push_back(p);
                    valid_pixel_coords.push_back({(int)x, (int)y});
                }
            }
        }
        
        gp_client_->async_send_request(request, [this, map_copy, samples_copy, valid_pixel_coords](rclcpp::Client<rad121_monitor::srv::GPPredict>::SharedFuture future) {
            auto response = future.get();
            if (!response || response->predicted_mean.size() != valid_pixel_coords.size()) {
                RCLCPP_ERROR(this->get_logger(), "GP service response is invalid or size mismatch.");
                std::lock_guard<std::mutex> lock(update_mutex_);
                is_updating_ = false;
                return;
            }

            cv::Mat raw_heatmap = cv::Mat::zeros(map_copy.info.height, map_copy.info.width, CV_32F);
            for (size_t i = 0; i < valid_pixel_coords.size(); ++i) {
                raw_heatmap.at<float>(valid_pixel_coords[i].second, valid_pixel_coords[i].first) = response->predicted_mean[i];
            }

            updateLocalRaycasting(raw_heatmap, map_copy, samples_copy);

            cv::Mat base_bgr = createBaseMapImage(map_copy);
            cv::Mat color_heatmap = createColorHeatmapImage(persistent_occluded_heatmap_, background_threshold_mrem_hr_);
            cv::Mat blended_image;
            cv::addWeighted(base_bgr, 0.6, color_heatmap, 0.4, 0.0, blended_image);

            std_msgs::msg::Header h;
            h.stamp = this->now();
            h.frame_id = map_frame_;
            heatmap_pub_.publish(*cv_bridge::CvImage(h, "bgr8", blended_image).toImageMsg());
            
            RCLCPP_INFO(this->get_logger(), "Update cycle complete.");
            std::lock_guard<std::mutex> lock(update_mutex_);
            is_updating_ = false;
        });
    }

    void updateLocalRaycasting(const cv::Mat& raw_heatmap, const nav_msgs::msg::OccupancyGrid& map, const std::vector<RadiationSample>& samples)
    {
        if (raw_heatmap.empty()) return;

        std::lock_guard<std::mutex> lock(data_mutex_);
        if (persistent_occluded_heatmap_.empty() || persistent_occluded_heatmap_.rows != (int)map.info.height || persistent_occluded_heatmap_.cols != (int)map.info.width) {
            persistent_occluded_heatmap_ = cv::Mat::zeros(map.info.height, map.info.width, CV_32F);
        }

        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_.lookupTransform(map_frame_, sensor_frame_, tf2::TimePointZero);
        } catch(const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get robot pose for local update: %s", ex.what());
            return;
        }

        cv::Mat map_image(map.info.height, map.info.width, CV_8UC1);
        for(unsigned int i=0; i<map.data.size(); ++i) { map_image.data[i] = (map.data[i] > 50) ? 255 : 0; }

        std::vector<cv::Point> source_pixels;
        double res = map.info.resolution;
        double origin_x = map.info.origin.position.x;
        double origin_y = map.info.origin.position.y;
        
        int start_index = std::max(0, (int)samples.size() - raycasting_max_sources_);
        for(size_t i = start_index; i < samples.size(); ++i) {
            int px = static_cast<int>((samples[i].x - origin_x) / res);
            int py = static_cast<int>((samples[i].y - origin_y) / res);
            if (px >= 0 && px < map_image.cols && py >= 0 && py < map_image.rows) {
                source_pixels.push_back(cv::Point(px, py));
            }
        }
        if (source_pixels.empty()) return;

        int radius_px = static_cast<int>(raycasting_update_radius_m_ / res);
        int robot_px = static_cast<int>((t.transform.translation.x - origin_x) / res);
        int robot_py = static_cast<int>((t.transform.translation.y - origin_y) / res);
        
        int start_c = std::max(0, robot_px - radius_px);
        int end_c = std::min((int)map.info.width, robot_px + radius_px);
        int start_r = std::max(0, robot_py - radius_px);
        int end_r = std::min((int)map.info.height, robot_py + radius_px);

        cv::parallel_for_(cv::Range(start_r, end_r), [&](const cv::Range& range){
            for (int r = range.start; r < range.end; ++r) {
                float* p_occluded = persistent_occluded_heatmap_.ptr<float>(r);
                const float* p_raw = raw_heatmap.ptr<float>(r);

                for (int c = start_c; c < end_c; ++c) {
                    if (map_image.at<uchar>(r, c) == 255) continue;
                    bool has_line_of_sight = false;
                    for (const auto& source_px : source_pixels) {
                        cv::LineIterator it(map_image, cv::Point(c, r), source_px);
                        bool blocked = false;
                        for(int i = 0; i < it.count; ++i, ++it) {
                            if (map_image.at<uchar>(it.pos()) == 255) {
                                blocked = true;
                                break;
                            }
                        }
                        if (!blocked) { has_line_of_sight = true; break; }
                    }
                    p_occluded[c] = has_line_of_sight ? p_raw[c] : 0.0f;
                }
            }
        });
    }
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    image_transport::Publisher heatmap_pub_;
    rclcpp::Client<rad121_monitor::srv::GPPredict>::SharedPtr gp_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string sensor_frame_, map_frame_;
    double background_threshold_mrem_hr_;
    int raycasting_max_sources_;
    double raycasting_update_radius_m_;
};

cv::Mat createBaseMapImage(const nav_msgs::msg::OccupancyGrid &map) {
    cv::Mat base_gray(map.info.height, map.info.width, CV_8UC1);
    for (unsigned int y = 0; y < map.info.height; ++y) {
        for (unsigned int x = 0; x < map.info.width; ++x) {
            int8_t o = map.data[y * map.info.width + x];
            if (o == -1) base_gray.at<uchar>(y, x) = 205;
            else if (o >= 50) base_gray.at<uchar>(y, x) = 0;
            else base_gray.at<uchar>(y, x) = 255;
        }
    }
    cv::Mat bgr;
    cv::cvtColor(base_gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
}

cv::Mat createColorHeatmapImage(const cv::Mat &field, double threshold) {
    double min_val, max_val;
    cv::minMaxLoc(field, &min_val, &max_val);
    if (max_val < threshold) max_val = threshold + 0.1;

    cv::Mat scaled_field;
    double range = max_val - min_val;
    if (range <= 1e-6) range = 1.0;

    field.convertTo(scaled_field, CV_8UC1, 255.0 / range, -255.0 * min_val / range);
    cv::Mat color_field;
    cv::applyColorMap(scaled_field, color_field, cv::COLORMAP_JET);

    cv::Mat mask;
    cv::compare(field, threshold, mask, cv::CMP_LT);
    color_field.setTo(cv::Scalar(0, 0, 0), mask);
    return color_field;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadiationHeatmapNode>();
    node->init();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}