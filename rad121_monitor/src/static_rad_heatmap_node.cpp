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
#include <map>

struct RadiationSample
{
    double x, y;
    int id;
};
cv::Mat createBaseMapImage(const nav_msgs::msg::OccupancyGrid &map);
cv::Mat createColorHeatmapImage(const cv::Mat &field, double threshold);
// MODIFIED: Updated function prototype
void addLegendToImage(cv::Mat &image, double min_val, double max_val, const std::string &unit_label);

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
        heatmap_pub_ = it.advertise("/rad_heatmap/mean_map", 1);
        std_dev_heatmap_pub_ = it.advertise("/rad_heatmap/std_dev_map", 1);

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
    int next_sample_id_ = 0;

    std::mutex update_mutex_;
    bool is_updating_ = false;

    cv::Mat current_raw_heatmap_;
    cv::Mat current_std_dev_heatmap_;
    std::map<int, cv::Mat> sample_occlusion_masks_;
    cv::Mat map_image_for_raycasting_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            base_map_ = *msg;
            map_received_ = true;
            map_image_for_raycasting_ = cv::Mat(base_map_.info.height, base_map_.info.width, CV_8UC1);
            for (unsigned int i = 0; i < base_map_.data.size(); ++i)
            {
                map_image_for_raycasting_.data[i] = (base_map_.data[i] > 50) ? 255 : 0;
            }
            sample_occlusion_masks_.clear();
        }
        triggerGPUpdate();
    }

    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!map_received_ || msg->data.size() < 2)
        {
            RCLCPP_WARN(this->get_logger(), "Map not received or invalid RAD message.");
            return;
        }

        geometry_msgs::msg::PoseStamped pose_in_map;
        try
        {
            geometry_msgs::msg::PoseStamped pose_in_sensor;
            pose_in_sensor.header.frame_id = sensor_frame_;
            pose_in_sensor.header.stamp = this->get_clock()->now();
            tf_buffer_.transform(pose_in_sensor, pose_in_map, map_frame_, std::chrono::milliseconds(200));

            int current_sample_id;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                current_sample_id = next_sample_id_++;
                collected_samples_.push_back({pose_in_map.pose.position.x, pose_in_map.pose.position.y, current_sample_id});
            }

            RCLCPP_INFO(this->get_logger(), "New radiation sample logged (ID %d). Triggering heatmap update.", current_sample_id);
            triggerGPUpdate();
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        }
    }

    void triggerGPUpdate()
    {
        {
            std::lock_guard<std::mutex> lock(update_mutex_);
            if (is_updating_)
            {
                return;
            }
            is_updating_ = true;
        }

        nav_msgs::msg::OccupancyGrid map_copy;
        std::vector<RadiationSample> samples_copy;
        cv::Mat map_image_raycasting_copy;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!map_received_ || collected_samples_.empty())
            {
                std::lock_guard<std::mutex> update_lock(update_mutex_);
                is_updating_ = false;
                RCLCPP_WARN(this->get_logger(), "Map not received or no samples, cannot update.");
                return;
            }
            map_copy = base_map_;
            samples_copy = collected_samples_;
            map_image_raycasting_copy = map_image_for_raycasting_.clone();
        }

        auto request = std::make_shared<rad121_monitor::srv::GPPredict::Request>();
        std::vector<std::pair<int, int>> valid_pixel_coords;
        double res = map_copy.info.resolution;
        double origin_x = map_copy.info.origin.position.x;
        double origin_y = map_copy.info.origin.position.y;

        for (unsigned int y = 0; y < map_copy.info.height; ++y)
        {
            for (unsigned int x = 0; x < map_copy.info.width; ++x)
            {
                if (map_copy.data[y * map_copy.info.width + x] < 50)
                {
                    geometry_msgs::msg::Point p;
                    p.x = origin_x + (x + 0.5) * res;
                    p.y = origin_y + (y + 0.5) * res;
                    request->points_to_predict.push_back(p);
                    valid_pixel_coords.push_back({(int)x, (int)y});
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Sending %zu points to GP service.", request->points_to_predict.size());

        gp_client_->async_send_request(request, [this, map_copy, samples_copy, valid_pixel_coords, map_image_raycasting_copy](rclcpp::Client<rad121_monitor::srv::GPPredict>::SharedFuture future)
                                       {
            auto response = future.get();
            if (!response || response->predicted_mean.size() != valid_pixel_coords.size() || response->predicted_std_dev.size() != valid_pixel_coords.size()) {
                RCLCPP_ERROR(this->get_logger(), "GP service response is invalid or size mismatch.");
                std::lock_guard<std::mutex> lock(update_mutex_);
                is_updating_ = false;
                return;
            }

            cv::Mat new_raw_heatmap = cv::Mat::zeros(map_copy.info.height, map_copy.info.width, CV_32F);
            cv::Mat new_std_dev_heatmap = cv::Mat::zeros(map_copy.info.height, map_copy.info.width, CV_32F);
            for (size_t i = 0; i < valid_pixel_coords.size(); ++i) {
                new_raw_heatmap.at<float>(valid_pixel_coords[i].second, valid_pixel_coords[i].first) = response->predicted_mean[i];
                new_std_dev_heatmap.at<float>(valid_pixel_coords[i].second, valid_pixel_coords[i].first) = response->predicted_std_dev[i];
            }
            
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                current_raw_heatmap_ = new_raw_heatmap;
                current_std_dev_heatmap_ = new_std_dev_heatmap;
            }

            for (const auto& sample : samples_copy) {
                bool is_new_sample;
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    is_new_sample = (sample_occlusion_masks_.find(sample.id) == sample_occlusion_masks_.end());
                }
                
                if (is_new_sample) {
                    cv::Mat single_sample_occlusion_mask = 
                        generateSingleSampleOcclusionMask(sample, map_image_raycasting_copy, map_copy.info.resolution, map_copy.info.origin.position.x, map_copy.info.origin.position.y);
                    {
                        std::lock_guard<std::mutex> lock(data_mutex_);
                        sample_occlusion_masks_[sample.id] = single_sample_occlusion_mask;
                    }
                }
            }
            
            cv::Mat base_bgr = createBaseMapImage(map_copy);

            // --- Generate and Publish MEAN Heatmap ---
            cv::Mat final_occluded_heatmap = combineOcclusionMasks(current_raw_heatmap_, samples_copy, sample_occlusion_masks_, map_image_raycasting_copy);
            cv::Mat color_heatmap = createColorHeatmapImage(final_occluded_heatmap, background_threshold_mrem_hr_);
            cv::Mat blended_image;
            cv::addWeighted(base_bgr, 0.6, color_heatmap, 0.4, 0.0, blended_image);

            double min_val, max_val;
            cv::minMaxLoc(final_occluded_heatmap, &min_val, &max_val);
            if (max_val < background_threshold_mrem_hr_) max_val = background_threshold_mrem_hr_ + 0.1;
            if (min_val >= max_val) min_val = max_val - 0.1;
            addLegendToImage(blended_image, min_val, max_val, "mrem/hr");

            std_msgs::msg::Header h;
            h.stamp = this->now();
            h.frame_id = map_frame_;
            heatmap_pub_.publish(*cv_bridge::CvImage(h, "bgr8", blended_image).toImageMsg());
            
            // --- Generate and Publish STD_DEV Heatmap ---
            {
                cv::Mat std_dev_color_heatmap = createColorHeatmapImage(current_std_dev_heatmap_, 0.0);
                cv::Mat std_dev_blended_image;
                cv::addWeighted(base_bgr, 0.6, std_dev_color_heatmap, 0.4, 0.0, std_dev_blended_image);

                double std_dev_min, std_dev_max;
                cv::minMaxLoc(current_std_dev_heatmap_, &std_dev_min, &std_dev_max);
                if (std_dev_max <= std_dev_min) std_dev_max = std_dev_min + 0.1;

                addLegendToImage(std_dev_blended_image, std_dev_min, std_dev_max, "Std. Dev.");
                
                std_msgs::msg::Header std_dev_header;
                std_dev_header.stamp = this->now();
                std_dev_header.frame_id = map_frame_;
                std_dev_heatmap_pub_.publish(*cv_bridge::CvImage(std_dev_header, "bgr8", std_dev_blended_image).toImageMsg());
            }

            RCLCPP_INFO(this->get_logger(), "Update cycle complete.");
            std::lock_guard<std::mutex> lock(update_mutex_);
            is_updating_ = false; });
    }

    cv::Mat generateSingleSampleOcclusionMask(const RadiationSample &sample, const cv::Mat &map_image, double res, double origin_x, double origin_y)
    {
        cv::Mat mask = cv::Mat::zeros(map_image.size(), CV_8UC1);

        int sample_px = static_cast<int>((sample.x - origin_x) / res);
        int sample_py = static_cast<int>((sample.y - origin_y) / res);

        if (sample_px < 0 || sample_px >= map_image.cols || sample_py < 0 || sample_py >= map_image.rows)
        {
            RCLCPP_WARN(this->get_logger(), "Sample (%.2f, %.2f) is outside map boundaries, cannot generate occlusion mask.", sample.x, sample.y);
            return mask;
        }

        cv::parallel_for_(cv::Range(0, map_image.rows), [&](const cv::Range &range)
                          {
            for (int r = range.start; r < range.end; ++r) {
                for (int c = 0; c < map_image.cols; ++c) {
                    if (map_image.at<uchar>(r, c) == 255) continue;

                    cv::LineIterator it(map_image, cv::Point(c, r), cv::Point(sample_px, sample_py));
                    bool blocked = false;
                    for(int i = 0; i < it.count; ++i, ++it) {
                        if (map_image.at<uchar>(it.pos()) == 255) {
                            blocked = true;
                            break;
                        }
                    }
                    if (!blocked) {
                        mask.at<uchar>(r, c) = 255;
                    }
                }
            } });
        return mask;
    }

    cv::Mat combineOcclusionMasks(const cv::Mat &raw_heatmap, const std::vector<RadiationSample> &samples, const std::map<int, cv::Mat> &occlusion_masks, const cv::Mat &map_image)
    {
        cv::Mat final_occluded_heatmap = cv::Mat::zeros(raw_heatmap.size(), CV_32F);

        cv::parallel_for_(cv::Range(0, raw_heatmap.rows), [&](const cv::Range &range)
                          {
            for (int r = range.start; r < range.end; ++r) {
                for (int c = 0; c < raw_heatmap.cols; ++c) {
                    if (map_image.at<uchar>(r, c) == 255) continue;

                    float max_visible_rad = 0.0f;
                    
                    for (const auto& sample : samples) {
                        auto it = occlusion_masks.find(sample.id);
                        if (it != occlusion_masks.end()) {
                            const cv::Mat& mask = it->second;
                            if (mask.at<uchar>(r, c) == 255) { 
                                max_visible_rad = std::max(max_visible_rad, raw_heatmap.at<float>(r, c));
                            }
                        }
                    }
                    final_occluded_heatmap.at<float>(r, c) = max_visible_rad;
                }
            } });

        return final_occluded_heatmap;
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    image_transport::Publisher heatmap_pub_;
    image_transport::Publisher std_dev_heatmap_pub_;
    rclcpp::Client<rad121_monitor::srv::GPPredict>::SharedPtr gp_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string sensor_frame_, map_frame_;
    double background_threshold_mrem_hr_;
    int raycasting_max_sources_;
    double raycasting_update_radius_m_;
};

cv::Mat createBaseMapImage(const nav_msgs::msg::OccupancyGrid &map)
{
    cv::Mat base_gray(map.info.height, map.info.width, CV_8UC1);
    for (unsigned int y = 0; y < map.info.height; ++y)
    {
        for (unsigned int x = 0; x < map.info.width; ++x)
        {
            int8_t o = map.data[y * map.info.width + x];
            if (o == -1)
                base_gray.at<uchar>(y, x) = 205;
            else if (o >= 50)
                base_gray.at<uchar>(y, x) = 0;
            else
                base_gray.at<uchar>(y, x) = 255;
        }
    }
    cv::Mat bgr;
    cv::cvtColor(base_gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
}

cv::Mat createColorHeatmapImage(const cv::Mat &field, double threshold)
{
    double min_val, max_val;
    cv::minMaxLoc(field, &min_val, &max_val);
    if (max_val < threshold)
        max_val = threshold + 0.1;

    cv::Mat scaled_field;
    double range = max_val - min_val;
    if (range <= 1e-6)
        range = 1.0;

    field.convertTo(scaled_field, CV_8UC1, 255.0 / range, -255.0 * min_val / range);
    cv::Mat color_field;
    cv::applyColorMap(scaled_field, color_field, cv::COLORMAP_JET);

    cv::Mat mask;
    cv::compare(field, threshold, mask, cv::CMP_LT);
    color_field.setTo(cv::Scalar(0, 0, 0), mask);
    return color_field;
}

void addLegendToImage(cv::Mat &image, double min_val, double max_val, const std::string &unit_label)
{
    int border_width = 150;
    cv::copyMakeBorder(image, image, 0, 0, 0, border_width, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

    int legend_width = 40;
    int legend_height = image.rows / 2;
    int legend_x = image.cols - border_width + 30;
    int legend_y = image.rows / 4;

    cv::Mat legend_bar(legend_height, legend_width, CV_8UC3);
    for (int r = 0; r < legend_height; ++r)
    {
        double ratio = (double)(legend_height - 1 - r) / (legend_height - 1);
        uchar val = static_cast<uchar>(ratio * 255.0);

        cv::Mat color_pixel(1, 1, CV_8UC1, val);
        cv::Mat mapped_color;
        cv::applyColorMap(color_pixel, mapped_color, cv::COLORMAP_JET);
        cv::line(legend_bar, cv::Point(0, r), cv::Point(legend_width, r), mapped_color.at<cv::Vec3b>(0, 0), 2);
    }

    legend_bar.copyTo(image(cv::Rect(legend_x, legend_y, legend_width, legend_height)));

    int text_x = legend_x + legend_width + 10;
    cv::putText(image, unit_label, cv::Point(legend_x, legend_y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
    cv::putText(image, cv::format("%.2f", max_val), cv::Point(text_x, legend_y + 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
    cv::putText(image, cv::format("%.2f", min_val), cv::Point(text_x, legend_y + legend_height),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
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