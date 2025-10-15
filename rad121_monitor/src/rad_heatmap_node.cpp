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
#include "rad121_monitor/srv/save_maps.hpp"

#include <vector>
#include <mutex>
#include <deque>
#include <thread>
#include <future>
#include <opencv2/opencv.hpp>
#include <map>
#include <filesystem>
#include <queue>

struct RadiationSample
{
    double x, y, value;
    int id;
};

// Forward declarations for helper functions
cv::Mat createBaseMapImage(const nav_msgs::msg::OccupancyGrid &map);
void addLegendToImage(cv::Mat &image, double min_val, double max_val, const std::string &unit_label);

class DynamicRadiationHeatmapNode : public rclcpp::Node
{
public:
    DynamicRadiationHeatmapNode() : Node("dynamic_rad_heatmap_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {}

    void init()
    {
        // Declare and retrieve parameters for node configuration
        this->declare_parameter<std::string>("sensor_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<double>("background_threshold_mrem_hr", 0.1);
        this->declare_parameter<int>("recent_samples_window_size", 20);
        this->declare_parameter<std::string>("output_directory", "/tmp");
        this->declare_parameter<int>("raycasting_max_sources", 5);
        this->declare_parameter<double>("raycasting_max_distance_m", 10.0);

        sensor_frame_ = this->get_parameter("sensor_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        background_threshold_mrem_hr_ = this->get_parameter("background_threshold_mrem_hr").as_double();
        recent_samples_window_size_ = this->get_parameter("recent_samples_window_size").as_int();
        output_directory_ = this->get_parameter("output_directory").as_string();
        raycasting_max_sources_ = this->get_parameter("raycasting_max_sources").as_int();
        raycasting_max_distance_m_ = this->get_parameter("raycasting_max_distance_m").as_double();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), std::bind(&DynamicRadiationHeatmapNode::mapCallback, this, std::placeholders::_1));
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/rad", 10, std::bind(&DynamicRadiationHeatmapNode::radCallback, this, std::placeholders::_1));

        // Setup publishers for four different heatmap visualizations
        image_transport::ImageTransport it(this->shared_from_this());
        historical_heatmap_pub_ = it.advertise("/rad_heatmap/historical_mean", 1);
        instantaneous_heatmap_pub_ = it.advertise("/rad_heatmap/instantaneous_mean", 1);
        historical_std_dev_pub_ = it.advertise("/rad_heatmap/historical_std_dev", 1);
        instantaneous_std_dev_pub_ = it.advertise("/rad_heatmap/instantaneous_std_dev", 1);

        gp_client_ = this->create_client<rad121_monitor::srv::GPPredict>("predict_gp");

        // Service to allow external nodes to request saving the generated maps
        save_maps_service_ = this->create_service<rad121_monitor::srv::SaveMaps>(
            "save_rad_maps",
            std::bind(&DynamicRadiationHeatmapNode::saveMapsCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node initialized. Saving maps to '%s'", output_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "Waiting for GP service 'predict_gp'...");
        if (!gp_client_->wait_for_service(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "GP service not available after waiting. Exiting.");
            if (rclcpp::ok())
            {
                rclcpp::shutdown();
            }
            return;
        }
        RCLCPP_INFO(this->get_logger(), "GP service is available. Node is ready.");
    }

private:
    // --- Member Variables ---

    // Mutexes for thread-safe access to shared data
    std::mutex data_mutex_;
    std::mutex update_mutex_;

    // Node state flags
    bool map_received_ = false;
    bool is_updating_ = false;
    int next_sample_id_ = 0;

    // Data Storage
    nav_msgs::msg::OccupancyGrid base_map_;
    std::vector<RadiationSample> historical_samples_;
    std::deque<RadiationSample> recent_samples_;

    // OpenCV Mats for image processing and storage
    cv::Mat map_image_for_raycasting_;
    cv::Mat latest_historical_mean_image_;
    cv::Mat latest_historical_std_dev_image_;

    // ROS Communications
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Service<rad121_monitor::srv::SaveMaps>::SharedPtr save_maps_service_;
    image_transport::Publisher historical_heatmap_pub_, instantaneous_heatmap_pub_;
    image_transport::Publisher historical_std_dev_pub_, instantaneous_std_dev_pub_;
    rclcpp::Client<rad121_monitor::srv::GPPredict>::SharedPtr gp_client_;

    // TF2 for coordinate transformations
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Parameters
    std::string sensor_frame_, map_frame_, output_directory_;
    double background_threshold_mrem_hr_;
    size_t recent_samples_window_size_;
    int raycasting_max_sources_;
    double raycasting_max_distance_m_;

    // --- Service and Topic Callbacks ---

    void saveMapsCallback(const std::shared_ptr<rad121_monitor::srv::SaveMaps::Request> request,
                          std::shared_ptr<rad121_monitor::srv::SaveMaps::Response> response)
    {
        cv::Mat mean_map_copy, std_dev_map_copy;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (latest_historical_mean_image_.empty() || latest_historical_std_dev_image_.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Save request received, but no historical maps have been generated yet.");
                response->success = false;
                response->message = "No historical heatmaps have been generated yet.";
                return;
            }
            mean_map_copy = latest_historical_mean_image_.clone();
            std_dev_map_copy = latest_historical_std_dev_image_.clone();
        }

        std::filesystem::path dir(output_directory_);
        std::filesystem::path mean_path = dir / (request->basename + "_historical_mean.png");
        std::filesystem::path std_dev_path = dir / (request->basename + "_historical_std_dev.png");
        std::filesystem::create_directories(dir);

        bool success1 = cv::imwrite(mean_path.string(), mean_map_copy);
        bool success2 = cv::imwrite(std_dev_path.string(), std_dev_map_copy);

        if (success1 && success2)
        {
            response->success = true;
            response->message = "Saved maps to " + mean_path.string() + " and " + std_dev_path.string();
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        }
        else
        {
            response->success = false;
            response->message = "Failed to save one or both maps.";
            RCLCPP_ERROR(this->get_logger(), "Failed to save maps. Mean success: %d, Std Dev success: %d", success1, success2);
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        base_map_ = *msg;
        map_received_ = true;
        // Pre-process the map into a binary image for faster raycasting
        map_image_for_raycasting_ = cv::Mat(base_map_.info.height, base_map_.info.width, CV_8UC1);
        for (unsigned int i = 0; i < base_map_.data.size(); ++i)
        {
            map_image_for_raycasting_.data[i] = (base_map_.data[i] > 50) ? 255 : 0; // 255=Wall, 0=Free
        }
        RCLCPP_INFO(this->get_logger(), "Map has been updated and stored.");
    }

    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!map_received_ || msg->data.size() < 2)
            return;

        try
        {
            geometry_msgs::msg::PoseStamped pose_in_sensor, pose_in_map;
            pose_in_sensor.header.frame_id = sensor_frame_;
            pose_in_sensor.header.stamp = this->get_clock()->now();
            tf_buffer_.transform(pose_in_sensor, pose_in_map, map_frame_, std::chrono::milliseconds(500));

            { // Scope for the lock_guard
                std::lock_guard<std::mutex> lock(data_mutex_);
                RadiationSample sample = {pose_in_map.pose.position.x, pose_in_map.pose.position.y, msg->data[1], next_sample_id_++};
                historical_samples_.push_back(sample);
                recent_samples_.push_back(sample);
                if (recent_samples_.size() > recent_samples_window_size_)
                {
                    recent_samples_.pop_front();
                }
            }
            RCLCPP_DEBUG(this->get_logger(), "New radiation sample logged. Triggering GP update.");
            triggerGPUpdate();
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF Error for logging sample: %s", ex.what());
        }
    }

    // --- Core Logic ---

    void triggerGPUpdate()
    {
        { // Lock scope to prevent concurrent updates
            std::lock_guard<std::mutex> lock(update_mutex_);
            if (is_updating_)
            {
                RCLCPP_INFO(this->get_logger(), "Update already in progress. Skipping trigger.");
                return;
            }
            is_updating_ = true;
        }

        nav_msgs::msg::OccupancyGrid map_copy;
        cv::Mat raycasting_map_copy;
        std::vector<RadiationSample> hist_samples_copy, inst_samples_copy;
        bool can_update = false;

        { // Lock scope to safely copy shared data
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (map_received_ && !historical_samples_.empty())
            {
                can_update = true;
                map_copy = base_map_;
                raycasting_map_copy = map_image_for_raycasting_.clone();
                hist_samples_copy = historical_samples_;
                inst_samples_copy.assign(recent_samples_.begin(), recent_samples_.end());
            }
        }

        if (!can_update)
        {
            std::lock_guard<std::mutex> lock(update_mutex_);
            is_updating_ = false;
            RCLCPP_WARN(this->get_logger(), "Map not received or no samples, cannot update.");
            return;
        }

        auto request_hist = std::make_shared<rad121_monitor::srv::GPPredict::Request>();
        for (const auto &s : hist_samples_copy)
        {
            request_hist->observed_x.push_back(s.x);
            request_hist->observed_y.push_back(s.y);
            request_hist->observed_values_mean.push_back(s.value);
        }

        auto request_inst = std::make_shared<rad121_monitor::srv::GPPredict::Request>();
        for (const auto &s : inst_samples_copy)
        {
            request_inst->observed_x.push_back(s.x);
            request_inst->observed_y.push_back(s.y);
            request_inst->observed_values_mean.push_back(s.value);
        }

        std::vector<geometry_msgs::msg::Point> points_to_predict;
        std::vector<std::pair<int, int>> valid_pixel_coords;
        double res = map_copy.info.resolution;
        double origin_x = map_copy.info.origin.position.x;
        double origin_y = map_copy.info.origin.position.y;
        for (unsigned int y = 0; y < map_copy.info.height; ++y)
        {
            for (unsigned int x = 0; x < map_copy.info.width; ++x)
            {
                // Predict only for free space (value < 50)
                if (map_copy.data[y * map_copy.info.width + x] < 50 && map_copy.data[y * map_copy.info.width + x] >= 0)
                {
                    geometry_msgs::msg::Point p;
                    p.x = origin_x + (x + 0.5) * res;
                    p.y = origin_y + (y + 0.5) * res;
                    points_to_predict.push_back(p);
                    valid_pixel_coords.push_back({(int)x, (int)y});
                }
            }
        }
        request_hist->points_to_predict = points_to_predict;
        request_inst->points_to_predict = points_to_predict;

        RCLCPP_INFO(this->get_logger(), "Sending requests: Historical (%zu samples), Instantaneous (%zu samples). Predicting for %zu points.",
                    hist_samples_copy.size(), inst_samples_copy.size(), valid_pixel_coords.size());

        auto future_hist = gp_client_->async_send_request(request_hist);
        auto future_inst = gp_client_->async_send_request(request_inst);

        std::thread([this, future_hist = std::move(future_hist), future_inst = std::move(future_inst),
                     map_copy, raycasting_map_copy, valid_pixel_coords, hist_samples_copy, inst_samples_copy]() mutable
                    {
                        try
                        {
                            auto response_hist = future_hist.get();
                            auto response_inst = future_inst.get();

                            cv::Mat hist_mean_raw, hist_std_raw, inst_mean_raw, inst_std_raw;

                            if (response_hist && response_hist->predicted_mean.size() == valid_pixel_coords.size())
                            {
                                hist_mean_raw = cv::Mat::zeros(map_copy.info.height, map_copy.info.width, CV_32F);
                                hist_std_raw = cv::Mat::zeros(map_copy.info.height, map_copy.info.width, CV_32F);
                                for (size_t i = 0; i < valid_pixel_coords.size(); ++i)
                                {
                                    hist_mean_raw.at<float>(valid_pixel_coords[i].second, valid_pixel_coords[i].first) = response_hist->predicted_mean[i];
                                    hist_std_raw.at<float>(valid_pixel_coords[i].second, valid_pixel_coords[i].first) = response_hist->predicted_std_dev[i];
                                }
                            } else { /* Error handling */ }

                            if (response_inst && response_inst->predicted_mean.size() == valid_pixel_coords.size())
                            {
                                inst_mean_raw = cv::Mat::zeros(map_copy.info.height, map_copy.info.width, CV_32F);
                                inst_std_raw = cv::Mat::zeros(map_copy.info.height, map_copy.info.width, CV_32F);
                                for (size_t i = 0; i < valid_pixel_coords.size(); ++i)
                                {
                                    inst_mean_raw.at<float>(valid_pixel_coords[i].second, valid_pixel_coords[i].first) = response_inst->predicted_mean[i];
                                    inst_std_raw.at<float>(valid_pixel_coords[i].second, valid_pixel_coords[i].first) = response_inst->predicted_std_dev[i];
                                }
                            } else { /* Error handling */ }

                            RCLCPP_INFO(this->get_logger(), "Both GP responses received. Applying occlusion...");

                            cv::Mat hist_mean_occluded = applyOcclusionFloodFill(hist_mean_raw, hist_samples_copy, map_copy, raycasting_map_copy);
                            cv::Mat inst_mean_occluded = applyOcclusionFloodFill(inst_mean_raw, inst_samples_copy, map_copy, raycasting_map_copy);
                            cv::Mat hist_std_occluded = applyOcclusionFloodFill(hist_std_raw, hist_samples_copy, map_copy, raycasting_map_copy);
                            cv::Mat inst_std_occluded = applyOcclusionFloodFill(inst_std_raw, inst_samples_copy, map_copy, raycasting_map_copy);

                            publishHeatmaps(hist_mean_occluded, inst_mean_occluded, hist_std_occluded, inst_std_occluded, map_copy);
                        }
                        catch (const std::exception &e)
                        {
                            RCLCPP_ERROR(this->get_logger(), "Exception caught in GP update thread: %s", e.what());
                        }

                        std::lock_guard<std::mutex> lock(update_mutex_);
                        is_updating_ = false;
                        RCLCPP_INFO(this->get_logger(), "Update cycle complete."); })
            .detach();
    }

    // --- Helper and Visualization Functions ---

    cv::Mat applyOcclusionFloodFill(const cv::Mat &raw_heatmap, const std::vector<RadiationSample> &sources,
                                    const nav_msgs::msg::OccupancyGrid &map_info, const cv::Mat &raycasting_map)
    {
        if (raw_heatmap.empty() || sources.empty())
            return cv::Mat::zeros(raw_heatmap.size(), raw_heatmap.type());

        cv::Mat occluded_heatmap = cv::Mat::zeros(raw_heatmap.size(), raw_heatmap.type());
        cv::Mat visible_mask = cv::Mat::zeros(raw_heatmap.size(), CV_8U);

        std::vector<RadiationSample> effective_sources;
        if (sources.size() > static_cast<size_t>(raycasting_max_sources_))
        {
            effective_sources.assign(sources.end() - raycasting_max_sources_, sources.end());
        }
        else
        {
            effective_sources = sources;
        }

        std::vector<cv::Point> source_pixels;
        double res = map_info.info.resolution;
        double origin_x = map_info.info.origin.position.x;
        double origin_y = map_info.info.origin.position.y;
        for (const auto &source : effective_sources)
        {
            int px = static_cast<int>((source.x - origin_x) / res);
            int py = static_cast<int>((source.y - origin_y) / res);
            if (px >= 0 && px < raycasting_map.cols && py >= 0 && py < raycasting_map.rows)
            {
                source_pixels.push_back(cv::Point(px, py));
            }
        }
        if (source_pixels.empty())
            return occluded_heatmap;

        for (const auto &start_node : source_pixels)
        {
            std::queue<cv::Point> q;
            cv::Mat visited = cv::Mat::zeros(raw_heatmap.size(), CV_8U);

            q.push(start_node);
            visited.at<uchar>(start_node) = 1;

            while (!q.empty())
            {
                cv::Point current = q.front();
                q.pop();
                visible_mask.at<uchar>(current) = 255;

                const cv::Point neighbors[] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
                for (const auto &offset : neighbors)
                {
                    cv::Point next = current + offset;

                    if (next.x < 0 || next.x >= raycasting_map.cols || next.y < 0 || next.y >= raycasting_map.rows)
                        continue;
                    if (visited.at<uchar>(next) == 1)
                        continue;
                    if (raycasting_map.at<uchar>(next) == 255)
                        continue;

                    double dist_m = cv::norm(next - start_node) * res;
                    if (dist_m > raycasting_max_distance_m_)
                        continue;

                    visited.at<uchar>(next) = 1;
                    q.push(next);
                }
            }
        }

        raw_heatmap.copyTo(occluded_heatmap, visible_mask);
        return occluded_heatmap;
    }

    void publishHeatmaps(const cv::Mat &hist_mean, const cv::Mat &inst_mean, const cv::Mat &hist_std, const cv::Mat &inst_std, const nav_msgs::msg::OccupancyGrid &map_copy)
    {
        cv::Mat base_bgr = createBaseMapImage(map_copy);
        std_msgs::msg::Header h;
        h.stamp = this->now();
        h.frame_id = map_frame_;

        // NEW: Create a mask of valid (free) space from the occupancy grid.
        // This mask will be used to ensure the heatmap is only drawn inside the map boundaries.
        cv::Mat free_space_mask(map_copy.info.height, map_copy.info.width, CV_8UC1);
        for (size_t i = 0; i < map_copy.data.size(); ++i)
        {
            // A pixel is valid if it's known free space (value between 0 and 50).
            free_space_mask.data[i] = (map_copy.data[i] >= 0 && map_copy.data[i] < 50) ? 255 : 0;
        }

        // Pass the new mask to each blending function
        auto blended_hist_mean = createBlendedMap(base_bgr, hist_mean, "mrem/hr", background_threshold_mrem_hr_, free_space_mask);
        if (!blended_hist_mean.empty())
        {
            historical_heatmap_pub_.publish(*cv_bridge::CvImage(h, "bgr8", blended_hist_mean).toImageMsg());
        }

        auto blended_hist_std = createBlendedMap(base_bgr, hist_std, "Std. Dev.", 0.0, free_space_mask);
        if (!blended_hist_std.empty())
        {
            historical_std_dev_pub_.publish(*cv_bridge::CvImage(h, "bgr8", blended_hist_std).toImageMsg());
        }

        auto blended_inst_mean = createBlendedMap(base_bgr, inst_mean, "mrem/hr", background_threshold_mrem_hr_, free_space_mask);
        if (!blended_inst_mean.empty())
        {
            instantaneous_heatmap_pub_.publish(*cv_bridge::CvImage(h, "bgr8", blended_inst_mean).toImageMsg());
        }

        auto blended_inst_std = createBlendedMap(base_bgr, inst_std, "Std. Dev.", 0.0, free_space_mask);
        if (!blended_inst_std.empty())
        {
            instantaneous_std_dev_pub_.publish(*cv_bridge::CvImage(h, "bgr8", blended_inst_std).toImageMsg());
        }

        // Store historical maps for the save service
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!blended_hist_mean.empty())
                latest_historical_mean_image_ = blended_hist_mean.clone();
            if (!blended_hist_std.empty())
                latest_historical_std_dev_image_ = blended_hist_std.clone();
        }
    }

    // MODIFIED: Added a const cv::Mat& mask parameter
    cv::Mat createBlendedMap(const cv::Mat &base_img, const cv::Mat &data, const std::string &unit, double threshold, const cv::Mat &mask)
    {
        if (data.empty())
            return {};

        double min_val, max_val;
        cv::minMaxLoc(data, &min_val, &max_val);

        if (max_val < threshold)
            max_val = threshold + 0.1;
        if (max_val - min_val < 1e-6)
            max_val = min_val + 0.1;

        cv::Mat color_heatmap = createColorHeatmapImage(data, min_val, max_val, threshold);

        // NEW: Zero out the color heatmap in areas that are not free space
        color_heatmap.setTo(cv::Scalar(0, 0, 0), ~mask);

        cv::Mat blended_image;
        cv::addWeighted(base_img, 0.6, color_heatmap, 0.4, 0.0, blended_image);
        addLegendToImage(blended_image, min_val, max_val, unit);

        return blended_image;
    }

    cv::Mat createColorHeatmapImage(const cv::Mat &field, double min_val, double max_val, double threshold)
    {
        cv::Mat scaled_field;
        double range = max_val - min_val;
        if (range <= 1e-6)
            range = 1.0;

        field.convertTo(scaled_field, CV_8UC1, 255.0 / range, -255.0 * min_val / range);

        cv::Mat color_field;
        cv::applyColorMap(scaled_field, color_field, cv::COLORMAP_JET);

        cv::Mat value_mask;
        cv::compare(field, threshold, value_mask, cv::CMP_LT);
        color_field.setTo(cv::Scalar(0, 0, 0, 0), value_mask); // Keep low values transparent
        return color_field;
    }
};

// --- Helper Function Implementations ---

cv::Mat createBaseMapImage(const nav_msgs::msg::OccupancyGrid &map)
{
    cv::Mat base_gray(map.info.height, map.info.width, CV_8UC1);
    for (unsigned int y = 0; y < map.info.height; ++y)
    {
        for (unsigned int x = 0; x < map.info.width; ++x)
        {
            int8_t o = map.data[y * map.info.width + x];
            if (o == -1)
                base_gray.at<uchar>(y, x) = 205; // Unknown
            else if (o >= 50)
                base_gray.at<uchar>(y, x) = 0; // Obstacle
            else
                base_gray.at<uchar>(y, x) = 255; // Free
        }
    }
    cv::Mat bgr;
    cv::cvtColor(base_gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
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
        double val_at_ratio = min_val + ratio * (max_val - min_val);

        cv::Mat temp_field(1, 1, CV_32FC1, (float)val_at_ratio);
        cv::Mat scaled_pixel;
        double range = max_val - min_val;
        if (range <= 1e-6)
            range = 1.0;
        temp_field.convertTo(scaled_pixel, CV_8UC1, 255.0 / range, -255.0 * min_val / range);

        cv::Mat mapped_color;
        cv::applyColorMap(scaled_pixel, mapped_color, cv::COLORMAP_JET);
        cv::line(legend_bar, cv::Point(0, r), cv::Point(legend_width, r), mapped_color.at<cv::Vec3b>(0, 0), 2);
    }

    legend_bar.copyTo(image(cv::Rect(legend_x, legend_y, legend_width, legend_height)));

    int text_x = legend_x + legend_width + 10;
    cv::putText(image, unit_label, cv::Point(legend_x, legend_y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
    cv::putText(image, cv::format("%.2f", max_val), cv::Point(text_x, legend_y + 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
    cv::putText(image, cv::format("%.2f", min_val), cv::Point(text_x, legend_y + legend_height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicRadiationHeatmapNode>();
    node->init();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}