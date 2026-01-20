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
#include "ros2_rad_map/srv/gp_predict.hpp"
#include "ros2_rad_map/srv/save_maps.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <vector>
#include <mutex>
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

cv::Mat createBaseMapImage(const nav_msgs::msg::OccupancyGrid &map);
void addLegendToImage(cv::Mat &image, double min_val, double max_val, const std::string &unit_label);

class DynamicRadiationHeatmapNode : public rclcpp::Node
{
public:
    DynamicRadiationHeatmapNode() : Node("dynamic_rad_heatmap_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {}

    void init()
    {
        this->declare_parameter<std::string>("sensor_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<double>("background_threshold_mrem_hr", 0.1);
        this->declare_parameter<std::string>("output_directory", "/tmp");
        this->declare_parameter<int>("raycasting_max_sources", 5);
        this->declare_parameter<double>("raycasting_max_distance_m", 10.0);

        sensor_frame_ = this->get_parameter("sensor_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        background_threshold_mrem_hr_ = this->get_parameter("background_threshold_mrem_hr").as_double();
        output_directory_ = this->get_parameter("output_directory").as_string();
        raycasting_max_sources_ = this->get_parameter("raycasting_max_sources").as_int();
        raycasting_max_distance_m_ = this->get_parameter("raycasting_max_distance_m").as_double();

        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local().reliable();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, std::bind(&DynamicRadiationHeatmapNode::mapCallback, this, std::placeholders::_1));
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/rad", 10, std::bind(&DynamicRadiationHeatmapNode::radCallback, this, std::placeholders::_1));
        zone_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/radiation_zone_markers", 10,
            std::bind(&DynamicRadiationHeatmapNode::zoneCallback, this, std::placeholders::_1));

        image_transport::ImageTransport it(this->shared_from_this());
        historical_heatmap_pub_ = it.advertise("/rad_heatmap/historical_mean", 1);
        historical_std_dev_pub_ = it.advertise("/rad_heatmap/historical_std_dev", 1);
        raw_mean_map_pub_ = it.advertise("/rad_heatmap/raw_mean_map", 1);

        gp_client_ = this->create_client<ros2_rad_map::srv::GPPredict>("predict_gp");

        save_maps_service_ = this->create_service<ros2_rad_map::srv::SaveMaps>(
            "save_rad_maps",
            std::bind(&DynamicRadiationHeatmapNode::saveMapsCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Heatmap Node initialized. Waiting for GP service...");
        if (!gp_client_->wait_for_service(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "GP service not available. Exiting.");
            if (rclcpp::ok()) { rclcpp::shutdown(); }
            return;
        }
        RCLCPP_INFO(this->get_logger(), "GP service available. Node is ready.");
    }

private:
    std::mutex data_mutex_;
    std::mutex update_mutex_;
    std::mutex zone_mutex_;

    bool map_received_ = false;
    bool is_updating_ = false;
    int next_sample_id_ = 0;

    nav_msgs::msg::OccupancyGrid base_map_;
    std::vector<RadiationSample> historical_samples_;
    std::vector<std::vector<cv::Point>> zone_contours_;

    cv::Mat map_image_for_raycasting_;
    cv::Mat latest_historical_mean_image_;
    cv::Mat latest_historical_std_dev_image_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr zone_sub_;
    rclcpp::Service<ros2_rad_map::srv::SaveMaps>::SharedPtr save_maps_service_;
    image_transport::Publisher historical_heatmap_pub_;
    image_transport::Publisher historical_std_dev_pub_;
    image_transport::Publisher raw_mean_map_pub_;
    rclcpp::Client<ros2_rad_map::srv::GPPredict>::SharedPtr gp_client_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string sensor_frame_, map_frame_, output_directory_;
    double background_threshold_mrem_hr_;
    int raycasting_max_sources_;
    double raycasting_max_distance_m_;

    void zoneCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(zone_mutex_);
        zone_contours_.clear();

        if (!map_received_) {
            return;
        }

        double res = base_map_.info.resolution;
        double origin_x = base_map_.info.origin.position.x;
        double origin_y = base_map_.info.origin.position.y;
        int map_height = base_map_.info.height;

        for (const auto& marker : msg->markers) {
            if (marker.ns == "zone_boundaries" && marker.type == visualization_msgs::msg::Marker::LINE_STRIP) {
                std::vector<cv::Point> contour;
                for (const auto& world_point : marker.points) {
                    int px = static_cast<int>((world_point.x - origin_x) / res);
                    int py = map_height - 1 - static_cast<int>((world_point.y - origin_y) / res);
                    contour.push_back(cv::Point(px, py));
                }
                if (!contour.empty()) {
                    zone_contours_.push_back(contour);
                }
            }
        }
    }

    void saveMapsCallback(const std::shared_ptr<ros2_rad_map::srv::SaveMaps::Request> request,
                          std::shared_ptr<ros2_rad_map::srv::SaveMaps::Response> response)
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
        map_image_for_raycasting_ = cv::Mat(base_map_.info.height, base_map_.info.width, CV_8UC1);
        for (unsigned int i = 0; i < base_map_.data.size(); ++i)
        {
            map_image_for_raycasting_.data[i] = (base_map_.data[i] > 50) ? 255 : 0;
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
            pose_in_sensor.header.stamp = rclcpp::Time(0);
            tf_buffer_.transform(pose_in_sensor, pose_in_map, map_frame_, std::chrono::milliseconds(500));

            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                RadiationSample sample = {pose_in_map.pose.position.x, pose_in_map.pose.position.y, msg->data[1], next_sample_id_++};
                historical_samples_.push_back(sample);
            }
            triggerGPUpdate();
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF Error for logging sample: %s", ex.what());
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
        cv::Mat raycasting_map_copy;
        std::vector<RadiationSample> hist_samples_copy;
        bool can_update = false;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (map_received_ && !historical_samples_.empty())
            {
                can_update = true;
                map_copy = base_map_;
                raycasting_map_copy = map_image_for_raycasting_.clone();
                hist_samples_copy = historical_samples_;
            }
        }

        if (!can_update)
        {
            std::lock_guard<std::mutex> lock(update_mutex_);
            is_updating_ = false;
            return;
        }

        auto request = std::make_shared<ros2_rad_map::srv::GPPredict::Request>();
        for (const auto &s : hist_samples_copy)
        {
            request->observed_x.push_back(s.x);
            request->observed_y.push_back(s.y);
            request->observed_values_mean.push_back(s.value);
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
        request->points_to_predict = points_to_predict;

        auto future = gp_client_->async_send_request(request);

        std::thread([this, future = std::move(future), map_copy, raycasting_map_copy, valid_pixel_coords, hist_samples_copy]() mutable {
            try
            {
                auto response = future.get();
                if (!response || response->predicted_mean.size() != valid_pixel_coords.size())
                {
                    RCLCPP_WARN(this->get_logger(), "GP service returned invalid data or timed out.");
                    std::lock_guard<std::mutex> lock(update_mutex_);
                    is_updating_ = false;
                    return;
                }
                
                cv::Mat hist_mean_raw = cv::Mat::zeros(map_copy.info.height, map_copy.info.width, CV_32F);
                cv::Mat hist_std_raw = cv::Mat::zeros(map_copy.info.height, map_copy.info.width, CV_32F);
                for (size_t i = 0; i < valid_pixel_coords.size(); ++i)
                {
                    hist_mean_raw.at<float>(valid_pixel_coords[i].second, valid_pixel_coords[i].first) = response->predicted_mean[i];
                    hist_std_raw.at<float>(valid_pixel_coords[i].second, valid_pixel_coords[i].first) = response->predicted_std_dev[i];
                }

                cv::Mat hist_mean_occluded = applyOcclusionFloodFill(hist_mean_raw, hist_samples_copy, map_copy, raycasting_map_copy);
                cv::Mat hist_std_occluded = applyOcclusionFloodFill(hist_std_raw, hist_samples_copy, map_copy, raycasting_map_copy);

                publishHeatmaps(hist_mean_occluded, hist_std_occluded, map_copy);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception caught in GP update thread: %s", e.what());
            }

            std::lock_guard<std::mutex> lock(update_mutex_);
            is_updating_ = false;
        }).detach();
    }

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

    void publishHeatmaps(const cv::Mat &hist_mean, const cv::Mat &hist_std, const nav_msgs::msg::OccupancyGrid &map_copy)
    {
        cv::Mat base_bgr = createBaseMapImage(map_copy);
        std_msgs::msg::Header h;
        h.stamp = this->now();
        h.frame_id = map_frame_;

        if (!hist_mean.empty()) {
            raw_mean_map_pub_.publish(*cv_bridge::CvImage(h, "32FC1", hist_mean).toImageMsg());
        }

        cv::Mat flipped_hist_mean, flipped_hist_std;
        if (!hist_mean.empty()) cv::flip(hist_mean, flipped_hist_mean, 0);
        if (!hist_std.empty()) cv::flip(hist_std, flipped_hist_std, 0);

        cv::Mat free_space_mask(map_copy.info.height, map_copy.info.width, CV_8UC1);
        for (unsigned int y = 0; y < map_copy.info.height; ++y) {
            for (unsigned int x = 0; x < map_copy.info.width; ++x) {
                unsigned int image_y = map_copy.info.height - 1 - y;
                free_space_mask.at<uchar>(image_y, x) = (map_copy.data[y * map_copy.info.width + x] >= 0 && map_copy.data[y * map_copy.info.width + x] < 50) ? 255 : 0;
            }
        }

        auto blended_hist_mean = createBlendedMap(base_bgr, flipped_hist_mean, "mrem/hr", background_threshold_mrem_hr_, free_space_mask);
        if (!blended_hist_mean.empty())
        {
            std::lock_guard<std::mutex> lock(zone_mutex_);
            if (!zone_contours_.empty()) {
                cv::drawContours(blended_hist_mean, zone_contours_, -1, cv::Scalar(255, 0, 255), 2);
            }
            historical_heatmap_pub_.publish(*cv_bridge::CvImage(h, "bgr8", blended_hist_mean).toImageMsg());
        }

        auto blended_hist_std = createBlendedMap(base_bgr, flipped_hist_std, "Std. Dev.", 0.0, free_space_mask);
        if (!blended_hist_std.empty())
        {
            std::lock_guard<std::mutex> lock(zone_mutex_);
            if (!zone_contours_.empty()) {
                cv::drawContours(blended_hist_std, zone_contours_, -1, cv::Scalar(255, 0, 255), 2);
            }
            historical_std_dev_pub_.publish(*cv_bridge::CvImage(h, "bgr8", blended_hist_std).toImageMsg());
        }

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!blended_hist_mean.empty())
                latest_historical_mean_image_ = blended_hist_mean.clone();
            if (!blended_hist_std.empty())
                latest_historical_std_dev_image_ = blended_hist_std.clone();
        }
    }

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
        color_field.setTo(cv::Scalar(0, 0, 0, 0), value_mask);
        return color_field;
    }
};

cv::Mat createBaseMapImage(const nav_msgs::msg::OccupancyGrid &map)
{
    cv::Mat base_gray(map.info.height, map.info.width, CV_8UC1);
    for (unsigned int y = 0; y < map.info.height; ++y)
    {
        for (unsigned int x = 0; x < map.info.width; ++x)
        {
            int8_t o = map.data[y * map.info.width + x];
            unsigned int image_y = map.info.height - 1 - y;
            if (o == -1)
                base_gray.at<uchar>(image_y, x) = 205;
            else if (o >= 50)
                base_gray.at<uchar>(image_y, x) = 0;
            else
                base_gray.at<uchar>(image_y, x) = 255;
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