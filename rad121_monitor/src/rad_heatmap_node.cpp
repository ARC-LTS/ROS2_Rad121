#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "time.h"

#include <vector>
#include <mutex>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

class RadiationHeatmapNode : public rclcpp::Node
{
private:
    struct RadiationSample
    {
        double x, y, cpm, mrem_hr;
    };
    
    std::string session_folder_;
    std::string sensor_frame_, map_frame_;
    std::string output_directory_;
    double min_valid_mrem_, min_sample_mrem_;
    double gaussian_sigma_, distance_cutoff_, background_threshold_mrem_hr_;
    int gaussian_blur_size_;
    double tf_timeout_seconds_;
    std::string data_collection_frame_;
    double draft_update_frequency_;

    std::vector<RadiationSample> final_samples_;
    std::mutex final_samples_mutex_;

    std::vector<RadiationSample> new_samples_for_draft_queue_;
    std::mutex new_samples_for_draft_mutex_;
    cv::Mat draft_rad_numerator_field_;
    cv::Mat draft_rad_denominator_field_;
    cv::Mat draft_max_rad_field_;
    
    nav_msgs::msg::OccupancyGrid base_map_;
    std::mutex map_mutex_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    image_transport::Publisher draft_image_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr draft_costmap_pub_;
    rclcpp::TimerBase::SharedPtr draft_update_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finalization_service_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void declareAndGetParameters()
    {
        this->declare_parameter<std::string>("sensor_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("output_directory", "");
        this->declare_parameter<double>("min_valid_mrem", 50.0);
        this->declare_parameter<double>("min_sample_mrem", 100.0);
        this->declare_parameter<double>("gaussian_sigma", 5.0);
        this->declare_parameter<int>("gaussian_blur_size", 15);
        this->declare_parameter<double>("distance_cutoff", 10.0);
        this->declare_parameter<double>("background_threshold_mrem_hr", 0.1);
        this->declare_parameter<double>("tf_timeout_seconds", 0.2);
        this->declare_parameter<std::string>("data_collection_frame", "odom");
        this->declare_parameter<double>("draft_update_frequency", 1.0);

        sensor_frame_ = this->get_parameter("sensor_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        output_directory_ = this->get_parameter("output_directory").as_string();
        min_valid_mrem_ = this->get_parameter("min_valid_mrem").as_double();
        min_sample_mrem_ = this->get_parameter("min_sample_mrem").as_double();
        gaussian_sigma_ = this->get_parameter("gaussian_sigma").as_double();
        gaussian_blur_size_ = this->get_parameter("gaussian_blur_size").as_int();
        distance_cutoff_ = this->get_parameter("distance_cutoff").as_double();
        background_threshold_mrem_hr_ = this->get_parameter("background_threshold_mrem_hr").as_double();
        tf_timeout_seconds_ = this->get_parameter("tf_timeout_seconds").as_double();
        data_collection_frame_ = this->get_parameter("data_collection_frame").as_string();
        draft_update_frequency_ = this->get_parameter("draft_update_frequency").as_double();

        if (gaussian_blur_size_ <= 0)
            gaussian_blur_size_ = 1;
        else if (gaussian_blur_size_ % 2 == 0)
            gaussian_blur_size_++;
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        bool has_map_changed = base_map_.data.empty() || base_map_.info.width != msg->info.width || base_map_.info.height != msg->info.height;
        
        if (has_map_changed) {
             RCLCPP_DEBUG(this->get_logger(), "Base map received. Initializing draft heatmap state.");
             base_map_ = *msg;
             draft_rad_numerator_field_ = cv::Mat::zeros(msg->info.height, msg->info.width, CV_32FC1);
             draft_rad_denominator_field_ = cv::Mat::zeros(msg->info.height, msg->info.width, CV_32FC1);
             draft_max_rad_field_ = cv::Mat::zeros(msg->info.height, msg->info.width, CV_32FC1);
        }
    }

    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.empty() || msg->data.size() < 2) return;
        double mrem_hr = msg->data[1];
        if (mrem_hr < min_sample_mrem_) return;

        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.header.frame_id = sensor_frame_;

        try
        {
            pose_out = tf_buffer_.transform(pose_in, data_collection_frame_, tf2::durationFromSec(tf_timeout_seconds_));
            
            RadiationSample sample = {pose_out.pose.position.x, pose_out.pose.position.y, msg->data[0], mrem_hr};

            {
                std::lock_guard<std::mutex> lock(new_samples_for_draft_mutex_);
                new_samples_for_draft_queue_.push_back(sample);
            }
            {
                std::lock_guard<std::mutex> lock(final_samples_mutex_);
                final_samples_.push_back(sample);
                RCLCPP_INFO(this->get_logger(), "Stored radiation sample #%zu (%.2f mrem/hr).", final_samples_.size(), mrem_hr);
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "TF Error for sample: %s", ex.what());
        }
    }
    
    /* Draft Functions */
    void updateDraftsCallback() {
        std::vector<RadiationSample> samples_to_process;
        {
            std::lock_guard<std::mutex> lock(new_samples_for_draft_mutex_);
            if (new_samples_for_draft_queue_.empty()) return;
            samples_to_process.swap(new_samples_for_draft_queue_);
        }
        RCLCPP_INFO(this->get_logger(), "Processing %zu new samples for draft heatmap...", samples_to_process.size());
        updateDraftFieldIncremental(samples_to_process);
        generateAndPublishDrafts();
    }

    void updateDraftFieldIncremental(const std::vector<RadiationSample>& new_samples) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (base_map_.data.empty()) return;

        double res = base_map_.info.resolution;
        double origin_x = base_map_.info.origin.position.x;
        double origin_y = base_map_.info.origin.position.y;
        double inv_2sigma2 = 1.0 / (2.0 * gaussian_sigma_ * gaussian_sigma_);
        double distance_cutoff_sq = distance_cutoff_ * distance_cutoff_;

        for (const auto& sample : new_samples) {
            geometry_msgs::msg::PointStamped point_in_collection_frame, point_in_map_frame;
            point_in_collection_frame.header.frame_id = data_collection_frame_;
            point_in_collection_frame.point.x = sample.x;
            point_in_collection_frame.point.y = sample.y;

            try {
                tf_buffer_.transform(point_in_collection_frame, point_in_map_frame, map_frame_, tf2::durationFromSec(0.1));
            } catch (const tf2::TransformException &ex) { continue; }

            double sample_map_x = point_in_map_frame.point.x;
            double sample_map_y = point_in_map_frame.point.y;

            int roi_min_x = static_cast<int>((sample_map_x - distance_cutoff_ - origin_x) / res);
            int roi_max_x = static_cast<int>((sample_map_x + distance_cutoff_ - origin_x) / res);
            int roi_min_y = static_cast<int>((sample_map_y - distance_cutoff_ - origin_y) / res);
            int roi_max_y = static_cast<int>((sample_map_y + distance_cutoff_ - origin_y) / res);

            for (int y = std::max(0, roi_min_y); y <= std::min((int)base_map_.info.height - 1, roi_max_y); ++y) {
                for (int x = std::max(0, roi_min_x); x <= std::min((int)base_map_.info.width - 1, roi_max_x); ++x) {
                    double cell_center_wx = origin_x + (x + 0.5) * res;
                    double cell_center_wy = origin_y + (y + 0.5) * res;
                    double dx_world = cell_center_wx - sample_map_x;
                    double dy_world = cell_center_wy - sample_map_y;
                    double dist_sq_world = dx_world * dx_world + dy_world * dy_world;

                    if (dist_sq_world > distance_cutoff_sq) continue;
                    
                    double weight = std::exp(-dist_sq_world * inv_2sigma2);
                    draft_rad_numerator_field_.at<float>(y, x) += weight * sample.mrem_hr;
                    draft_rad_denominator_field_.at<float>(y, x) += weight;
                }
            }
        }
    }
    
    void generateAndPublishDrafts() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (base_map_.data.empty()) return;

        cv::Mat current_draft_field = cv::Mat::zeros(base_map_.info.height, base_map_.info.width, CV_32FC1);
        cv::divide(draft_rad_numerator_field_, draft_rad_denominator_field_, current_draft_field, 1.0, CV_32FC1);
        cv::patchNaNs(current_draft_field, 0.0);
        cv::max(current_draft_field, draft_max_rad_field_, draft_max_rad_field_);
        
        cv::Mat display_field = draft_max_rad_field_.clone();
        applyGaussianBlur(display_field);
        auto [true_min, true_max] = applyThresholdAndGetRange(display_field);

        cv::Mat base_bgr = createBaseMapImage(base_map_);
        cv::Mat color_heatmap = createColorHeatmapImage(display_field, true_min, true_max);
        cv::Mat blended_image;
        cv::addWeighted(base_bgr, 0.5, color_heatmap, 0.5, 0.0, blended_image);
        addLegendToImage(blended_image, true_min, true_max);
        std_msgs::msg::Header h;
        h.stamp = this->now();
        h.frame_id = map_frame_;
        draft_image_pub_.publish(*cv_bridge::CvImage(h, "bgr8", blended_image).toImageMsg());
        
        nav_msgs::msg::OccupancyGrid draft_costmap;
        draft_costmap.header = h;
        draft_costmap.info = base_map_.info;
        draft_costmap.data.assign(base_map_.info.width * base_map_.info.height, 0);
        updateCostmapData(draft_costmap, display_field, true_min, true_max);
        draft_costmap_pub_->publish(draft_costmap);
    }
    
    /* Final Heatmap Functions*/    
    void finalizeHeatmapCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        RCLCPP_INFO(this->get_logger(), "Finalization service called. Generating and saving final heatmap...");
        
        generateAndSaveFinalHeatmap();

        response->success = true;
        response->message = "Final heatmap generated and saved successfully.";
        RCLCPP_INFO(this->get_logger(), "Finalization complete.");
    }

    void generateAndSaveFinalHeatmap() {
        std::lock_guard<std::mutex> lock(final_samples_mutex_);
        RCLCPP_INFO(this->get_logger(), "Beginning final processing...");

        if (base_map_.data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No base map was ever received. Cannot generate final heatmap.");
            return;
        }
        if (final_samples_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No radiation samples collected. Saving map without heatmap.");
            cv::Mat base_bgr = createBaseMapImage(base_map_);
            saveMap(base_bgr);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Processing %zu collected radiation samples for final map...", final_samples_.size());

        cv::Mat rad_field = computeRawRadiationField();
        cv::Mat display_field = rad_field.clone();
        applyGaussianBlur(display_field);
        auto [true_min, true_max] = applyThresholdAndGetRange(display_field);
        cv::Mat base_bgr = createBaseMapImage(base_map_);
        cv::Mat color_heatmap = createColorHeatmapImage(display_field, true_min, true_max);
        cv::Mat blended_image_output;
        cv::addWeighted(base_bgr, 0.5, color_heatmap, 0.5, 0.0, blended_image_output);
        addLegendToImage(blended_image_output, true_min, true_max);
        saveMap(blended_image_output);
    }

    cv::Mat computeRawRadiationField() {
        cv::Mat rad_field = cv::Mat::zeros(base_map_.info.height, base_map_.info.width, CV_32FC1);
        double res = base_map_.info.resolution;
        double origin_x = base_map_.info.origin.position.x;
        double origin_y = base_map_.info.origin.position.y;
        double inv_2sigma2 = 1.0 / (2.0 * gaussian_sigma_ * gaussian_sigma_);
        double distance_cutoff_sq = distance_cutoff_ * distance_cutoff_;

        int total_rows = rad_field.rows;
        int last_percent = -1;
        RCLCPP_INFO(this->get_logger(), "Starting computationally intensive map generation...");

        for (int y = 0; y < total_rows; ++y) {
            int percent_done = static_cast<int>((static_cast<float>(y+1) / total_rows) * 100.0f);
            if (percent_done > last_percent) {
                printf("\rGenerating Final Heatmap: %d%% complete...", percent_done);
                fflush(stdout);
                last_percent = percent_done;
            }

            for (int x = 0; x < rad_field.cols; ++x) {
                if (isMapOccupied(base_map_, x, y)) continue;
                double wx_center = origin_x + (x + 0.5) * res;
                double wy_center = origin_y + (y + 0.5) * res;
                auto [numerator, denominator] = calculateCellContribution(x, y, wx_center, wy_center, inv_2sigma2, distance_cutoff_sq);
                rad_field.at<float>(y, x) = (denominator > 1e-9) ? static_cast<float>(numerator / denominator) : 0.0f;
            }
        }
        printf("\n");
        RCLCPP_INFO(this->get_logger(), "Map generation finished.");
        return rad_field;
    }
    
    std::pair<double, double> calculateCellContribution(int x_cell, int y_cell, double wx_cell_center, double wy_cell_center, double inv_2sigma2, double distance_cutoff_sq)
    {
        double numerator = 0.0;
        double denominator = 0.0;
        for (const auto &sample : final_samples_) {
            geometry_msgs::msg::PointStamped point_in_collection_frame, point_in_map_frame;
            point_in_collection_frame.header.frame_id = data_collection_frame_;
            point_in_collection_frame.point.x = sample.x;
            point_in_collection_frame.point.y = sample.y;
            try {
                tf_buffer_.transform(point_in_collection_frame, point_in_map_frame, map_frame_, tf2::durationFromSec(0.1));
            } catch (const tf2::TransformException &ex) { continue; }

            double sample_map_x = point_in_map_frame.point.x;
            double sample_map_y = point_in_map_frame.point.y;
            int sx_sample_cell = static_cast<int>((sample_map_x - base_map_.info.origin.position.x) / base_map_.info.resolution);
            int sy_sample_cell = static_cast<int>((sample_map_y - base_map_.info.origin.position.y) / base_map_.info.resolution);

            if (!isVisible(base_map_, sx_sample_cell, sy_sample_cell, x_cell, y_cell)) continue;
            
            double dx_world = wx_cell_center - sample_map_x;
            double dy_world = wy_cell_center - sample_map_y;
            double dist_sq_world = dx_world * dx_world + dy_world * dy_world;
            if (dist_sq_world > distance_cutoff_sq) continue;
            
            double weight = std::exp(-dist_sq_world * inv_2sigma2);
            numerator += weight * sample.mrem_hr;
            denominator += weight;
        }
        return {numerator, denominator};
    }
    
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

    void saveMap(const cv::Mat& image)
    {
        if (session_folder_.empty()) {
            RCLCPP_ERROR(get_logger(), "Session folder not set. Cannot save map.");
            return;
        }
        std::string path = session_folder_ + "/map_with_heatmap.png";
        try {
            if (!cv::imwrite(path, image)) {
                RCLCPP_ERROR(get_logger(), "Failed to save final map to %s", path.c_str());
            } else {
                RCLCPP_INFO(get_logger(), "Successfully saved final map to %s", path.c_str());
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Exception while saving map: %s", e.what());
        }
    }

    bool isMapOccupied(const nav_msgs::msg::OccupancyGrid& map, int x, int y) {
        if (x < 0 || x >= (int)map.info.width || y < 0 || y >= (int)map.info.height) return true;
        int idx = y * map.info.width + x;
        return map.data[idx] >= 50;
    }

    bool isVisible(const nav_msgs::msg::OccupancyGrid& map, int x0, int y0, int x1, int y1) {
        int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;
        while (true) {
            if (isMapOccupied(map, x0, y0)) return false;
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
        return true;
    }

    void applyGaussianBlur(cv::Mat &field) {
        if (gaussian_blur_size_ < 3 || gaussian_sigma_ <= 0) return;
        cv::GaussianBlur(field, field, cv::Size(gaussian_blur_size_, gaussian_blur_size_), gaussian_sigma_);
    }

    std::pair<float, float> applyThresholdAndGetRange(cv::Mat &field) {
        float true_min = std::numeric_limits<float>::max();
        float true_max = std::numeric_limits<float>::lowest();
        bool found_value_above_threshold = false;
        for (int y = 0; y < field.rows; ++y) {
            for (int x = 0; x < field.cols; ++x) {
                float &val = field.at<float>(y, x);
                if (val < background_threshold_mrem_hr_) {
                    val = 0.0f;
                } else {
                    true_min = std::min(true_min, val);
                    true_max = std::max(true_max, val);
                    found_value_above_threshold = true;
                }
            }
        }
        if (!found_value_above_threshold) {
            true_min = static_cast<float>(background_threshold_mrem_hr_);
            true_max = true_min + 0.01f;
        } else if (true_max <= true_min) {
            true_max = true_min + 0.01f;
        }
        return {true_min, true_max};
    }

    cv::Mat createBaseMapImage(const nav_msgs::msg::OccupancyGrid& map) {
        cv::Mat base_gray(map.info.height, map.info.width, CV_8UC1);
        for (unsigned int y = 0; y < map.info.height; ++y) {
            for (unsigned int x = 0; x < map.info.width; ++x) {
                int8_t o = map.data[y * map.info.width + x];
                base_gray.at<uchar>(y, x) = (o == -1) ? 127 : (o >= 50 ? 0 : 255);
            }
        }
        cv::Mat bgr;
        cv::cvtColor(base_gray, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }
    
    cv::Mat createColorHeatmapImage(const cv::Mat &field, float min_val, float max_val) {
        float log_min_val = std::log(std::max(min_val, 0.001f));
        float log_max_val = std::log(std::max(max_val, min_val + 0.01f));
        float log_range = log_max_val - log_min_val;
        if (log_range <= 1e-6) log_range = 1.0f;

        cv::Mat scaled_field = cv::Mat::zeros(field.size(), CV_8UC1);
        for (int y = 0; y < field.rows; ++y) {
            for (int x = 0; x < field.cols; ++x) {
                float val = field.at<float>(y, x);
                if (val >= background_threshold_mrem_hr_) {
                    float log_val = std::log(std::max(val, 0.001f));
                    float scaled_intensity = 255.0f * (log_val - log_min_val) / log_range;
                    scaled_field.at<uchar>(y, x) = static_cast<uchar>(std::clamp(scaled_intensity, 0.0f, 255.0f));
                }
            }
        }
        cv::Mat color_field;
        cv::applyColorMap(scaled_field, color_field, cv::COLORMAP_JET);
        cv::Mat mask;
        cv::compare(field, background_threshold_mrem_hr_, mask, cv::CMP_LT);
        color_field.setTo(cv::Scalar(0, 0, 0), mask);
        return color_field;
    }

    void addLegendToImage(cv::Mat &image, float min_val, float max_val) {
        int legend_width = 50;
        int legend_height = image.rows / 2;
        int legend_y_offset = image.rows / 4;
        int border_padding = 100;

        cv::copyMakeBorder(image, image, 0, 0, 0, legend_width + border_padding, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

        cv::Rect legend_rect(image.cols - legend_width - (border_padding / 2), legend_y_offset, legend_width, legend_height);
        cv::Mat legend_bar = image(legend_rect);

        for (int i = 0; i < legend_bar.rows; i++) {
            float ratio = static_cast<float>(i) / static_cast<float>(legend_bar.rows);
            uchar val = static_cast<uchar>(255.0f * (1.0f - ratio));
            
            cv::Mat color_pixel(1, 1, CV_8UC1, val);
            cv::Mat mapped_color;
            cv::applyColorMap(color_pixel, mapped_color, cv::COLORMAP_JET);
            
            legend_bar.row(i) = mapped_color.at<cv::Vec3b>(0, 0);
        }

        int text_x_pos = legend_rect.x + legend_width + 10;
        cv::putText(image, cv::format("%.2f", max_val), cv::Point(text_x_pos, legend_rect.y + 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
        cv::putText(image, cv::format("%.2f", min_val), cv::Point(text_x_pos, legend_rect.y + legend_rect.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
        cv::putText(image, "mrem/hr", cv::Point(text_x_pos, legend_rect.y + legend_rect.height / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
    }
    
    void updateCostmapData(nav_msgs::msg::OccupancyGrid& costmap, const cv::Mat &field, float min_val, float max_val) {
        float log_min_val = std::log(std::max(min_val, 0.001f));
        float log_max_val = std::log(std::max(max_val, min_val + 0.01f));
        float log_range = log_max_val - log_min_val;
        if (log_range <= 1e-6) log_range = 1.0f;

        for (int y = 0; y < field.rows; ++y) {
            for (int x = 0; x < field.cols; ++x) {
                float val = field.at<float>(y, x);
                if (val >= background_threshold_mrem_hr_) {
                    int idx = y * field.cols + x;
                    float log_val = std::log(std::max(val, 0.001f));
                    float scaled_cost = 100.0f * (log_val - log_min_val) / log_range;
                    costmap.data[idx] = static_cast<int8_t>(std::clamp(scaled_cost, 0.0f, 100.0f));
                }
            }
        }
    }

public:
    RadiationHeatmapNode()
        : Node("radiation_heatmap_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        declareAndGetParameters();
        createSessionFolder();
    }
    
    void init() {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&RadiationHeatmapNode::mapCallback, this, std::placeholders::_1));
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/rad", 10, std::bind(&RadiationHeatmapNode::radCallback, this, std::placeholders::_1));

        image_transport::ImageTransport it(this->shared_from_this());
        draft_image_pub_ = it.advertise("/rad_heatmap_image/draft", 1);
        draft_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/rad_heatmap/draft", 10);

        auto period = std::chrono::duration<double>(1.0 / draft_update_frequency_);
        draft_update_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&RadiationHeatmapNode::updateDraftsCallback, this));

        finalization_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/finalize_heatmap",
            std::bind(&RadiationHeatmapNode::finalizeHeatmapCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Radiation data collector initialized.");
        RCLCPP_INFO(this->get_logger(), "--> Publishing real-time draft to /rad_heatmap_image/draft");
        RCLCPP_INFO(this->get_logger(), "--> Call /finalize_heatmap service to generate and save final map.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadiationHeatmapNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}