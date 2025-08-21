#include "rclcpp/rclcpp.hpp"
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
#include "std_srvs/srv/trigger.hpp"
#include "rad121_monitor/srv/gp_predict.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"

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
#include <future>

class RadiationHeatmapNode : public rclcpp::Node
{
private:
    struct RadiationSample
    {
        double x, y, cpm, mrem_hr_real, mrem_hr_predicted;
    };

    // Heatmap Data & State
    cv::Mat gp_mean_field_;
    cv::Mat gp_std_dev_field_;
    bool gp_queried_ = false;
    nav_msgs::msg::OccupancyGrid base_map_;
    bool map_received_ = false;
    std::mutex map_mutex_;

    // Parameters
    std::string session_folder_, output_directory_, map_frame_, sensor_frame_, data_collection_frame_;
    double background_threshold_mrem_hr_, gaussian_sigma_, tf_timeout_seconds_;
    int gaussian_blur_size_;

    // ROS objects
    std::vector<RadiationSample> collected_samples_;
    std::mutex collected_samples_mutex_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    image_transport::Publisher gp_mean_heatmap_pub_;
    image_transport::Publisher gp_std_dev_heatmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gp_costmap_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finalization_service_;
    rclcpp::Client<rad121_monitor::srv::GPPredict>::SharedPtr gp_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void declareAndGetParameters()
    {
        this->declare_parameter<std::string>("sensor_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("data_collection_frame", "odom");
        this->declare_parameter<std::string>("output_directory", "");
        this->declare_parameter<double>("gaussian_sigma", 5.0);
        this->declare_parameter<int>("gaussian_blur_size", 15);
        this->declare_parameter<double>("background_threshold_mrem_hr", 0.1);
        this->declare_parameter<double>("tf_timeout_seconds", 0.2);

        sensor_frame_ = this->get_parameter("sensor_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        data_collection_frame_ = this->get_parameter("data_collection_frame").as_string();
        output_directory_ = this->get_parameter("output_directory").as_string();
        gaussian_sigma_ = this->get_parameter("gaussian_sigma").as_double();
        gaussian_blur_size_ = this->get_parameter("gaussian_blur_size").as_int();
        background_threshold_mrem_hr_ = this->get_parameter("background_threshold_mrem_hr").as_double();
        tf_timeout_seconds_ = this->get_parameter("tf_timeout_seconds").as_double();

        if (gaussian_blur_size_ <= 0)
            gaussian_blur_size_ = 1;
        else if (gaussian_blur_size_ % 2 == 0)
            gaussian_blur_size_++;
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (map_received_)
            return;

        RCLCPP_INFO(this->get_logger(), "Base map received. Preparing to query GP model for the entire map...");
        base_map_ = *msg;
        map_received_ = true;

        auto request = std::make_shared<rad121_monitor::srv::GPPredict::Request>();
        std::vector<std::pair<int, int>> valid_pixel_coords;
        double res = base_map_.info.resolution;
        double origin_x = base_map_.info.origin.position.x;
        double origin_y = base_map_.info.origin.position.y;

        for (unsigned int y = 0; y < base_map_.info.height; ++y)
        {
            for (unsigned int x = 0; x < base_map_.info.width; ++x)
            {
                if (base_map_.data[y * base_map_.info.width + x] < 50)
                { // If not occupied
                    geometry_msgs::msg::Point p;
                    // Note: We are sending MAP coordinates to the GP service.
                    p.x = origin_x + (x + 0.5) * res;
                    p.y = origin_y + (y + 0.5) * res;
                    p.z = 0;
                    request->points_to_predict.push_back(p);
                    valid_pixel_coords.push_back({x, y});
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Sending %zu points to GP prediction service. This may take a moment...", request->points_to_predict.size());

        using ServiceResponseFuture = rclcpp::Client<rad121_monitor::srv::GPPredict>::SharedFuture;
        auto response_received_callback = [this, valid_pixel_coords](ServiceResponseFuture future)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Received response from GP service.");

            if (response->predicted_mean.size() != valid_pixel_coords.size())
            {
                RCLCPP_ERROR(this->get_logger(), "Mismatch between requested points and GP response size!");
                return;
            }

            std::lock_guard<std::mutex> lock(this->map_mutex_);
            this->gp_mean_field_ = cv::Mat::zeros(this->base_map_.info.height, this->base_map_.info.width, CV_32F);
            this->gp_std_dev_field_ = cv::Mat::zeros(this->base_map_.info.height, this->base_map_.info.width, CV_32F);

            for (size_t i = 0; i < valid_pixel_coords.size(); ++i)
            {
                int px = valid_pixel_coords[i].first;
                int py = valid_pixel_coords[i].second;
                this->gp_mean_field_.at<float>(py, px) = response->predicted_mean[i];
                this->gp_std_dev_field_.at<float>(py, px) = response->predicted_std_dev[i];
            }
            this->gp_queried_ = true;
            RCLCPP_INFO(this->get_logger(), "GP fields populated successfully.");
            this->publishGPHeatmaps();
        };

        gp_client_->async_send_request(request, response_received_callback);
    }

    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!gp_queried_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "GP fields not ready yet. Skipping radiation sample.");
            return;
        }
        if (msg->data.empty() || msg->data.size() < 2)
            return;

        geometry_msgs::msg::PoseStamped pose_in_sensor, pose_in_map;
        pose_in_sensor.header.frame_id = sensor_frame_;
        pose_in_sensor.header.stamp = this->get_clock()->now();

        try
        {
            tf_buffer_.transform(pose_in_sensor, pose_in_map, map_frame_, tf2::durationFromSec(tf_timeout_seconds_));

            std::lock_guard<std::mutex> lock(map_mutex_);
            double res = base_map_.info.resolution;
            int px = static_cast<int>((pose_in_map.pose.position.x - base_map_.info.origin.position.x) / res);
            int py = static_cast<int>((pose_in_map.pose.position.y - base_map_.info.origin.position.y) / res);

            double predicted_mrem = 0.0;
            if (py >= 0 && py < gp_mean_field_.rows && px >= 0 && px < gp_mean_field_.cols)
            {
                predicted_mrem = gp_mean_field_.at<float>(py, px);
            }

            RadiationSample sample = {pose_in_map.pose.position.x, pose_in_map.pose.position.y, msg->data[0], msg->data[1], predicted_mrem};
            std::lock_guard<std::mutex> samples_lock(collected_samples_mutex_);
            collected_samples_.push_back(sample);

            RCLCPP_INFO(this->get_logger(), "Logged sample. Real: %.2f, Predicted: %.2f", sample.mrem_hr_real, sample.mrem_hr_predicted);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "TF Error for logging sample: %s", ex.what());
        }
    }

    void publishGPHeatmaps()
    {
        if (!gp_queried_)
            return;
        std::lock_guard<std::mutex> lock(map_mutex_);

        std_msgs::msg::Header h;
        h.stamp = this->now();
        h.frame_id = map_frame_;

        cv::Mat gp_mean_display = gp_mean_field_.clone();
        applyGaussianBlur(gp_mean_display);
        auto [mean_min, mean_max] = applyThresholdAndGetRange(gp_mean_display);
        cv::Mat base_bgr = createBaseMapImage(base_map_);
        cv::Mat mean_color_map = createColorHeatmapImage(gp_mean_display, mean_min, mean_max);
        cv::Mat blended_mean_image;
        cv::addWeighted(base_bgr, 0.5, mean_color_map, 0.5, 0.0, blended_mean_image);
        addLegendToImage(blended_mean_image, mean_min, mean_max, "GP Mean (mrem/hr)");
        gp_mean_heatmap_pub_.publish(*cv_bridge::CvImage(h, "bgr8", blended_mean_image).toImageMsg());

        cv::Mat gp_std_dev_display = gp_std_dev_field_.clone();
        applyGaussianBlur(gp_std_dev_display);
        auto [std_min, std_max] = applyThresholdAndGetRange(gp_std_dev_display, 0.0);
        cv::Mat std_dev_color_map = createColorHeatmapImage(gp_std_dev_display, std_min, std_max);
        cv::Mat blended_std_dev_image;
        cv::addWeighted(base_bgr.clone(), 0.5, std_dev_color_map, 0.5, 0.0, blended_std_dev_image);
        addLegendToImage(blended_std_dev_image, std_min, std_max, "GP Std. Dev.");
        gp_std_dev_heatmap_pub_.publish(*cv_bridge::CvImage(h, "bgr8", blended_std_dev_image).toImageMsg());

        nav_msgs::msg::OccupancyGrid gp_costmap;
        gp_costmap.header = h;
        gp_costmap.info = base_map_.info;
        gp_costmap.data.assign(base_map_.info.width * base_map_.info.height, 0);
        updateCostmapData(gp_costmap, gp_mean_display, mean_min, mean_max);
        gp_costmap_pub_->publish(gp_costmap);

        RCLCPP_INFO(this->get_logger(), "Published GP Mean/StdDev heatmaps and costmap.");
    }

    void finalizeHeatmapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Finalization service called. Saving all maps and data...");

        if (!gp_queried_)
        {
            response->success = false;
            response->message = "GP model has not been successfully queried. Cannot finalize.";
            RCLCPP_ERROR(this->get_logger(), response->message.c_str());
            return;
        }

        saveImage(gp_mean_field_, "final_gp_mean_heatmap_raw.png");
        saveImage(gp_std_dev_field_, "final_gp_std_dev_heatmap_raw.png");
        saveCollectedDataToCSV();

        response->success = true;
        response->message = "Final raw heatmaps and data saved successfully.";
        RCLCPP_INFO(this->get_logger(), "Finalization complete.");
    }

    void createSessionFolder()
    {
        std::stringstream ss;
        std::time_t now = std::time(nullptr);
        ss << "rad_" << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
        std::string session_name = ss.str();
        if (output_directory_.empty())
        {
            output_directory_ = std::filesystem::current_path().string();
        }
        session_folder_ = (std::filesystem::path(output_directory_) / session_name).string();
        try
        {
            std::filesystem::create_directories(session_folder_);
            RCLCPP_INFO(this->get_logger(), "Session folder created: %s", session_folder_.c_str());
        }
        catch (const std::filesystem::filesystem_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not create session folder: %s", e.what());
            session_folder_ = "";
        }
    }

    void saveImage(const cv::Mat &image, const std::string &filename)
    {
        if (session_folder_.empty())
            return;
        std::string path = (std::filesystem::path(session_folder_) / filename).string();
        try
        {
            cv::Mat savable_image;
            if (image.type() == CV_32F || image.type() == CV_64F)
            {
                cv::normalize(image, savable_image, 0, 255, cv::NORM_MINMAX, CV_8U);
            }
            else
            {
                savable_image = image;
            }
            if (!cv::imwrite(path, savable_image))
            {
                RCLCPP_ERROR(get_logger(), "Failed to save image to %s", path.c_str());
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Successfully saved image to %s", path.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Exception while saving image: %s", e.what());
        }
    }

    void saveCollectedDataToCSV()
    {
        if (session_folder_.empty())
            return;
        std::string path = (std::filesystem::path(session_folder_) / "collected_samples.csv").string();
        std::ofstream file(path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing.", path.c_str());
            return;
        }
        file << "map_x,map_y,cpm,mrem_hr_real,mrem_hr_predicted,error\n";
        std::lock_guard<std::mutex> lock(collected_samples_mutex_);
        for (const auto &sample : collected_samples_)
        {
            file << sample.x << "," << sample.y << "," << sample.cpm << ","
                 << sample.mrem_hr_real << "," << sample.mrem_hr_predicted << ","
                 << std::abs(sample.mrem_hr_real - sample.mrem_hr_predicted) << "\n";
        }
        file.close();
        RCLCPP_INFO(this->get_logger(), "Saved %zu samples to %s", collected_samples_.size(), path.c_str());
    }

    void applyGaussianBlur(cv::Mat &field)
    {
        if (gaussian_blur_size_ < 3 || gaussian_sigma_ <= 0)
            return;
        cv::GaussianBlur(field, field, cv::Size(gaussian_blur_size_, gaussian_blur_size_), gaussian_sigma_);
    }

    std::pair<float, float> applyThresholdAndGetRange(cv::Mat &field, float custom_threshold = -1.0f)
    {
        float threshold = (custom_threshold >= 0) ? custom_threshold : background_threshold_mrem_hr_;
        float true_min = std::numeric_limits<float>::max();
        float true_max = std::numeric_limits<float>::lowest();
        bool found_value_above_threshold = false;
        for (int y = 0; y < field.rows; ++y)
        {
            for (int x = 0; x < field.cols; ++x)
            {
                float &val = field.at<float>(y, x);
                if (val < threshold)
                {
                    val = 0.0f;
                }
                else
                {
                    true_min = std::min(true_min, val);
                    true_max = std::max(true_max, val);
                    found_value_above_threshold = true;
                }
            }
        }
        if (!found_value_above_threshold)
        {
            true_min = threshold;
            true_max = true_min + 0.01f;
        }
        else if (true_max <= true_min)
        {
            true_max = true_min + 0.01f;
        }
        return {true_min, true_max};
    }

    cv::Mat createBaseMapImage(const nav_msgs::msg::OccupancyGrid &map)
    {
        cv::Mat base_gray(map.info.height, map.info.width, CV_8UC1);
        for (unsigned int y = 0; y < map.info.height; ++y)
        {
            for (unsigned int x = 0; x < map.info.width; ++x)
            {
                int8_t o = map.data[y * map.info.width + x];
                base_gray.at<uchar>(y, x) = (o == -1) ? 127 : (o >= 50 ? 0 : 255);
            }
        }
        cv::Mat bgr;
        cv::cvtColor(base_gray, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }

    cv::Mat createColorHeatmapImage(const cv::Mat &field, float min_val, float max_val)
    {
        float log_min_val = std::log(std::max(min_val, 0.001f));
        float log_max_val = std::log(std::max(max_val, min_val + 0.01f));
        float log_range = log_max_val - log_min_val;
        if (log_range <= 1e-6)
            log_range = 1.0f;

        cv::Mat scaled_field = cv::Mat::zeros(field.size(), CV_8UC1);
        for (int y = 0; y < field.rows; ++y)
        {
            for (int x = 0; x < field.cols; ++x)
            {
                float val = field.at<float>(y, x);
                if (val >= background_threshold_mrem_hr_)
                {
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

    void addLegendToImage(cv::Mat &image, float min_val, float max_val, const std::string &title)
    {
        int legend_width = 60;
        int legend_height = image.rows / 2;
        int legend_y_offset = image.rows / 4;
        int border_padding = 150;

        cv::copyMakeBorder(image, image, 0, 0, 0, legend_width + border_padding, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
        cv::Rect legend_rect(image.cols - legend_width - (border_padding / 2) - 20, legend_y_offset, legend_width, legend_height);
        cv::Mat legend_bar = image(legend_rect);

        for (int i = 0; i < legend_bar.rows; i++)
        {
            float ratio = static_cast<float>(i) / static_cast<float>(legend_bar.rows);
            uchar val = static_cast<uchar>(255.0f * (1.0f - ratio));
            cv::Mat color_pixel(1, 1, CV_8UC1, val);
            cv::Mat mapped_color;
            cv::applyColorMap(color_pixel, mapped_color, cv::COLORMAP_JET);
            legend_bar.row(i) = mapped_color.at<cv::Vec3b>(0, 0);
        }

        int text_x_pos = legend_rect.x + legend_width + 10;
        cv::putText(image, title, cv::Point(legend_rect.x, legend_rect.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
        cv::putText(image, cv::format("%.2f", max_val), cv::Point(text_x_pos, legend_rect.y + 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
        cv::putText(image, cv::format("%.2f", min_val), cv::Point(text_x_pos, legend_rect.y + legend_rect.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
    }

    void updateCostmapData(nav_msgs::msg::OccupancyGrid &costmap, const cv::Mat &field, float min_val, float max_val)
    {
        float log_min_val = std::log(std::max(min_val, 0.001f));
        float log_max_val = std::log(std::max(max_val, min_val + 0.01f));
        float log_range = log_max_val - log_min_val;
        if (log_range <= 1e-6)
            log_range = 1.0f;

        for (int y = 0; y < field.rows; ++y)
        {
            for (int x = 0; x < field.cols; ++x)
            {
                float val = field.at<float>(y, x);
                if (val >= background_threshold_mrem_hr_)
                {
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

    void init()
    {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&RadiationHeatmapNode::mapCallback, this, std::placeholders::_1));
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/rad", 10, std::bind(&RadiationHeatmapNode::radCallback, this, std::placeholders::_1));

        image_transport::ImageTransport it(this->shared_from_this());
        gp_mean_heatmap_pub_ = it.advertise("/rad_heatmap/gp_mean", 1);
        gp_std_dev_heatmap_pub_ = it.advertise("/rad_heatmap/gp_std_dev", 1);

        gp_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/rad_heatmap/gp_costmap", rclcpp::QoS(1).transient_local().reliable());

        gp_client_ = this->create_client<rad121_monitor::srv::GPPredict>("predict_gp");
        RCLCPP_INFO(this->get_logger(), "Waiting for predict_gp service...");
        while (!gp_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        RCLCPP_INFO(this->get_logger(), "Service is available.");

        finalization_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/finalize_heatmap",
            std::bind(&RadiationHeatmapNode::finalizeHeatmapCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Radiation GP monitor initialized.");
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