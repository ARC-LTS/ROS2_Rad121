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
#include "rad121_monitor/srv/gp_predict.hpp"

#include <vector>
#include <mutex>
#include <deque>
#include <thread>
#include <future>
#include <opencv2/opencv.hpp>

class RadiationHeatmapNode : public rclcpp::Node
{
private:
    struct RadiationSample {
        double x, y;
        double value;
    };

    nav_msgs::msg::OccupancyGrid base_map_;
    bool map_received_ = false;
    std::mutex map_mutex_;
    
    std::vector<RadiationSample> historical_samples_;
    std::deque<RadiationSample> recent_samples_;
    std::mutex samples_mutex_;
    
    bool is_updating_ = false;
    std::mutex update_mutex_;

    std::string map_frame_, sensor_frame_;
    int recent_samples_window_size_;
    double background_threshold_mrem_hr_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    image_transport::Publisher historical_heatmap_pub_;
    image_transport::Publisher instantaneous_heatmap_pub_;
    rclcpp::Client<rad121_monitor::srv::GPPredict>::SharedPtr gp_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    //================================================================================
    // CORE LOGIC
    //================================================================================

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            base_map_ = *msg;
            map_received_ = true;
        }
        RCLCPP_INFO(this->get_logger(), "Map updated. Triggering GP update.");
        triggerGPUpdate();
    }

    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (!map_received_ || msg->data.size() < 2) return;

        geometry_msgs::msg::PoseStamped pose_in_sensor, pose_in_map;
        pose_in_sensor.header.frame_id = sensor_frame_;
        pose_in_sensor.header.stamp = this->get_clock()->now();

        try {
            tf_buffer_.transform(pose_in_sensor, pose_in_map, map_frame_, tf2::durationFromSec(0.5));
            RadiationSample sample = {pose_in_map.pose.position.x, pose_in_map.pose.position.y, msg->data[1]};
            
            {
                std::lock_guard<std::mutex> lock(samples_mutex_);
                historical_samples_.push_back(sample);
                recent_samples_.push_back(sample);
                if (recent_samples_.size() > (size_t)recent_samples_window_size_) {
                    recent_samples_.pop_front();
                }
            }
            RCLCPP_INFO(this->get_logger(), "New radiation sample logged. Triggering GP update.");
            triggerGPUpdate();
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error for logging sample: %s", ex.what());
        }
    }

    void triggerGPUpdate() {
        nav_msgs::msg::OccupancyGrid map_copy; // FIX: Create a copy of the map for this update cycle.
        {
            std::lock_guard<std::mutex> lock(update_mutex_);
            if (is_updating_ || !map_received_ || historical_samples_.empty()) {
                return;
            }
            is_updating_ = true;
            std::lock_guard<std::mutex> map_lock(map_mutex_);
            map_copy = base_map_; // Make the copy
        }

        auto request_hist = std::make_shared<rad121_monitor::srv::GPPredict::Request>();
        auto request_inst = std::make_shared<rad121_monitor::srv::GPPredict::Request>();
        std::vector<RadiationSample> hist_samples_copy, inst_samples_copy;

        {
            std::lock_guard<std::mutex> lock(samples_mutex_);
            hist_samples_copy = historical_samples_;
            for(const auto& s : recent_samples_) inst_samples_copy.push_back(s);
        }

        for(const auto& s : hist_samples_copy) {
            request_hist->observed_x.push_back(s.x);
            request_hist->observed_y.push_back(s.y);
            request_hist->observed_values_mean.push_back(s.value);
        }
        for(const auto& s : inst_samples_copy) {
            request_inst->observed_x.push_back(s.x);
            request_inst->observed_y.push_back(s.y);
            request_inst->observed_values_mean.push_back(s.value);
        }

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
                    request_hist->points_to_predict.push_back(p);
                    request_inst->points_to_predict.push_back(p);
                    valid_pixel_coords.push_back({(int)x, (int)y});
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Sending requests: Historical (%zu samples), Instantaneous (%zu samples). Predicting for %zu points.",
            hist_samples_copy.size(), inst_samples_copy.size(), valid_pixel_coords.size());

        auto future_hist = gp_client_->async_send_request(request_hist);
        auto future_inst = gp_client_->async_send_request(request_inst);
        
        // FIX: Pass the map_copy into the detached thread's capture list
        std::thread([this, future_hist = std::move(future_hist), future_inst = std::move(future_inst), 
                     map_copy, valid_pixel_coords, hist_samples_copy, inst_samples_copy]() mutable {
            
            cv::Mat hist_heatmap, inst_heatmap;
            
            auto response_hist = future_hist.get();
            if (response_hist && response_hist->predicted_mean.size() == valid_pixel_coords.size()) {
                hist_heatmap = cv::Mat::zeros(map_copy.info.height, map_copy.info.width, CV_32F);
                for (size_t i = 0; i < valid_pixel_coords.size(); ++i) {
                    hist_heatmap.at<float>(valid_pixel_coords[i].second, valid_pixel_coords[i].first) = response_hist->predicted_mean[i];
                }
            } else {
                 RCLCPP_ERROR(this->get_logger(), "Historical response invalid or size mismatch!");
            }

            auto response_inst = future_inst.get();
             if (response_inst && response_inst->predicted_mean.size() == valid_pixel_coords.size()) {
                inst_heatmap = cv::Mat::zeros(map_copy.info.height, map_copy.info.width, CV_32F);
                for (size_t i = 0; i < valid_pixel_coords.size(); ++i) {
                    inst_heatmap.at<float>(valid_pixel_coords[i].second, valid_pixel_coords[i].first) = response_inst->predicted_mean[i];
                }
            } else {
                 RCLCPP_ERROR(this->get_logger(), "Instantaneous response invalid or size mismatch!");
            }
            
            RCLCPP_INFO(this->get_logger(), "Both GP responses received. Applying raycasting...");
            
            // FIX: Pass the consistent map_copy to the helper functions
            cv::Mat occluded_hist = applyRaycastingOcclusion(hist_heatmap, hist_samples_copy, map_copy);
            cv::Mat occluded_inst = applyRaycastingOcclusion(inst_heatmap, inst_samples_copy, map_copy);

            publishHeatmaps(occluded_hist, occluded_inst, map_copy);
            
            std::lock_guard<std::mutex> lock(update_mutex_);
            is_updating_ = false;
            RCLCPP_INFO(this->get_logger(), "Update cycle complete.");
        }).detach();
    }
    
    //================================================================================
    // VISUALIZATION & HELPERS
    //================================================================================

    // FIX: Function now takes the map as an argument to ensure consistency
    cv::Mat applyRaycastingOcclusion(const cv::Mat& raw_heatmap, const std::vector<RadiationSample>& sources, const nav_msgs::msg::OccupancyGrid& map_copy) {
        if (raw_heatmap.empty()) return {};
        cv::Mat occluded_heatmap = cv::Mat::zeros(raw_heatmap.size(), raw_heatmap.type());
        cv::Mat map_image(map_copy.info.height, map_copy.info.width, CV_8UC1);
        for(unsigned int i=0; i<map_copy.data.size(); ++i) {
            map_image.data[i] = (map_copy.data[i] > 50) ? 255 : 0; // 255=Wall, 0=Free
        }

        std::vector<cv::Point> source_pixels;
        double res = map_copy.info.resolution;
        double origin_x = map_copy.info.origin.position.x;
        double origin_y = map_copy.info.origin.position.y;
        for(const auto& source : sources) {
            int px = static_cast<int>((source.x - origin_x) / res);
            int py = static_cast<int>((source.y - origin_y) / res);
            if (px >= 0 && px < map_image.cols && py >= 0 && py < map_image.rows) {
                source_pixels.push_back(cv::Point(px, py));
            }
        }

        if (source_pixels.empty()) return occluded_heatmap;

        for (int r = 0; r < raw_heatmap.rows; ++r) {
            for (int c = 0; c < raw_heatmap.cols; ++c) {
                if (map_image.at<uchar>(r, c) == 255) continue;

                for (const auto& source_px : source_pixels) {
                    cv::LineIterator it(map_image, cv::Point(c, r), source_px);
                    bool blocked = false;
                    for(int i = 0; i < it.count; ++i, ++it) {
                        if (map_image.at<uchar>(it.pos()) == 255) {
                            blocked = true;
                            break;
                        }
                    }
                    if (!blocked) {
                        occluded_heatmap.at<float>(r, c) = raw_heatmap.at<float>(r, c);
                        break;
                    }
                }
            }
        }
        return occluded_heatmap;
    }
    
    // FIX: Function now takes the map as an argument
    void publishHeatmaps(const cv::Mat& historical_map, const cv::Mat& instantaneous_map, const nav_msgs::msg::OccupancyGrid& map_copy) {
        cv::Mat base_bgr = createBaseMapImage(map_copy);
        
        std_msgs::msg::Header h;
        h.stamp = this->now();
        h.frame_id = map_frame_;

        publishSingleHeatmap(historical_heatmap_pub_, base_bgr.clone(), historical_map, h);
        publishSingleHeatmap(instantaneous_heatmap_pub_, base_bgr.clone(), instantaneous_map, h);
    }
    
    void publishSingleHeatmap(image_transport::Publisher& pub, const cv::Mat& base_img, const cv::Mat& heatmap_data, const std_msgs::msg::Header& h) {
        if (heatmap_data.empty() || heatmap_data.rows != base_img.rows || heatmap_data.cols != base_img.cols) {
             RCLCPP_WARN(this->get_logger(), "Skipping publish due to empty or mismatched heatmap size.");
             return;
        }

        double min_val, max_val;
        cv::minMaxLoc(heatmap_data, &min_val, &max_val);

        if (max_val < background_threshold_mrem_hr_) max_val = background_threshold_mrem_hr_ + 0.1;
        if (min_val >= max_val) min_val = max_val - 0.1;

        cv::Mat color_heatmap = createColorHeatmapImage(heatmap_data, min_val, max_val);
        
        cv::Mat blended_image;
        cv::addWeighted(base_img, 0.6, color_heatmap, 0.4, 0.0, blended_image);

        pub.publish(*cv_bridge::CvImage(h, "bgr8", blended_image).toImageMsg());
    }

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

    cv::Mat createColorHeatmapImage(const cv::Mat &field, double min_val, double max_val) {
        cv::Mat scaled_field;
        double range = max_val - min_val;
        if (range <= 1e-6) range = 1.0;
        
        field.convertTo(scaled_field, CV_8UC1, 255.0 / range, -255.0 * min_val / range);
        
        cv::Mat color_field;
        cv::applyColorMap(scaled_field, color_field, cv::COLORMAP_JET);

        cv::Mat mask;
        cv::compare(field, background_threshold_mrem_hr_, mask, cv::CMP_LT);
        color_field.setTo(cv::Scalar(0, 0, 0), mask);
        return color_field;
    }

public:
    RadiationHeatmapNode() : Node("radiation_heatmap_node"), 
                             tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>("sensor_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<int>("recent_samples_window_size", 20);
        this->declare_parameter<double>("background_threshold_mrem_hr", 0.1);

        sensor_frame_ = this->get_parameter("sensor_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        recent_samples_window_size_ = this->get_parameter("recent_samples_window_size").as_int();
        background_threshold_mrem_hr_ = this->get_parameter("background_threshold_mrem_hr").as_double();
    }
    
    void init() {
       map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), std::bind(&RadiationHeatmapNode::mapCallback, this, std::placeholders::_1));
       rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/rad", 10, std::bind(&RadiationHeatmapNode::radCallback, this, std::placeholders::_1));

       image_transport::ImageTransport it(this->shared_from_this());
       historical_heatmap_pub_ = it.advertise("/rad_heatmap/historical", 1);
       instantaneous_heatmap_pub_ = it.advertise("/rad_heatmap/instantaneous", 1);

       gp_client_ = this->create_client<rad121_monitor::srv::GPPredict>("predict_gp");
       RCLCPP_INFO(this->get_logger(), "Waiting for predict_gp service...");
       while (!gp_client_->wait_for_service(std::chrono::seconds(1))) {
           if (!rclcpp::ok()) {
               RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
               return;
           }
           RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
       }
       RCLCPP_INFO(this->get_logger(), "Radiation GP monitor initialized.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadiationHeatmapNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}