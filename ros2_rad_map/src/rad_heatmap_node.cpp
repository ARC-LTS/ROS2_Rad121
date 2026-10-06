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
#include <deque>
#include <numeric>
#include <mutex>
#include <thread>
#include <future>
#include <opencv2/opencv.hpp>
#include <map>
#include <filesystem>
#include <queue>
#include <fstream>
#include <limits>

struct RadiationSample
{
    double x, y, value;
    int id;
    bool is_background;
};

cv::Mat createBaseMapImage(const nav_msgs::msg::OccupancyGrid &map);
void addLegendToImage(cv::Mat &image, double min_val, double max_val, const std::string &unit_label);

#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class DynamicRadiationHeatmapNode : public rclcpp::Node
{
public:
    DynamicRadiationHeatmapNode() : Node("dynamic_rad_heatmap_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), current_background_level_(0.0) {}

    void init()
    {
        this->declare_parameter<std::string>("sensor_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<double>("background_threshold_mrem_hr", 0.1);
        this->declare_parameter<std::string>("output_directory", "/tmp");
        this->declare_parameter<int>("raycasting_max_sources", 5);
        this->declare_parameter<double>("raycasting_max_distance_m", 10.0);
        this->declare_parameter<double>("gp_update_frequency", 0.2);

        // Known-field oracle. When true, the published heatmap is replaced by the
        // analytic true radiation field (the simulator's own inverse-square point-
        // source model) over the entire mapped extent, with near-zero uncertainty
        // and no visibility-mask restriction, so the robot has perfect radiation
        // knowledge wherever the map exists rather than having to estimate it from
        // sparse measurements. This is the estimation-quality ablation for the
        // H1/H2 dose-and-safety null: if a perfect field lets the informed strategy
        // beat the radiation-blind baseline on dose efficiency and violations, the
        // exploration null was an estimation limit; if not, it was a strategy limit.
        // false (default) reproduces the GP-estimated pipeline exactly. It is an
        // upper-bound diagnostic, not an operational mode -- a real survey does not
        // know the field a priori. effective_range matches the sensor simulator so
        // the injected field is the one the robot would converge to given perfect
        // measurements.
        this->declare_parameter<bool>("known_field_mode", false);
        this->declare_parameter<double>("known_field_effective_range_m", 2.0);

        sensor_frame_ = this->get_parameter("sensor_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        background_threshold_mrem_hr_ = this->get_parameter("background_threshold_mrem_hr").as_double();
        output_directory_ = this->get_parameter("output_directory").as_string();
        raycasting_max_sources_ = this->get_parameter("raycasting_max_sources").as_int();
        raycasting_max_distance_m_ = this->get_parameter("raycasting_max_distance_m").as_double();
        double gp_update_frequency = this->get_parameter("gp_update_frequency").as_double();
        known_field_mode_ = this->get_parameter("known_field_mode").as_bool();
        known_field_effective_range_ = this->get_parameter("known_field_effective_range_m").as_double();

        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local().reliable();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, std::bind(&DynamicRadiationHeatmapNode::mapCallback, this, std::placeholders::_1));
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/rad", 10, std::bind(&DynamicRadiationHeatmapNode::radCallback, this, std::placeholders::_1));
        
        background_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/rad_field_server/background_radiation", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                this->current_background_level_ = msg->data;
            });

        zone_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/radiation_zone_markers", 10,
            std::bind(&DynamicRadiationHeatmapNode::zoneCallback, this, std::placeholders::_1));

        // Only subscribe to ground truth when the oracle is active. Parses the
        // same /rad_field_server/sources SPHERE markers the evaluator reads;
        // intensity is recovered from marker scale by the inverse of the
        // simulator's normalization (scale = intensity_norm * 0.8 + 0.2,
        // intensity = norm * 7.5 + 2.5).
        if (known_field_mode_) {
            known_src_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
                "/rad_field_server/sources", 10,
                [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                    std::vector<KnownSource> srcs;
                    for (const auto &m : msg->markers) {
                        if (m.action != visualization_msgs::msg::Marker::ADD ||
                            m.type != visualization_msgs::msg::Marker::SPHERE) continue;
                        double norm = (m.scale.x - 0.2) / 0.8;
                        srcs.push_back({m.pose.position.x, m.pose.position.y, norm * 7.5 + 2.5});
                    }
                    std::lock_guard<std::mutex> lock(known_src_mutex_);
                    known_sources_ = std::move(srcs);
                });
            RCLCPP_WARN(this->get_logger(),
                "KNOWN-FIELD ORACLE ACTIVE: heatmap will publish the analytic true field, "
                "not the GP estimate. Localization is near-circular in this mode.");
        }

        image_transport::ImageTransport it(this->shared_from_this());
        historical_heatmap_pub_ = it.advertise("/rad_heatmap/historical_mean", 1);
        historical_std_dev_pub_ = it.advertise("/rad_heatmap/historical_std_dev", 1);
        historical_mean_overlay_pub_ = it.advertise("/rad_heatmap/historical_mean_overlay", 1);
        raw_mean_map_pub_ = it.advertise("/rad_heatmap/raw_mean_map", 1);
        gp_mean_map_pub_ = it.advertise("/rad_heatmap/gp_mean_map", 1);
        gp_std_dev_pub_ = it.advertise("/rad_heatmap/gp_std_dev", 1);

        gp_client_ = this->create_client<ros2_rad_map::srv::GPPredict>("predict_gp");
        source_estimate_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/rad_heatmap/source_estimates",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        // Published each time a stable elevated sample is committed.
        // The exploration node subscribes to this to know when to leave radiation mode.
        stable_source_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/rad_heatmap/stable_source_captured", 10);

        // Latched so any new subscriber immediately sees the last computed boundaries.
        zone_boundary_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/rad_heatmap/zone_boundaries",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

        save_maps_service_ = this->create_service<std_srvs::srv::Trigger>(
            "save_rad_maps",
            std::bind(&DynamicRadiationHeatmapNode::saveMapsCallback, this, std::placeholders::_1, std::placeholders::_2));

        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
            "reset_rad_heatmap",
            std::bind(&DynamicRadiationHeatmapNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
        
        gp_update_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / gp_update_frequency),
            std::bind(&DynamicRadiationHeatmapNode::triggerGPUpdate, this));

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

    // Background estimation (GM tube — Poisson statistics)
    static constexpr size_t BG_BUFFER_SIZE   = 50;
    static constexpr double ELEVATION_SIGMA  = 2.5;   // multiples of σ above background
    static constexpr size_t STABLE_COUNT     = 5;     // readings to average before committing
    static constexpr double STABLE_RADIUS_M  = 1.0;   // max spread of a "stable" cluster (m)

    std::deque<double> bg_buffer_;
    double bg_mean_ = 0.0;
    double bg_std_  = 1.0;   // conservative until buffer fills

    struct PendingPoint { double x, y, value; };
    std::vector<PendingPoint> pending_elevated_;

    bool map_received_ = false;
    bool is_updating_ = false;
    int next_sample_id_ = 0;

    nav_msgs::msg::OccupancyGrid base_map_;
    std::vector<RadiationSample> historical_samples_;
    std::vector<std::vector<cv::Point>> zone_contours_;

    cv::Mat map_image_for_raycasting_;
    cv::Mat latest_historical_mean_image_;
    cv::Mat latest_historical_std_dev_image_;
    cv::Mat raw_mean_data_, raw_std_data_;
    cv::Mat persistent_visibility_mask_;

    // Historical (monotonic) color-scale bounds: the legend/colormap range must
    // only ever expand as new extremes are observed, never shrink back down to
    // whatever the current frame happens to contain — otherwise the scale (and
    // therefore the displayed colors) jitters frame-to-frame instead of giving a
    // stable view of where readings actually sit relative to the whole run.
    double hist_min_mean_ = std::numeric_limits<double>::infinity();
    double hist_max_mean_ = -std::numeric_limits<double>::infinity();
    double hist_min_std_  = std::numeric_limits<double>::infinity();
    double hist_max_std_  = -std::numeric_limits<double>::infinity();

    // Diagnostic state
    struct GPDiagnostics {
        double log_likelihood;
        double lengthscale;
        double variance;
        double noise;
        double mse;
        double prior_mse;
        double rho;
    } latest_diagnostics_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr zone_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr known_src_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_maps_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::TimerBase::SharedPtr gp_update_timer_;
    image_transport::Publisher historical_heatmap_pub_;
    image_transport::Publisher historical_std_dev_pub_;
    image_transport::Publisher historical_mean_overlay_pub_;
    image_transport::Publisher raw_mean_map_pub_;
    image_transport::Publisher gp_mean_map_pub_;
    image_transport::Publisher gp_std_dev_pub_;
    rclcpp::Client<ros2_rad_map::srv::GPPredict>::SharedPtr gp_client_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr background_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr source_estimate_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr stable_source_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zone_boundary_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string sensor_frame_, map_frame_, output_directory_;
    double background_threshold_mrem_hr_;
    double current_background_level_;
    int raycasting_max_sources_;
    double raycasting_max_distance_m_;

    // Known-field oracle state.
    struct KnownSource { double x, y, intensity; };
    bool known_field_mode_{false};
    double known_field_effective_range_{2.0};
    std::vector<KnownSource> known_sources_;
    std::mutex known_src_mutex_;

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

    void resetCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // --- 1. Clear all radiation data under the data lock ---
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            historical_samples_.clear();
            bg_buffer_.clear();
            bg_mean_        = 0.0;
            bg_std_         = 1.0;
            pending_elevated_.clear();
            next_sample_id_ = 0;

            hist_min_mean_ = std::numeric_limits<double>::infinity();
            hist_max_mean_ = -std::numeric_limits<double>::infinity();
            hist_min_std_  = std::numeric_limits<double>::infinity();
            hist_max_std_  = -std::numeric_limits<double>::infinity();

            if (!persistent_visibility_mask_.empty()) {
                persistent_visibility_mask_ = cv::Mat::zeros(
                    persistent_visibility_mask_.size(), CV_8U);
            }

            // Clear cached display images so stale frames don't persist
            latest_historical_mean_image_ = cv::Mat();
            latest_historical_std_dev_image_ = cv::Mat();
            raw_mean_data_ = cv::Mat();
            raw_std_data_  = cv::Mat();
        }

        // --- 2. Clear display contours ---
        {
            std::lock_guard<std::mutex> lock(zone_mutex_);
            zone_contours_.clear();
        }

        // --- 3. Publish DELETEALL to all latched marker topics ---
        auto make_deleteall = [&](const std::string & ns) {
            visualization_msgs::msg::MarkerArray ma;
            visualization_msgs::msg::Marker m;
            m.header.frame_id = map_frame_;
            m.header.stamp    = this->now();
            m.ns     = ns;
            m.action = visualization_msgs::msg::Marker::DELETEALL;
            ma.markers.push_back(m);
            return ma;
        };
        zone_boundary_pub_->publish(make_deleteall("zone_boundaries"));
        source_estimate_pub_->publish(make_deleteall("source_estimates"));
        source_estimate_pub_->publish(make_deleteall("source_estimate_labels"));

        // --- 4. Publish blank float maps so subscribers (e.g. explore node) see a clean slate ---
        if (map_received_) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            cv::Mat blank = cv::Mat::zeros(base_map_.info.height, base_map_.info.width, CV_32F);
            std_msgs::msg::Header h;
            h.stamp    = this->now();
            h.frame_id = map_frame_;
            auto blank_msg = cv_bridge::CvImage(h, "32FC1", blank).toImageMsg();
            raw_mean_map_pub_.publish(*blank_msg);
            gp_mean_map_pub_.publish(*blank_msg);
        }

        response->success = true;
        response->message = "Radiation heatmap reset: all samples, background, and markers cleared.";
        RCLCPP_INFO(this->get_logger(), "Heatmap node reset via service.");
    }

    void saveMapsCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        cv::Mat mean_img, std_dev_img, raw_mean, raw_std;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (latest_historical_mean_image_.empty() || latest_historical_std_dev_image_.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Save request received, but no maps generated.");
                response->success = false;
                response->message = "No historical heatmaps available.";
                return;
            }
            mean_img = latest_historical_mean_image_.clone();
            std_dev_img = latest_historical_std_dev_image_.clone();
            raw_mean = raw_mean_data_.clone();
            raw_std = raw_std_data_.clone();
        }

        // Calculate actual ranges for dynamic legend
        double min_mean, max_mean, min_std, max_std;
        cv::minMaxLoc(raw_mean, &min_mean, &max_mean);
        cv::minMaxLoc(raw_std, &min_std, &max_std);

        // Standardize min values to 0 for cleaner visuals if close to background
        if (min_mean < 0.1) min_mean = 0.0;
        if (max_mean < 0.1) max_mean = 0.1;

        // Add legend using the dynamically detected maximums
        addLegendToImage(mean_img, min_mean, max_mean, "mrem/hr");
        addLegendToImage(std_dev_img, min_std, max_std, "Std. Dev.");

        std::filesystem::path dir(output_directory_);
        std::filesystem::path mean_path = dir / "historical_mean.png";
        std::filesystem::path std_dev_path = dir / "historical_std_dev.png";
        bool success1 = cv::imwrite(mean_path.string(), mean_img);
        bool success2 = cv::imwrite(std_dev_path.string(), std_dev_img);

        // Save GP Diagnostics to CSV
        std::filesystem::path diag_path = dir / "gp_diagnostics.csv";
        bool file_exists = std::filesystem::exists(diag_path);
        std::ofstream diag_out(diag_path.string(), std::ios::app);
        
        if (diag_out.is_open()) {
            if (!file_exists) {
                diag_out << "timestamp,log_marginal_likelihood,kernel_lengthscale,kernel_variance,noise_variance,training_mse,prior_mse,rho\n";
            }
            
            auto now = std::time(nullptr);
            diag_out << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") << ","
                     << latest_diagnostics_.log_likelihood << ","
                     << latest_diagnostics_.lengthscale << ","
                     << latest_diagnostics_.variance << ","
                     << latest_diagnostics_.noise << ","
                     << latest_diagnostics_.mse << ","
                     << latest_diagnostics_.prior_mse << ","
                     << latest_diagnostics_.rho << "\n";
            diag_out.close();
        }

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
        std_msgs::msg::Header h;
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        bool size_changed = false;
        unsigned int old_width = 0, old_height = 0;
        double old_origin_x = 0, old_origin_y = 0;

        if (!base_map_.data.empty()) {
            old_width = base_map_.info.width;
            old_height = base_map_.info.height;
            old_origin_x = base_map_.info.origin.position.x;
            old_origin_y = base_map_.info.origin.position.y;

            if (old_width != msg->info.width || old_height != msg->info.height ||
                old_origin_x != msg->info.origin.position.x || old_origin_y != msg->info.origin.position.y) {
                size_changed = true;
            }
        }

        base_map_ = *msg;
        map_received_ = true;
        
        // Handle Mask Preservation during Map Resizing
        if (persistent_visibility_mask_.empty()) {
            persistent_visibility_mask_ = cv::Mat::zeros(base_map_.info.height, base_map_.info.width, CV_8U);
        }
        else if (size_changed) {
            cv::Mat new_mask = cv::Mat::zeros(base_map_.info.height, base_map_.info.width, CV_8U);
            
            // Map old mask coordinates to new mask coordinates based on origins
            int offset_x = static_cast<int>((old_origin_x - base_map_.info.origin.position.x) / base_map_.info.resolution);
            int offset_y = static_cast<int>((old_origin_y - base_map_.info.origin.position.y) / base_map_.info.resolution);

            for (int y = 0; y < static_cast<int>(old_height); ++y) {
                for (int x = 0; x < static_cast<int>(old_width); ++x) {
                    int nx = x + offset_x;
                    int ny = y + offset_y;
                    if (nx >= 0 && nx < static_cast<int>(base_map_.info.width) && ny >= 0 && ny < static_cast<int>(base_map_.info.height)) {
                        new_mask.at<uchar>(ny, nx) = persistent_visibility_mask_.at<uchar>(y, x);
                    }
                }
            }
            persistent_visibility_mask_ = new_mask;
            RCLCPP_INFO(this->get_logger(), "Persistent mask resized and data preserved. New size: %ux%u", base_map_.info.width, base_map_.info.height);
        }

        map_image_for_raycasting_ = cv::Mat(base_map_.info.height, base_map_.info.width, CV_8UC1);
        for (unsigned int y = 0; y < base_map_.info.height; ++y) {
            for (unsigned int x = 0; x < base_map_.info.width; ++x) {
                int8_t occupancy = base_map_.data[y * base_map_.info.width + x];
                map_image_for_raycasting_.at<uchar>(y, x) = (occupancy > 50) ? 255 : 0;
            }
        }
        
        RCLCPP_INFO_ONCE(this->get_logger(), "Initial map processed and memory allocated.");
    }

    // Recompute bg_mean_ / bg_std_ from bg_buffer_. Must be called under data_mutex_.
    void updateBackgroundStats()
    {
        if (bg_buffer_.empty()) return;
        double sum = std::accumulate(bg_buffer_.begin(), bg_buffer_.end(), 0.0);
        bg_mean_ = sum / bg_buffer_.size();
        if (bg_buffer_.size() > 1) {
            double sq = 0.0;
            for (double v : bg_buffer_) sq += (v - bg_mean_) * (v - bg_mean_);
            bg_std_ = std::sqrt(sq / (bg_buffer_.size() - 1));
        }
    }

    // Commit the current pending_elevated_ cluster as a single averaged sample.
    // Must be called under data_mutex_.
    void commitPendingSample()
    {
        if (pending_elevated_.empty()) return;
        double mx = 0, my = 0, mv = 0;
        for (const auto &p : pending_elevated_) { mx += p.x; my += p.y; mv += p.value; }
        double n = static_cast<double>(pending_elevated_.size());
        RadiationSample s = {mx / n, my / n, mv / n, next_sample_id_++, false};
        historical_samples_.push_back(s);
        RCLCPP_INFO(this->get_logger(),
            "Committed elevated sample #%d at (%.2f, %.2f): %.3f mrem/hr (avg of %zu readings)",
            s.id, s.x, s.y, s.value, pending_elevated_.size());

        // Notify the exploration node that a stable measurement has been captured
        // so it can exit radiation mode and resume spatial exploration.
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.point.x = s.x;
        msg.point.y = s.y;
        msg.point.z = 0.0;
        stable_source_pub_->publish(msg);

        pending_elevated_.clear();
    }

    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!map_received_ || msg->data.size() < 2) return;

        double reading = msg->data[1]; // mrem/hr

        geometry_msgs::msg::PoseStamped pose_in_sensor, pose_in_map;
        pose_in_sensor.header.frame_id = sensor_frame_;
        pose_in_sensor.header.stamp = rclcpp::Time(0);
        try {
            tf_buffer_.transform(pose_in_sensor, pose_in_map, map_frame_, std::chrono::milliseconds(500));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
            return;
        }
        double x = pose_in_map.pose.position.x;
        double y = pose_in_map.pose.position.y;

        std::lock_guard<std::mutex> lock(data_mutex_);

        // Elevation threshold: background mean + 2.5 * max(empirical σ, Poisson σ).
        // GM tube counts follow Poisson statistics, so the irreducible noise floor is
        // sqrt(mean) — we use whichever is larger so the threshold is never too tight.
        double poisson_std = std::sqrt(std::max(bg_mean_, 0.01));
        double threshold   = bg_mean_ + ELEVATION_SIGMA * std::max(bg_std_, poisson_std);

        if (reading <= threshold) {
            // --- Background reading ---
            // Update rolling background estimate for threshold calculation.
            bg_buffer_.push_back(reading);
            if (bg_buffer_.size() > BG_BUFFER_SIZE) bg_buffer_.pop_front();
            updateBackgroundStats();

            // Commit any partial elevated cluster — robot has left the hot region.
            if (!pending_elevated_.empty()) commitPendingSample();

            // Also store sparsely in historical_samples_ so the GP has anchor points
            // showing where radiation is LOW. Without these the GP extrapolates source
            // readings into unvisited background regions and the whole map turns hot.
            // Use a coarser 1 m spacing — enough to anchor the GP, won't flood the buffer.
            bool far_enough = true;
            if (!historical_samples_.empty()) {
                const auto &last_bg = historical_samples_.back();
                double d2 = std::pow(last_bg.x - x, 2) + std::pow(last_bg.y - y, 2);
                if (d2 < 1.0) far_enough = false;  // 1 m spacing for background
            }
            if (!far_enough) return;

            RadiationSample s = {x, y, reading, next_sample_id_++, true};
            historical_samples_.push_back(s);

            // When the buffer is full evict only background-level samples — never
            // elevated ones — so committed source readings stay locked in.
            // Restricting the search to is_background entries (via +inf for
            // elevated ones) guarantees this never selects an elevated sample,
            // even after every background anchor has already been evicted.
            if (historical_samples_.size() > 1000) {
                auto evict = std::min_element(
                    historical_samples_.begin(), historical_samples_.end(),
                    [](const RadiationSample &a, const RadiationSample &b) {
                        double av = a.is_background ? a.value : std::numeric_limits<double>::infinity();
                        double bv = b.is_background ? b.value : std::numeric_limits<double>::infinity();
                        return av < bv;
                    });
                if (evict != historical_samples_.end() && evict->is_background) {
                    historical_samples_.erase(evict);
                }
            }
            return;
        }

        // --- Elevated reading ---

        // If the robot has wandered too far from where this cluster started, commit first.
        if (!pending_elevated_.empty()) {
            const auto &origin = pending_elevated_.front();
            double d2 = std::pow(origin.x - x, 2) + std::pow(origin.y - y, 2);
            if (d2 > STABLE_RADIUS_M * STABLE_RADIUS_M) {
                commitPendingSample();
            }
        }

        // Spatial downsampling within the cluster (0.2 m).
        if (!pending_elevated_.empty()) {
            const auto &last = pending_elevated_.back();
            double d2 = std::pow(last.x - x, 2) + std::pow(last.y - y, 2);
            if (d2 < 0.04) return;
        }

        pending_elevated_.push_back({x, y, reading});

        // Once we have enough stable readings, commit the averaged sample.
        if (pending_elevated_.size() >= STABLE_COUNT) {
            commitPendingSample();
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
        cv::Mat visibility_mask_copy;
        std::vector<RadiationSample> hist_samples_copy;
        double bg_mean_copy, bg_std_copy;
        bool can_update = false;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (map_received_ && !historical_samples_.empty())
            {
                can_update = true;
                map_copy = base_map_;
                raycasting_map_copy = map_image_for_raycasting_.clone();
                hist_samples_copy = historical_samples_;
                bg_mean_copy = bg_mean_;
                bg_std_copy  = bg_std_;

                if (persistent_visibility_mask_.empty()) {
                    persistent_visibility_mask_ = cv::Mat::zeros(map_copy.info.height, map_copy.info.width, CV_8U);
                }
                visibility_mask_copy = persistent_visibility_mask_.clone();
            }
        }

        if (!can_update)
        {
            std::lock_guard<std::mutex> lock(update_mutex_);
            is_updating_ = false;
            return;
        }

        auto request = std::make_shared<ros2_rad_map::srv::GPPredict::Request>();
        request->mean_value = current_background_level_;
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
                // Predict at all KNOWN cells (free AND occupied/shelves), skip unknown (-1).
                // This allows the GP to show radiation inside shelf structures where sources
                // may be placed, since the simulator uses 1/r² with no obstacle attenuation.
                if (map_copy.data[y * map_copy.info.width + x] >= 0)
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

        std::thread([this, future = std::move(future), map_copy, raycasting_map_copy, visibility_mask_copy, valid_pixel_coords, hist_samples_copy, bg_mean_copy, bg_std_copy]() mutable {
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

                // Update Diagnostics
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    latest_diagnostics_.log_likelihood = response->log_marginal_likelihood;
                    latest_diagnostics_.lengthscale = response->kernel_lengthscale;
                    latest_diagnostics_.variance = response->kernel_variance;
                    latest_diagnostics_.noise = response->noise_variance;
                    latest_diagnostics_.mse = response->training_mse;
                    latest_diagnostics_.prior_mse = response->prior_mse;
                    latest_diagnostics_.rho = response->rho;
                }

                // Update the persistent mask locally first
                cv::Mat local_mask = visibility_mask_copy.clone();
                updatePersistentMask(local_mask, hist_samples_copy, map_copy, raycasting_map_copy);

                // Improved Gap Filling: Use a combination of visibility mask and uncertainty threshold
                // to allow GP to interpolate into gaps while still masking truly unknown/distant areas.
                double bg = current_background_level_;
                cv::Mat hist_mean_occluded(hist_mean_raw.size(), hist_mean_raw.type(), cv::Scalar(bg));
                cv::Mat hist_std_occluded(hist_std_raw.size(), hist_std_raw.type(), cv::Scalar(0.0));

                for (int r = 0; r < hist_mean_raw.rows; ++r) {
                    for (int c = 0; c < hist_mean_raw.cols; ++c) {
                        float uncertainty = hist_std_raw.at<float>(r, c);
                        bool is_visible = (local_mask.at<uchar>(r, c) == 255);

                        // Map row r to map-grid row (image is y-flipped relative to map).
                        // We need the ORIGINAL map y (bottom-up) from the image row (top-down).
                        unsigned int map_grid_y = map_copy.info.height - 1 - r;
                        int8_t occ = map_copy.data[map_grid_y * map_copy.info.width + c];
                        bool is_occupied = (occ >= 50);

                        // Free-space cells only get GP values when the flood-fill can reach them from
                        // a sample position (is_visible). Occupied cells (shelves) are never reachable
                        // by the flood-fill, so they get a fallback when uncertainty is low enough.
                        // This prevents the GP from extrapolating into unexplored frontier areas.
                        if (is_visible || (is_occupied && uncertainty < 2.0f)) {
                            hist_mean_occluded.at<float>(r, c) = hist_mean_raw.at<float>(r, c);
                            hist_std_occluded.at<float>(r, c) = hist_std_raw.at<float>(r, c);
                        }
                    }
                }

                // Known-field oracle: overwrite the estimated field with the
                // analytic true field over every mapped cell, bypassing the
                // visibility mask so radiation is known wherever the map extends
                // (not only where it has been measured). Uses the simulator's own
                // point-source model, intensity * min(1, range^2/r^2), summed over
                // sources plus the current background estimate. std is pinned near
                // zero: the field is treated as certain.
                if (known_field_mode_) {
                    std::vector<KnownSource> srcs;
                    { std::lock_guard<std::mutex> lock(known_src_mutex_); srcs = known_sources_; }
                    const double res = map_copy.info.resolution;
                    const double ox = map_copy.info.origin.position.x;
                    const double oy = map_copy.info.origin.position.y;
                    const double er_sq = known_field_effective_range_ * known_field_effective_range_;
                    const double bg_lvl = current_background_level_;
                    for (int r = 0; r < hist_mean_occluded.rows; ++r) {
                        for (int c = 0; c < hist_mean_occluded.cols; ++c) {
                            unsigned int map_grid_y = map_copy.info.height - 1 - r;
                            double wx = ox + (c + 0.5) * res;
                            double wy = oy + (map_grid_y + 0.5) * res;
                            double val = bg_lvl;
                            for (const auto &s : srcs) {
                                double dx = wx - s.x, dy = wy - s.y;
                                double dsq = dx * dx + dy * dy;
                                double ratio = (dsq > 1e-9) ? std::min(1.0, er_sq / dsq) : 1.0;
                                val += s.intensity * ratio;
                            }
                            hist_mean_occluded.at<float>(r, c) = static_cast<float>(val);
                            hist_std_occluded.at<float>(r, c) = 0.01f;
                        }
                    }
                }

                // Final Smoothing Pass to fill small micro-gaps and soften edges
                cv::GaussianBlur(hist_mean_occluded, hist_mean_occluded, cv::Size(3, 3), 0);
                cv::GaussianBlur(hist_std_occluded, hist_std_occluded, cv::Size(3, 3), 0);

                // Push local mask back to global persistent mask
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    if (persistent_visibility_mask_.size() == local_mask.size()) {
                        cv::bitwise_or(persistent_visibility_mask_, local_mask, persistent_visibility_mask_);
                    }
                }

                // Compute zone boundaries from the GP map and update the overlay contours.
                auto display_contours = publishZoneBoundaries(
                    hist_mean_occluded, map_copy, bg_mean_copy, bg_std_copy);
                {
                    std::lock_guard<std::mutex> lk(zone_mutex_);
                    zone_contours_ = std::move(display_contours);
                }

                publishHeatmaps(hist_mean_occluded, hist_std_occluded, map_copy);
                publishSourceEstimates(hist_mean_occluded, map_copy, bg_mean_copy, bg_std_copy);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception caught in GP update thread: %s", e.what());
            }

            std::lock_guard<std::mutex> lock(update_mutex_);
            is_updating_ = false;
        }).detach();
    }

    void updatePersistentMask(cv::Mat &mask, const std::vector<RadiationSample> &samples,
                              const nav_msgs::msg::OccupancyGrid &map_info, const cv::Mat &raycasting_map)
    {
        std::vector<RadiationSample> effective_sources;
        if (samples.size() > 20) {
            effective_sources.assign(samples.end() - 20, samples.end());
        } else {
            effective_sources = samples;
        }

        double res = map_info.info.resolution;
        double origin_x = map_info.info.origin.position.x;
        double origin_y = map_info.info.origin.position.y;

        for (const auto &source : effective_sources)
        {
            int start_px = static_cast<int>((source.x - origin_x) / res);
            int start_py = static_cast<int>((source.y - origin_y) / res);
            
            if (start_px < 0 || start_px >= raycasting_map.cols || start_py < 0 || start_py >= raycasting_map.rows)
                continue;

            std::queue<cv::Point> q;
            cv::Mat local_visited = cv::Mat::zeros(mask.size(), CV_8U);
            cv::Point start_node(start_px, start_py);

            q.push(start_node);
            local_visited.at<uchar>(start_node) = 1;
            mask.at<uchar>(start_node) = 255;

            while (!q.empty())
            {
                cv::Point current = q.front();
                q.pop();

                const cv::Point neighbors[] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
                for (const auto &offset : neighbors)
                {
                    cv::Point next = current + offset;

                    if (next.x < 0 || next.x >= raycasting_map.cols || next.y < 0 || next.y >= raycasting_map.rows)
                        continue;
                    if (local_visited.at<uchar>(next) == 1)
                        continue;
                    if (raycasting_map.at<uchar>(next) == 255)
                        continue;

                    double dist_m = cv::norm(next - start_node) * res;
                    if (dist_m > raycasting_max_distance_m_)
                        continue;

                    local_visited.at<uchar>(next) = 1;
                    mask.at<uchar>(next) = 255;
                    q.push(next);
                }
            }
        }
    }

    // Compute the boundary of every elevated zone in the GP mean map and publish:
    //   1. A MarkerArray of LINE_STRIP markers on /rad_heatmap/zone_boundaries (RViz 3-D)
    //   2. Display-image-space contours for the heatmap overlay (returned for caller to use)
    //
    // Algorithm: threshold → MORPH_CLOSE (fill gaps) → MORPH_OPEN (remove specks)
    //            → findContours(RETR_EXTERNAL) → approxPolyDP (smooth grid aliasing)
    //
    // gp_mean is in map-grid coordinates (no y-flip): pixel (col, row) = world (x, y).
    std::vector<std::vector<cv::Point>> publishZoneBoundaries(
        const cv::Mat &gp_mean,
        const nav_msgs::msg::OccupancyGrid &map_info,
        double bg_mean, double bg_std)
    {
        if (gp_mean.empty()) return {};

        double poisson_std = std::sqrt(std::max(bg_mean, 0.01));
        double threshold   = bg_mean + ELEVATION_SIGMA * std::max(bg_std, poisson_std);

        // Need at least some background data before the threshold is meaningful.
        if (bg_mean < 0.01 && bg_std < 0.01) return {};

        // --- 1. Binary mask ---
        cv::Mat binary;
        cv::threshold(gp_mean, binary, threshold, 255.0, cv::THRESH_BINARY);
        binary.convertTo(binary, CV_8U);

        // --- 2. Morphological cleaning ---
        // CLOSE: fills small holes / bridges narrow gaps between elevated blobs
        // OPEN:  removes isolated speck noise below ~3-cell diameter
        cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        cv::Mat open_kernel  = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, close_kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN,  open_kernel);

        // --- 3. Find outermost contours only ---
        std::vector<std::vector<cv::Point>> raw_contours;
        cv::findContours(binary, raw_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // --- 4. approxPolyDP: smooth grid-aliasing staircase ---
        // epsilon=3 px keeps the shape accurate while removing pixel-level jaggedness.
        std::vector<std::vector<cv::Point>> smooth;
        for (const auto &c : raw_contours) {
            if (std::abs(cv::contourArea(c)) < 9.0) continue;  // skip tiny noise
            std::vector<cv::Point> approx;
            cv::approxPolyDP(c, approx, 3.0 /*px*/, true /*closed*/);
            if (approx.size() >= 3) smooth.push_back(approx);
        }

        // --- 5. Publish LINE_STRIP markers for RViz ---
        double res      = map_info.info.resolution;
        double origin_x = map_info.info.origin.position.x;
        double origin_y = map_info.info.origin.position.y;
        int    height   = static_cast<int>(map_info.info.height);

        visualization_msgs::msg::MarkerArray markers;
        {
            visualization_msgs::msg::Marker del;
            del.header.frame_id = map_frame_;
            del.header.stamp    = this->now();
            del.ns              = "zone_boundaries";
            del.action          = visualization_msgs::msg::Marker::DELETEALL;
            markers.markers.push_back(del);
        }

        int id = 0;
        for (const auto &contour : smooth) {
            visualization_msgs::msg::Marker line;
            line.header.frame_id    = map_frame_;
            line.header.stamp       = this->now();
            line.ns                 = "zone_boundaries";
            line.id                 = id++;
            line.type               = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action             = visualization_msgs::msg::Marker::ADD;
            line.scale.x            = 0.15;   // line width in metres
            line.color.r = 1.0f; line.color.g = 0.45f; line.color.b = 0.0f; line.color.a = 1.0f;
            line.pose.orientation.w = 1.0;
            line.lifetime           = rclcpp::Duration::from_seconds(0);  // persist until replaced

            for (const auto &pt : contour) {
                // gp_mean pixel (col=pt.x, row=pt.y) → world coords (no y-flip needed)
                geometry_msgs::msg::Point wp;
                wp.x = origin_x + (pt.x + 0.5) * res;
                wp.y = origin_y + (pt.y + 0.5) * res;
                wp.z = 0.05;
                line.points.push_back(wp);
            }
            if (!line.points.empty()) line.points.push_back(line.points.front());  // close loop
            markers.markers.push_back(line);
        }
        zone_boundary_pub_->publish(markers);

        // --- 6. Return display-image-space contours (y-flipped) for heatmap overlay ---
        std::vector<std::vector<cv::Point>> display_contours;
        display_contours.reserve(smooth.size());
        for (const auto &c : smooth) {
            std::vector<cv::Point> dc;
            dc.reserve(c.size());
            for (const auto &pt : c)
                dc.push_back(cv::Point(pt.x, height - 1 - pt.y));
            display_contours.push_back(std::move(dc));
        }
        return display_contours;
    }

    // Find local maxima in the GP mean map and publish a MarkerArray of source estimates.
    // gp_mean is in map-grid coordinates (not y-flipped): pixel (col, row) → world (x, y).
    void publishSourceEstimates(const cv::Mat &gp_mean,
                                const nav_msgs::msg::OccupancyGrid &map_info,
                                double bg_mean, double bg_std)
    {
        double poisson_std = std::sqrt(std::max(bg_mean, 0.01));
        double threshold   = bg_mean + ELEVATION_SIGMA * std::max(bg_std, poisson_std);

        // Threshold → binary mask of elevated regions
        cv::Mat elevated;
        cv::threshold(gp_mean, elevated, threshold, 1.0, cv::THRESH_BINARY);
        elevated.convertTo(elevated, CV_8U, 255.0);

        // Find connected components
        cv::Mat labels, stats, centroids;
        int n = cv::connectedComponentsWithStats(elevated, labels, stats, centroids);

        visualization_msgs::msg::MarkerArray markers;

        // Clear previous markers
        visualization_msgs::msg::Marker del;
        del.header.frame_id = map_frame_;
        del.header.stamp = this->now();
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(del);

        double res      = map_info.info.resolution;
        double origin_x = map_info.info.origin.position.x;
        double origin_y = map_info.info.origin.position.y;
        int    height   = static_cast<int>(map_info.info.height);

        int marker_id = 0;
        for (int i = 1; i < n; ++i) {  // label 0 is background
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < 9) continue;  // reject tiny noise clusters

            // Find the pixel with the peak GP value inside this component
            cv::Mat component_mask = (labels == i);
            double min_v, max_v;
            cv::Point max_loc;
            cv::minMaxLoc(gp_mean, &min_v, &max_v, nullptr, &max_loc, component_mask);

            // Convert map-grid pixel to world coordinates.
            // hist_mean_raw uses map-grid coords directly (no y-flip), so:
            //   world_x = origin_x + (col + 0.5) * res
            //   world_y = origin_y + (row + 0.5) * res
            double wx = origin_x + (max_loc.x + 0.5) * res;
            double wy = origin_y + (max_loc.y + 0.5) * res;

            // Sphere at the estimated source location
            visualization_msgs::msg::Marker sphere;
            sphere.header.frame_id = map_frame_;
            sphere.header.stamp    = this->now();
            sphere.ns              = "source_estimates";
            sphere.id              = marker_id++;
            sphere.type            = visualization_msgs::msg::Marker::SPHERE;
            sphere.action          = visualization_msgs::msg::Marker::ADD;
            sphere.pose.position.x = wx;
            sphere.pose.position.y = wy;
            sphere.pose.position.z = 0.5;
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.6;
            sphere.color.r = 1.0f; sphere.color.g = 0.8f; sphere.color.b = 0.0f; sphere.color.a = 0.9f;
            markers.markers.push_back(sphere);

            // Text label showing estimated peak dose rate
            visualization_msgs::msg::Marker text;
            text.header        = sphere.header;
            text.ns            = "source_estimate_labels";
            text.id            = marker_id++;
            text.type          = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action        = visualization_msgs::msg::Marker::ADD;
            text.pose.position = sphere.pose.position;
            text.pose.position.z = 1.3;
            text.pose.orientation.w = 1.0;
            text.scale.z       = 0.35;
            text.color.r = text.color.g = text.color.b = 1.0f; text.color.a = 1.0f;
            char buf[64];
            std::snprintf(buf, sizeof(buf), "~%.2f mrem/hr", max_v);
            text.text = buf;
            markers.markers.push_back(text);
        }

        source_estimate_pub_->publish(markers);
    }

    void publishHeatmaps(const cv::Mat &hist_mean, const cv::Mat &hist_std, const nav_msgs::msg::OccupancyGrid &map_copy)
    {
        cv::Mat base_bgr = createBaseMapImage(map_copy);
        std_msgs::msg::Header h;
        h.stamp = this->now();
        h.frame_id = map_frame_;

        if (!hist_mean.empty()) {
            raw_mean_map_pub_.publish(*cv_bridge::CvImage(h, "32FC1", hist_mean).toImageMsg());
            gp_mean_map_pub_.publish(*cv_bridge::CvImage(h, "32FC1", hist_mean).toImageMsg());
            std::lock_guard<std::mutex> d_lock(data_mutex_);
            raw_mean_data_ = hist_mean.clone();
        }
        
        if (!hist_std.empty()) {
            gp_std_dev_pub_.publish(*cv_bridge::CvImage(h, "32FC1", hist_std).toImageMsg());
            std::lock_guard<std::mutex> d_lock(data_mutex_);
            raw_std_data_ = hist_std.clone();
        }

        cv::Mat flipped_hist_mean, flipped_hist_std;
        if (!hist_mean.empty()) cv::flip(hist_mean, flipped_hist_mean, 0);
        if (!hist_std.empty()) cv::flip(hist_std, flipped_hist_std, 0);

        cv::Mat free_space_mask(map_copy.info.height, map_copy.info.width, CV_8UC1);
        cv::Mat wall_mask(map_copy.info.height, map_copy.info.width, CV_8UC1);
        
        for (unsigned int y = 0; y < map_copy.info.height; ++y) {
            for (unsigned int x = 0; x < map_copy.info.width; ++x) {
                unsigned int image_y = map_copy.info.height - 1 - y;
                int8_t occupancy = map_copy.data[y * map_copy.info.width + x];

                // Include occupied cells (shelves) in the painted area so that sources
                // inside shelf structures are visible on the heatmap.
                // Unknown cells (-1) are still excluded — no data, no prediction.
                free_space_mask.at<uchar>(image_y, x) = (occupancy >= 0) ? 255 : 0;
                wall_mask.at<uchar>(image_y, x) = (occupancy == -1) ? 255 : 0;
            }
        }

        // Expand the historical (monotonic) color-scale bounds with whatever this
        // frame contains. These only ever grow — they never shrink back down just
        // because the current frame's range happens to be narrower — so the legend
        // and colormap stay stable across the run instead of jittering every cycle.
        if (!hist_mean.empty()) {
            double cur_min, cur_max;
            cv::minMaxLoc(hist_mean, &cur_min, &cur_max);
            hist_min_mean_ = std::min(hist_min_mean_, cur_min);
            hist_max_mean_ = std::max(hist_max_mean_, cur_max);
        }
        if (!hist_std.empty()) {
            double cur_min, cur_max;
            cv::minMaxLoc(hist_std, &cur_min, &cur_max);
            hist_min_std_ = std::min(hist_min_std_, cur_min);
            hist_max_std_ = std::max(hist_max_std_, cur_max);
        }

        // 1. Overlay topic: No legend, 1:1 scaling with map
        auto mean_overlay = createBlendedMap(base_bgr, flipped_hist_mean, "mrem/hr", background_threshold_mrem_hr_, free_space_mask, false, hist_min_mean_, hist_max_mean_);
        if (!mean_overlay.empty()) {
            historical_mean_overlay_pub_.publish(*cv_bridge::CvImage(h, "bgr8", mean_overlay).toImageMsg());
        }

        // 2. Display topic: Includes adaptive legend
        auto mean_display = createBlendedMap(base_bgr, flipped_hist_mean, "mrem/hr", background_threshold_mrem_hr_, free_space_mask, true, hist_min_mean_, hist_max_mean_);
        if (!mean_display.empty())
        {
            std::lock_guard<std::mutex> lock(zone_mutex_);
            if (!zone_contours_.empty()) {
                cv::drawContours(mean_display, zone_contours_, -1, cv::Scalar(0, 165, 255), 2);
            }
            historical_heatmap_pub_.publish(*cv_bridge::CvImage(h, "bgr8", mean_display).toImageMsg());

            std::lock_guard<std::mutex> d_lock(data_mutex_);
            latest_historical_mean_image_ = mean_display.clone();
        }

        auto blended_hist_std = createBlendedMap(base_bgr, flipped_hist_std, "Std. Dev.", 0.0, free_space_mask, true, hist_min_std_, hist_max_std_);
        if (!blended_hist_std.empty())
        {
            std::lock_guard<std::mutex> lock(zone_mutex_);
            if (!zone_contours_.empty()) {
                cv::drawContours(blended_hist_std, zone_contours_, -1, cv::Scalar(0, 165, 255), 2);
            }
            historical_std_dev_pub_.publish(*cv_bridge::CvImage(h, "bgr8", blended_hist_std).toImageMsg());

            std::lock_guard<std::mutex> d_lock(data_mutex_);
            latest_historical_std_dev_image_ = blended_hist_std.clone();
        }
    }

    cv::Mat createBlendedMap(const cv::Mat &base_img, const cv::Mat &data, const std::string &unit, double threshold, const cv::Mat &mask, bool add_legend, double min_val, double max_val)
    {
        if (data.empty())
            return {};

        if (max_val < threshold)
            max_val = threshold + 0.1;
        if (max_val - min_val < 1e-6)
            max_val = min_val + 0.1;

        cv::Mat color_heatmap = createColorHeatmapImage(data, min_val, max_val, threshold);
        
        // Smart Blending: Only apply radiation color where mask is true (free space)
        cv::Mat blended_image = base_img.clone();
        cv::Mat colored_roi;
        cv::addWeighted(base_img, 0.4, color_heatmap, 0.6, 0.0, colored_roi);
        colored_roi.copyTo(blended_image, mask);

        if (add_legend) {
            addLegendToImage(blended_image, min_val, max_val, unit);
        }

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