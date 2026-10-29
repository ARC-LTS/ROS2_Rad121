#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <algorithm>
#include "ros2_rad_map/srv/find_peaks.hpp"
#include "ros2_rad_map/msg/rad_zone.hpp"
#include "ros2_rad_map/msg/rad_zone_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <vector>
#include <deque>
#include <mutex>
#include <iomanip>
#include <cmath>
#include "std_msgs/msg/float64.hpp"

// Historical sample buffer cap: previously unbounded, growing (and getting
// copied wholesale + re-marshalled into a service request) for the entire
// mission. Mirrors gp_predictor_node.py's equivalent 1000-sample cap.
static constexpr size_t kMaxHistoricalSamples = 1000;

std_msgs::msg::ColorRGBA valueToColor(double value, double min_val, double max_val) {
    std_msgs::msg::ColorRGBA color;
    double normalized_value = (value - min_val) / (max_val - min_val);
    normalized_value = std::max(0.0, std::min(1.0, normalized_value));

    if (normalized_value < 0.25) {
        color.r = 0.0;
        color.g = normalized_value * 4.0;
        color.b = 1.0;
    } else if (normalized_value < 0.5) {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 1.0 - (normalized_value - 0.25) * 4.0;
    } else if (normalized_value < 0.75) {
        color.r = (normalized_value - 0.5) * 4.0;
        color.g = 1.0;
        color.b = 0.0;
    } else {
        color.r = 1.0;
        color.g = 1.0 - (normalized_value - 0.75) * 4.0;
        color.b = 0.0;
    }
    color.a = 1.0;
    return color;
}

struct RadSample {
    double x, y, value;
};

class PeakFinderClientNode : public rclcpp::Node
{
public:
    PeakFinderClientNode() : Node("peak_finder_client_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>("sensor_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<int>("num_starts", 30);
        this->declare_parameter<double>("marker_scale", 0.5);
        this->declare_parameter<double>("matching_threshold_m", 1.5);
        this->declare_parameter<double>("min_peak_mrem_hr", 1.5);
        // A peak must be found in the same location at least this many consecutive
        // service calls before it is promoted to a "confirmed source".
        // At the default 5 s poll interval, 3 = confirmed after ~15 s of stability.
        this->declare_parameter<int>("source_confirmation_count", 3);
        // Seconds between /find_rad_peaks calls. Confirmation needs
        // source_confirmation_count consecutive hits, so at the old hardcoded
        // 15 s a source first seen mid-mission needed 45 s of stability -- on
        // a short mission (66 s on rectangle_obst_1, 2026-10-29) the second
        // source was never confirmed before return-to-home. 5 s confirms in ~15 s.
        this->declare_parameter<double>("peak_poll_interval_s", 5.0);

        sensor_frame_ = this->get_parameter("sensor_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        num_starts_ = this->get_parameter("num_starts").as_int();
        marker_scale_ = this->get_parameter("marker_scale").as_double();
        matching_threshold_m_ = this->get_parameter("matching_threshold_m").as_double();
        min_peak_mrem_hr_ = this->get_parameter("min_peak_mrem_hr").as_double();
        source_confirmation_count_ = this->get_parameter("source_confirmation_count").as_int();

        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rad", 10, std::bind(&PeakFinderClientNode::radCallback, this, std::placeholders::_1));

        background_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/rad_field_server/background_radiation", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                this->current_background_level_ = msg->data;
            });

        peak_finder_client_ = this->create_client<ros2_rad_map::srv::FindPeaks>("/find_rad_peaks");

        // Candidate peaks: all zones above threshold, whether confirmed or not
        zone_waypoints_pub_ = this->create_publisher<ros2_rad_map::msg::RadZoneArray>("/radiation_zones", 10);
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/radiation_peak_markers", 10);

        // Confirmed sources: peaks that have recurred in the same location >= source_confirmation_count times
        confirmed_sources_pub_ = this->create_publisher<ros2_rad_map::msg::RadZoneArray>("/confirmed_rad_sources", 10);
        confirmed_source_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/confirmed_rad_source_markers", 10);

        const double poll_s = this->get_parameter("peak_poll_interval_s").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(std::max(0.5, poll_s) * 1000.0)),
            std::bind(&PeakFinderClientNode::callPeakFinderService, this));

        RCLCPP_INFO(this->get_logger(), "Peak Finder Client initialized. Confirmation threshold: %d detections.", source_confirmation_count_);
        if (!peak_finder_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Service /find_rad_peaks not available. Exiting.");
            rclcpp::shutdown();
        }
    }

private:
    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) return;

        geometry_msgs::msg::PoseStamped pose_in_sensor, pose_in_map;
        try {
            pose_in_sensor.header.frame_id = sensor_frame_;
            pose_in_sensor.header.stamp = rclcpp::Time(0);
            // No timeout (defaults to 0): this ran at /rad's publish rate and
            // a 500ms blocking wait here stalled every other callback on this
            // node's (default single-threaded) executor. TF is virtually
            // always already available by the time a sample arrives; if not,
            // skip this one sample rather than block.
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

    void callPeakFinderService()
    {
        if (is_service_running_) {
            return;
        }

        std::deque<RadSample> samples_copy;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (historical_samples_.empty()) {
                return;
            }
            samples_copy = historical_samples_;
        }

        is_service_running_ = true;
        auto request = std::make_shared<ros2_rad_map::srv::FindPeaks::Request>();
        request->num_starts = num_starts_;
        request->mean_value = current_background_level_;

        for (const auto &s : samples_copy) {
            request->observed_x.push_back(s.x);
            request->observed_y.push_back(s.y);
            request->observed_values_mean.push_back(s.value);
        }

        peak_finder_client_->async_send_request(request, [this](rclcpp::Client<ros2_rad_map::srv::FindPeaks>::SharedFuture future) {
            try {
                auto response = future.get();
                // Single timestamp for all markers in this response batch so rviz2
                // sees a coherent update across every namespace simultaneously.
                auto stamp = this->get_clock()->now();

                std::lock_guard<std::mutex> lock(data_mutex_);

                std::vector<bool> matched_new_peaks(response->peaks.size(), false);
                std::vector<bool> matched_persistent_zones(persistent_zones_.size(), false);

                // Match each new peak to the nearest existing zone within matching_threshold_m_.
                for (size_t i = 0; i < response->peaks.size(); ++i) {
                    const auto& new_peak = response->peaks[i];
                    double min_dist_sq = std::pow(matching_threshold_m_, 2);
                    int best_match_idx = -1;

                    for (size_t j = 0; j < persistent_zones_.size(); ++j) {
                        if (matched_persistent_zones[j]) continue;

                        double dx = new_peak.x - persistent_zones_[j].center.x;
                        double dy = new_peak.y - persistent_zones_[j].center.y;
                        double dist_sq = dx*dx + dy*dy;

                        if (dist_sq < min_dist_sq) {
                            min_dist_sq = dist_sq;
                            best_match_idx = j;
                        }
                    }

                    if (best_match_idx != -1) {
                        // Running-average position update: the estimate converges toward the
                        // true source location as more detections accumulate.
                        double n = static_cast<double>(zone_detection_counts_[best_match_idx]);
                        persistent_zones_[best_match_idx].center.x =
                            (persistent_zones_[best_match_idx].center.x * n + new_peak.x) / (n + 1.0);
                        persistent_zones_[best_match_idx].center.y =
                            (persistent_zones_[best_match_idx].center.y * n + new_peak.y) / (n + 1.0);
                        // Keep the peak radiation value seen at this location — never average down.
                        persistent_zones_[best_match_idx].center.z =
                            std::max(persistent_zones_[best_match_idx].center.z, new_peak.z);

                        zone_detection_counts_[best_match_idx]++;

                        matched_new_peaks[i] = true;
                        matched_persistent_zones[best_match_idx] = true;
                    }
                }

                // Unmatched new peaks become new candidate zones (count = 1).
                for (size_t i = 0; i < response->peaks.size(); ++i) {
                    if (!matched_new_peaks[i]) {
                        ros2_rad_map::msg::RadZone new_zone;
                        new_zone.label = std::string(1, next_zone_label_++);
                        new_zone.center = response->peaks[i];
                        persistent_zones_.push_back(new_zone);
                        zone_detection_counts_.push_back(1);
                        zone_miss_counts_.push_back(0);
                    }
                }

                // Forget zones the GP has stopped reporting. A transient bump
                // between two sources got its 3 hits early, then was never seen
                // again but stayed "confirmed" (a fake third source on the map
                // for the whole run, auto3 2026-10-29). zone_miss_counts_ is
                // parallel to persistent_zones_; matched_persistent_zones only
                // covers zones that existed before this poll's additions.
                for (size_t j = 0; j < matched_persistent_zones.size(); ++j) {
                    zone_miss_counts_[j] = matched_persistent_zones[j] ? 0 : zone_miss_counts_[j] + 1;
                }
                for (size_t j = persistent_zones_.size(); j-- > 0;) {
                    if (zone_miss_counts_[j] > zone_max_misses_) {
                        RCLCPP_INFO(this->get_logger(), "Dropping zone %s at (%.2f, %.2f): not seen for %d polls.",
                            persistent_zones_[j].label.c_str(), persistent_zones_[j].center.x,
                            persistent_zones_[j].center.y, zone_miss_counts_[j]);
                        persistent_zones_.erase(persistent_zones_.begin() + j);
                        zone_detection_counts_.erase(zone_detection_counts_.begin() + j);
                        zone_miss_counts_.erase(zone_miss_counts_.begin() + j);
                    }
                }

                // --- Publish candidate peaks (unchanged behaviour) ---
                std::vector<ros2_rad_map::msg::RadZone> zones_to_publish;
                for (const auto& zone : persistent_zones_) {
                    if (zone.center.z >= min_peak_mrem_hr_) {
                        zones_to_publish.push_back(zone);
                    }
                }

                auto zone_array_msg = std::make_unique<ros2_rad_map::msg::RadZoneArray>();
                zone_array_msg->header.stamp = stamp;
                zone_array_msg->header.frame_id = map_frame_;
                zone_array_msg->zones = zones_to_publish;

                visualization_msgs::msg::MarkerArray marker_array;
                visualization_msgs::msg::Marker clear_marker;
                clear_marker.header.stamp = stamp;
                clear_marker.header.frame_id = map_frame_;
                clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
                marker_array.markers.push_back(clear_marker);

                int marker_id = 0;
                double max_rad_val = min_peak_mrem_hr_;
                for (const auto& zone : zones_to_publish) {
                    if (zone.center.z > max_rad_val) max_rad_val = zone.center.z;
                }

                for (const auto& zone : zones_to_publish) {
                    visualization_msgs::msg::Marker point_marker;
                    point_marker.header.stamp = stamp;
                    point_marker.header.frame_id = map_frame_;
                    point_marker.ns = "radiation_peaks";
                    point_marker.id = marker_id++;
                    point_marker.type = visualization_msgs::msg::Marker::SPHERE;
                    point_marker.action = visualization_msgs::msg::Marker::ADD;
                    point_marker.pose.position = zone.center;
                    point_marker.pose.orientation.w = 1.0;
                    point_marker.scale.x = marker_scale_;
                    point_marker.scale.y = marker_scale_;
                    point_marker.scale.z = marker_scale_;
                    point_marker.color = valueToColor(zone.center.z, min_peak_mrem_hr_, max_rad_val);
                    marker_array.markers.push_back(point_marker);

                    visualization_msgs::msg::Marker text_marker;
                    text_marker.header.stamp = stamp;
                    text_marker.header.frame_id = map_frame_;
                    text_marker.ns = "radiation_peak_labels";
                    text_marker.id = marker_id++;
                    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                    text_marker.action = visualization_msgs::msg::Marker::ADD;
                    text_marker.pose.position = zone.center;
                    text_marker.pose.position.z += marker_scale_ * 1.5;
                    text_marker.pose.orientation.w = 1.0;
                    text_marker.scale.z = marker_scale_ * 0.75;
                    text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; text_marker.color.a = 1.0;
                    std::stringstream ss;
                    ss << zone.label << ": " << std::fixed << std::setprecision(2) << zone.center.z;
                    text_marker.text = ss.str();
                    marker_array.markers.push_back(text_marker);
                }

                zone_waypoints_pub_->publish(std::move(zone_array_msg));
                marker_array_pub_->publish(marker_array);

                // --- Publish confirmed sources ---
                // A zone is "confirmed" once it has been detected in the same location
                // at least source_confirmation_count_ times.  At that point we have
                // enough spatial convergence to assert a point source exists there.
                std::vector<ros2_rad_map::msg::RadZone> confirmed_zones;
                std::vector<int> confirmed_counts;
                for (size_t j = 0; j < persistent_zones_.size(); ++j) {
                    if (zone_detection_counts_[j] >= source_confirmation_count_ &&
                        persistent_zones_[j].center.z >= min_peak_mrem_hr_)
                    {
                        confirmed_zones.push_back(persistent_zones_[j]);
                        confirmed_counts.push_back(zone_detection_counts_[j]);
                    }
                }

                if (!confirmed_zones.empty()) {
                    auto confirmed_msg = std::make_unique<ros2_rad_map::msg::RadZoneArray>();
                    confirmed_msg->header.stamp = stamp;
                    confirmed_msg->header.frame_id = map_frame_;
                    confirmed_msg->zones = confirmed_zones;
                    confirmed_sources_pub_->publish(std::move(confirmed_msg));

                    // Distinct visual: larger solid-red cylinders with count annotation
                    visualization_msgs::msg::MarkerArray confirmed_markers;
                    visualization_msgs::msg::Marker conf_clear;
                    conf_clear.header.stamp = stamp;
                    conf_clear.header.frame_id = map_frame_;
                    conf_clear.action = visualization_msgs::msg::Marker::DELETEALL;
                    confirmed_markers.markers.push_back(conf_clear);

                    int conf_id = 0;
                    double conf_max_rad = min_peak_mrem_hr_;
                    for (const auto& z : confirmed_zones) {
                        if (z.center.z > conf_max_rad) conf_max_rad = z.center.z;
                    }

                    for (size_t k = 0; k < confirmed_zones.size(); ++k) {
                        const auto& zone = confirmed_zones[k];
                        double scale = marker_scale_ * 1.6;  // larger than candidate peaks

                        // Cylinder body
                        visualization_msgs::msg::Marker cyl;
                        cyl.header.stamp = stamp;
                        cyl.header.frame_id = map_frame_;
                        cyl.ns = "confirmed_sources";
                        cyl.id = conf_id++;
                        cyl.type = visualization_msgs::msg::Marker::CYLINDER;
                        cyl.action = visualization_msgs::msg::Marker::ADD;
                        cyl.pose.position = zone.center;
                        cyl.pose.position.z = 0.0;
                        cyl.pose.orientation.w = 1.0;
                        cyl.scale.x = scale;
                        cyl.scale.y = scale;
                        cyl.scale.z = 0.1;
                        // Solid red — unambiguous "source confirmed" indicator
                        cyl.color.r = 1.0f; cyl.color.g = 0.0f; cyl.color.b = 0.0f; cyl.color.a = 0.85f;
                        confirmed_markers.markers.push_back(cyl);

                        // Text: label, radiation value, detection count
                        visualization_msgs::msg::Marker txt;
                        txt.header.stamp = stamp;
                        txt.header.frame_id = map_frame_;
                        txt.ns = "confirmed_source_labels";
                        txt.id = conf_id++;
                        txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                        txt.action = visualization_msgs::msg::Marker::ADD;
                        txt.pose.position = zone.center;
                        txt.pose.position.z += scale * 1.2;
                        txt.pose.orientation.w = 1.0;
                        txt.scale.z = marker_scale_ * 0.85;
                        txt.color.r = 1.0f; txt.color.g = 0.4f; txt.color.b = 0.4f; txt.color.a = 1.0f;
                        std::stringstream ss2;
                        ss2 << "SRC-" << zone.label
                            << "  " << std::fixed << std::setprecision(1) << zone.center.z << " mrem/hr"
                            << "  [n=" << confirmed_counts[k] << "]";
                        txt.text = ss2.str();
                        confirmed_markers.markers.push_back(txt);

                        RCLCPP_INFO(
                            this->get_logger(),
                            "Confirmed source SRC-%s at (%.2f, %.2f): %.2f mrem/hr  [detections=%d]",
                            zone.label.c_str(), zone.center.x, zone.center.y,
                            zone.center.z, confirmed_counts[k]);
                    }

                    confirmed_source_markers_pub_->publish(confirmed_markers);
                }

            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
            is_service_running_ = false;
        });
    }

    std::mutex data_mutex_;
    std::deque<RadSample> historical_samples_;
    std::vector<ros2_rad_map::msg::RadZone> persistent_zones_;
    // Parallel to persistent_zones_: how many service calls have found a peak here.
    std::vector<int> zone_detection_counts_;
    std::vector<int> zone_miss_counts_;   // consecutive polls without a match
    static constexpr int zone_max_misses_ = 6;  // ~30 s at the 5 s poll
    char next_zone_label_ = 'A';
    bool is_service_running_ = false;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr background_sub_;
    rclcpp::Client<ros2_rad_map::srv::FindPeaks>::SharedPtr peak_finder_client_;
    rclcpp::Publisher<ros2_rad_map::msg::RadZoneArray>::SharedPtr zone_waypoints_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    rclcpp::Publisher<ros2_rad_map::msg::RadZoneArray>::SharedPtr confirmed_sources_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr confirmed_source_markers_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string sensor_frame_, map_frame_;
    int num_starts_;
    double marker_scale_;
    double matching_threshold_m_;
    double min_peak_mrem_hr_;
    int source_confirmation_count_;
    double current_background_level_{0.0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PeakFinderClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
