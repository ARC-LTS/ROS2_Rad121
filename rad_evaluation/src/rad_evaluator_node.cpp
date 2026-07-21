#include "ament_index_cpp/get_package_share_directory.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "ros2_rad_map/msg/rad_zone_array.hpp"

#include <fstream>
#include <vector>
#include <chrono>
#include <limits>
#include <cmath>
#include <mutex>
#include <filesystem>
#include <iomanip>

struct GroundTruthSource {
    int id;
    double x, y, intensity;
};

class RadEvaluatorNode : public rclcpp::Node
{
public:
    RadEvaluatorNode() : Node("rad_evaluator_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // Parameters
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("robot_frame", "base_link");
        this->declare_parameter<double>("critical_dose_mrem_hr", 2.0);
        this->declare_parameter<std::string>("output_dir",
            ament_index_cpp::get_package_share_directory("rad_evaluation") + "/eval_reports");
        this->declare_parameter<std::string>("test_case_name", "unnamed_trial");
        this->declare_parameter<double>("max_radiation_dose", 100.0);

        map_frame_ = this->get_parameter("map_frame").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        critical_dose_limit_ = this->get_parameter("critical_dose_mrem_hr").as_double();
        output_dir_ = this->get_parameter("output_dir").as_string();
        test_case_name_ = this->get_parameter("test_case_name").as_string();
        max_radiation_dose_ = this->get_parameter("max_radiation_dose").as_double();

        // Subscriptions
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rad", 10, std::bind(&RadEvaluatorNode::radCallback, this, std::placeholders::_1));
        
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&RadEvaluatorNode::mapCallback, this, std::placeholders::_1));

        gt_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/rad_field_server/sources", 10, std::bind(&RadEvaluatorNode::gtCallback, this, std::placeholders::_1));

        peak_sub_ = this->create_subscription<ros2_rad_map::msg::RadZoneArray>(
            "/radiation_zones", 10, std::bind(&RadEvaluatorNode::peakCallback, this, std::placeholders::_1));

        // stuck_escape_node (rad_local_recovery) publishes one message per
        // completed stuck-and-recover cycle, dose-independent -- a robot
        // physically wedged (e.g. a corner) rather than reacting to
        // radiation. Tracked separately from the violation/recovery-time
        // metrics above, which are specifically the radiation-triggered path.
        stuck_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/stuck_escape/event", 10, std::bind(&RadEvaluatorNode::stuckCallback, this, std::placeholders::_1));

        // Service to save report
        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/save_report", std::bind(&RadEvaluatorNode::saveReportCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Dose-budget authority: this node's cumulative-dose integration is
        // the campaign's reported ground truth, so it is also the single
        // source of the remaining-budget fraction every dose-thresholded
        // layer (explore_node, local_recovery_node, rad_safety_monitor_node)
        // scales its threshold by. Publishing it, rather than letting each
        // layer integrate /rad independently, makes the cross-layer
        // threshold contract coherent by construction.
        budget_fraction_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/dose_budget/remaining_fraction", 10);

        // Contract-coherence monitor: each consumer publishes the relaxation
        // multiplier it is actually using; this node records the maximum
        // pairwise divergence and any time spent with explore_node's
        // multiplier below local_recovery_node's (the mis-ordering that
        // reopens the yield-without-rescue dead band). Both go into the
        // per-trial report as verification evidence.
        for (const auto & pair : std::vector<std::pair<int, std::string>>{
                 {0, "/explore_node/dose_budget_multiplier"},
                 {1, "/local_recovery_node/dose_budget_multiplier"},
                 {2, "/rad_safety_monitor_node/dose_budget_multiplier"}}) {
            int idx = pair.first;
            multiplier_subs_.push_back(this->create_subscription<std_msgs::msg::Float64>(
                pair.second, 10,
                [this, idx](const std_msgs::msg::Float64::SharedPtr msg) {
                    layer_multiplier_[idx] = msg->data;
                    layer_multiplier_time_[idx] = this->get_clock()->now();
                }));
        }

        contract_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&RadEvaluatorNode::contractTick, this));

        // Timers
        stats_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&RadEvaluatorNode::updateStats, this));

        start_time_ = this->get_clock()->now();
        last_stats_time_ = start_time_;

        RCLCPP_INFO(this->get_logger(), "Radiation Evaluator Node started for test case: %s", test_case_name_.c_str());
    }

private:
    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) return;
        
        double current_dose = msg->data[1];
        auto now = this->get_clock()->now();
        double dt = (now - last_rad_time_).seconds();
        
        if (last_rad_time_.seconds() > 0 && dt > 0 && dt < 1.0) {
            // current_dose is mrem/hr, dt is seconds -- convert to hours
            // before integrating, or this silently overcounts every trial's
            // cumulative dose by 3600x (confirmed directly: avg_dose_rate
            // below, computed as cumulative_dose_/total_time_seconds, was
            // only coincidentally correct in mrem/hr because that missing
            // /3600 here and the choice of seconds instead of hours in the
            // denominator there happened to cancel out -- cum_dose_mrem
            // itself, and dose_per_meter which is derived from it, were not
            // so lucky).
            cumulative_dose_ += current_dose * (dt / 3600.0);
        }
        last_rad_time_ = now;

        // Peak dose detection
        if (current_dose > max_dose_encountered_) {
            max_dose_encountered_ = current_dose;
        }

        // Critical violation tracking
        if (current_dose > critical_dose_limit_) {
            if (!in_critical_zone_) {
                violation_count_++;
                critical_entry_time_ = now;
                in_critical_zone_ = true;
            }
            // Track penetration (how far above limit)
            double penetration = current_dose - critical_dose_limit_;
            if (penetration > max_penetration_depth_) {
                max_penetration_depth_ = penetration;
            }
        } else {
            if (in_critical_zone_) {
                double recovery_time = (now - critical_entry_time_).seconds();
                total_recovery_time_ += recovery_time;
                in_critical_zone_ = false;
            }
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        long explored_cells = 0;
        for (const auto &cell : msg->data) {
            if (cell != -1) explored_cells++;
        }
        area_explored_ = explored_cells * std::pow(msg->info.resolution, 2);
    }

    void gtCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        gt_sources_.clear();
        for (const auto &m : msg->markers) {
            if (m.action == visualization_msgs::msg::Marker::ADD && m.type == visualization_msgs::msg::Marker::SPHERE) {
                GroundTruthSource s;
                s.id = m.id;
                s.x = m.pose.position.x;
                s.y = m.pose.position.y;
                // Reverse normalization: intensity = norm * 7.5 + 2.5
                double norm = (m.scale.x - 0.2) / 0.8;
                s.intensity = norm * 7.5 + 2.5;
                gt_sources_.push_back(s);
            }
        }
    }

    void peakCallback(const ros2_rad_map::msg::RadZoneArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        predicted_peaks_ = msg->zones;
    }

    void stuckCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        stuck_count_++;
        total_stuck_time_ += msg->data;
    }

    void contractTick()
    {
        // Publish the authoritative remaining-budget fraction.
        std_msgs::msg::Float64 f;
        f.data = (max_radiation_dose_ > 0.0)
            ? std::max(0.0, (max_radiation_dose_ - cumulative_dose_) / max_radiation_dose_)
            : 1.0;
        budget_fraction_pub_->publish(f);

        // Evaluate coherence over the layers heard from recently (10s).
        auto now = this->get_clock()->now();
        double mn = std::numeric_limits<double>::max();
        double mx = std::numeric_limits<double>::lowest();
        int fresh = 0;
        for (int i = 0; i < 3; ++i) {
            if (layer_multiplier_time_[i].nanoseconds() > 0 &&
                (now - layer_multiplier_time_[i]).seconds() < 10.0) {
                mn = std::min(mn, layer_multiplier_[i]);
                mx = std::max(mx, layer_multiplier_[i]);
                fresh++;
            }
        }
        if (fresh >= 2) {
            max_multiplier_divergence_ = std::max(max_multiplier_divergence_, mx - mn);
        }
        // Mis-ordering: explore (idx 0) yielding below local_recovery's
        // (idx 1) trigger is the dead-band condition; both bases are equal,
        // so the multipliers carry the whole comparison.
        bool both_fresh = layer_multiplier_time_[0].nanoseconds() > 0 &&
            layer_multiplier_time_[1].nanoseconds() > 0 &&
            (now - layer_multiplier_time_[0]).seconds() < 10.0 &&
            (now - layer_multiplier_time_[1]).seconds() < 10.0;
        if (both_fresh && layer_multiplier_[0] + 1e-6 < layer_multiplier_[1]) {
            contract_misorder_s_ += 1.0;
        }
    }

    void updateStats()
    {
        auto now = this->get_clock()->now();
        
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_.lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
            double rx = t.transform.translation.x;
            double ry = t.transform.translation.y;

            if (first_pose_received_) {
                double dist = std::sqrt(std::pow(rx - last_x_, 2) + std::pow(ry - last_y_, 2));
                if (dist > 0.01 && dist < 1.0) { // Filter jitter and jumps
                    total_distance_ += dist;
                }
            }
            last_x_ = rx;
            last_y_ = ry;
            first_pose_received_ = true;
        } catch (const tf2::TransformException &ex) {
            // Silence TF errors to avoid spam
        }
    }

    void saveReportCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        auto now = this->get_clock()->now();
        double total_time = (now - start_time_).seconds();

        // Calculate Localization Error
        double total_localization_error = 0;
        int matched_count = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto &gt : gt_sources_) {
                double min_dist_sq = 100.0; // 10m limit
                bool found = false;
                for (const auto &peak : predicted_peaks_) {
                    double d2 = std::pow(gt.x - peak.center.x, 2) + std::pow(gt.y - peak.center.y, 2);
                    if (d2 < min_dist_sq) {
                        min_dist_sq = d2;
                        found = true;
                    }
                }
                if (found) {
                    total_localization_error += std::sqrt(min_dist_sq);
                    matched_count++;
                }
            }
        }
        double avg_loc_error = (matched_count > 0) ? total_localization_error / matched_count : -1.0;

        // Write to CSV
        std::filesystem::create_directories(output_dir_);
        std::string filename = output_dir_ + "/" + test_case_name_ + "_report.csv";
        
        bool file_exists = std::filesystem::exists(filename);
        std::ofstream fout(filename, std::ios::app);

        if (!file_exists) {
            fout << "timestamp,test_case,total_time_s,distance_m,area_explored_m2,cum_dose_mrem,avg_dose_rate,dose_per_meter,max_dose,violations,avg_recovery_s,max_penetration,avg_loc_error_m,stuck_count,avg_stuck_recovery_s,max_multiplier_divergence,contract_misorder_s\n";
        }

        // total_time is in seconds; cumulative_dose_ is now correctly in
        // mrem (see radCallback), so convert total_time to hours here too --
        // dividing by raw seconds would silently reintroduce the same
        // magnitude of error this fix is for, just relocated.
        double avg_dose_rate = (total_time > 0) ? cumulative_dose_ / (total_time / 3600.0) : 0;
        double dose_per_m = (total_distance_ > 0) ? cumulative_dose_ / total_distance_ : 0;
        double avg_recovery = (violation_count_ > 0) ? total_recovery_time_ / violation_count_ : 0;
        double avg_stuck_recovery = (stuck_count_ > 0) ? total_stuck_time_ / stuck_count_ : 0;

        auto t_now = std::time(nullptr);
        fout << std::put_time(std::localtime(&t_now), "%Y-%m-%d %H:%M:%S") << ","
             << test_case_name_ << ","
             << std::fixed << std::setprecision(3)
             << total_time << ","
             << total_distance_ << ","
             << area_explored_ << ","
             << cumulative_dose_ << ","
             << avg_dose_rate << ","
             << dose_per_m << ","
             << max_dose_encountered_ << ","
             << violation_count_ << ","
             << avg_recovery << ","
             << max_penetration_depth_ << ","
             << avg_loc_error << ","
             << stuck_count_ << ","
             << avg_stuck_recovery << ","
             << max_multiplier_divergence_ << ","
             << contract_misorder_s_ << "\n";
        
        fout.close();

        response->success = true;
        response->message = "Report saved to " + filename;
        RCLCPP_INFO(this->get_logger(), "Evaluation report saved successfully.");
    }

    // ROS
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr gt_sub_;
    rclcpp::Subscription<ros2_rad_map::msg::RadZoneArray>::SharedPtr peak_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr stuck_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Params
    std::string map_frame_, robot_frame_, output_dir_, test_case_name_;
    double critical_dose_limit_;

    // Metrics State
    double cumulative_dose_ = 0;
    double max_radiation_dose_ = 100.0;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr budget_fraction_pub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> multiplier_subs_;
    rclcpp::TimerBase::SharedPtr contract_timer_;
    double layer_multiplier_[3] = {0.0, 0.0, 0.0};
    rclcpp::Time layer_multiplier_time_[3] = {
        rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Time(0, 0, RCL_ROS_TIME),
        rclcpp::Time(0, 0, RCL_ROS_TIME)};
    double max_multiplier_divergence_ = 0.0;
    double contract_misorder_s_ = 0.0;
    double total_distance_ = 0;
    double area_explored_ = 0;
    double max_dose_encountered_ = 0;
    int violation_count_ = 0;
    double total_recovery_time_ = 0;
    double max_penetration_depth_ = 0;
    int stuck_count_ = 0;
    double total_stuck_time_ = 0;

    // Internal tracking
    rclcpp::Time start_time_;
    rclcpp::Time last_rad_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_stats_time_;
    rclcpp::Time critical_entry_time_;
    bool in_critical_zone_ = false;
    bool first_pose_received_ = false;
    double last_x_, last_y_;

    std::mutex mutex_;
    std::vector<GroundTruthSource> gt_sources_;
    std::vector<ros2_rad_map::msg::RadZone> predicted_peaks_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadEvaluatorNode>());
    rclcpp::shutdown();
    return 0;
}
