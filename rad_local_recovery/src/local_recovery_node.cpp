#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>

class LocalRecoveryNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    LocalRecoveryNode()
        : Node("local_recovery_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          is_recovering_(false),
          exploration_finished_(false)
    {
        // Parameters
        this->declare_parameter<double>("critical_dose_mrem_hr", 2.0);
        this->declare_parameter<double>("safe_dose_threshold_mrem_hr", 0.25);
        this->declare_parameter<double>("recovery_search_distance_m", 15.0); // Increased radius
        this->declare_parameter<std::string>("robot_base_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<double>("fallback_retreat_distance_m", 2.0);
        this->declare_parameter<bool>("return_to_origin", true);
        this->declare_parameter<double>("origin_x", 0.0);
        this->declare_parameter<double>("origin_y", 0.0);

        critical_dose_ = this->get_parameter("critical_dose_mrem_hr").as_double();
        safe_dose_threshold_ = this->get_parameter("safe_dose_threshold_mrem_hr").as_double();
        recovery_search_distance_ = this->get_parameter("recovery_search_distance_m").as_double();
        robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        fallback_retreat_distance_ = this->get_parameter("fallback_retreat_distance_m").as_double();
        return_to_origin_ = this->get_parameter("return_to_origin").as_bool();
        origin_x_ = this->get_parameter("origin_x").as_double();
        origin_y_ = this->get_parameter("origin_y").as_double();

        // Subscriptions
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rad", 10, std::bind(&LocalRecoveryNode::radCallback, this, std::placeholders::_1));
        
        rclcpp::QoS costmap_qos(rclcpp::KeepLast(1));
        costmap_qos.transient_local().reliable();
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", costmap_qos, std::bind(&LocalRecoveryNode::costmapCallback, this, std::placeholders::_1));
        
        heatmap_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rad_heatmap/gp_mean_map", 10, std::bind(&LocalRecoveryNode::heatmapCallback, this, std::placeholders::_1));

        explore_finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "explore/is_finished", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->exploration_finished_ = msg->data;
                if (this->exploration_finished_) {
                    RCLCPP_INFO(this->get_logger(), "Exploration finished detected. Disabling local recovery logic.");
                }
            });

        // Action Client
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Dynamic recovery timer
        recovery_timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&LocalRecoveryNode::recoveryTick, this));

        RCLCPP_INFO(this->get_logger(), "Local Recovery Node overhaul complete. Enhanced search radius: %.1fm", recovery_search_distance_);
    }

private:
    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) return;
        current_rad_val_ = msg->data[1];
    }

    void recoveryTick()
    {
        if (exploration_finished_) return;
        
        if (current_rad_val_ > critical_dose_) {
            triggerRecovery();
        }
    }

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        latest_costmap_ = msg;
    }

    void heatmapCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(heatmap_mutex_);
        latest_heatmap_ = msg;
    }

    void triggerRecovery()
    {
        auto now = this->get_clock()->now();
        
        // Prevent frequent preemption unless we've been stuck for a while
        if (is_recovering_ && (now - last_goal_time_).seconds() < 5.0) {
            return;
        }

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Critical radiation (%.2f)! Executing recovery.", current_rad_val_);
        
        geometry_msgs::msg::PoseStamped current_pose;
        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_.lookupTransform(
                map_frame_, robot_base_frame_, tf2::TimePointZero);
            current_pose.header = t.header;
            current_pose.pose.position.x = t.transform.translation.x;
            current_pose.pose.position.y = t.transform.translation.y;
            current_pose.pose.orientation = t.transform.rotation;
        } catch (...) { return; }

        geometry_msgs::msg::Point escape_point;
        bool point_found = false;

        if (return_to_origin_) {
            escape_point.x = origin_x_;
            escape_point.y = origin_y_;
            escape_point.z = 0.0;
            point_found = true;
            RCLCPP_INFO(this->get_logger(), "Evacuating to origin (%.2f, %.2f)", origin_x_, origin_y_);
        } else {
            point_found = findSafePoint(current_pose.pose.position, escape_point);
        }

        if (!point_found) {
            // Fallback: Just move backwards relative to current orientation
            tf2::Quaternion q;
            tf2::fromMsg(current_pose.pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            escape_point.x = current_pose.pose.position.x - fallback_retreat_distance_ * cos(yaw);
            escape_point.y = current_pose.pose.position.y - fallback_retreat_distance_ * sin(yaw);
            RCLCPP_ERROR(this->get_logger(), "No safe point found! Performing emergency retreat.");
        }

        auto goal_msg = std::make_shared<NavigateToPose::Goal>();
        goal_msg->pose.header.frame_id = map_frame_;
        goal_msg->pose.header.stamp = now;
        goal_msg->pose.pose.position = escape_point;
        goal_msg->pose.pose.orientation = current_pose.pose.orientation;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&LocalRecoveryNode::recoveryResultCallback, this, std::placeholders::_1);
        
        nav_action_client_->async_send_goal(*goal_msg, send_goal_options);
        last_goal_time_ = now;
        is_recovering_ = true;
    }
    
    bool findSafePoint(const geometry_msgs::msg::Point& robot_pos, geometry_msgs::msg::Point& safe_point)
    {
        nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
        sensor_msgs::msg::Image::SharedPtr heatmap;
        {
            std::lock_guard<std::mutex> map_lock(map_mutex_);
            std::lock_guard<std::mutex> heatmap_lock(heatmap_mutex_);
            if (!latest_costmap_ || !latest_heatmap_) return false;
            costmap = latest_costmap_;
            heatmap = latest_heatmap_;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(heatmap, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (...) { return false; }

        float res = costmap->info.resolution;
        float origin_x = costmap->info.origin.position.x;
        float origin_y = costmap->info.origin.position.y;
        int width = costmap->info.width;
        int height = costmap->info.height;

        int robot_px = (robot_pos.x - origin_x) / res;
        int robot_py = (robot_pos.y - origin_y) / res;
        int max_search_px = recovery_search_distance_ / res;

        float best_utility = -std::numeric_limits<float>::max();
        bool found = false;

        // More granular search: balance safety and distance
        for (int r = 5; r < max_search_px; r += 5) {
            for (int i = 0; i < 360; i += 15) {
                double angle = i * M_PI / 180.0;
                int px = robot_px + r * cos(angle);
                int py = robot_py + r * sin(angle);

                if (px < 0 || px >= width || py < 0 || py >= height) continue;

                int map_idx = py * width + px;
                // Check if navigable (cost < 100 avoids obstacles and high inflation)
                if (costmap->data[map_idx] >= 0 && costmap->data[map_idx] < 100) {
                    float rad = cv_ptr->image.at<float>(py, px);
                    
                    // Utility = -Radiation - Normalized_Distance
                    // We want low radiation AND low distance, but safety is priority
                    float dist_m = r * res;
                    float utility = -rad * 10.0f - dist_m;

                    if (utility > best_utility) {
                        best_utility = utility;
                        safe_point.x = origin_x + (px + 0.5) * res;
                        safe_point.y = origin_y + (py + 0.5) * res;
                        safe_point.z = 0;
                        found = true;
                    }
                    
                    // If we found a genuinely safe point, we can stop early if it's close enough
                    if (rad < safe_dose_threshold_ && dist_m > 2.0) {
                        return true;
                    }
                }
            }
        }
        return found;
    }

    void recoveryResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            is_recovering_ = false;
            RCLCPP_INFO(this->get_logger(), "Successfully reached safe zone.");
        }
    }

    // Parameters
    double critical_dose_;
    double safe_dose_threshold_;
    double recovery_search_distance_;
    std::string robot_base_frame_;
    std::string map_frame_;
    double fallback_retreat_distance_;
    bool return_to_origin_;
    double origin_x_;
    double origin_y_;

    // ROS Comms
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr heatmap_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_finished_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp::TimerBase::SharedPtr recovery_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // State
    double current_rad_val_ = 0.0;
    bool is_recovering_ = false;
    bool exploration_finished_ = false;
    rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    sensor_msgs::msg::Image::SharedPtr latest_heatmap_;
    std::mutex map_mutex_;
    std::mutex heatmap_mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalRecoveryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
