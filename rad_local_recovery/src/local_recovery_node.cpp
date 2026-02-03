#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
          is_recovering_(false)
    {
        // Parameters
        this->declare_parameter<double>("critical_dose_mrem_hr", 10.0);
        this->declare_parameter<double>("safe_dose_threshold_mrem_hr", 1.0);
        this->declare_parameter<double>("recovery_search_distance_m", 5.0);
        this->declare_parameter<std::string>("robot_base_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<double>("fallback_retreat_distance_m", 1.5);

        critical_dose_ = this->get_parameter("critical_dose_mrem_hr").as_double();
        safe_dose_threshold_ = this->get_parameter("safe_dose_threshold_mrem_hr").as_double();
        recovery_search_distance_ = this->get_parameter("recovery_search_distance_m").as_double();
        robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        fallback_retreat_distance_ = this->get_parameter("fallback_retreat_distance_m").as_double();

        // Subscriptions
        rad_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rad", 10, std::bind(&LocalRecoveryNode::radCallback, this, std::placeholders::_1));
        
        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local().reliable();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos, std::bind(&LocalRecoveryNode::mapCallback, this, std::placeholders::_1));
        
        heatmap_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rad_heatmap/gp_mean_map", 10, std::bind(&LocalRecoveryNode::heatmapCallback, this, std::placeholders::_1));

        // Action Client
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "Local Recovery Node has started.");
        RCLCPP_INFO(this->get_logger(), "Critical dose set to: %.2f mrem/hr", critical_dose_);
    }

private:
    void radCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) return;
        
        double current_mR_hr = msg->data[1];
        if (current_mR_hr > critical_dose_) {
            triggerRecovery();
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        latest_map_ = msg;
    }

    void heatmapCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(heatmap_mutex_);
        latest_heatmap_ = msg;
    }

    void triggerRecovery()
    {
        if (is_recovering_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Recovery already in progress.");
            return;
        }

        RCLCPP_WARN(this->get_logger(), "CRITICAL: Radiation threshold exceeded! Triggering recovery behavior.");
        is_recovering_ = true;

        geometry_msgs::msg::PoseStamped current_pose;
        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_.lookupTransform(
                map_frame_, robot_base_frame_, tf2::TimePointZero);
            
            current_pose.header = t.header;
            current_pose.pose.position.x = t.transform.translation.x;
            current_pose.pose.position.y = t.transform.translation.y;
            current_pose.pose.orientation = t.transform.rotation;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not get robot pose: %s", ex.what());
            is_recovering_ = false;
            return;
        }

        geometry_msgs::msg::Point safe_point;
        bool safe_point_found = findSafePoint(current_pose.pose.position, safe_point);

        auto goal_msg = std::make_shared<NavigateToPose::Goal>();
        goal_msg->pose.header.frame_id = map_frame_;
        goal_msg->pose.header.stamp = this->get_clock()->now();

        if (safe_point_found) {
            RCLCPP_INFO(this->get_logger(), "Safe point found. Navigating to (%.2f, %.2f)", safe_point.x, safe_point.y);
            goal_msg->pose.pose.position = safe_point;
            goal_msg->pose.pose.orientation = current_pose.pose.orientation; // Keep orientation
        } else {
            RCLCPP_WARN(this->get_logger(), "No safe point found. Performing fallback retreat.");
            tf2::Quaternion q;
            tf2::fromMsg(current_pose.pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            goal_msg->pose.pose.position.x = current_pose.pose.position.x - fallback_retreat_distance_ * cos(yaw);
            goal_msg->pose.pose.position.y = current_pose.pose.position.y - fallback_retreat_distance_ * sin(yaw);
            goal_msg->pose.pose.orientation = current_pose.pose.orientation;
        }
        
        RCLCPP_INFO(this->get_logger(), "Sending recovery goal to (%.2f, %.2f)",
            goal_msg->pose.pose.position.x, goal_msg->pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&LocalRecoveryNode::recoveryResultCallback, this, std::placeholders::_1);
        
        nav_action_client_->async_send_goal(*goal_msg, send_goal_options);
    }
    
    bool findSafePoint(const geometry_msgs::msg::Point& robot_pos, geometry_msgs::msg::Point& safe_point)
    {
        nav_msgs::msg::OccupancyGrid::SharedPtr map;
        sensor_msgs::msg::Image::SharedPtr heatmap;
        {
            std::lock_guard<std::mutex> map_lock(map_mutex_);
            std::lock_guard<std::mutex> heatmap_lock(heatmap_mutex_);
            if (!latest_map_ || !latest_heatmap_) {
                RCLCPP_WARN(this->get_logger(), "Map or heatmap not available for finding safe point.");
                return false;
            }
            map = latest_map_;
            heatmap = latest_heatmap_;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(heatmap, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception in recovery: %s", e.what());
            return false;
        }

        float res = map->info.resolution;
        float origin_x = map->info.origin.position.x;
        float origin_y = map->info.origin.position.y;
        int width = map->info.width;
        int height = map->info.height;

        int robot_px = (robot_pos.x - origin_x) / res;
        int robot_py = (robot_pos.y - origin_y) / res;
        
        int search_radius_px = recovery_search_distance_ / res;

        for (int r = 1; r < search_radius_px; ++r) {
            for (int i = 0; i < 360; i += 15) { // Search in a circle
                double angle = i * M_PI / 180.0;
                int px = robot_px + r * cos(angle);
                int py = robot_py + r * sin(angle);

                if (px < 0 || px >= width || py < 0 || py >= height) continue;

                int map_idx = py * width + px;
                if (map->data[map_idx] == 0) { // Free space
                    if (cv_ptr->image.at<float>(py, px) < safe_dose_threshold_) {
                        safe_point.x = origin_x + (px + 0.5) * res;
                        safe_point.y = origin_y + (py + 0.5) * res;
                        safe_point.z = 0; // Assuming 2D
                        return true;
                    }
                }
            }
        }

        return false;
    }

    void recoveryResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Recovery navigation successful.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Recovery navigation failed or was preempted.");
        }
        is_recovering_ = false;
    }

    // Parameters
    double critical_dose_;
    double safe_dose_threshold_;
    double recovery_search_distance_;
    std::string robot_base_frame_;
    std::string map_frame_;
    double fallback_retreat_distance_;

    // ROS Comms
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr heatmap_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // State
    bool is_recovering_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
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
