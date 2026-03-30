#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
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
          exploration_finished_(false),
          consecutive_recovery_failures_(0)
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
        this->declare_parameter<std::string>("stop_explore_service", "/explore_node/stop");
        this->declare_parameter<std::string>("start_explore_service", "/explore_node/start");

        critical_dose_ = this->get_parameter("critical_dose_mrem_hr").as_double();
        safe_dose_threshold_ = this->get_parameter("safe_dose_threshold_mrem_hr").as_double();
        recovery_search_distance_ = this->get_parameter("recovery_search_distance_m").as_double();
        robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        fallback_retreat_distance_ = this->get_parameter("fallback_retreat_distance_m").as_double();
        return_to_origin_ = this->get_parameter("return_to_origin").as_bool();
        origin_x_ = this->get_parameter("origin_x").as_double();
        origin_y_ = this->get_parameter("origin_y").as_double();
        stop_explore_srv_name_ = this->get_parameter("stop_explore_service").as_string();
        start_explore_srv_name_ = this->get_parameter("start_explore_service").as_string();

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

        // Service Clients
        stop_explore_client_ = this->create_client<std_srvs::srv::Trigger>(stop_explore_srv_name_);
        start_explore_client_ = this->create_client<std_srvs::srv::Trigger>(start_explore_srv_name_);

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

        // Don't preempt an active recovery within the cooldown window
        if (is_recovering_ && (now - last_goal_time_).seconds() < 10.0) {
            return;
        }

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Critical radiation (%.2f)! Executing recovery (attempt #%d).",
            current_rad_val_, consecutive_recovery_failures_ + 1);

        if (!is_recovering_) {
            callTriggerService(stop_explore_client_, "Stop Exploration");
        }

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

        // After 4 consecutive failures, override everything and go to origin —
        // the robot is genuinely stuck and needs a guaranteed safe destination.
        if (consecutive_recovery_failures_ >= 4) {
            RCLCPP_WARN(this->get_logger(),
                "4+ recovery failures. Forcing navigation to map origin as last resort.");
            escape_point.x = origin_x_;
            escape_point.y = origin_y_;
            escape_point.z = 0.0;
            point_found = true;
        } else if (return_to_origin_) {
            escape_point.x = origin_x_;
            escape_point.y = origin_y_;
            escape_point.z = 0.0;
            std::lock_guard<std::mutex> map_lock(map_mutex_);
            if (latest_costmap_) {
                float res = latest_costmap_->info.resolution;
                float ox = latest_costmap_->info.origin.position.x;
                float oy = latest_costmap_->info.origin.position.y;
                int w = latest_costmap_->info.width;
                int h = latest_costmap_->info.height;
                int px = (escape_point.x - ox) / res;
                int py = (escape_point.y - oy) / res;
                if (px < 2 || px >= w - 2 || py < 2 || py >= h - 2) {
                    RCLCPP_WARN(this->get_logger(), "Origin out of map bounds. Falling back to safe point search.");
                    return_to_origin_ = false;
                    // Expand radius after failures to break out of local minima
                    double search_radius = recovery_search_distance_ *
                        (consecutive_recovery_failures_ >= 2 ? 1.5 : 1.0);
                    point_found = findSafePoint(current_pose.pose.position, escape_point, search_radius);
                } else {
                    point_found = true;
                    RCLCPP_INFO(this->get_logger(), "Evacuating to origin (%.2f, %.2f)", origin_x_, origin_y_);
                }
            }
        } else {
            double search_radius = recovery_search_distance_ *
                (consecutive_recovery_failures_ >= 2 ? 1.5 : 1.0);
            if (consecutive_recovery_failures_ >= 2) {
                RCLCPP_INFO(this->get_logger(),
                    "Expanding search radius to %.1fm after %d failures.",
                    search_radius, consecutive_recovery_failures_);
            }
            point_found = findSafePoint(current_pose.pose.position, escape_point, search_radius);
        }

        if (!point_found) {
            // Improved fallback: sample 8 directions and pick the one with the
            // lowest costmap cost that hasn't already been tried.
            tf2::Quaternion q;
            tf2::fromMsg(current_pose.pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            float best_cost = std::numeric_limits<float>::max();
            bool directional_found = false;
            {
                std::lock_guard<std::mutex> map_lock(map_mutex_);
                if (latest_costmap_) {
                    float res = latest_costmap_->info.resolution;
                    float ox = latest_costmap_->info.origin.position.x;
                    float oy = latest_costmap_->info.origin.position.y;
                    int w = latest_costmap_->info.width;
                    int h = latest_costmap_->info.height;
                    for (int i = 0; i < 8; ++i) {
                        double angle = yaw + i * M_PI / 4.0;
                        geometry_msgs::msg::Point candidate;
                        candidate.x = current_pose.pose.position.x + fallback_retreat_distance_ * cos(angle);
                        candidate.y = current_pose.pose.position.y + fallback_retreat_distance_ * sin(angle);
                        int px = (candidate.x - ox) / res;
                        int py = (candidate.y - oy) / res;
                        if (px < 2 || px >= w - 2 || py < 2 || py >= h - 2) continue;
                        unsigned char cost = latest_costmap_->data[py * w + px];
                        if (cost >= 70) continue;
                        // Skip if too close to a previously failed escape point
                        bool near_failed = false;
                        for (const auto& fe : failed_escape_points_) {
                            double dx = candidate.x - fe.x, dy = candidate.y - fe.y;
                            if (dx * dx + dy * dy < 4.0) { near_failed = true; break; }
                        }
                        if (!near_failed && static_cast<float>(cost) < best_cost) {
                            best_cost = static_cast<float>(cost);
                            escape_point = candidate;
                            directional_found = true;
                        }
                    }
                }
            }
            if (!directional_found) {
                // Last resort: go straight backward
                escape_point.x = current_pose.pose.position.x - fallback_retreat_distance_ * cos(yaw);
                escape_point.y = current_pose.pose.position.y - fallback_retreat_distance_ * sin(yaw);
            }
            RCLCPP_ERROR(this->get_logger(), "No safe point found! Using %s fallback.",
                directional_found ? "best-direction" : "backward");
        }

        last_escape_attempt_ = escape_point;

        // Orient toward the escape point so the robot faces its destination
        // and doesn't need to spin at the goal.
        double escape_yaw = atan2(escape_point.y - current_pose.pose.position.y,
                                   escape_point.x - current_pose.pose.position.x);
        tf2::Quaternion escape_q;
        escape_q.setRPY(0.0, 0.0, escape_yaw);
        escape_q.normalize();

        auto goal_msg = std::make_shared<NavigateToPose::Goal>();
        goal_msg->pose.header.frame_id = map_frame_;
        goal_msg->pose.header.stamp = now;
        goal_msg->pose.pose.position = escape_point;
        goal_msg->pose.pose.orientation = tf2::toMsg(escape_q);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandleNavigateToPose::SharedPtr &handle) {
                if (!handle) {
                    RCLCPP_WARN(this->get_logger(), "Recovery goal rejected by Nav2. Will retry.");
                    is_recovering_ = false;
                }
            };
        send_goal_options.result_callback =
            std::bind(&LocalRecoveryNode::recoveryResultCallback, this, std::placeholders::_1);

        nav_action_client_->async_send_goal(*goal_msg, send_goal_options);
        last_goal_time_ = now;
        is_recovering_ = true;
    }
    
    bool findSafePoint(const geometry_msgs::msg::Point& robot_pos,
                       geometry_msgs::msg::Point& safe_point,
                       double search_radius = -1.0)
    {
        if (search_radius < 0.0) search_radius = recovery_search_distance_;

        nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
        sensor_msgs::msg::Image::SharedPtr heatmap;
        {
            std::scoped_lock lock(map_mutex_, heatmap_mutex_);
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
        int max_search_px = search_radius / res;

        float best_utility = -std::numeric_limits<float>::max();
        bool found = false;

        // Safety margin from map edges
        int margin = 5;

        // More granular search: balance safety and distance
        for (int r = 5; r < max_search_px; r += 5) {
            for (int i = 0; i < 360; i += 15) {
                double angle = i * M_PI / 180.0;
                int px = robot_px + r * cos(angle);
                int py = robot_py + r * sin(angle);

                // Strict boundary check
                if (px < margin || px >= width - margin || py < margin || py >= height - margin) continue;

                int map_idx = py * width + px;
                unsigned char cost = costmap->data[map_idx];
                // Check if navigable (cost < 70 avoids inflation from walls better)
                if (cost >= 0 && cost < 70) {
                    // Guard against heatmap/costmap dimension mismatch
                    float rad = (px < cv_ptr->image.cols && py < cv_ptr->image.rows)
                                ? cv_ptr->image.at<float>(py, px) : 0.0f;
                    float dist_m = r * res;

                    geometry_msgs::msg::Point candidate;
                    candidate.x = origin_x + (px + 0.5) * res;
                    candidate.y = origin_y + (py + 0.5) * res;

                    // Skip points too close to previously failed escape destinations
                    bool near_failed = false;
                    for (const auto& fe : failed_escape_points_) {
                        double dx = candidate.x - fe.x, dy = candidate.y - fe.y;
                        if (dx * dx + dy * dy < 4.0) { near_failed = true; break; }
                    }
                    if (near_failed) continue;

                    // Utility = -Radiation + Normalized_Distance - Normalized_Cost
                    float utility = -rad * 10.0f +
                                    (dist_m / search_radius) -
                                    (static_cast<float>(cost) / 50.0f);

                    if (utility > best_utility) {
                        best_utility = utility;
                        safe_point = candidate;
                        safe_point.z = 0;
                        found = true;
                    }

                    // Early exit: genuinely safe and far enough away
                    if (rad < safe_dose_threshold_ && dist_m > 5.0) {
                        return true;
                    }
                }
            }
        }
        return found;
    }

    void recoveryResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        is_recovering_ = false;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Successfully reached safe zone. Resuming exploration.");
            consecutive_recovery_failures_ = 0;
            failed_escape_points_.clear();
            callTriggerService(start_explore_client_, "Start Exploration");
        } else {
            consecutive_recovery_failures_++;
            failed_escape_points_.push_back(last_escape_attempt_);
            RCLCPP_WARN(this->get_logger(),
                "Recovery navigation failed (code %d, attempt #%d). "
                "Blacklisting escape point (%.2f, %.2f). Will retry.",
                static_cast<int>(result.code), consecutive_recovery_failures_,
                last_escape_attempt_.x, last_escape_attempt_.y);
        }
    }

    void callTriggerService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, const std::string& label)
    {
        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available!", client->get_service_name());
            return;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        client->async_send_request(request, [this, label](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "%s succeeded: %s", label.c_str(), response->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "%s failed: %s", label.c_str(), response->message.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call %s failed: %s", label.c_str(), e.what());
            }
        });
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
    std::string stop_explore_srv_name_;
    std::string start_explore_srv_name_;

    // ROS Comms
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rad_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr heatmap_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_finished_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_explore_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_explore_client_;
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
    // Recovery escalation tracking
    int consecutive_recovery_failures_;
    std::vector<geometry_msgs::msg::Point> failed_escape_points_;
    geometry_msgs::msg::Point last_escape_attempt_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalRecoveryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
