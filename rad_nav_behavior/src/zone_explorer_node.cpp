#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "ros2_rad_map/msg/rad_zone_array.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class ZoneExplorerNode : public rclcpp::Node
{
public:
    ZoneExplorerNode() : Node("zone_explorer_node"), is_navigating_(false), orbit_idx_(0)
    {
        this->declare_parameter<std::string>("zones_topic", "/radiation_zones");
        this->declare_parameter<double>("survey_radius_m", 1.5);
        this->declare_parameter<int>("survey_waypoints", 6);
        this->declare_parameter<int>("dwell_per_waypoint_s", 2);

        std::string zones_topic = this->get_parameter("zones_topic").as_string();
        survey_radius_       = this->get_parameter("survey_radius_m").as_double();
        survey_waypoints_    = this->get_parameter("survey_waypoints").as_int();
        dwell_per_waypoint_s_ = this->get_parameter("dwell_per_waypoint_s").as_int();

        zone_sub_ = this->create_subscription<ros2_rad_map::msg::RadZoneArray>(
            zones_topic, 10,
            std::bind(&ZoneExplorerNode::zoneCallback, this, std::placeholders::_1));

        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "Zone Explorer Node started. survey_radius=%.1fm, waypoints=%d, dwell=%ds",
            survey_radius_, survey_waypoints_, dwell_per_waypoint_s_);
        RCLCPP_INFO(this->get_logger(), "Waiting for navigation action server...");
        nav_action_client_->wait_for_action_server();
        RCLCPP_INFO(this->get_logger(), "Navigation action server is available.");

        process_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ZoneExplorerNode::processZoneQueue, this));
    }

private:
    void zoneCallback(const ros2_rad_map::msg::RadZoneArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        for (const auto& zone : msg->zones)
        {
            bool found = false;
            for (const auto& existing_zone : zone_queue_)
            {
                if (existing_zone.label == zone.label) { found = true; break; }
            }
            if (is_navigating_ && current_goal_.label == zone.label) found = true;

            if (!found)
            {
                RCLCPP_INFO(this->get_logger(), "Adding new zone '%s' to the queue.", zone.label.c_str());
                zone_queue_.push_back(zone);
            }
        }
    }

    void processZoneQueue()
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (is_navigating_ || zone_queue_.empty()) return;

        current_goal_ = zone_queue_.front();
        zone_queue_.pop_front();
        is_navigating_ = true;

        RCLCPP_INFO(this->get_logger(), "Navigating to zone '%s' at (%.2f, %.2f)",
            current_goal_.label.c_str(), current_goal_.center.x, current_goal_.center.y);

        auto goal_msg = std::make_shared<NavigateToPose::Goal>();
        goal_msg->pose.header.frame_id = "map";
        goal_msg->pose.header.stamp = this->get_clock()->now();
        goal_msg->pose.pose.position.x = current_goal_.center.x;
        goal_msg->pose.pose.position.y = current_goal_.center.y;
        goal_msg->pose.pose.orientation.w = 1.0;

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        opts.goal_response_callback =
            std::bind(&ZoneExplorerNode::goalResponseCallback, this, std::placeholders::_1);
        opts.result_callback =
            std::bind(&ZoneExplorerNode::zoneResultCallback, this, std::placeholders::_1);

        nav_action_client_->async_send_goal(*goal_msg, opts);
    }

    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            is_navigating_ = false;
        }
    }

    // Called when the robot reaches (or fails to reach) the zone center.
    void zoneResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(),
                    "Reached zone '%s'. Starting orbital survey (r=%.1fm, %d pts, dwell=%ds).",
                    current_goal_.label.c_str(), survey_radius_, survey_waypoints_, dwell_per_waypoint_s_);
                startOrbitalSurvey();
                return;  // is_navigating_ stays true until survey finishes
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Navigation to zone '%s' aborted.", current_goal_.label.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Navigation to zone '%s' canceled.", current_goal_.label.c_str());
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code for zone '%s'.", current_goal_.label.c_str());
                break;
        }
        is_navigating_ = false;
    }

    // Generate an equally-spaced orbit of waypoints around the zone center,
    // oriented so the robot faces inward toward the peak.
    void startOrbitalSurvey()
    {
        orbit_waypoints_.clear();
        orbit_idx_ = 0;
        double cx = current_goal_.center.x;
        double cy = current_goal_.center.y;
        for (int i = 0; i < survey_waypoints_; ++i)
        {
            double angle = 2.0 * M_PI * i / survey_waypoints_;
            geometry_msgs::msg::Point p;
            p.x = cx + survey_radius_ * std::cos(angle);
            p.y = cy + survey_radius_ * std::sin(angle);
            p.z = 0.0;
            orbit_waypoints_.push_back(p);
        }
        sendNextOrbitGoal();
    }

    void sendNextOrbitGoal()
    {
        if (orbit_idx_ >= orbit_waypoints_.size())
        {
            RCLCPP_INFO(this->get_logger(),
                "Orbital survey of zone '%s' complete.", current_goal_.label.c_str());
            is_navigating_ = false;
            return;
        }

        const auto& wp = orbit_waypoints_[orbit_idx_];
        double cx = current_goal_.center.x;
        double cy = current_goal_.center.y;
        // Face inward toward the zone center from each orbit point
        double yaw = std::atan2(cy - wp.y, cx - wp.x);

        auto goal_msg = std::make_shared<NavigateToPose::Goal>();
        goal_msg->pose.header.frame_id = "map";
        goal_msg->pose.header.stamp = this->get_clock()->now();
        goal_msg->pose.pose.position = wp;
        goal_msg->pose.pose.orientation.z = std::sin(yaw / 2.0);
        goal_msg->pose.pose.orientation.w = std::cos(yaw / 2.0);

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        opts.result_callback =
            std::bind(&ZoneExplorerNode::orbitResultCallback, this, std::placeholders::_1);

        nav_action_client_->async_send_goal(*goal_msg, opts);
        RCLCPP_INFO(this->get_logger(), "Surveying orbit point %zu/%zu at (%.2f, %.2f).",
            orbit_idx_ + 1, orbit_waypoints_.size(), wp.x, wp.y);
    }

    void orbitResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        // If the goal was canceled, something (e.g. local_recovery) preempted us — abort survey cleanly.
        if (result.code == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_WARN(this->get_logger(),
                "Orbit goal canceled (preempted). Aborting survey of zone '%s'.",
                current_goal_.label.c_str());
            orbit_waypoints_.clear();
            is_navigating_ = false;
            return;
        }

        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_WARN(this->get_logger(),
                "Orbit point %zu navigation failed. Skipping to next.", orbit_idx_);
        }

        orbit_idx_++;

        if (dwell_per_waypoint_s_ > 0)
        {
            // Non-blocking dwell: fire a one-shot timer, then move to next waypoint.
            dwell_timer_ = this->create_wall_timer(
                std::chrono::seconds(dwell_per_waypoint_s_),
                [this]() {
                    dwell_timer_->cancel();
                    dwell_timer_.reset();
                    sendNextOrbitGoal();
                });
        }
        else
        {
            sendNextOrbitGoal();
        }
    }

    // --- Members ---
    rclcpp::Subscription<ros2_rad_map::msg::RadZoneArray>::SharedPtr zone_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp::TimerBase::SharedPtr process_timer_;
    rclcpp::TimerBase::SharedPtr dwell_timer_;

    std::mutex queue_mutex_;
    std::deque<ros2_rad_map::msg::RadZone> zone_queue_;
    ros2_rad_map::msg::RadZone current_goal_;
    bool is_navigating_;

    // Orbital survey state
    double survey_radius_;
    int survey_waypoints_;
    int dwell_per_waypoint_s_;
    std::vector<geometry_msgs::msg::Point> orbit_waypoints_;
    size_t orbit_idx_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZoneExplorerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
