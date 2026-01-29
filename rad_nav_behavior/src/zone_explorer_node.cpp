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
    ZoneExplorerNode() : Node("zone_explorer_node"), is_navigating_(false)
    {
        this->declare_parameter<std::string>("zones_topic", "/radiation_zones");
        this->declare_parameter<int>("secondary_scan_duration_s", 5);

        std::string zones_topic = this->get_parameter("zones_topic").as_string();
        secondary_scan_duration_s_ = this->get_parameter("secondary_scan_duration_s").as_int();

        zone_sub_ = this->create_subscription<ros2_rad_map::msg::RadZoneArray>(
            zones_topic, 10,
            std::bind(&ZoneExplorerNode::zoneCallback, this, std::placeholders::_1));

        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "Zone Explorer Node has started.");
        RCLCPP_INFO(this->get_logger(), "Waiting for navigation action server...");
        nav_action_client_->wait_for_action_server();
        RCLCPP_INFO(this->get_logger(), "Navigation action server is available.");

        // Timer to process the queue
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
            // A simple check to avoid adding duplicate zones by label
            bool found = false;
            for (const auto& existing_zone : zone_queue_)
            {
                if (existing_zone.label == zone.label)
                {
                    found = true;
                    break;
                }
            }
             // Also check the current goal
            if(is_navigating_ && current_goal_.label == zone.label)
            {
                found = true;
            }

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
        if (is_navigating_ || zone_queue_.empty())
        {
            return;
        }

        current_goal_ = zone_queue_.front();
        zone_queue_.pop_front();
        is_navigating_ = true;

        RCLCPP_INFO(this->get_logger(), "Processing next zone: '%s' at (%.2f, %.2f)",
            current_goal_.label.c_str(), current_goal_.center.x, current_goal_.center.y);

        auto goal_msg = std::make_shared<NavigateToPose::Goal>();
        goal_msg->pose.header.frame_id = "map";
        goal_msg->pose.header.stamp = this->get_clock()->now();
        goal_msg->pose.pose.position.x = current_goal_.center.x;
        goal_msg->pose.pose.position.y = current_goal_.center.y;
        goal_msg->pose.pose.orientation.w = 1.0; // Pointing forward

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ZoneExplorerNode::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&ZoneExplorerNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&ZoneExplorerNode::resultCallback, this, std::placeholders::_1);

        nav_action_client_->async_send_goal(*goal_msg, send_goal_options);
    }

    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            is_navigating_ = false;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedbackCallback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // Could log feedback if needed, e.g., distance remaining
        // RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f", feedback->distance_remaining);
    }

    void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Successfully navigated to zone '%s'.", current_goal_.label.c_str());
                RCLCPP_INFO(this->get_logger(), "Performing secondary scan for %d seconds...", secondary_scan_duration_s_);
                // This is a simple blocking sleep. In a more complex robot,
                // you might trigger a separate action or state.
                rclcpp::sleep_for(std::chrono::seconds(secondary_scan_duration_s_));
                RCLCPP_INFO(this->get_logger(), "Secondary scan complete.");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        is_navigating_ = false;
    }

    rclcpp::Subscription<ros2_rad_map::msg::RadZoneArray>::SharedPtr zone_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp::TimerBase::SharedPtr process_timer_;

    std::mutex queue_mutex_;
    std::deque<ros2_rad_map::msg::RadZone> zone_queue_;
    ros2_rad_map::msg::RadZone current_goal_;
    bool is_navigating_;
    int secondary_scan_duration_s_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZoneExplorerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
