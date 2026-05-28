#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "ros2_rad_map/msg/rad_zone_array.hpp"
#include "std_srvs/srv/trigger.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class ZoneExplorerNode : public rclcpp::Node
{
public:
    enum class Mode { EXPLORATION, SURVEILLANCE };

    ZoneExplorerNode() : Node("zone_explorer_node"), is_navigating_(false), orbit_idx_(0)
    {
        this->declare_parameter<std::string>("mode", "exploration");
        this->declare_parameter<std::string>("zones_topic", "/radiation_zones");
        this->declare_parameter<double>("survey_radius_m", 1.5);
        this->declare_parameter<int>("survey_waypoints", 6);
        this->declare_parameter<int>("dwell_per_waypoint_s", 2);
        // Exploration mode: service names to start/stop explore_lite
        this->declare_parameter<std::string>("explore_start_service", "explore_node/start");
        this->declare_parameter<std::string>("explore_stop_service", "explore_node/stop");

        std::string mode_str = this->get_parameter("mode").as_string();
        mode_ = (mode_str == "surveillance") ? Mode::SURVEILLANCE : Mode::EXPLORATION;

        std::string zones_topic = this->get_parameter("zones_topic").as_string();
        survey_radius_        = this->get_parameter("survey_radius_m").as_double();
        survey_waypoints_     = this->get_parameter("survey_waypoints").as_int();
        dwell_per_waypoint_s_ = this->get_parameter("dwell_per_waypoint_s").as_int();

        // Always subscribe to the zones topic to catch dynamically discovered sources
        zone_sub_ = this->create_subscription<ros2_rad_map::msg::RadZoneArray>(
            zones_topic, 10,
            std::bind(&ZoneExplorerNode::zoneCallback, this, std::placeholders::_1));

        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        if (mode_ == Mode::EXPLORATION) {
            std::string start_svc = this->get_parameter("explore_start_service").as_string();
            std::string stop_svc  = this->get_parameter("explore_stop_service").as_string();
            explore_start_client_ = this->create_client<std_srvs::srv::Trigger>(start_svc);
            explore_stop_client_  = this->create_client<std_srvs::srv::Trigger>(stop_svc);
            RCLCPP_INFO(this->get_logger(),
                "Mode: EXPLORATION — frontier navigation active. survey_radius=%.1fm, waypoints=%d, dwell=%ds",
                survey_radius_, survey_waypoints_, dwell_per_waypoint_s_);
        } else {
            RCLCPP_INFO(this->get_logger(),
                "Mode: SURVEILLANCE — waiting for zones on '%s'. survey_radius=%.1fm, waypoints=%d, dwell=%ds",
                this->get_parameter("zones_topic").as_string().c_str(),
                survey_radius_, survey_waypoints_, dwell_per_waypoint_s_);
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for navigation action server...");
        nav_action_client_->wait_for_action_server();
        RCLCPP_INFO(this->get_logger(), "Navigation action server is available.");

        if (mode_ == Mode::EXPLORATION) {
            callExploreStart();
        }

        process_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ZoneExplorerNode::processZoneQueue, this));
    }

private:
    void callExploreStart()
    {
        if (!explore_start_client_) return;
        if (!explore_start_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "explore start service unavailable; frontier navigation may not resume.");
            return;
        }
        explore_start_client_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>(),
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture) {
                RCLCPP_INFO(this->get_logger(), "Frontier exploration resumed.");
            });
    }

    void callExploreStop()
    {
        if (!explore_stop_client_) return;
        if (!explore_stop_client_->service_is_ready()) return;
        explore_stop_client_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>(),
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture) {
                RCLCPP_INFO(this->get_logger(), "Frontier exploration paused for zone survey.");
            });
    }

    // Receive dynamically discovered zones (both modes) and add new ones to the queue.
    void zoneCallback(const ros2_rad_map::msg::RadZoneArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        for (const auto& zone : msg->zones) {
            bool found = false;
            for (const auto& existing : zone_queue_) {
                if (existing.label == zone.label) { found = true; break; }
            }
            if (is_navigating_ && current_goal_.label == zone.label) found = true;

            if (!found) {
                RCLCPP_INFO(this->get_logger(), "New zone '%s' discovered at (%.2f, %.2f) — queued.",
                    zone.label.c_str(), zone.center.x, zone.center.y);
                zone_queue_.push_back(zone);
            }
        }
    }

    void processZoneQueue()
    {
        ros2_rad_map::msg::RadZone next_goal;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (is_navigating_ || zone_queue_.empty()) return;
            current_goal_ = zone_queue_.front();
            zone_queue_.pop_front();
            is_navigating_ = true;
            next_goal = current_goal_;
        }

        if (mode_ == Mode::EXPLORATION) {
            // Pause frontier navigation so it doesn't fight with our goal.
            callExploreStop();
            RCLCPP_INFO(this->get_logger(), "Navigating to zone '%s' at (%.2f, %.2f) for survey.",
                next_goal.label.c_str(), next_goal.center.x, next_goal.center.y);
            sendCenterGoal();
        } else {
            // Surveillance: skip navigating to the hotspot center — go straight to the orbit.
            // Nav2's radiation costmap layer will also route around high-dose areas naturally.
            RCLCPP_INFO(this->get_logger(),
                "Surveillance: beginning orbital survey of zone '%s' at (%.2f, %.2f).",
                next_goal.label.c_str(), next_goal.center.x, next_goal.center.y);
            startOrbitalSurvey();
        }
    }

    // Exploration mode: navigate to the zone center first, then orbit.
    void sendCenterGoal()
    {
        auto goal_msg = std::make_shared<NavigateToPose::Goal>();
        goal_msg->pose.header.frame_id = "map";
        goal_msg->pose.header.stamp    = this->get_clock()->now();
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
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
            is_navigating_ = false;
            if (mode_ == Mode::EXPLORATION) callExploreStart();
        }
    }

    void zoneResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
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
                RCLCPP_ERROR(this->get_logger(), "Unknown result for zone '%s'.", current_goal_.label.c_str());
                break;
        }
        is_navigating_ = false;
        if (mode_ == Mode::EXPLORATION) callExploreStart();
    }

    // Build equally-spaced orbit waypoints around the zone center.
    // In surveillance mode this is called directly (skipping center navigation).
    void startOrbitalSurvey()
    {
        orbit_waypoints_.clear();
        orbit_idx_ = 0;
        double cx = current_goal_.center.x;
        double cy = current_goal_.center.y;
        for (int i = 0; i < survey_waypoints_; ++i) {
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
        if (orbit_idx_ >= orbit_waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(),
                "Orbital survey of zone '%s' complete.", current_goal_.label.c_str());
            is_navigating_ = false;
            // In exploration mode, resume frontier nav when there are no more zones pending.
            if (mode_ == Mode::EXPLORATION) {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (zone_queue_.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Queue empty — resuming frontier exploration.");
                    callExploreStart();
                }
            }
            return;
        }

        const auto& wp  = orbit_waypoints_[orbit_idx_];
        double cx = current_goal_.center.x;
        double cy = current_goal_.center.y;
        double yaw = std::atan2(cy - wp.y, cx - wp.x);  // face inward toward zone center

        auto goal_msg = std::make_shared<NavigateToPose::Goal>();
        goal_msg->pose.header.frame_id = "map";
        goal_msg->pose.header.stamp    = this->get_clock()->now();
        goal_msg->pose.pose.position   = wp;
        goal_msg->pose.pose.orientation.z = std::sin(yaw / 2.0);
        goal_msg->pose.pose.orientation.w = std::cos(yaw / 2.0);

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        opts.result_callback =
            std::bind(&ZoneExplorerNode::orbitResultCallback, this, std::placeholders::_1);

        nav_action_client_->async_send_goal(*goal_msg, opts);
        RCLCPP_INFO(this->get_logger(), "Orbit point %zu/%zu at (%.2f, %.2f).",
            orbit_idx_ + 1, orbit_waypoints_.size(), wp.x, wp.y);
    }

    void orbitResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_WARN(this->get_logger(),
                "Orbit goal canceled (preempted). Aborting survey of zone '%s'.",
                current_goal_.label.c_str());
            orbit_waypoints_.clear();
            is_navigating_ = false;
            if (mode_ == Mode::EXPLORATION) callExploreStart();
            return;
        }

        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_WARN(this->get_logger(), "Orbit point %zu failed — skipping.", orbit_idx_);
        }

        orbit_idx_++;

        if (dwell_per_waypoint_s_ > 0) {
            dwell_timer_ = this->create_wall_timer(
                std::chrono::seconds(dwell_per_waypoint_s_),
                [this]() {
                    dwell_timer_->cancel();
                    dwell_timer_.reset();
                    sendNextOrbitGoal();
                });
        } else {
            sendNextOrbitGoal();
        }
    }

    // --- Members ---
    Mode mode_;

    rclcpp::Subscription<ros2_rad_map::msg::RadZoneArray>::SharedPtr zone_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp::TimerBase::SharedPtr process_timer_;
    rclcpp::TimerBase::SharedPtr dwell_timer_;

    // Exploration mode only
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr explore_start_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr explore_stop_client_;

    std::mutex queue_mutex_;
    std::deque<ros2_rad_map::msg::RadZone> zone_queue_;
    ros2_rad_map::msg::RadZone current_goal_;
    bool is_navigating_;

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
