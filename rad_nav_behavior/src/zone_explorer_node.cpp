#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <cmath>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "ros2_rad_map/msg/rad_zone_array.hpp"
#include "ros2_rad_map/srv/orbit_zone.hpp"
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
        this->declare_parameter<int>("orbit_waypoint_timeout_s", 60);
        this->declare_parameter<std::string>("explore_start_service", "explore_node/start");
        this->declare_parameter<std::string>("explore_stop_service", "explore_node/stop");

        std::string mode_str = this->get_parameter("mode").as_string();
        mode_ = (mode_str == "surveillance") ? Mode::SURVEILLANCE : Mode::EXPLORATION;

        std::string zones_topic   = this->get_parameter("zones_topic").as_string();
        survey_radius_            = this->get_parameter("survey_radius_m").as_double();
        survey_waypoints_         = this->get_parameter("survey_waypoints").as_int();
        dwell_per_waypoint_s_     = this->get_parameter("dwell_per_waypoint_s").as_int();
        orbit_waypoint_timeout_s_ = this->get_parameter("orbit_waypoint_timeout_s").as_int();

        zone_sub_ = this->create_subscription<ros2_rad_map::msg::RadZoneArray>(
            zones_topic, 10,
            std::bind(&ZoneExplorerNode::zoneCallback, this, std::placeholders::_1));

        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        std::string start_svc = this->get_parameter("explore_start_service").as_string();
        std::string stop_svc  = this->get_parameter("explore_stop_service").as_string();
        explore_start_client_ = this->create_client<std_srvs::srv::Trigger>(start_svc);
        explore_stop_client_  = this->create_client<std_srvs::srv::Trigger>(stop_svc);

        exploration_mode_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/exploration_mode",
            std::bind(&ZoneExplorerNode::setExplorationMode, this,
                std::placeholders::_1, std::placeholders::_2));
        surveillance_mode_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/surveillance_mode",
            std::bind(&ZoneExplorerNode::setSurveillanceMode, this,
                std::placeholders::_1, std::placeholders::_2));
        orbit_zone_srv_ = this->create_service<ros2_rad_map::srv::OrbitZone>(
            "~/orbit_zone",
            std::bind(&ZoneExplorerNode::orbitZoneCallback, this,
                std::placeholders::_1, std::placeholders::_2));
        clear_queue_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/clear_queue",
            std::bind(&ZoneExplorerNode::clearQueueCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Waiting for navigation action server...");
        nav_action_client_->wait_for_action_server();
        RCLCPP_INFO(this->get_logger(), "Navigation action server is available.");

        logMode();

        process_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ZoneExplorerNode::processZoneQueue, this));
    }

private:
    // -------------------------------------------------------------------------
    // Mode switch services
    // -------------------------------------------------------------------------

    void setExplorationMode(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (mode_ == Mode::EXPLORATION) {
            response->success = true;
            response->message = "Already in exploration mode.";
            return;
        }
        mode_ = Mode::EXPLORATION;
        RCLCPP_INFO(this->get_logger(), "Switched to EXPLORATION mode via service.");
        callExploreStart();
        response->success = true;
        response->message = "Switched to exploration mode. Frontier navigation started.";
    }

    void setSurveillanceMode(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (mode_ == Mode::SURVEILLANCE) {
            response->success = true;
            response->message = "Already in surveillance mode.";
            return;
        }
        mode_ = Mode::SURVEILLANCE;
        RCLCPP_INFO(this->get_logger(), "Switched to SURVEILLANCE mode via service.");
        callExploreStop(nullptr);
        response->success = true;
        response->message = "Switched to surveillance mode. Frontier navigation stopped.";
    }

    void clearQueueCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        size_t n = zone_queue_.size();
        zone_queue_.clear();
        RCLCPP_INFO(this->get_logger(), "Zone queue cleared (%zu zone(s) removed).", n);
        response->success = true;
        response->message = std::to_string(n) + " zone(s) cleared from queue.";
    }

    void orbitZoneCallback(
        const std::shared_ptr<ros2_rad_map::srv::OrbitZone::Request> request,
        std::shared_ptr<ros2_rad_map::srv::OrbitZone::Response> response)
    {
        ros2_rad_map::msg::RadZone z;
        z.label = request->label.empty()
            ? ("manual_" + std::to_string(this->now().seconds()))
            : request->label;
        z.center.x = request->x;
        z.center.y = request->y;
        z.center.z = 0.0;

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            zone_queue_.push_front(z);
        }

        RCLCPP_INFO(this->get_logger(),
            "orbit_zone service: queued '%s' at (%.2f, %.2f) — will survey next.",
            z.label.c_str(), z.center.x, z.center.y);

        response->success = true;
        response->message = "Zone '" + z.label + "' queued for orbital survey.";
    }

    // -------------------------------------------------------------------------
    // Explore start / stop
    // -------------------------------------------------------------------------

    void callExploreStart()
    {
        if (!explore_start_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_WARN(this->get_logger(),
                "explore start service unavailable after 10s; frontier navigation may not resume.");
            return;
        }
        explore_start_client_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>(),
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture) {
                RCLCPP_INFO(this->get_logger(), "Frontier exploration resumed.");
            });
    }

    // Stop explore, then call on_stopped() once the service confirms.
    // Pass nullptr when no follow-up action is needed.
    void callExploreStop(std::function<void()> on_stopped)
    {
        if (!explore_stop_client_->service_is_ready()) {
            if (on_stopped) on_stopped();
            return;
        }
        explore_stop_client_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>(),
            [this, on_stopped](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture) {
                RCLCPP_INFO(this->get_logger(), "Frontier exploration paused.");
                if (on_stopped) on_stopped();
            });
    }

    // -------------------------------------------------------------------------
    // Zone queue
    // -------------------------------------------------------------------------

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
                RCLCPP_INFO(this->get_logger(), "New zone '%s' at (%.2f, %.2f) — queued.",
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
            // Stop explore first; only send the nav goal once explore confirms.
            RCLCPP_INFO(this->get_logger(), "Stopping frontier nav before surveying zone '%s'.",
                next_goal.label.c_str());
            callExploreStop([this, next_goal]() {
                RCLCPP_INFO(this->get_logger(), "Navigating to zone '%s' at (%.2f, %.2f).",
                    next_goal.label.c_str(), next_goal.center.x, next_goal.center.y);
                sendCenterGoal();
            });
        } else {
            RCLCPP_INFO(this->get_logger(),
                "Surveillance: beginning orbital survey of zone '%s' at (%.2f, %.2f).",
                next_goal.label.c_str(), next_goal.center.x, next_goal.center.y);
            startOrbitalSurvey();
        }
    }

    // -------------------------------------------------------------------------
    // Center navigation (exploration mode)
    // -------------------------------------------------------------------------

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
                return;
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

    // -------------------------------------------------------------------------
    // Orbital survey
    // -------------------------------------------------------------------------

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
        cancelOrbitTimeout();

        if (orbit_idx_ >= orbit_waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(),
                "Orbital survey of zone '%s' complete.", current_goal_.label.c_str());
            is_navigating_ = false;
            if (mode_ == Mode::EXPLORATION) {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (zone_queue_.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Queue empty — resuming frontier exploration.");
                    callExploreStart();
                }
            }
            return;
        }

        const auto& wp = orbit_waypoints_[orbit_idx_];
        double cx  = current_goal_.center.x;
        double cy  = current_goal_.center.y;
        double yaw = std::atan2(cy - wp.y, cx - wp.x);

        auto goal_msg = std::make_shared<NavigateToPose::Goal>();
        goal_msg->pose.header.frame_id = "map";
        goal_msg->pose.header.stamp    = this->get_clock()->now();
        goal_msg->pose.pose.position   = wp;
        goal_msg->pose.pose.orientation.z = std::sin(yaw / 2.0);
        goal_msg->pose.pose.orientation.w = std::cos(yaw / 2.0);

        orbit_timed_out_ = false;

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        opts.goal_response_callback =
            [this](const GoalHandleNavigateToPose::SharedPtr & handle) {
                if (!handle) {
                    // Nav2 rejected the goal outright (e.g. goal in lethal obstacle).
                    // result_callback will never fire, so we must advance here.
                    RCLCPP_WARN(this->get_logger(),
                        "Orbit waypoint %zu rejected by Nav2 — skipping.", orbit_idx_);
                    cancelOrbitTimeout();
                    orbit_idx_++;
                    sendNextOrbitGoal();
                    return;
                }
                current_orbit_goal_handle_ = handle;
            };
        opts.result_callback =
            std::bind(&ZoneExplorerNode::orbitResultCallback, this, std::placeholders::_1);

        nav_action_client_->async_send_goal(*goal_msg, opts);
        RCLCPP_INFO(this->get_logger(), "Orbit point %zu/%zu at (%.2f, %.2f) — timeout %ds.",
            orbit_idx_ + 1, orbit_waypoints_.size(), wp.x, wp.y, orbit_waypoint_timeout_s_);

        orbit_timeout_timer_ = this->create_wall_timer(
            std::chrono::seconds(orbit_waypoint_timeout_s_),
            [this]() {
                orbit_timed_out_ = true;
                cancelOrbitTimeout();
                RCLCPP_WARN(this->get_logger(),
                    "Orbit waypoint %zu timed out after %ds — skipping.",
                    orbit_idx_, orbit_waypoint_timeout_s_);
                if (current_orbit_goal_handle_) {
                    nav_action_client_->async_cancel_goal(current_orbit_goal_handle_);
                    current_orbit_goal_handle_.reset();
                }
            });
    }

    void cancelOrbitTimeout()
    {
        if (orbit_timeout_timer_) {
            orbit_timeout_timer_->cancel();
            orbit_timeout_timer_.reset();
        }
    }

    void orbitResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        cancelOrbitTimeout();
        current_orbit_goal_handle_.reset();

        if (result.code == rclcpp_action::ResultCode::CANCELED) {
            if (!orbit_timed_out_) {
                // External preempt (e.g. rad_safety canceling the goal) — abort survey.
                RCLCPP_WARN(this->get_logger(),
                    "Orbit goal externally preempted. Aborting survey of zone '%s'.",
                    current_goal_.label.c_str());
                orbit_waypoints_.clear();
                is_navigating_ = false;
                if (mode_ == Mode::EXPLORATION) callExploreStart();
                return;
            }
            // Timeout skip — fall through and advance index.
        } else if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
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

    void logMode()
    {
        if (mode_ == Mode::EXPLORATION) {
            RCLCPP_INFO(this->get_logger(),
                "Mode: EXPLORATION — frontier navigation active. "
                "survey_radius=%.1fm, waypoints=%d, dwell=%ds, waypoint_timeout=%ds",
                survey_radius_, survey_waypoints_, dwell_per_waypoint_s_, orbit_waypoint_timeout_s_);
        } else {
            RCLCPP_INFO(this->get_logger(),
                "Mode: SURVEILLANCE — orbiting known zones, no frontier nav. "
                "survey_radius=%.1fm, waypoints=%d, dwell=%ds, waypoint_timeout=%ds",
                survey_radius_, survey_waypoints_, dwell_per_waypoint_s_, orbit_waypoint_timeout_s_);
        }
    }

    // --- Members ---
    Mode mode_;

    rclcpp::Subscription<ros2_rad_map::msg::RadZoneArray>::SharedPtr zone_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp::TimerBase::SharedPtr process_timer_;
    rclcpp::TimerBase::SharedPtr dwell_timer_;
    rclcpp::TimerBase::SharedPtr orbit_timeout_timer_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr explore_start_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr explore_stop_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr exploration_mode_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr surveillance_mode_srv_;
    rclcpp::Service<ros2_rad_map::srv::OrbitZone>::SharedPtr orbit_zone_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_queue_srv_;

    GoalHandleNavigateToPose::SharedPtr current_orbit_goal_handle_;
    bool orbit_timed_out_{false};

    std::mutex queue_mutex_;
    std::deque<ros2_rad_map::msg::RadZone> zone_queue_;
    ros2_rad_map::msg::RadZone current_goal_;
    bool is_navigating_;

    double survey_radius_;
    int survey_waypoints_;
    int dwell_per_waypoint_s_;
    int orbit_waypoint_timeout_s_;
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
